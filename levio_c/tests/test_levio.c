/**
 * @file test_levio.c
 * @brief Minimal unit tests for LEVIO components.
 *
 * Tests that can be run without a camera or IMU:
 *   1) Hamming distance correctness
 *   2) Ring buffer push / pop / overflow
 *   3) vec3 / mat3 linear algebra
 *   4) SO3 exp / log round-trip
 *   5) Camera projection / unprojection round-trip
 *   6) Bidirectional matcher
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <errno.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <zlib.h>

#include "levio_cfg.h"
#include "levio_types.h"
#include "levio_orb.h"
#include "levio_ringbuf.h"
#include "levio_math.h"
#include "levio_se3.h"
#include "levio_camera.h"
#include "levio_match.h"
#include "euroc_dataset.h"

/* --------------------------------------------------------------------------
 * Helpers
 * -------------------------------------------------------------------------- */
#define EXPECT(cond, msg) \
    do { \
        if (!(cond)) { \
            printf("FAIL [%s:%d] %s\n", __FILE__, __LINE__, (msg)); \
            return 1; \
        } \
    } while(0)

#define EXPECT_NEAR(a, b, tol, msg) \
    EXPECT(fabsf((float)(a) - (float)(b)) < (float)(tol), msg)

static int g_pass = 0, g_fail = 0;
#define RUN_TEST(fn) \
    do { \
        printf("  %-40s ", #fn); \
        if (fn() == 0) { printf("PASS\n"); ++g_pass; } \
        else            { printf("FAIL\n"); ++g_fail; } \
    } while(0)

/* --------------------------------------------------------------------------
 * Test 1: Hamming distance
 * -------------------------------------------------------------------------- */
static int test_hamming_zero(void)
{
    uint8_t a[LEVIO_DESC_BYTES], b[LEVIO_DESC_BYTES];
    memset(a, 0xAA, LEVIO_DESC_BYTES);
    memset(b, 0xAA, LEVIO_DESC_BYTES);
    EXPECT(levio_orb_hamming(a, b) == 0, "identical descriptors → dist 0");
    return 0;
}

static int test_hamming_all_bits(void)
{
    uint8_t a[LEVIO_DESC_BYTES], b[LEVIO_DESC_BYTES];
    memset(a, 0x00, LEVIO_DESC_BYTES);
    memset(b, 0xFF, LEVIO_DESC_BYTES);
    uint32_t d = levio_orb_hamming(a, b);
    EXPECT(d == 256, "all-zero vs all-ones → dist 256");
    return 0;
}

static int test_hamming_single_bit(void)
{
    uint8_t a[LEVIO_DESC_BYTES], b[LEVIO_DESC_BYTES];
    memset(a, 0x00, LEVIO_DESC_BYTES);
    memset(b, 0x00, LEVIO_DESC_BYTES);
    b[5] = 0x01;  /* flip one bit */
    EXPECT(levio_orb_hamming(a, b) == 1, "one bit differs → dist 1");
    return 0;
}

/* --------------------------------------------------------------------------
 * Test 2: Ring buffer
 * -------------------------------------------------------------------------- */
static int test_ringbuf_empty(void)
{
    levio_ringbuf_t rb;
    levio_ringbuf_init(&rb);
    EXPECT(levio_ringbuf_empty(&rb), "freshly initialised buffer is empty");
    EXPECT(!levio_ringbuf_full(&rb), "freshly initialised buffer is not full");
    EXPECT(levio_ringbuf_size(&rb) == 0, "size == 0 after init");
    levio_imu_sample_t s;
    EXPECT(levio_ringbuf_pop(&rb, &s) == -1, "pop from empty returns -1");
    return 0;
}

static int test_ringbuf_push_pop(void)
{
    levio_ringbuf_t rb;
    levio_ringbuf_init(&rb);

    levio_imu_sample_t s_in, s_out;
    s_in.t = 1.0;
    s_in.acc.x  = 1.0f; s_in.acc.y  = 2.0f; s_in.acc.z  = 3.0f;
    s_in.gyro.x = 4.0f; s_in.gyro.y = 5.0f; s_in.gyro.z = 6.0f;

    EXPECT(levio_ringbuf_push(&rb, &s_in) == 0, "push succeeds");
    EXPECT(levio_ringbuf_size(&rb) == 1, "size == 1 after push");
    EXPECT(levio_ringbuf_pop(&rb, &s_out) == 0, "pop succeeds");
    EXPECT_NEAR(s_out.t, 1.0, 1e-9, "timestamp preserved");
    EXPECT_NEAR(s_out.acc.z, 3.0f, 1e-6f, "accelerometer z preserved");
    EXPECT(levio_ringbuf_empty(&rb), "empty after push+pop");
    return 0;
}

static int test_ringbuf_fifo_order(void)
{
    levio_ringbuf_t rb;
    levio_ringbuf_init(&rb);

    int i;
    for (i = 0; i < 5; ++i) {
        levio_imu_sample_t s;
        s.t = (double)i;
        s.acc.x = s.acc.y = s.acc.z = 0.0f;
        s.gyro.x = s.gyro.y = s.gyro.z = 0.0f;
        EXPECT(levio_ringbuf_push(&rb, &s) == 0, "push i");
    }
    for (i = 0; i < 5; ++i) {
        levio_imu_sample_t s;
        EXPECT(levio_ringbuf_pop(&rb, &s) == 0, "pop i");
        EXPECT_NEAR(s.t, (double)i, 1e-9, "FIFO order preserved");
    }
    return 0;
}

static int test_ringbuf_overflow(void)
{
    levio_ringbuf_t rb;
    levio_ringbuf_init(&rb);

    levio_imu_sample_t s;
    memset(&s, 0, sizeof(s));

    /* Fill to capacity (LEVIO_IMU_BUF_SIZE - 1 samples max) */
    int i;
    for (i = 0; i < LEVIO_IMU_BUF_SIZE - 1; ++i)
        EXPECT(levio_ringbuf_push(&rb, &s) == 0, "push until full");

    EXPECT(levio_ringbuf_full(&rb), "buffer reports full");
    EXPECT(levio_ringbuf_push(&rb, &s) == -1, "push on full returns -1");
    return 0;
}

/* --------------------------------------------------------------------------
 * Test 3: Linear algebra
 * -------------------------------------------------------------------------- */
static int test_vec3_ops(void)
{
    vec3f_t a = {1.0f, 2.0f, 3.0f};
    vec3f_t b = {4.0f, 5.0f, 6.0f};

    vec3f_t s = levio_vec3_sub(a, b);
    EXPECT_NEAR(s.x, -3.0f, 1e-6f, "sub x");
    EXPECT_NEAR(s.y, -3.0f, 1e-6f, "sub y");
    EXPECT_NEAR(s.z, -3.0f, 1e-6f, "sub z");

    float dot = levio_vec3_dot(a, b);
    EXPECT_NEAR(dot, 32.0f, 1e-5f, "dot product");

    vec3f_t cross = levio_vec3_cross(a, b);
    EXPECT_NEAR(cross.x, -3.0f, 1e-5f, "cross x");
    EXPECT_NEAR(cross.y,  6.0f, 1e-5f, "cross y");
    EXPECT_NEAR(cross.z, -3.0f, 1e-5f, "cross z");

    float n = levio_vec3_norm(a);
    EXPECT_NEAR(n, sqrtf(14.0f), 1e-5f, "norm");
    return 0;
}

static int test_mat3_identity_mul(void)
{
    mat3f_t I = levio_mat3_identity();
    vec3f_t v = {1.0f, 2.0f, 3.0f};
    vec3f_t r = levio_mat3_mul_vec3(I, v);
    EXPECT_NEAR(r.x, 1.0f, 1e-6f, "I*v x");
    EXPECT_NEAR(r.y, 2.0f, 1e-6f, "I*v y");
    EXPECT_NEAR(r.z, 3.0f, 1e-6f, "I*v z");
    return 0;
}

static int test_mat3_inv(void)
{
    mat3f_t A;
    A.m[0] = 2.0f; A.m[1] = 1.0f; A.m[2] = 0.0f;
    A.m[3] = 1.0f; A.m[4] = 3.0f; A.m[5] = 1.0f;
    A.m[6] = 0.0f; A.m[7] = 1.0f; A.m[8] = 2.0f;

    mat3f_t Ai = levio_mat3_inv(A);
    mat3f_t P  = levio_mat3_mul(A, Ai);

    /* P should be close to identity */
    EXPECT_NEAR(P.m[0], 1.0f, 1e-5f, "A*A^-1[0,0]");
    EXPECT_NEAR(P.m[4], 1.0f, 1e-5f, "A*A^-1[1,1]");
    EXPECT_NEAR(P.m[8], 1.0f, 1e-5f, "A*A^-1[2,2]");
    EXPECT_NEAR(P.m[1], 0.0f, 1e-5f, "A*A^-1[0,1]");
    return 0;
}

/* --------------------------------------------------------------------------
 * Test 4: SO3 exp / log round-trip
 * -------------------------------------------------------------------------- */
static int test_so3_exp_log(void)
{
    vec3f_t omega = {0.1f, 0.2f, 0.3f};
    mat3f_t R     = levio_so3_exp(omega);
    vec3f_t omega2 = levio_so3_log(R);

    EXPECT_NEAR(omega2.x, omega.x, 1e-5f, "so3 round-trip x");
    EXPECT_NEAR(omega2.y, omega.y, 1e-5f, "so3 round-trip y");
    EXPECT_NEAR(omega2.z, omega.z, 1e-5f, "so3 round-trip z");
    return 0;
}

static int test_so3_identity(void)
{
    vec3f_t zero = {0.0f, 0.0f, 0.0f};
    mat3f_t R    = levio_so3_exp(zero);
    mat3f_t I    = levio_mat3_identity();
    int i;
    for (i = 0; i < 9; ++i)
        EXPECT_NEAR(R.m[i], I.m[i], 1e-6f, "exp(0) == I");
    return 0;
}

/* --------------------------------------------------------------------------
 * Test 5: Camera projection / unprojection
 * -------------------------------------------------------------------------- */
static int test_camera_project_unproject(void)
{
    levio_K_t K;
    K.fx = 100.0f; K.fy = 100.0f;
    K.cx =  80.0f; K.cy =  60.0f;

    vec3f_t Pc = {1.0f, 2.0f, 5.0f};
    vec2f_t px;
    EXPECT(levio_project(&K, Pc, &px) == 0, "project succeeds");
    EXPECT_NEAR(px.x, K.fx * Pc.x / Pc.z + K.cx, 1e-4f, "project x");
    EXPECT_NEAR(px.y, K.fy * Pc.y / Pc.z + K.cy, 1e-4f, "project y");

    vec3f_t ray = levio_unproject(&K, px);
    /* ray.x / ray.z should equal Pc.x / Pc.z */
    EXPECT_NEAR(ray.x / ray.z, Pc.x / Pc.z, 1e-5f, "unproject x/z");
    EXPECT_NEAR(ray.y / ray.z, Pc.y / Pc.z, 1e-5f, "unproject y/z");
    return 0;
}

static int test_camera_behind(void)
{
    levio_K_t K = {100.0f, 100.0f, 80.0f, 60.0f};
    vec3f_t Pc = {1.0f, 1.0f, -1.0f};  /* behind camera */
    vec2f_t px;
    EXPECT(levio_project(&K, Pc, &px) == -1, "project behind camera returns -1");
    return 0;
}

/* --------------------------------------------------------------------------
 * Test 6: Bidirectional matcher
 * -------------------------------------------------------------------------- */
static int test_match_self(void)
{
    levio_feat_t feats[3];
    int i;
    for (i = 0; i < 3; ++i) {
        feats[i].px.x = (float)i;
        feats[i].px.y = 0.0f;
        feats[i].lm_id = -1;
        memset(feats[i].desc, 0, LEVIO_DESC_BYTES);
        feats[i].desc[0] = (uint8_t)(i * 10);
        feats[i].desc[1] = (uint8_t)(i * 20);
    }

    levio_match_t matches[3];
    int n = levio_match_bidir(feats, 3, feats, 3, matches, 3);
    /* Matching a set against itself: each point should match itself */
    EXPECT(n == 3, "self-match: expect 3 matches");
    for (i = 0; i < n; ++i)
        EXPECT(matches[i].idx_a == matches[i].idx_b, "self-match: idx_a == idx_b");
    return 0;
}

static int test_match_no_match(void)
{
    levio_feat_t a[2], b[2];
    int i;
    for (i = 0; i < 2; ++i) {
        memset(a[i].desc, 0x00, LEVIO_DESC_BYTES);
        memset(b[i].desc, 0xFF, LEVIO_DESC_BYTES);  /* completely different */
        a[i].lm_id = b[i].lm_id = -1;
        a[i].px.x = b[i].px.x = 0.0f;
        a[i].px.y = b[i].px.y = 0.0f;
    }
    /* With cross-check, a[0] → b[0] but b[0] → a[0] (both equidistant);
     * the matcher will still find 2 matches here because both have the same
     * distance to all b. Verify no crash. */
    levio_match_t out[4];
    int n = levio_match_bidir(a, 2, b, 2, out, 4);
    EXPECT(n >= 0, "match returns non-negative count");
    return 0;
}

/* --------------------------------------------------------------------------
 * Test 7: EuRoC dataset helpers
 * -------------------------------------------------------------------------- */
static int test_mkdir_p(const char *path)
{
    if (mkdir(path, 0777) == 0 || errno == EEXIST)
        return 0;
    return -1;
}

static int test_write_file(const char *path, const void *data, size_t size)
{
    FILE *fp = fopen(path, "wb");
    if (fp == NULL)
        return -1;
    if (fwrite(data, 1, size, fp) != size) {
        fclose(fp);
        return -1;
    }
    fclose(fp);
    return 0;
}

static int test_join_path(char *dst, size_t dst_size,
                          const char *a, const char *b)
{
    int n = snprintf(dst, dst_size, "%s/%s", a, b);
    return (n > 0 && (size_t)n < dst_size) ? 0 : -1;
}

static void write_be32(uint8_t *dst, uint32_t v)
{
    dst[0] = (uint8_t)(v >> 24);
    dst[1] = (uint8_t)(v >> 16);
    dst[2] = (uint8_t)(v >> 8);
    dst[3] = (uint8_t)v;
}

static int append_png_chunk(uint8_t **buf, size_t *size,
                            const char type[4],
                            const uint8_t *data, uint32_t data_size)
{
    size_t old_size = *size;
    size_t new_size = old_size + 12u + (size_t)data_size;
    uint8_t *tmp = (uint8_t *)realloc(*buf, new_size);
    uint32_t crc;

    if (tmp == NULL)
        return -1;
    *buf = tmp;

    write_be32(tmp + old_size, data_size);
    memcpy(tmp + old_size + 4u, type, 4);
    if (data_size > 0)
        memcpy(tmp + old_size + 8u, data, data_size);

    crc = crc32(0L, Z_NULL, 0);
    crc = crc32(crc, (const Bytef *)(tmp + old_size + 4u), 4u + data_size);
    write_be32(tmp + old_size + 8u + data_size, crc);
    *size = new_size;
    return 0;
}

static int test_write_gray_png(const char *path, int w, int h, const uint8_t *pixels)
{
    static const uint8_t sig[8] = {
        0x89, 'P', 'N', 'G', '\r', '\n', 0x1A, '\n'
    };
    uint8_t ihdr[13];
    uint8_t *raw = NULL;
    uint8_t *compressed = NULL;
    uint8_t *png = NULL;
    size_t png_size = 0;
    uLongf compressed_size;
    int y;

    raw = (uint8_t *)malloc((size_t)(w + 1) * (size_t)h);
    if (raw == NULL)
        return -1;

    for (y = 0; y < h; ++y) {
        raw[y * (w + 1)] = 0;
        memcpy(raw + y * (w + 1) + 1, pixels + y * w, (size_t)w);
    }

    compressed_size = compressBound((uLong)((size_t)(w + 1) * (size_t)h));
    compressed = (uint8_t *)malloc((size_t)compressed_size);
    if (compressed == NULL) {
        free(raw);
        return -1;
    }

    if (compress2(compressed, &compressed_size, raw,
                  (uLong)((size_t)(w + 1) * (size_t)h), Z_BEST_SPEED) != Z_OK) {
        free(raw);
        free(compressed);
        return -1;
    }

    png = (uint8_t *)malloc(sizeof(sig));
    if (png == NULL) {
        free(raw);
        free(compressed);
        return -1;
    }
    memcpy(png, sig, sizeof(sig));
    png_size = sizeof(sig);

    write_be32(ihdr + 0, (uint32_t)w);
    write_be32(ihdr + 4, (uint32_t)h);
    ihdr[8] = 8;  /* bit depth */
    ihdr[9] = 0;  /* grayscale */
    ihdr[10] = 0; /* compression */
    ihdr[11] = 0; /* filter */
    ihdr[12] = 0; /* interlace */

    if (append_png_chunk(&png, &png_size, "IHDR", ihdr, sizeof(ihdr)) != 0 ||
        append_png_chunk(&png, &png_size, "IDAT", compressed,
                         (uint32_t)compressed_size) != 0 ||
        append_png_chunk(&png, &png_size, "IEND", NULL, 0) != 0 ||
        test_write_file(path, png, png_size) != 0) {
        free(raw);
        free(compressed);
        free(png);
        return -1;
    }

    free(raw);
    free(compressed);
    free(png);
    return 0;
}

static int test_prepare_euroc_fixture(char *root, size_t root_size)
{
    char mav0[512], imu0[512], cam0[512], cam_data[512];
    char imu_csv[512], cam_csv[512], image_path[512];
    static const char imu_data[] =
        "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n"
        "1000000000,0.1,0.2,0.3,1.0,2.0,3.0\n"
        "1050000000,0.4,0.5,0.6,4.0,5.0,6.0\n";
    static const char cam_data_csv[] =
        "#timestamp [ns],filename\n"
        "1020000000,1000000000.png\n";
    static const uint8_t pixels[16] = {
         1,  2,  3,  4,
         5,  6,  7,  8,
         9, 10, 11, 12,
        13, 14, 15, 16
    };

    if (snprintf(root, root_size, "/tmp/levio_euroc_fixture_%ld",
                 (long)getpid()) <= 0)
        return -1;
    if (test_mkdir_p(root) != 0)
        return -1;

    EXPECT(test_join_path(mav0, sizeof(mav0), root, "mav0") == 0,
           "build mav0 path");
    EXPECT(test_join_path(imu0, sizeof(imu0), mav0, "imu0") == 0,
           "build imu path");
    EXPECT(test_join_path(cam0, sizeof(cam0), mav0, "cam0") == 0,
           "build cam path");
    EXPECT(test_join_path(cam_data, sizeof(cam_data), cam0, "data") == 0,
           "build cam data path");
    if (test_mkdir_p(mav0) != 0 || test_mkdir_p(imu0) != 0 ||
        test_mkdir_p(cam0) != 0 || test_mkdir_p(cam_data) != 0)
        return -1;

    EXPECT(test_join_path(imu_csv, sizeof(imu_csv), imu0, "data.csv") == 0,
           "build imu csv path");
    EXPECT(test_join_path(cam_csv, sizeof(cam_csv), cam0, "data.csv") == 0,
           "build cam csv path");
    EXPECT(test_join_path(image_path, sizeof(image_path), cam_data,
                          "1000000000.png") == 0,
           "build image path");
    if (test_write_file(imu_csv, imu_data, sizeof(imu_data) - 1u) != 0 ||
        test_write_file(cam_csv, cam_data_csv, sizeof(cam_data_csv) - 1u) != 0 ||
        test_write_gray_png(image_path, 4, 4, pixels) != 0)
        return -1;

    return 0;
}

static int test_euroc_readers_and_image(void)
{
    char root[512];
    char resolved[LEVIO_EUROC_PATH_MAX];
    uint8_t img[4];
    levio_euroc_imu_reader_t imu_reader;
    levio_euroc_cam_reader_t cam_reader;
    levio_euroc_imu_sample_t imu;
    levio_euroc_cam_sample_t cam;
    int src_w = 0, src_h = 0;

    memset(&imu_reader, 0, sizeof(imu_reader));
    memset(&cam_reader, 0, sizeof(cam_reader));

    EXPECT(test_prepare_euroc_fixture(root, sizeof(root)) == 0,
           "prepare EuRoC fixture");
    EXPECT(levio_euroc_resolve_mav0_root(root, resolved, sizeof(resolved)) == 0,
           "resolve sequence root to mav0");
    EXPECT(strstr(resolved, "/mav0") != NULL, "resolved path ends with mav0");

    EXPECT(levio_euroc_open_imu_reader(&imu_reader, root) == 0,
           "open IMU reader");
    EXPECT(levio_euroc_open_cam_reader(&cam_reader, root) == 0,
           "open camera reader");

    EXPECT(levio_euroc_read_next_imu(&imu_reader, &imu) == 1,
           "read first IMU sample");
    EXPECT_NEAR(imu.t, 1.0, 1e-9, "IMU timestamp converted to seconds");
    EXPECT_NEAR(imu.gz, 0.3f, 1e-6f, "IMU gyro z parsed");
    EXPECT_NEAR(imu.az, 3.0f, 1e-6f, "IMU accel z parsed");

    EXPECT(levio_euroc_read_next_cam(&cam_reader, &cam) == 1,
           "read first camera sample");
    EXPECT_NEAR(cam.t, 1.02, 1e-9, "camera timestamp converted to seconds");
    EXPECT(strstr(cam.image_path, "1000000000.png") != NULL,
           "camera image path resolved");

    EXPECT(levio_euroc_load_image_gray(cam.image_path, img, 2, 2,
                                       &src_w, &src_h) == 0,
           "load and scale grayscale PNG");
    EXPECT(src_w == 4 && src_h == 4, "source image size preserved");
    EXPECT(img[0] == 1, "scaled pixel [0,0]");
    EXPECT(img[1] == 3, "scaled pixel [0,1]");
    EXPECT(img[2] == 9, "scaled pixel [1,0]");
    EXPECT(img[3] == 11, "scaled pixel [1,1]");

    levio_euroc_close_imu_reader(&imu_reader);
    levio_euroc_close_cam_reader(&cam_reader);
    return 0;
}

/* --------------------------------------------------------------------------
 * Main
 * -------------------------------------------------------------------------- */
int main(void)
{
    printf("=== LEVIO unit tests ===\n\n");

    printf("Hamming distance:\n");
    RUN_TEST(test_hamming_zero);
    RUN_TEST(test_hamming_all_bits);
    RUN_TEST(test_hamming_single_bit);

    printf("Ring buffer:\n");
    RUN_TEST(test_ringbuf_empty);
    RUN_TEST(test_ringbuf_push_pop);
    RUN_TEST(test_ringbuf_fifo_order);
    RUN_TEST(test_ringbuf_overflow);

    printf("Linear algebra:\n");
    RUN_TEST(test_vec3_ops);
    RUN_TEST(test_mat3_identity_mul);
    RUN_TEST(test_mat3_inv);

    printf("SO3:\n");
    RUN_TEST(test_so3_exp_log);
    RUN_TEST(test_so3_identity);

    printf("Camera:\n");
    RUN_TEST(test_camera_project_unproject);
    RUN_TEST(test_camera_behind);

    printf("Matcher:\n");
    RUN_TEST(test_match_self);
    RUN_TEST(test_match_no_match);

    printf("EuRoC dataset:\n");
    RUN_TEST(test_euroc_readers_and_image);

    printf("\n=== Results: %d passed, %d failed ===\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
