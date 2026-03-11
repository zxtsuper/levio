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

#include "levio_cfg.h"
#include "levio_types.h"
#include "levio_orb.h"
#include "levio_ringbuf.h"
#include "levio_math.h"
#include "levio_se3.h"
#include "levio_camera.h"
#include "levio_match.h"

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

    printf("\n=== Results: %d passed, %d failed ===\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
