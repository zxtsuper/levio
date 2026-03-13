#include "euroc_dataset.h"

#include <errno.h>
#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#include <zlib.h>

static int path_exists(const char *path)
{
    struct stat st;
    return stat(path, &st) == 0;
}

static int join_path(char *dst, size_t dst_size,
                     const char *a, const char *b)
{
    int n = snprintf(dst, dst_size, "%s/%s", a, b);
    return (n > 0 && (size_t)n < dst_size) ? 0 : -1;
}

static int next_data_line(FILE *fp, char *line, size_t line_size)
{
    if (line_size > (size_t)INT_MAX)
        return 0;

    while (fgets(line, (int)line_size, fp) != NULL) {
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r')
            continue;
        return 1;
    }
    return 0;
}

int levio_euroc_resolve_mav0_root(const char *root,
                                  char *mav0_root, size_t mav0_root_size)
{
    char path[LEVIO_EUROC_PATH_MAX];

    if (root == NULL || mav0_root == NULL || mav0_root_size == 0)
        return -1;

    if (join_path(path, sizeof(path), root, "imu0/data.csv") == 0 &&
        path_exists(path) &&
        join_path(path, sizeof(path), root, "cam0/data.csv") == 0 &&
        path_exists(path)) {
        int n = snprintf(mav0_root, mav0_root_size, "%s", root);
        return (n > 0 && (size_t)n < mav0_root_size) ? 0 : -1;
    }

    if (join_path(path, sizeof(path), root, "mav0/imu0/data.csv") == 0 &&
        path_exists(path) &&
        join_path(path, sizeof(path), root, "mav0/cam0/data.csv") == 0 &&
        path_exists(path)) {
        int n = snprintf(mav0_root, mav0_root_size, "%s/mav0", root);
        return (n > 0 && (size_t)n < mav0_root_size) ? 0 : -1;
    }

    return -1;
}

int levio_euroc_open_imu_reader(levio_euroc_imu_reader_t *reader,
                                const char *root)
{
    char mav0_root[LEVIO_EUROC_PATH_MAX];
    char csv_path[LEVIO_EUROC_PATH_MAX];

    if (reader == NULL || levio_euroc_resolve_mav0_root(root, mav0_root,
                                                        sizeof(mav0_root)) != 0)
        return -1;

    if (join_path(csv_path, sizeof(csv_path), mav0_root, "imu0/data.csv") != 0)
        return -1;

    reader->fp = fopen(csv_path, "r");
    return (reader->fp != NULL) ? 0 : -1;
}

int levio_euroc_open_cam_reader(levio_euroc_cam_reader_t *reader,
                                const char *root)
{
    char mav0_root[LEVIO_EUROC_PATH_MAX];
    char csv_path[LEVIO_EUROC_PATH_MAX];

    if (reader == NULL || levio_euroc_resolve_mav0_root(root, mav0_root,
                                                        sizeof(mav0_root)) != 0)
        return -1;

    if (join_path(csv_path, sizeof(csv_path), mav0_root, "cam0/data.csv") != 0 ||
        join_path(reader->data_dir, sizeof(reader->data_dir), mav0_root,
                  "cam0/data") != 0)
        return -1;

    reader->fp = fopen(csv_path, "r");
    return (reader->fp != NULL) ? 0 : -1;
}

void levio_euroc_close_imu_reader(levio_euroc_imu_reader_t *reader)
{
    if (reader != NULL && reader->fp != NULL) {
        fclose(reader->fp);
        reader->fp = NULL;
    }
}

void levio_euroc_close_cam_reader(levio_euroc_cam_reader_t *reader)
{
    if (reader != NULL && reader->fp != NULL) {
        fclose(reader->fp);
        reader->fp = NULL;
    }
}

int levio_euroc_read_next_imu(levio_euroc_imu_reader_t *reader,
                              levio_euroc_imu_sample_t *sample)
{
    char line[512];
    unsigned long long timestamp_ns;
    double gx, gy, gz, ax, ay, az;

    if (reader == NULL || sample == NULL || reader->fp == NULL)
        return -1;

    if (!next_data_line(reader->fp, line, sizeof(line)))
        return 0;

    if (sscanf(line, "%llu,%lf,%lf,%lf,%lf,%lf,%lf",
               &timestamp_ns, &gx, &gy, &gz, &ax, &ay, &az) != 7)
        return -1;

    sample->timestamp_ns = (uint64_t)timestamp_ns;
    sample->t = (double)timestamp_ns * 1e-9;
    sample->gx = (float)gx; sample->gy = (float)gy; sample->gz = (float)gz;
    sample->ax = (float)ax; sample->ay = (float)ay; sample->az = (float)az;
    return 1;
}

int levio_euroc_read_next_cam(levio_euroc_cam_reader_t *reader,
                              levio_euroc_cam_sample_t *sample)
{
    char line[512];
    unsigned long long timestamp_ns;
    char filename[256];

    if (reader == NULL || sample == NULL || reader->fp == NULL)
        return -1;

    if (!next_data_line(reader->fp, line, sizeof(line)))
        return 0;

    if (sscanf(line, "%llu,%255[^\r\n]", &timestamp_ns, filename) != 2)
        return -1;

    sample->timestamp_ns = (uint64_t)timestamp_ns;
    sample->t = (double)timestamp_ns * 1e-9;
    return join_path(sample->image_path, sizeof(sample->image_path),
                     reader->data_dir, filename) == 0 ? 1 : -1;
}

static uint32_t read_be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8)  | (uint32_t)p[3];
}

static uint8_t paeth_predictor(uint8_t a, uint8_t b, uint8_t c)
{
    int p = (int)a + (int)b - (int)c;
    int pa = abs(p - (int)a);
    int pb = abs(p - (int)b);
    int pc = abs(p - (int)c);

    if (pa <= pb && pa <= pc) return a;
    if (pb <= pc) return b;
    return c;
}

static int decode_png_gray8(const uint8_t *png_data, size_t png_size,
                            uint8_t **out_pixels, int *width, int *height)
{
    static const uint8_t png_sig[8] = {
        0x89, 'P', 'N', 'G', '\r', '\n', 0x1A, '\n'
    };
    size_t offset = 8;
    uint8_t *idat = NULL;
    size_t idat_size = 0;
    int w = 0, h = 0;
    uint8_t *raw = NULL;
    uint8_t *pixels = NULL;
    size_t row_bytes, raw_size;
    int y;

    if (png_data == NULL || png_size < sizeof(png_sig) ||
        memcmp(png_data, png_sig, sizeof(png_sig)) != 0)
        return -1;

    while (offset + 12 <= png_size) {
        uint32_t chunk_size = read_be32(png_data + offset);
        const uint8_t *type = png_data + offset + 4;
        const uint8_t *chunk = png_data + offset + 8;
        size_t next_offset = offset + 12u + (size_t)chunk_size;
        uint8_t *tmp;

        if (next_offset > png_size)
            goto fail;

        if (memcmp(type, "IHDR", 4) == 0) {
            if (chunk_size != 13)
                goto fail;
            w = (int)read_be32(chunk + 0);
            h = (int)read_be32(chunk + 4);
            if (w <= 0 || h <= 0 || chunk[8] != 8 || chunk[9] != 0 ||
                chunk[10] != 0 || chunk[11] != 0 || chunk[12] != 0)
                goto fail;
        } else if (memcmp(type, "IDAT", 4) == 0) {
            tmp = (uint8_t *)realloc(idat, idat_size + chunk_size);
            if (tmp == NULL)
                goto fail;
            idat = tmp;
            memcpy(idat + idat_size, chunk, chunk_size);
            idat_size += chunk_size;
        } else if (memcmp(type, "IEND", 4) == 0) {
            break;
        }

        offset = next_offset;
    }

    if (w <= 0 || h <= 0 || idat == NULL)
        goto fail;

    row_bytes = (size_t)w;
    raw_size = (row_bytes + 1u) * (size_t)h;
    raw = (uint8_t *)malloc(raw_size);
    pixels = (uint8_t *)malloc(row_bytes * (size_t)h);
    if (raw == NULL || pixels == NULL)
        goto fail;

    {
        uLongf uncompressed_size = (uLongf)raw_size;
        if (uncompress(raw, &uncompressed_size, idat, (uLong)idat_size) != Z_OK ||
            uncompressed_size != (uLongf)raw_size)
            goto fail;
    }

    for (y = 0; y < h; ++y) {
        const uint8_t *src = raw + y * (row_bytes + 1u) + 1u;
        uint8_t *dst = pixels + (size_t)y * row_bytes;
        uint8_t filter = raw[y * (row_bytes + 1u)];
        size_t x;

        for (x = 0; x < row_bytes; ++x) {
            uint8_t left = (x > 0) ? dst[x - 1u] : 0;
            uint8_t up = (y > 0) ? pixels[(size_t)(y - 1) * row_bytes + x] : 0;
            uint8_t up_left = (x > 0 && y > 0)
                ? pixels[(size_t)(y - 1) * row_bytes + x - 1u] : 0;
            uint8_t avg = (uint8_t)(((int)left + (int)up) / 2);

            switch (filter) {
            case 0: dst[x] = src[x]; break;
            case 1: dst[x] = (uint8_t)(src[x] + left); break;
            case 2: dst[x] = (uint8_t)(src[x] + up); break;
            case 3: dst[x] = (uint8_t)(src[x] + avg); break;
            case 4: dst[x] = (uint8_t)(src[x] + paeth_predictor(left, up, up_left)); break;
            default: goto fail;
            }
        }
    }

    free(idat);
    free(raw);
    *out_pixels = pixels;
    *width = w;
    *height = h;
    return 0;

fail:
    free(idat);
    free(raw);
    free(pixels);
    return -1;
}

int levio_euroc_load_image_gray(const char *path,
                                uint8_t *dst, int dst_w, int dst_h,
                                int *src_w, int *src_h)
{
    FILE *fp;
    uint8_t *png_data = NULL;
    uint8_t *pixels = NULL;
    int *x_map = NULL;
    long file_size;
    size_t read_size;
    int w = 0, h = 0;
    int x;
    int y;

    if (path == NULL || dst == NULL || dst_w <= 0 || dst_h <= 0)
        return -1;

    fp = fopen(path, "rb");
    if (fp == NULL)
        return -1;

    if (fseek(fp, 0, SEEK_END) != 0) {
        fclose(fp);
        return -1;
    }
    file_size = ftell(fp);
    if (file_size <= 0 || fseek(fp, 0, SEEK_SET) != 0) {
        fclose(fp);
        return -1;
    }

    png_data = (uint8_t *)malloc((size_t)file_size);
    if (png_data == NULL) {
        fclose(fp);
        return -1;
    }

    read_size = fread(png_data, 1, (size_t)file_size, fp);
    fclose(fp);
    if (read_size != (size_t)file_size ||
        decode_png_gray8(png_data, (size_t)file_size, &pixels, &w, &h) != 0) {
        free(png_data);
        return -1;
    }
    free(png_data);

    x_map = (int *)malloc((size_t)dst_w * sizeof(*x_map));
    if (x_map == NULL) {
        free(pixels);
        return -1;
    }

    for (x = 0; x < dst_w; ++x)
        x_map[x] = (int)((long long)x * w / dst_w);

    for (y = 0; y < dst_h; ++y) {
        int src_y = (int)((long long)y * h / dst_h);
        for (x = 0; x < dst_w; ++x) {
            dst[y * dst_w + x] = pixels[src_y * w + x_map[x]];
        }
    }

    free(x_map);
    free(pixels);
    if (src_w != NULL) *src_w = w;
    if (src_h != NULL) *src_h = h;
    return 0;
}
