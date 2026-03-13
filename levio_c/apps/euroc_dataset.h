#ifndef LEVIO_EUROC_DATASET_H
#define LEVIO_EUROC_DATASET_H

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#define LEVIO_EUROC_PATH_MAX 1024

typedef struct {
    double t;
    uint64_t timestamp_ns;
    float ax, ay, az;
    float gx, gy, gz;
} levio_euroc_imu_sample_t;

typedef struct {
    double t;
    uint64_t timestamp_ns;
    char image_path[LEVIO_EUROC_PATH_MAX];
} levio_euroc_cam_sample_t;

typedef struct {
    FILE *fp;
} levio_euroc_imu_reader_t;

typedef struct {
    FILE *fp;
    char data_dir[LEVIO_EUROC_PATH_MAX];
} levio_euroc_cam_reader_t;

int levio_euroc_resolve_mav0_root(const char *root,
                                  char *mav0_root, size_t mav0_root_size);

int levio_euroc_open_imu_reader(levio_euroc_imu_reader_t *reader,
                                const char *root);
int levio_euroc_open_cam_reader(levio_euroc_cam_reader_t *reader,
                                const char *root);
void levio_euroc_close_imu_reader(levio_euroc_imu_reader_t *reader);
void levio_euroc_close_cam_reader(levio_euroc_cam_reader_t *reader);

int levio_euroc_read_next_imu(levio_euroc_imu_reader_t *reader,
                              levio_euroc_imu_sample_t *sample);
int levio_euroc_read_next_cam(levio_euroc_cam_reader_t *reader,
                              levio_euroc_cam_sample_t *sample);

int levio_euroc_load_image_gray(const char *path,
                                uint8_t *dst, int dst_w, int dst_h,
                                int *src_w, int *src_h);

#endif
