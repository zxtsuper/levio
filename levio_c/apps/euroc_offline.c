/**
 * @file euroc_offline.c
 * @brief Offline EuRoC dataset demo application for LEVIO.
 *
 * Demonstrates how to wire up the LEVIO API (levio_push_imu / levio_push_image)
 * against a real EuRoC sequence on disk.
 *
 * Build and run:
 *   cmake -S levio_c -B build && cmake --build build
 *   ./build/levio_euroc_demo /path/to/MH_01_easy [max_frames]
 */
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <limits.h>

#include "levio.h"
#include "euroc_dataset.h"

#define DEMO_IMG_W    LEVIO_IMG_W
#define DEMO_IMG_H    LEVIO_IMG_H

/* --------------------------------------------------------------------------
 * Main
 * -------------------------------------------------------------------------- */

int main(int argc, char **argv)
{
    const char *dataset_root;
    int max_frames = -1;
    char *endptr = NULL;
    levio_euroc_imu_reader_t imu_reader;
    levio_euroc_cam_reader_t cam_reader;
    levio_euroc_imu_sample_t imu_sample;
    levio_euroc_cam_sample_t cam_sample;
    int imu_status;
    int cam_status;
    int frame = 0;
    int src_w = 0, src_h = 0;

    if (argc < 2) {
        fprintf(stderr, "Usage: %s <EuRoC sequence root|mav0 root> [max_frames]\n",
                argv[0]);
        return 1;
    }

    dataset_root = argv[1];
    if (argc >= 3) {
        long parsed_frames;
        errno = 0;
        parsed_frames = strtol(argv[2], &endptr, 10);
        if (errno != 0 || endptr == argv[2] || *endptr != '\0' ||
            parsed_frames <= 0 || parsed_frames > INT_MAX) {
            fprintf(stderr, "Invalid max_frames value: %s\n", argv[2]);
            return 1;
        }
        max_frames = (int)parsed_frames;
    }

    printf("LEVIO EuRoC offline demo\n");
    printf("  Dataset root       : %s\n", dataset_root);
    printf("  Image: %dx%d (QQVGA, paper §3.4)\n", LEVIO_IMG_W, LEVIO_IMG_H);
    printf("  Max frame features : %d  (paper §3.2)\n", LEVIO_MAX_FRAME_FEATS);
    printf("  Max world points   : %d  (paper §3.2)\n", LEVIO_MAX_WORLD_POINTS);
    printf("  PnP min inliers    : %d  (paper §3.2)\n", LEVIO_PNP_MIN_INLIERS);
    printf("  Window size        : %d  (paper §3.3)\n", LEVIO_WINDOW_SIZE);
    printf("  KF parallax thr    : %.1f px (paper §3.2)\n\n", (double)LEVIO_KF_PARALLAX_PX);

    /* Camera intrinsics: EuRoC MH_01 cam0 (approximate, QQVGA scaled) */
    levio_K_t K;
    K.fx = 458.654f * ((float)LEVIO_IMG_W  / 752.0f);
    K.fy = 457.296f * ((float)LEVIO_IMG_H  / 480.0f);
    K.cx = 367.215f * ((float)LEVIO_IMG_W  / 752.0f);
    K.cy = 248.375f * ((float)LEVIO_IMG_H  / 480.0f);

    vec3f_t g_w = {0.0f, 0.0f, -9.81f};

    levio_t vio;
    levio_init(&vio, &K, g_w);

    static uint8_t img_buf[DEMO_IMG_W * DEMO_IMG_H];
    memset(&imu_reader, 0, sizeof(imu_reader));
    memset(&cam_reader, 0, sizeof(cam_reader));

    if (levio_euroc_open_imu_reader(&imu_reader, dataset_root) != 0 ||
        levio_euroc_open_cam_reader(&cam_reader, dataset_root) != 0) {
        fprintf(stderr, "Failed to open EuRoC dataset under: %s\n", dataset_root);
        fprintf(stderr, "Expected either <root>/mav0/{imu0,cam0} or <root>/{imu0,cam0}\n");
        levio_euroc_close_imu_reader(&imu_reader);
        levio_euroc_close_cam_reader(&cam_reader);
        return 1;
    }

    imu_status = levio_euroc_read_next_imu(&imu_reader, &imu_sample);
    if (imu_status < 0) {
        fprintf(stderr, "Failed to parse IMU CSV\n");
        levio_euroc_close_imu_reader(&imu_reader);
        levio_euroc_close_cam_reader(&cam_reader);
        return 1;
    }

    while ((max_frames < 0 || frame < max_frames) &&
           (cam_status = levio_euroc_read_next_cam(&cam_reader, &cam_sample)) > 0) {
        levio_pose_t pose;

        while (imu_status > 0 && imu_sample.t <= cam_sample.t) {
            levio_push_imu(&vio, imu_sample.t,
                           imu_sample.ax, imu_sample.ay, imu_sample.az,
                           imu_sample.gx, imu_sample.gy, imu_sample.gz);
            imu_status = levio_euroc_read_next_imu(&imu_reader, &imu_sample);
            if (imu_status < 0) {
                fprintf(stderr, "Failed to parse IMU CSV\n");
                levio_euroc_close_imu_reader(&imu_reader);
                levio_euroc_close_cam_reader(&cam_reader);
                return 1;
            }
        }

        if (levio_euroc_load_image_gray(cam_sample.image_path, img_buf,
                                        DEMO_IMG_W, DEMO_IMG_H,
                                        &src_w, &src_h) != 0) {
            fprintf(stderr, "Failed to load image: %s\n", cam_sample.image_path);
            levio_euroc_close_imu_reader(&imu_reader);
            levio_euroc_close_cam_reader(&cam_reader);
            return 1;
        }

        if (frame == 0)
            printf("  Source image       : %dx%d\n\n", src_w, src_h);

        pose = levio_push_image(&vio, cam_sample.t,
                                img_buf,
                                DEMO_IMG_W, DEMO_IMG_H,
                                DEMO_IMG_W);

        printf("Frame %3d  t=%.6f  pose_valid=%d  landmarks=%d\n",
               frame, cam_sample.t, pose.valid, (int)vio.fe.n_lms);
        ++frame;
    }

    if (cam_status < 0) {
        fprintf(stderr, "Failed to parse camera CSV\n");
        levio_euroc_close_imu_reader(&imu_reader);
        levio_euroc_close_cam_reader(&cam_reader);
        return 1;
    }

    levio_euroc_close_imu_reader(&imu_reader);
    levio_euroc_close_cam_reader(&cam_reader);

    if (frame == 0) {
        fprintf(stderr, "No camera frames found in dataset\n");
        return 1;
    }

    printf("\nDemo complete.  Processed %d EuRoC frames.\n", frame);
    printf("Back-end and RANSAC are still stubs – see TODOs.\n");
    return 0;
}
