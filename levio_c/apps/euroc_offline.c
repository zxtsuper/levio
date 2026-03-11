/**
 * @file euroc_offline.c
 * @brief Offline EuRoC dataset demo application for LEVIO.
 *
 * Demonstrates how to wire up the LEVIO API (levio_push_imu / levio_push_image)
 * using placeholder dataset hooks.  On a real system, replace the stub
 * functions below with actual CSV/MAV reader code.
 *
 * Build and run:
 *   cmake -S levio_c -B build && cmake --build build
 *   ./build/levio_euroc_demo
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "levio.h"

/* --------------------------------------------------------------------------
 * Placeholder dataset types – replace with a real EuRoC CSV reader
 * -------------------------------------------------------------------------- */

#define DEMO_IMG_W    LEVIO_IMG_W
#define DEMO_IMG_H    LEVIO_IMG_H

/** Synthetic IMU sample (constant circular motion for demo). */
static void gen_imu_sample(double t, float *ax, float *ay, float *az,
                            float *gx, float *gy, float *gz)
{
    /* Simulate slow rotation about Z-axis with gravity along -Z */
    float omega = 0.1f;  /* rad/s */
    *ax = 0.0f;
    *ay = 0.0f;
    *az = 9.81f;
    *gx = 0.0f;
    *gy = 0.0f;
    *gz = omega;
    (void)t;
}

/** Synthetic greyscale image (gradient pattern for demo). */
static void gen_image(double t, uint8_t *buf, int w, int h)
{
    int r, c;
    for (r = 0; r < h; ++r)
        for (c = 0; c < w; ++c)
            buf[r * w + c] = (uint8_t)((r + c + (int)(t * 10.0)) & 0xFF);
}

/* --------------------------------------------------------------------------
 * Main
 * -------------------------------------------------------------------------- */

int main(void)
{
    printf("LEVIO EuRoC offline demo\n");
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

    /* Simulation parameters */
    double t_img  = 0.0;
    double t_imu  = 0.0;
    double dt_img = 1.0 / 20.0;   /* 20 Hz camera */
    double dt_imu = 1.0 / 200.0;  /* 200 Hz IMU */
    int n_frames  = 30;
    int frame;

    for (frame = 0; frame < n_frames; ++frame) {
        double t_img_next = t_img + dt_img;

        /* Feed IMU samples up to next image timestamp */
        while (t_imu < t_img_next) {
            float ax, ay, az, gx, gy, gz;
            gen_imu_sample(t_imu, &ax, &ay, &az, &gx, &gy, &gz);
            levio_push_imu(&vio, t_imu, ax, ay, az, gx, gy, gz);
            t_imu += dt_imu;
        }

        /* Generate and feed image */
        gen_image(t_img, img_buf, DEMO_IMG_W, DEMO_IMG_H);
        levio_pose_t pose = levio_push_image(&vio, t_img,
                                              img_buf,
                                              DEMO_IMG_W, DEMO_IMG_H,
                                              DEMO_IMG_W);

        printf("Frame %3d  t=%.3f  pose_valid=%d  landmarks=%d\n",
               frame, t_img, pose.valid, (int)vio.fe.n_lms);

        t_img = t_img_next;
    }

    printf("\nDemo complete.  Back-end and RANSAC are stubs – see TODOs.\n");
    return 0;
}
