/**
 * @file main.c
 * @brief GAP9 PMSIS entry point for LEVIO visual-inertial odometry.
 *
 * Architecture:
 *   FC (Fabric Controller) – sensor initialisation, state machine,
 *                            lightweight serial logic.
 *   Cluster (8 × PE cores) – descriptor matching, RANSAC inlier counting.
 *
 * Build (GAP9 cross-compile):
 *   mkdir build_gap9 && cd build_gap9
 *   cmake -DCMAKE_TOOLCHAIN_FILE=../../cmake/toolchains/gap9-pmsis.cmake \
 *         -DGAP_SDK_HOME=$GAP_SDK_HOME \
 *         -DLEVIO_USE_CLUSTER=ON \
 *         -DCMAKE_BUILD_TYPE=Release \
 *         ../levio_c/gap9
 *   make
 *
 * Build (desktop simulation, no GAP SDK):
 *   mkdir build_gap9_sim && cd build_gap9_sim
 *   cmake -DLEVIO_USE_CLUSTER=OFF -DCMAKE_BUILD_TYPE=Debug ../levio_c/gap9
 *   make && ./levio_gap9_demo
 *
 * Run on board:
 *   gapy --target=gap9_evk --exec-prepare --exec run
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>

/* -----------------------------------------------------------------------
 * PMSIS headers (GAP9) or minimal stubs (desktop)
 * --------------------------------------------------------------------- */
#ifdef LEVIO_USE_GAP9_CLUSTER
#  include "pmsis.h"
#else
/* Minimal stubs so main.c compiles without the GAP SDK */
   typedef struct { int id; } pi_cluster_conf_t;
   typedef struct { void (*entry)(void*); void *arg; } pi_cluster_task_t;
   typedef struct { int _dummy; } pi_device_t;

   static inline void  pi_cluster_conf_init(pi_cluster_conf_t *c) { (void)c; }
   static inline int   pi_open_from_conf(pi_device_t *d, const pi_cluster_conf_t *c)
                       { (void)d; (void)c; return 0; }
   static inline int   pi_cluster_open(pi_device_t *d)  { (void)d; return 0; }
   static inline void  pi_cluster_close(pi_device_t *d) { (void)d; }
   static inline void  pi_cluster_task(pi_cluster_task_t *t,
                                        void (*fn)(void*), void *arg)
                       { t->entry = fn; t->arg = arg; }
   static inline void  pi_cluster_send_task_to_cl(pi_device_t *d,
                                                   pi_cluster_task_t *t)
                       { (void)d; t->entry(t->arg); }
   static inline int   pmsis_kickoff(void (*fn)(void)) { fn(); return 0; }
#endif /* LEVIO_USE_GAP9_CLUSTER */

/* -----------------------------------------------------------------------
 * LEVIO headers
 * --------------------------------------------------------------------- */
#include "levio.h"
#include "levio_cfg.h"
#include "levio_types.h"
#include "levio_camera.h"
#include "levio_orb.h"
#include "levio_frontend.h"
#include "levio_backend.h"
#include "kernels/levio_kernels.h"

/* --------------------------------------------------------------------------
 * Static allocations (no heap)
 * -------------------------------------------------------------------------- */

/* Synthetic test image (QQVGA) */
static uint8_t s_image[LEVIO_IMG_H * LEVIO_IMG_W];

/* LEVIO pipeline state */
static levio_t         s_levio;
static levio_frontend_t s_frontend;
static levio_backend_t  s_backend;

/* Cluster device (GAP9 only, but struct defined for both builds) */
static pi_device_t       s_cluster_dev;
static pi_cluster_task_t s_cl_task;

/* --------------------------------------------------------------------------
 * Cluster task: parallel descriptor matching
 * -------------------------------------------------------------------------- */
static levio_match_args_t s_match_args;
static levio_feat_t       s_feats_a[LEVIO_MAX_FRAME_FEATS];
static levio_feat_t       s_feats_b[LEVIO_MAX_FRAME_FEATS];
static levio_match_t      s_matches[LEVIO_MAX_FRAME_FEATS];
static int                s_n_feats_a;
static int                s_n_feats_b;

static void cluster_main_task(void *arg)
{
    (void)arg;
    s_match_args.feats_a = s_feats_a;
    s_match_args.n_a     = s_n_feats_a;
    s_match_args.feats_b = s_feats_b;
    s_match_args.n_b     = s_n_feats_b;
    s_match_args.out     = s_matches;
    s_match_args.max_out = LEVIO_MAX_FRAME_FEATS;
    s_match_args.ratio   = LEVIO_MATCH_RATIO_THR;
    levio_cl_match_ratio_fork(&s_match_args);
}

/* --------------------------------------------------------------------------
 * Synthetic data helpers
 * -------------------------------------------------------------------------- */

static void gen_test_image(uint8_t *img, int w, int h, int frame)
{
    int x, y;
    for (y = 0; y < h; ++y)
        for (x = 0; x < w; ++x)
            img[y * w + x] = (uint8_t)((x ^ y ^ frame) & 0xFFu);

    /* Bright cross to give FAST corners to detect */
    int cx = w / 2, cy = h / 2;
    for (x = cx - 20; x < cx + 20; ++x)
        if (x >= 0 && x < w) img[cy * w + x] = 230;
    for (y = cy - 15; y < cy + 15; ++y)
        if (y >= 0 && y < h) img[y * w + cx] = 230;
}

/* --------------------------------------------------------------------------
 * Application entry point (called from main / pmsis_kickoff)
 * -------------------------------------------------------------------------- */

void levio_gap9_main(void)
{
    /* Camera intrinsics (approximate QQVGA pinhole) */
    levio_K_t K;
    K.fx = 100.0f; K.fy = 100.0f;
    K.cx =  80.0f; K.cy =  60.0f;

    /* Gravity vector (NED frame, Z-down → +9.81 along Z) */
    vec3f_t g_w;
    g_w.x = 0.0f; g_w.y = 0.0f; g_w.z = 9.81f;

    /* IMU noise parameters (typical MEMS defaults) */
    levio_imu_noise_t imu_noise;
    imu_noise.sigma_gyro_c   = 1.7e-4f;
    imu_noise.sigma_accel_c  = 2.0e-3f;
    imu_noise.sigma_gyro_rw  = 1.9e-5f;
    imu_noise.sigma_accel_rw = 3.0e-3f;

    /* Initialise pipeline */
    levio_init(&s_levio, &K, g_w);
    levio_frontend_init(&s_frontend, &K);
    levio_backend_init(&s_backend, &K, g_w, &imu_noise);

#ifdef LEVIO_USE_GAP9_CLUSTER
    pi_cluster_conf_t cl_conf;
    pi_cluster_conf_init(&cl_conf);
    cl_conf.id = 0;
    pi_open_from_conf(&s_cluster_dev, &cl_conf);
    if (pi_cluster_open(&s_cluster_dev)) {
        printf("[LEVIO] ERROR: failed to open cluster\n");
        return;
    }
    printf("[LEVIO] GAP9 Cluster opened (8 PEs)\n");
#endif

    printf("[LEVIO] Pipeline ready. Image %dx%d, max_feats=%d\n",
           LEVIO_IMG_W, LEVIO_IMG_H, LEVIO_MAX_FRAME_FEATS);

    int total_matches = 0;
    int frame;

    for (frame = 0; frame < 30; ++frame) {
        double t_img = frame * 0.05;  /* 20 Hz */

        /* Push synthetic IMU samples (200 Hz → 10 per camera frame) */
        int imu_i;
        for (imu_i = 0; imu_i < 10; ++imu_i) {
            double t_imu = t_img - 0.045 + imu_i * 0.005;
            levio_push_imu(&s_levio, t_imu,
                           0.0f, 0.0f, 9.81f,   /* acc: gravity */
                           0.0f, 0.0f, 0.0f);    /* gyro: static */
        }

        /* Generate synthetic image */
        gen_test_image(s_image, LEVIO_IMG_W, LEVIO_IMG_H, frame);

        /* Extract ORB features */
        int n_new = levio_orb_extract(s_image, LEVIO_IMG_W, LEVIO_IMG_H,
                                       LEVIO_IMG_W,
                                       s_feats_b, LEVIO_MAX_FRAME_FEATS);

        if (frame > 0 && s_n_feats_a > 0 && n_new > 0) {
            s_n_feats_b = n_new;

            /* -----------------------------------------------------------
             * Cluster task: parallel descriptor matching A → B
             * --------------------------------------------------------- */
#ifdef LEVIO_USE_GAP9_CLUSTER
            pi_cluster_task(&s_cl_task, cluster_main_task, NULL);
            pi_cluster_send_task_to_cl(&s_cluster_dev, &s_cl_task);
#else
            cluster_main_task(NULL);
#endif
            int n_matches = s_match_args.n_matches;
            total_matches += n_matches;

            /* -----------------------------------------------------------
             * FC-side: frontend pose estimation
             * --------------------------------------------------------- */
            levio_frontend_result_t fe_res;
            levio_frontend_process(&s_frontend, t_img, s_image,
                                    LEVIO_IMG_W, LEVIO_IMG_H, LEVIO_IMG_W,
                                    &fe_res);

            printf("[LEVIO] Frame %3d: feats_prev=%d feats_cur=%d "
                   "matches=%d kf=%d pose_valid=%d\n",
                   frame, s_n_feats_a, n_new,
                   n_matches, fe_res.is_keyframe, fe_res.pose.valid);
        } else {
            /* First frame: no previous features */
            levio_frontend_result_t fe_res;
            levio_frontend_process(&s_frontend, t_img, s_image,
                                    LEVIO_IMG_W, LEVIO_IMG_H, LEVIO_IMG_W,
                                    &fe_res);
            printf("[LEVIO] Frame %3d: first frame, feats=%d kf=%d\n",
                   frame, n_new, fe_res.is_keyframe);
        }

        /* Rotate: B → A for next iteration */
        {
            int fi;
            for (fi = 0; fi < n_new; ++fi)
                s_feats_a[fi] = s_feats_b[fi];
            s_n_feats_a = n_new;
        }
    }

    printf("[LEVIO] Done.  Total cluster matches across 30 frames: %d\n",
           total_matches);

#ifdef LEVIO_USE_GAP9_CLUSTER
    pi_cluster_close(&s_cluster_dev);
    printf("[LEVIO] Cluster closed.\n");
#endif
}

/* --------------------------------------------------------------------------
 * C entry point
 * -------------------------------------------------------------------------- */
int main(void)
{
#ifdef LEVIO_USE_GAP9_CLUSTER
    return pmsis_kickoff((void (*)(void))levio_gap9_main);
#else
    levio_gap9_main();
    return 0;
#endif
}
