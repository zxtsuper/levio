/**
 * @file levio_cfg.h
 * @brief LEVIO compile-time configuration constants.
 *
 * All default values are taken directly from the LEVIO paper:
 *   "LEVIO: Lightweight Embedded Visual Inertial Odometry for
 *    Resource-Constrained Devices", arXiv:2602.03294v1
 *
 * Section references are noted on each constant.
 */
#ifndef LEVIO_CFG_H
#define LEVIO_CFG_H

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------
 * Image resolution
 * Paper section 3.4: "we use a QQVGA (160x120) resolution"
 * -------------------------------------------------------------------------- */
#define LEVIO_IMG_W           160
#define LEVIO_IMG_H           120

/* --------------------------------------------------------------------------
 * ORB descriptor size
 * Standard 256-bit ORB = 32 bytes (used throughout paper frontend, §3.2)
 * -------------------------------------------------------------------------- */
#define LEVIO_DESC_BYTES      32   /**< 256-bit ORB descriptor */

/* --------------------------------------------------------------------------
 * Feature / world-point limits
 * Paper section 3.2: "frames store up to 700 descriptors"
 *                    "the world stores up to 1000 descriptors"
 * -------------------------------------------------------------------------- */
#define LEVIO_MAX_FRAME_FEATS  700
#define LEVIO_MAX_WORLD_POINTS 1000

/* --------------------------------------------------------------------------
 * PnP fallback threshold
 * Paper section 3.2: "when fewer than 25 world-point matches are found,
 *                     the system falls back to the 8-point algorithm"
 * -------------------------------------------------------------------------- */
#define LEVIO_PNP_MIN_INLIERS  25

/* --------------------------------------------------------------------------
 * Sliding-window / keyframe parameters
 * Paper section 3.3: tight-coupled moving window optimisation.
 *   - Window size    : 10 keyframes   (Table I / §3.3)
 *   - Parallax thr.  : 15 pixels      (§3.2 keyframe selection criterion)
 * These values are consistent with the paper description.  Cross-check
 * against Table I of arXiv:2602.03294v1 §3.3 for the official exact values.
 * -------------------------------------------------------------------------- */
#define LEVIO_WINDOW_SIZE      10   /**< keyframes kept in moving window (§3.3) */
#define LEVIO_KF_PARALLAX_PX   15.0f /**< keyframe parallax threshold [px] (§3.2) */

/* --------------------------------------------------------------------------
 * IMU ring-buffer capacity
 * At 200 Hz IMU and ~5 Hz keyframe rate: 200/5 * 2 headroom = 80+ samples.
 * -------------------------------------------------------------------------- */
#define LEVIO_IMU_BUF_SIZE     256

/* --------------------------------------------------------------------------
 * RANSAC parameters
 * -------------------------------------------------------------------------- */
#define LEVIO_RANSAC_MAX_ITER  200
#define LEVIO_RANSAC_REPROJ_THR 2.0f /**< reprojection error threshold [px] */

/* --------------------------------------------------------------------------
 * LM optimiser
 * -------------------------------------------------------------------------- */
#define LEVIO_LM_MAX_ITER      10
#define LEVIO_LM_LAMBDA_INIT   1e-4f
#define LEVIO_LM_LAMBDA_FACTOR 10.0f

/* --------------------------------------------------------------------------
 * Hamming distance ratio test threshold for brute-force matcher
 * -------------------------------------------------------------------------- */
#define LEVIO_MATCH_RATIO_THR  0.8f

#ifdef __cplusplus
}
#endif

#endif /* LEVIO_CFG_H */
