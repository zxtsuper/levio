/**
 * @file levio_factors.h
 * @brief Cost factors for the tight-coupled bundle-adjustment optimizer.
 *
 * Paper §3.3: the pose graph has two types of edges:
 *   1) Visual reprojection factor  – 2-D observation of a world landmark.
 *   2) IMU pre-integration factor  – constraint between adjacent keyframes.
 *   3) Prior / marginalisation factor – encodes information from dropped
 *      keyframes after Schur-complement marginalisation (§3.3 / eq. 5–6).
 *
 * Each factor provides:
 *   - A residual vector
 *   - Jacobians w.r.t. all connected state variables
 *   - A (robust) information matrix
 */
#ifndef LEVIO_FACTORS_H
#define LEVIO_FACTORS_H

#include "levio_types.h"
#include "levio_camera.h"
#include "levio_imu_preint.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Visual reprojection factor
 * ========================================================================== */

/**
 * @brief Compute reprojection residual and Jacobians for one observation.
 *
 * Residual (2-vector): r = z - π(R_cw * P_w + t_cw)
 *
 * @param K       Camera intrinsics.
 * @param Rcw     Rotation world→camera (3×3 row-major).
 * @param tcw     Translation world→camera.
 * @param Pw      3-D world point.
 * @param obs     2-D pixel observation.
 * @param res     Output residual (2 floats).
 * @param J_pose  Output Jacobian w.r.t. camera pose (2×6 row-major, or NULL).
 * @param J_point Output Jacobian w.r.t. world point (2×3 row-major, or NULL).
 * @return        0 on success, -1 if point is behind camera.
 */
int levio_factor_visual(const levio_K_t *K,
                         const float Rcw[9], vec3f_t tcw,
                         vec3f_t Pw, vec2f_t obs,
                         float res[2],
                         float J_pose[12],   /* 2×6 */
                         float J_point[6]);  /* 2×3 */

/* ==========================================================================
 * IMU pre-integration factor
 * ========================================================================== */

/**
 * @brief Compute IMU factor residual and Jacobians.
 *
 * Residual (9-vector): [r_R; r_v; r_p] (paper §3.3, eq. 2–4 in error form).
 *
 * @param pi     Pre-integration between keyframe i and j.
 * @param kf_i   State at keyframe i.
 * @param kf_j   State at keyframe j.
 * @param g_w    Gravity in world frame.
 * @param res    Output residual (9 floats).
 * @param J_i    Jacobian w.r.t. state_i (9×15, or NULL).   TODO
 * @param J_j    Jacobian w.r.t. state_j (9×15, or NULL).   TODO
 * @return       0 on success.
 */
int levio_factor_imu(const levio_preint_t *pi,
                      const levio_keyframe_t *kf_i,
                      const levio_keyframe_t *kf_j,
                      vec3f_t g_w,
                      float res[LEVIO_PREINT_DIM],
                      float *J_i,   /* 9×15 */
                      float *J_j);  /* 9×15 */

/* ==========================================================================
 * Prior / marginalisation factor
 * ========================================================================== */

/** Maximum prior state dimension (conservative allocation). */
#define LEVIO_PRIOR_DIM  (LEVIO_WINDOW_SIZE * 15)

/**
 * @brief Prior factor resulting from Schur-complement marginalisation (eq. 5–6).
 *
 * Stores the linearised prior: Λ (x - x̄) where Λ is the Schur-complement
 * information matrix.
 *
 * TODO: populate after each window drop in levio_backend_marginalise().
 */
typedef struct {
    int    dim;                         /**< active dimension */
    float  x_lin[LEVIO_PRIOR_DIM];      /**< linearisation point */
    float  H[LEVIO_PRIOR_DIM * LEVIO_PRIOR_DIM]; /**< information matrix (row-major) */
    float  b[LEVIO_PRIOR_DIM];          /**< information vector */
} levio_prior_t;

/**
 * @brief Compute prior factor residual and add it to the normal equations.
 *
 * @param prior  Prior factor.
 * @param x_cur  Current state vector (dim floats).
 * @param res    Output residual (dim floats).
 * @return       0 on success.
 */
int levio_factor_prior(const levio_prior_t *prior,
                        const float *x_cur,
                        float *res);

#ifdef __cplusplus
}
#endif

#endif /* LEVIO_FACTORS_H */
