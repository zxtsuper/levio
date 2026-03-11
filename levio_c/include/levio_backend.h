/**
 * @file levio_backend.h
 * @brief Sliding-window back-end: tight-coupled BA + IMU optimiser.
 *
 * Paper §3.3 + §3.4:
 *   - Maintains a moving window of LEVIO_WINDOW_SIZE keyframes.
 *   - Each keyframe node stores: pose (R, p), velocity v, IMU bias (bg, ba).
 *   - Edges: visual reprojection factors + IMU pre-integration factors.
 *   - Joint optimisation via Levenberg–Marquardt (LM).
 *   - Landmark states marginalised via Schur complement (eq. 5→6) to
 *     reduce the linear system to pose-only variables.
 *   - When the window is full, the oldest keyframe is marginalised and its
 *     information is folded into the prior factor (eq. 5–6).
 *
 * State layout (for the reduced system after Schur, eq. 6):
 *   x = [pose_0, ..., pose_{N-1}, v_0, ..., v_{N-1}, bg_0, ..., ba_{N-1}]
 *   dim_per_kf = 6 (pose) + 3 (vel) + 3 (bg) + 3 (ba) = 15
 */
#ifndef LEVIO_BACKEND_H
#define LEVIO_BACKEND_H

#include "levio_types.h"
#include "levio_camera.h"
#include "levio_imu_preint.h"
#include "levio_factors.h"
#include "levio_cfg.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Per-keyframe state dimension (pose 6 + vel 3 + bg 3 + ba 3). */
#define LEVIO_KF_DIM  15

/** Maximum reduced-system dimension. */
#define LEVIO_RSYS_DIM  (LEVIO_WINDOW_SIZE * LEVIO_KF_DIM)

/**
 * @brief Association between a landmark and an observing keyframe.
 */
typedef struct {
    uint16_t lm_id;   /**< index into levio_backend_t::lms */
    uint8_t  kf_idx;  /**< index into levio_backend_t::win */
    vec2f_t  obs;     /**< 2-D pixel observation */
} levio_obs_t;

/** Maximum number of observations in the window. */
#define LEVIO_MAX_OBS  (LEVIO_MAX_WORLD_POINTS * 4)

/**
 * @brief Sliding-window back-end state.
 */
typedef struct {
    /* Keyframe window */
    levio_keyframe_t  win[LEVIO_WINDOW_SIZE];  /**< circular keyframe buffer */
    int               n_kf;                    /**< keyframes currently in window */
    int               win_head;                /**< index of oldest keyframe */

    /* World landmarks */
    levio_landmark_t  lms[LEVIO_MAX_WORLD_POINTS];
    uint16_t          n_lms;

    /* Observations */
    levio_obs_t       obs[LEVIO_MAX_OBS];
    uint32_t          n_obs;

    /* IMU pre-integration between consecutive keyframes */
    levio_preint_t    preint[LEVIO_WINDOW_SIZE];

    /* Prior factor (marginalised information from dropped keyframes) */
    levio_prior_t     prior;

    /* Camera intrinsics */
    levio_K_t         K;

    /* Gravity vector in world frame [m/s²] */
    vec3f_t           g_w;

    /* IMU noise parameters */
    levio_imu_noise_t imu_noise;

    /* LM state */
    float             lm_lambda;

    /* Normal equations (reduced system after Schur, eq. 6):
     *   H_pp x_p = b_p
     * where x_p stacks all pose/velocity/bias increments.
     * TODO: consider sparse representation for larger windows.
     */
    float             H_pp[LEVIO_RSYS_DIM * LEVIO_RSYS_DIM]; /**< reduced Hessian */
    float             b_pp[LEVIO_RSYS_DIM];                   /**< reduced gradient */

} levio_backend_t;

/* --------------------------------------------------------------------------
 * Lifecycle
 * -------------------------------------------------------------------------- */

/**
 * @brief Initialise the back-end.
 *
 * @param be        Back-end state.
 * @param K         Camera intrinsics.
 * @param g_w       Gravity vector in world frame.
 * @param imu_noise IMU noise parameters.
 */
void levio_backend_init(levio_backend_t *be, const levio_K_t *K,
                         vec3f_t g_w, const levio_imu_noise_t *imu_noise);

/**
 * @brief Add a new keyframe to the sliding window.
 *
 * If the window is full (n_kf == LEVIO_WINDOW_SIZE), the oldest keyframe
 * is first marginalised via levio_backend_marginalise().
 *
 * @param be          Back-end state.
 * @param kf          New keyframe (copied into the window).
 * @param preint      Pre-integration result from the previous keyframe.
 * @param lms         Newly triangulated landmarks.
 * @param n_lms       Number of new landmarks.
 * @param obs         Observations from this keyframe.
 * @param n_obs       Number of observations.
 * @return            0 on success.
 */
int levio_backend_add_keyframe(levio_backend_t *be,
                                const levio_keyframe_t *kf,
                                const levio_preint_t *preint,
                                const levio_landmark_t *lms, int n_lms,
                                const levio_obs_t *obs, int n_obs);

/* --------------------------------------------------------------------------
 * Optimisation (paper §3.3 + §3.4)
 * -------------------------------------------------------------------------- */

/**
 * @brief Run one full LM iteration over the sliding window.
 *
 * Steps (paper §3.3 + eq. 5–6):
 *   1) Linearise all visual and IMU factors.
 *   2) Build the full Hessian [H_pp H_pl; H_lp H_ll] + prior.
 *   3) Schur complement: eliminate landmark block → reduced system (eq. 5–6).
 *   4) Solve reduced system via Cholesky (levio_math.h).
 *   5) Back-substitute to update landmark positions.
 *   6) Update all states with the increment.
 *   7) Adjust LM damping λ.
 *
 * @param be     Back-end state.
 * @param n_iter Number of LM iterations to run.
 * @return       0 on success, -1 on solver failure.
 *
 * TODO: implement full Hessian construction and Schur complement solver.
 */
int levio_backend_optimize(levio_backend_t *be, int n_iter);

/**
 * @brief Marginalise the oldest keyframe out of the window.
 *
 * Computes the Schur complement of the oldest keyframe's block in the
 * current linearised system and folds it into the prior factor (§3.3).
 *
 * @param be  Back-end state.
 * @return    0 on success.
 *
 * TODO: implement Schur-complement marginalisation.
 */
int levio_backend_marginalise(levio_backend_t *be);

/**
 * @brief Get the most recent pose estimate from the back-end.
 *
 * @param be    Back-end state.
 * @param pose  Output pose.
 */
void levio_backend_get_pose(const levio_backend_t *be, levio_pose_t *pose);

#ifdef __cplusplus
}
#endif

#endif /* LEVIO_BACKEND_H */
