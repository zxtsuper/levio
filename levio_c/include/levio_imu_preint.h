/**
 * @file levio_imu_preint.h
 * @brief IMU pre-integration for the tight-coupled back-end.
 *
 * Paper §3.3, equations (2)–(4):
 *
 *   ΔR_{ij} = ∏_{k=i}^{j-1} Exp(( ω̃_k - b^g_i ) Δt)             ... (2)
 *   Δv_{ij} = Σ_{k=i}^{j-1} ΔR_{ik} (ã_k - b^a_i) Δt             ... (3)
 *   Δp_{ij} = Σ_{k=i}^{j-1} [ Δv_{ik} Δt + ½ ΔR_{ik}(ã_k-b^a_i)Δt² ] ... (4)
 *
 * where ω̃, ã are raw IMU measurements and b^g, b^a are the biases at
 * the start of the integration window (held fixed during integration).
 *
 * The residuals are also linearised w.r.t. the biases to allow first-order
 * bias-correction without re-integrating (standard technique; see
 * Forster et al., TRO 2017).
 *
 * TODO: implement covariance propagation and bias-correction Jacobians.
 */
#ifndef LEVIO_IMU_PREINT_H
#define LEVIO_IMU_PREINT_H

#include "levio_types.h"
#include "levio_cfg.h"

#ifdef __cplusplus
extern "C" {
#endif

/** State dimension for preintegration covariance: [δφ, δv, δp] = 9 */
#define LEVIO_PREINT_DIM  9

/**
 * @brief IMU pre-integration accumulator between two keyframes.
 *
 * Accumulates ΔR, Δv, Δp (equations (2)–(4) of the paper) and the
 * associated covariance and bias Jacobians.
 */
typedef struct {
    /* Pre-integrated measurements (paper eq. 2, 3, 4) */
    mat3f_t  dR;     /**< ΔR_{ij}: accumulated rotation */
    vec3f_t  dv;     /**< Δv_{ij}: accumulated velocity change */
    vec3f_t  dp;     /**< Δp_{ij}: accumulated position change */

    double   dt_sum; /**< total integration time [s] */
    int      n_imu;  /**< number of IMU samples integrated */

    /* Linearisation point (biases at start of window) */
    vec3f_t  bg0;    /**< gyroscope bias at linearisation */
    vec3f_t  ba0;    /**< accelerometer bias at linearisation */

    /* Bias Jacobians for first-order correction:
     *   dR_corrected ≈ dR * Exp(J_dR_bg * δbg)
     *   dv_corrected ≈ dv + J_dv_bg * δbg + J_dv_ba * δba
     *   dp_corrected ≈ dp + J_dp_bg * δbg + J_dp_ba * δba
     * TODO: populate during integration.
     */
    float    J_dR_bg[9];   /**< 3×3 Jacobian of ΔR w.r.t. gyro bias */
    float    J_dv_bg[9];   /**< 3×3 Jacobian of Δv w.r.t. gyro bias */
    float    J_dv_ba[9];   /**< 3×3 Jacobian of Δv w.r.t. accel bias */
    float    J_dp_bg[9];   /**< 3×3 Jacobian of Δp w.r.t. gyro bias */
    float    J_dp_ba[9];   /**< 3×3 Jacobian of Δp w.r.t. accel bias */

    /* Covariance of the pre-integrated measurement (9×9) */
    float    cov[LEVIO_PREINT_DIM * LEVIO_PREINT_DIM];

} levio_preint_t;

/** IMU noise parameters */
typedef struct {
    float sigma_gyro_c;    /**< gyroscope continuous noise density [rad/s/√Hz] */
    float sigma_accel_c;   /**< accelerometer continuous noise density [m/s²/√Hz] */
    float sigma_gyro_rw;   /**< gyroscope random-walk noise density */
    float sigma_accel_rw;  /**< accelerometer random-walk noise density */
} levio_imu_noise_t;

/**
 * @brief Reset / initialise a pre-integration accumulator.
 *
 * @param pi   Pre-integration state.
 * @param bg0  Gyroscope bias at the start of the integration window.
 * @param ba0  Accelerometer bias.
 */
void levio_preint_reset(levio_preint_t *pi, vec3f_t bg0, vec3f_t ba0);

/**
 * @brief Integrate one IMU sample into the accumulator.
 *
 * Implements the discrete-time update for equations (2)–(4).
 *
 * @param pi    Pre-integration accumulator (updated in place).
 * @param dt    Time step [s].
 * @param acc   Accelerometer measurement [m/s²].
 * @param gyro  Gyroscope measurement [rad/s].
 * @param noise IMU noise parameters (used for covariance propagation).
 *
 * TODO: propagate covariance and bias Jacobians.
 */
void levio_preint_push(levio_preint_t *pi, float dt,
                        vec3f_t acc, vec3f_t gyro,
                        const levio_imu_noise_t *noise);

/**
 * @brief Compute the IMU factor residual between two states.
 *
 * Residual = [r_R, r_v, r_p]^T (9-vector) as defined implicitly by
 * equations (2)–(4) of the paper (state-error form).
 *
 * @param pi    Pre-integration result (ΔR, Δv, Δp).
 * @param s_i   State at start of window (pose, velocity, bias).
 * @param s_j   State at end   of window.
 * @param g_w   Gravity vector in world frame [m/s²].
 * @param res   Output residual (9 floats: [δφ; δv; δp]).
 *
 * TODO: implement full residual computation and Jacobians.
 */
void levio_preint_residual(const levio_preint_t *pi,
                            const levio_keyframe_t *s_i,
                            const levio_keyframe_t *s_j,
                            vec3f_t g_w,
                            float res[LEVIO_PREINT_DIM]);

#ifdef __cplusplus
}
#endif

#endif /* LEVIO_IMU_PREINT_H */
