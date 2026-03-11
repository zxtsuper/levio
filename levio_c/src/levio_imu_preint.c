/**
 * @file levio_imu_preint.c
 * @brief IMU pre-integration (equations 2–4 of the paper).
 *
 * TODO: Complete covariance propagation and bias Jacobian computation.
 */
#include "levio_imu_preint.h"
#include "levio_se3.h"
#include "levio_math.h"
#include <string.h>
#include <math.h>

void levio_preint_reset(levio_preint_t *pi, vec3f_t bg0, vec3f_t ba0)
{
    memset(pi, 0, sizeof(*pi));
    pi->dR = levio_mat3_identity();  /* ΔR = I */
    /* dv, dp already zero */
    pi->bg0     = bg0;
    pi->ba0     = ba0;
    pi->dt_sum  = 0.0;
    pi->n_imu   = 0;
}

void levio_preint_push(levio_preint_t *pi, float dt,
                        vec3f_t acc, vec3f_t gyro,
                        const levio_imu_noise_t *noise)
{
    /* Correct raw measurements with current bias estimates */
    vec3f_t gyro_c = levio_vec3_sub(gyro, pi->bg0);
    vec3f_t acc_c  = levio_vec3_sub(acc,  pi->ba0);

    /* -----------------------------------------------------------------------
     * Equation (2): ΔR_{k+1} = ΔR_k * Exp(gyro_c * dt)
     * --------------------------------------------------------------------- */
    vec3f_t omega = levio_vec3_scale(gyro_c, dt);
    mat3f_t dR_k  = levio_so3_exp(omega);
    pi->dR        = levio_mat3_mul(pi->dR, dR_k);

    /* -----------------------------------------------------------------------
     * Equation (3): Δv_{k+1} = Δv_k + ΔR_k * acc_c * dt
     * --------------------------------------------------------------------- */
    vec3f_t acc_rot = levio_mat3_mul_vec3(pi->dR, acc_c);
    pi->dv = levio_vec3_add(pi->dv, levio_vec3_scale(acc_rot, dt));

    /* -----------------------------------------------------------------------
     * Equation (4): Δp_{k+1} = Δp_k + Δv_k*dt + 0.5*ΔR_k*acc_c*dt²
     * --------------------------------------------------------------------- */
    pi->dp = levio_vec3_add(pi->dp, levio_vec3_scale(pi->dv, dt));
    pi->dp = levio_vec3_add(pi->dp,
                 levio_vec3_scale(acc_rot, 0.5f * dt * dt));

    pi->dt_sum += (double)dt;
    pi->n_imu  += 1;

    /* TODO: propagate 9×9 covariance matrix and bias Jacobians.
     *       F_k (transition matrix) and G_k (noise matrix) blocks:
     *       δφ_{k+1} = ΔR_k^T δφ_k + Jr * δbg * dt + noise
     *       δv_{k+1} = δv_k - ΔR_k * [acc_c]× δφ_k * dt - ΔR_k*δba*dt + noise
     *       δp_{k+1} = δp_k + δv_k*dt + ...
     *
     *       Covariance:  P_{k+1} = F_k P_k F_k^T + G_k Q G_k^T
     *       where Q = diag(σ_g², σ_a²) * dt.
     */
    (void)noise;
}

void levio_preint_residual(const levio_preint_t *pi,
                            const levio_keyframe_t *s_i,
                            const levio_keyframe_t *s_j,
                            vec3f_t g_w,
                            float res[LEVIO_PREINT_DIM])
{
    /* Residual = [r_R; r_v; r_p] as in §3.3 error-state form.
     *
     * TODO: implement the full residual using:
     *   r_R = Log(ΔR^T * R_i^T * R_j)
     *   r_v = R_i^T * (v_j - v_i - g*dt) - Δv
     *   r_p = R_i^T * (p_j - p_i - v_i*dt - 0.5*g*dt²) - Δp
     */
    (void)pi; (void)s_i; (void)s_j; (void)g_w;

    int i;
    for (i = 0; i < LEVIO_PREINT_DIM; ++i)
        res[i] = 0.0f;
}
