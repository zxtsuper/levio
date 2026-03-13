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
    /* Residual = [r_R; r_v; r_p] (9-vector) per Forster et al., TRO 2017.
     *
     *   r_R = Log( ΔR^T * R_i^T * R_j )               (3)
     *   r_v = R_i^T * (v_j - v_i - g_w * dt) - Δv     (3)
     *   r_p = R_i^T * (p_j - p_i - v_i*dt             (3)
     *                  - 0.5*g_w*dt^2) - Δp
     */
    float dt = (float)pi->dt_sum;
    float dt2 = 0.5f * dt * dt;

    mat3f_t Ri  = s_i->Rwb;   /* R_wb at i */
    mat3f_t RiT = levio_mat3_transpose(Ri);
    mat3f_t Rj  = s_j->Rwb;   /* R_wb at j */

    /* r_R = Log( ΔR^T * R_i^T * R_j ) */
    mat3f_t dRt  = levio_mat3_transpose(pi->dR);
    mat3f_t RiTRj = levio_mat3_mul(RiT, Rj);
    mat3f_t dRtRiTRj = levio_mat3_mul(dRt, RiTRj);
    vec3f_t r_R  = levio_so3_log(dRtRiTRj);

    /* r_v = R_i^T * (v_j - v_i - g_w*dt) - Δv */
    vec3f_t dv_world = levio_vec3_sub(s_j->vwb, s_i->vwb);
    dv_world = levio_vec3_sub(dv_world, levio_vec3_scale(g_w, dt));
    vec3f_t r_v = levio_vec3_sub(levio_mat3_mul_vec3(RiT, dv_world), pi->dv);

    /* r_p = R_i^T * (p_j - p_i - v_i*dt - 0.5*g_w*dt^2) - Δp */
    vec3f_t dp_world = levio_vec3_sub(s_j->pwb, s_i->pwb);
    dp_world = levio_vec3_sub(dp_world, levio_vec3_scale(s_i->vwb, dt));
    dp_world = levio_vec3_sub(dp_world, levio_vec3_scale(g_w, dt2));
    vec3f_t r_p = levio_vec3_sub(levio_mat3_mul_vec3(RiT, dp_world), pi->dp);

    res[0] = r_R.x; res[1] = r_R.y; res[2] = r_R.z;
    res[3] = r_v.x; res[4] = r_v.y; res[5] = r_v.z;
    res[6] = r_p.x; res[7] = r_p.y; res[8] = r_p.z;
}
