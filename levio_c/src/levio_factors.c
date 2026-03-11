/**
 * @file levio_factors.c
 * @brief Visual reprojection factor, IMU factor, and prior factor.
 */
#include "levio_factors.h"
#include "levio_camera.h"
#include "levio_math.h"
#include <string.h>

/* --------------------------------------------------------------------------
 * Visual reprojection factor
 * -------------------------------------------------------------------------- */

int levio_factor_visual(const levio_K_t *K,
                         const float Rcw[9], vec3f_t tcw,
                         vec3f_t Pw, vec2f_t obs,
                         float res[2],
                         float J_pose[12],
                         float J_point[6])
{
    /* Transform Pw to camera frame */
    mat3f_t Rm;
    int i;
    for (i = 0; i < 9; ++i) Rm.m[i] = Rcw[i];
    vec3f_t Pc = levio_vec3_add(levio_mat3_mul_vec3(Rm, Pw), tcw);

    vec2f_t proj;
    if (levio_project(K, Pc, &proj) != 0)
        return -1;

    res[0] = proj.x - obs.x;
    res[1] = proj.y - obs.y;

    if (J_pose)
        levio_reproj_jacobian_pose(K, Pc, J_pose);
    if (J_point)
        levio_reproj_jacobian_point(K, Rcw, Pc, J_point);

    return 0;
}

/* --------------------------------------------------------------------------
 * IMU pre-integration factor
 * -------------------------------------------------------------------------- */

int levio_factor_imu(const levio_preint_t *pi,
                      const levio_keyframe_t *kf_i,
                      const levio_keyframe_t *kf_j,
                      vec3f_t g_w,
                      float res[LEVIO_PREINT_DIM],
                      float *J_i,
                      float *J_j)
{
    levio_preint_residual(pi, kf_i, kf_j, g_w, res);

    /* TODO: compute 9×15 Jacobians J_i, J_j */
    if (J_i) memset(J_i, 0, LEVIO_PREINT_DIM * 15 * sizeof(float));
    if (J_j) memset(J_j, 0, LEVIO_PREINT_DIM * 15 * sizeof(float));

    return 0;
}

/* --------------------------------------------------------------------------
 * Prior factor
 * -------------------------------------------------------------------------- */

int levio_factor_prior(const levio_prior_t *prior,
                        const float *x_cur,
                        float *res)
{
    /* r = x_cur - x_lin (linearised prior residual) */
    int i;
    for (i = 0; i < prior->dim; ++i)
        res[i] = x_cur[i] - prior->x_lin[i];

    /* TODO: apply the full quadratic form: r^T Λ r,
     *       and return the gradient Λ r for the LM step.
     */
    return 0;
}
