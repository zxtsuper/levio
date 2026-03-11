/**
 * @file levio_camera.c
 * @brief Camera projection / unprojection with known pinhole intrinsics.
 */
#include "levio_camera.h"
#include "levio_math.h"
#include <math.h>

int levio_project(const levio_K_t *K, vec3f_t Pc, vec2f_t *px)
{
    if (Pc.z <= 0.0f)
        return -1;
    float iz = 1.0f / Pc.z;
    px->x = K->fx * Pc.x * iz + K->cx;
    px->y = K->fy * Pc.y * iz + K->cy;
    return 0;
}

vec3f_t levio_unproject(const levio_K_t *K, vec2f_t px)
{
    vec3f_t ray;
    ray.x = (px.x - K->cx) / K->fx;
    ray.y = (px.y - K->cy) / K->fy;
    ray.z = 1.0f;
    return ray;
}

int levio_reproj_error(const levio_K_t *K,
                       const float Rcw[9], vec3f_t tcw,
                       vec3f_t Pw, vec2f_t obs,
                       vec2f_t *err)
{
    /* Transform world point to camera frame */
    mat3f_t Rm;
    int i;
    for (i = 0; i < 9; ++i) Rm.m[i] = Rcw[i];
    vec3f_t Pc = levio_vec3_add(levio_mat3_mul_vec3(Rm, Pw), tcw);

    vec2f_t proj;
    if (levio_project(K, Pc, &proj) != 0)
        return -1;

    err->x = obs.x - proj.x;
    err->y = obs.y - proj.y;
    return 0;
}

void levio_reproj_jacobian_pose(const levio_K_t *K, vec3f_t Pc, float J26[12])
{
    /* 2×6 Jacobian: ∂[proj] / ∂[δφ, δt] (left perturbation on SE3)
     *
     * Let p = [X, Y, Z]^T (camera frame).
     *   u = fx * X/Z + cx,  v = fy * Y/Z + cy
     *
     * ∂u/∂Pc = [fx/Z,     0, -fx*X/Z²]
     * ∂v/∂Pc = [   0, fy/Z, -fy*Y/Z²]
     *
     * ∂Pc/∂[δφ] = -[Pc]×   (skew)
     * ∂Pc/∂[δt] =  I
     *
     * Full Jacobian (2×6): [∂[u,v]/∂Pc * ∂Pc/∂δφ,  ∂[u,v]/∂Pc * I]
     */
    float X = Pc.x, Y = Pc.y, Z = Pc.z;
    float iz  = 1.0f / Z;
    float iz2 = iz * iz;

    /* ∂proj/∂Pc  (2×3, row-major) */
    float dp_dPc[6];
    dp_dPc[0] = K->fx * iz;
    dp_dPc[1] = 0.0f;
    dp_dPc[2] = -K->fx * X * iz2;
    dp_dPc[3] = 0.0f;
    dp_dPc[4] = K->fy * iz;
    dp_dPc[5] = -K->fy * Y * iz2;

    /* ∂Pc/∂δφ = -[Pc]×  (3×3 row-major, negative skew) */
    float dPc_dphi[9];
    dPc_dphi[0] =  0.0f; dPc_dphi[1] =  Z;    dPc_dphi[2] = -Y;
    dPc_dphi[3] = -Z;    dPc_dphi[4] =  0.0f; dPc_dphi[5] =  X;
    dPc_dphi[6] =  Y;    dPc_dphi[7] = -X;    dPc_dphi[8] =  0.0f;

    /* J_rot = dp_dPc (2×3) * dPc_dphi (3×3) → 2×3 */
    float J_rot[6];
    levio_matmul(dp_dPc, dPc_dphi, J_rot, 2, 3, 3);

    /* J_trans = dp_dPc (2×3) * I (3×3) = dp_dPc */
    /* Assemble 2×6 output: [J_rot | J_trans] */
    J26[0] = J_rot[0]; J26[1] = J_rot[1]; J26[2]  = J_rot[2];
    J26[3] = dp_dPc[0]; J26[4] = dp_dPc[1]; J26[5]  = dp_dPc[2];
    J26[6] = J_rot[3]; J26[7] = J_rot[4]; J26[8]  = J_rot[5];
    J26[9] = dp_dPc[3]; J26[10]= dp_dPc[4]; J26[11] = dp_dPc[5];
}

void levio_reproj_jacobian_point(const levio_K_t *K,
                                  const float Rcw[9], vec3f_t Pc,
                                  float J23[6])
{
    /* 2×3 Jacobian: ∂[proj] / ∂Pw
     *   ∂proj/∂Pw = ∂proj/∂Pc * ∂Pc/∂Pw
     *   ∂Pc/∂Pw   = Rcw (3×3)
     */
    float X = Pc.x, Y = Pc.y, Z = Pc.z;
    float iz  = 1.0f / Z;
    float iz2 = iz * iz;

    /* ∂proj/∂Pc (2×3) */
    float dp_dPc[6];
    dp_dPc[0] = K->fx * iz;  dp_dPc[1] = 0.0f;       dp_dPc[2] = -K->fx * X * iz2;
    dp_dPc[3] = 0.0f;        dp_dPc[4] = K->fy * iz;  dp_dPc[5] = -K->fy * Y * iz2;

    /* J23 = dp_dPc (2×3) * Rcw (3×3) → 2×3 */
    levio_matmul(dp_dPc, Rcw, J23, 2, 3, 3);
}
