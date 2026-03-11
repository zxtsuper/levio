/**
 * @file levio_se3.c
 * @brief SO3 / SE3 Lie-group implementations.
 */
#include "levio_se3.h"
#include "levio_math.h"
#include <math.h>
#include <string.h>

mat3f_t levio_so3_exp(vec3f_t omega)
{
    float theta = levio_vec3_norm(omega);
    mat3f_t I   = levio_mat3_identity();

    if (theta < 1e-8f)
        return I;  /* near-zero rotation: return identity */

    vec3f_t axis = levio_vec3_scale(omega, 1.0f / theta);
    mat3f_t K    = levio_mat3_skew(axis);
    mat3f_t K2   = levio_mat3_mul(K, K);

    float s = sinf(theta);
    float c = cosf(theta);

    /* R = I + sin(θ)[k]× + (1-cos(θ))[k]×² */
    mat3f_t R = I;
    int i;
    for (i = 0; i < 9; ++i)
        R.m[i] += s * K.m[i] + (1.0f - c) * K2.m[i];
    return R;
}

vec3f_t levio_so3_log(mat3f_t R)
{
    float trace = R.m[0] + R.m[4] + R.m[8];
    float cos_theta = (trace - 1.0f) * 0.5f;
    /* Clamp to [-1, 1] for numerical safety */
    if (cos_theta >  1.0f) cos_theta =  1.0f;
    if (cos_theta < -1.0f) cos_theta = -1.0f;
    float theta = acosf(cos_theta);

    vec3f_t omega;
    if (fabsf(theta) < 1e-8f) {
        /* Small angle: log R ≈ (R - R^T) / 2 */
        omega.x = (R.m[7] - R.m[5]) * 0.5f;
        omega.y = (R.m[2] - R.m[6]) * 0.5f;
        omega.z = (R.m[3] - R.m[1]) * 0.5f;
    } else {
        float s = theta / (2.0f * sinf(theta));
        omega.x = (R.m[7] - R.m[5]) * s;
        omega.y = (R.m[2] - R.m[6]) * s;
        omega.z = (R.m[3] - R.m[1]) * s;
    }
    return omega;
}

void levio_so3_right_jacobian(vec3f_t omega, mat3f_t *Jr_out)
{
    float theta = levio_vec3_norm(omega);
    mat3f_t I   = levio_mat3_identity();

    if (theta < 1e-8f) {
        *Jr_out = I;
        return;
    }

    mat3f_t K   = levio_mat3_skew(omega);
    mat3f_t K2  = levio_mat3_mul(K, K);
    float t2    = theta * theta;
    float t3    = t2 * theta;

    float c1 = (1.0f - cosf(theta)) / t2;
    float c2 = (theta - sinf(theta)) / t3;

    int i;
    mat3f_t Jr = I;
    for (i = 0; i < 9; ++i)
        Jr.m[i] += -c1 * K.m[i] + c2 * K2.m[i];
    *Jr_out = Jr;
}

/* --------------------------------------------------------------------------
 * SE3 operations
 * -------------------------------------------------------------------------- */

levio_se3_t levio_se3_identity(void)
{
    levio_se3_t T;
    T.R = levio_mat3_identity();
    T.t.x = T.t.y = T.t.z = 0.0f;
    return T;
}

levio_se3_t levio_se3_mul(levio_se3_t T_AC, levio_se3_t T_CB)
{
    levio_se3_t T_AB;
    T_AB.R = levio_mat3_mul(T_AC.R, T_CB.R);
    T_AB.t = levio_vec3_add(levio_mat3_mul_vec3(T_AC.R, T_CB.t), T_AC.t);
    return T_AB;
}

levio_se3_t levio_se3_inv(levio_se3_t T)
{
    levio_se3_t Ti;
    Ti.R = levio_mat3_transpose(T.R);
    Ti.t = levio_mat3_mul_vec3(Ti.R, levio_vec3_scale(T.t, -1.0f));
    return Ti;
}

vec3f_t levio_se3_transform(levio_se3_t T, vec3f_t p)
{
    return levio_vec3_add(levio_mat3_mul_vec3(T.R, p), T.t);
}

levio_se3_t levio_se3_boxplus(levio_se3_t T, const float delta[6])
{
    /* δξ = [δφ (rotation), δρ (translation)] */
    vec3f_t dphi = { delta[0], delta[1], delta[2] };
    vec3f_t drho = { delta[3], delta[4], delta[5] };

    mat3f_t dR = levio_so3_exp(dphi);
    levio_se3_t T_new;
    T_new.R = levio_mat3_mul(T.R, dR);
    T_new.t = levio_vec3_add(T.t, drho);
    return T_new;
}
