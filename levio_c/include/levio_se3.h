/**
 * @file levio_se3.h
 * @brief SO3 / SE3 Lie-group helpers for LEVIO embedded VIO.
 *
 * Implements the minimal operations needed by the optimizer:
 *  - SO3 exponential map (axis-angle → rotation matrix)
 *  - SO3 logarithm (rotation matrix → axis-angle)
 *  - Right Jacobian of SO3 (needed for preintegration residual, §3.3)
 *  - SE3 pose composition and inversion
 */
#ifndef LEVIO_SE3_H
#define LEVIO_SE3_H

#include "levio_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------
 * SO3 – rotation matrices
 * -------------------------------------------------------------------------- */

/**
 * @brief Exponential map: axis-angle ω → R ∈ SO3.
 *        Uses Rodrigues formula.
 * @param omega axis-angle vector (direction = axis, magnitude = angle [rad])
 * @return rotation matrix (row-major, 3×3)
 */
mat3f_t levio_so3_exp(vec3f_t omega);

/**
 * @brief Logarithm map: R ∈ SO3 → axis-angle ω.
 * @param R  rotation matrix
 * @return axis-angle vector
 */
vec3f_t levio_so3_log(mat3f_t R);

/**
 * @brief Right Jacobian of SO3 at ω.
 *        Jr(ω) = I - (1-cos|ω|)/|ω|² [ω]× + (|ω|-sin|ω|)/|ω|³ [ω]×²
 * @param omega  axis-angle
 * @param Jr_out 3×3 output (row-major)
 */
void levio_so3_right_jacobian(vec3f_t omega, mat3f_t *Jr_out);

/* --------------------------------------------------------------------------
 * SE3 – rigid body transformations
 * -------------------------------------------------------------------------- */

/** Compact SE3 pose: R (3×3) + t (3) */
typedef struct {
    mat3f_t R;
    vec3f_t t;
} levio_se3_t;

/** Identity pose */
levio_se3_t levio_se3_identity(void);

/** T_AB = T_AC * T_CB */
levio_se3_t levio_se3_mul(levio_se3_t T_AC, levio_se3_t T_CB);

/** T^{-1}: R^T, -R^T t */
levio_se3_t levio_se3_inv(levio_se3_t T);

/** Transform point: p_b = T_ab * p_a */
vec3f_t levio_se3_transform(levio_se3_t T, vec3f_t p);

/** Box-plus on pose (6-dof perturbation [δφ; δρ] in body tangent space) */
levio_se3_t levio_se3_boxplus(levio_se3_t T, const float delta[6]);

#ifdef __cplusplus
}
#endif

#endif /* LEVIO_SE3_H */
