/**
 * @file levio_camera.h
 * @brief Camera projection / unprojection with known pinhole intrinsics.
 *
 * Paper §3.2 assumes a monocular camera with known intrinsics.
 * No distortion model is applied here (assume pre-rectified images, or
 * extend by adding a radtan/equidist layer before calling project()).
 */
#ifndef LEVIO_CAMERA_H
#define LEVIO_CAMERA_H

#include "levio_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Project a 3-D point (in camera frame) to pixel coordinates.
 *
 * px = [fx * X/Z + cx,  fy * Y/Z + cy]
 *
 * @param K   camera intrinsics
 * @param Pc  3-D point in camera frame
 * @param px  output pixel coordinates
 * @return    0 on success, -1 if point is behind camera (Z <= 0)
 */
int levio_project(const levio_K_t *K, vec3f_t Pc, vec2f_t *px);

/**
 * @brief Unproject a pixel to a unit bearing vector in the camera frame.
 *
 * ray = [(u - cx)/fx,  (v - cy)/fy,  1]^T  (NOT unit, use levio_normalize if needed)
 *
 * @param K   camera intrinsics
 * @param px  pixel coordinates
 * @return    bearing ray (z=1 normalised plane)
 */
vec3f_t levio_unproject(const levio_K_t *K, vec2f_t px);

/**
 * @brief Compute reprojection error (pixel distance) for a world point.
 *
 * @param K    camera intrinsics
 * @param Rcw  rotation world→camera (row-major 3×3)
 * @param tcw  translation world→camera
 * @param Pw   3-D world point
 * @param obs  observed pixel
 * @param err  output 2-vector [ex, ey]
 * @return     0 on success, -1 if behind camera
 */
int levio_reproj_error(const levio_K_t *K,
                       const float Rcw[9], vec3f_t tcw,
                       vec3f_t Pw, vec2f_t obs,
                       vec2f_t *err);

/**
 * @brief Compute the 2×6 Jacobian of the reprojection error w.r.t. a
 *        small SE3 left-perturbation of the camera pose.
 *
 * Used in the LM back-end for the visual reprojection factor Jacobian.
 * The 2×6 matrix is stored row-major: [∂ex/∂ξ; ∂ey/∂ξ]
 *
 * @param K    camera intrinsics
 * @param Pc   3-D point in camera frame (already transformed)
 * @param J26  output 2×6 Jacobian (row-major)
 */
void levio_reproj_jacobian_pose(const levio_K_t *K, vec3f_t Pc, float J26[12]);

/**
 * @brief Compute the 2×3 Jacobian of the reprojection error w.r.t. the
 *        3-D world point position.
 *
 * @param K    camera intrinsics
 * @param Rcw  rotation world→camera (row-major 3×3)
 * @param Pc   point in camera frame
 * @param J23  output 2×3 Jacobian (row-major)
 */
void levio_reproj_jacobian_point(const levio_K_t *K,
                                 const float Rcw[9], vec3f_t Pc,
                                 float J23[6]);

#ifdef __cplusplus
}
#endif

#endif /* LEVIO_CAMERA_H */
