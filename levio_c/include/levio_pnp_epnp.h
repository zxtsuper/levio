/**
 * @file levio_pnp_epnp.h
 * @brief EPnP + RANSAC for 2D-3D absolute pose estimation.
 *
 * Paper §3.2 (fourth segment):
 *   "When LEVIO_PNP_MIN_INLIERS or more world-point matches are available,
 *    EPnP + RANSAC is used to compute the absolute camera pose."
 *
 * EPnP algorithm overview (Lepetit et al., IJCV 2009):
 *   1) Express each world point as a weighted combination of 4 control points.
 *   2) Set up a 12×12 linear system M^T M and compute its null-space.
 *   3) Recover control-point coordinates in the camera frame from the null
 *      vectors (1, 2, or 4 null vectors depending on noise level).
 *   4) Recover R, t from the aligned control points.
 *
 * TODO: implement the full EPnP solver (step 2–4 require a 12×12 null-space
 *       computation and a 3×3 SVD for absolute orientation).
 */
#ifndef LEVIO_PNP_EPNP_H
#define LEVIO_PNP_EPNP_H

#include "levio_types.h"
#include "levio_camera.h"
#include "levio_cfg.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Result of EPnP + RANSAC.
 */
typedef struct {
    float   Rcw[9];     /**< rotation world→camera (row-major 3×3) */
    vec3f_t tcw;        /**< translation world→camera */
    int     n_inliers;  /**< RANSAC inlier count */
    uint8_t valid;      /**< 1 if a valid pose was recovered */
} levio_pnp_result_t;

/**
 * @brief Estimate camera pose from 2D-3D correspondences via EPnP + RANSAC.
 *
 * @param Pw       3-D world points  (n entries).
 * @param px       2-D pixel observations corresponding to Pw (n entries).
 * @param n        Number of correspondences.
 * @param K        Camera intrinsics.
 * @param out      Result structure.
 * @return         0 on success, -1 if fewer than 4 correspondences.
 *
 * TODO: full EPnP implementation with null-space solver.
 */
int levio_epnp_ransac(const vec3f_t *Pw, const vec2f_t *px, int n,
                      const levio_K_t *K, levio_pnp_result_t *out);

/**
 * @brief Minimal 4-point EPnP solver (RANSAC hypothesis generator).
 *
 * @param Pw4   Exactly 4 world points.
 * @param px4   Corresponding 4 pixel observations.
 * @param K     Camera intrinsics.
 * @param Rcw   Output rotation (3×3 row-major).
 * @param tcw   Output translation.
 * @return      0 on success.
 *
 * TODO: implement EPnP inner solver.
 */
int levio_epnp_minimal(const vec3f_t Pw4[4], const vec2f_t px4[4],
                        const levio_K_t *K, float Rcw[9], vec3f_t *tcw);

#ifdef __cplusplus
}
#endif

#endif /* LEVIO_PNP_EPNP_H */
