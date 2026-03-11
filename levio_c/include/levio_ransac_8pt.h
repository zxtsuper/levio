/**
 * @file levio_ransac_8pt.h
 * @brief 8-point algorithm + RANSAC for essential-matrix estimation.
 *
 * Paper §3.2 (third segment and fallback):
 *   "When fewer than LEVIO_PNP_MIN_INLIERS world-point matches are available,
 *    the system falls back to the 8-point algorithm to recover a frame-to-frame
 *    relative pose (R, t) from 2D-2D correspondences."
 *
 * Algorithm:
 *   1) RANSAC: sample 8 normalised correspondences, solve for E via SVD.
 *   2) Enforce rank-2 constraint on E (set smallest singular value to 0).
 *   3) Recover (R, t) from E: four candidate decompositions, disambiguate
 *      via cheirality test (triangulate one point, check positive depth).
 *   4) Count inliers via symmetric epipolar distance threshold.
 *
 * TODO: Implement the SVD (needed for step 1 and 3).  A minimal SVD for
 *       8×9 and 3×3 matrices is sufficient; see levio_math.h for hooks.
 */
#ifndef LEVIO_RANSAC_8PT_H
#define LEVIO_RANSAC_8PT_H

#include "levio_types.h"
#include "levio_camera.h"
#include "levio_cfg.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Result of 8-point essential-matrix RANSAC.
 */
typedef struct {
    float   R[9];       /**< rotation (row-major 3×3), current←reference */
    vec3f_t t;          /**< translation direction (unit vector, scale unknown) */
    int     n_inliers;  /**< number of RANSAC inliers */
    uint8_t valid;      /**< 1 if a valid pose was recovered */
} levio_8pt_result_t;

/**
 * @brief Estimate relative pose (R, t) from 2D-2D correspondences via
 *        8-point algorithm with RANSAC.
 *
 * @param px1      Pixel coordinates in frame 1 (reference).
 * @param px2      Pixel coordinates in frame 2 (current).
 * @param n        Number of correspondences.
 * @param K        Camera intrinsics (for normalisation).
 * @param out      Result structure.
 * @return         0 on success, -1 if insufficient correspondences (< 8).
 *
 * TODO: full SVD-based essential matrix solver.
 */
int levio_ransac_8pt(const vec2f_t *px1, const vec2f_t *px2, int n,
                     const levio_K_t *K, levio_8pt_result_t *out);

/**
 * @brief Compute essential matrix from 8 normalised point correspondences.
 *        Helper for the RANSAC inner loop.
 *
 * Solves the linear system A e = 0 where A is 8×9 and e = vec(E).
 * Then enforces rank-2 by zeroing the smallest singular value.
 *
 * @param n_pts  points in normalised coordinates from frame 1 (3×8)
 * @param n_cur  points in normalised coordinates from frame 2 (3×8)
 * @param E_out  output essential matrix (row-major 3×3)
 * @return       0 on success
 *
 * TODO: implement SVD decomposition.
 */
int levio_compute_essential(const vec3f_t n_pts[8], const vec3f_t n_cur[8],
                             float E_out[9]);

#ifdef __cplusplus
}
#endif

#endif /* LEVIO_RANSAC_8PT_H */
