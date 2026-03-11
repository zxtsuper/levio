/**
 * @file levio_triangulate.h
 * @brief Two-view triangulation for new world-point creation.
 *
 * Paper §3.2 (sixth segment):
 *   "New world points are triangulated from matches between the current
 *    keyframe and the previous keyframe."
 *
 * Method: Direct Linear Transform (DLT) – for each correspondence, form two
 * equations from the cross-product   p × (P x) = 0.  Stack into a 4×4
 * system and solve via SVD (take last right singular vector).
 *
 * TODO: implement the 4×4 SVD required by levio_triangulate_dlt().
 */
#ifndef LEVIO_TRIANGULATE_H
#define LEVIO_TRIANGULATE_H

#include "levio_types.h"
#include "levio_camera.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Triangulate a single 3-D point from two views using DLT.
 *
 * @param R1   Rotation    world→camera1 (3×3 row-major).
 * @param t1   Translation world→camera1.
 * @param R2   Rotation    world→camera2 (3×3 row-major).
 * @param t2   Translation world→camera2.
 * @param K    Camera intrinsics (same for both views, monocular).
 * @param px1  Observation in camera1.
 * @param px2  Observation in camera2.
 * @param Pw   Output 3-D world point.
 * @return     0 on success, -1 if the point is behind either camera or
 *             the triangulation is degenerate (near-zero baseline).
 *
 * TODO: implement the SVD step.
 */
int levio_triangulate_dlt(const float R1[9], vec3f_t t1,
                           const float R2[9], vec3f_t t2,
                           const levio_K_t *K,
                           vec2f_t px1, vec2f_t px2,
                           vec3f_t *Pw);

/**
 * @brief Batch triangulate a set of matched feature pairs between two keyframes.
 *
 * Iterates over all matches and calls levio_triangulate_dlt() for each.
 * Successfully triangulated points are added to the world-point map
 * (managed externally).
 *
 * @param R1        Rotation world→camera1.
 * @param t1        Translation world→camera1.
 * @param R2        Rotation world→camera2.
 * @param t2        Translation world→camera2.
 * @param K         Camera intrinsics.
 * @param matches   Array of match pairs (idx_a = idx in feats1, idx_b in feats2).
 * @param n_matches Number of matches.
 * @param feats1    Features from camera1 frame.
 * @param feats2    Features from camera2 frame.
 * @param lms_out   Output landmark array (caller-provided, capacity n_matches).
 * @return          Number of successfully triangulated points.
 */
int levio_triangulate_batch(const float R1[9], vec3f_t t1,
                             const float R2[9], vec3f_t t2,
                             const levio_K_t *K,
                             const levio_match_t *matches, int n_matches,
                             const levio_feat_t *feats1,
                             const levio_feat_t *feats2,
                             levio_landmark_t *lms_out);

#ifdef __cplusplus
}
#endif

#endif /* LEVIO_TRIANGULATE_H */
