/**
 * @file levio_match.h
 * @brief Bidirectional brute-force ORB descriptor matcher.
 *
 * Paper §3.2 (second segment): "bidirectional brute-force matching" using
 * Hamming distance on 256-bit ORB descriptors.
 *
 * A match (a_i, b_j) is accepted only when:
 *   1) b_j is the nearest neighbour of a_i  AND
 *   2) a_i is the nearest neighbour of b_j  (cross-check / bidirectional test).
 *
 * An optional ratio test (LEVIO_MATCH_RATIO_THR) can further reduce outliers.
 */
#ifndef LEVIO_MATCH_H
#define LEVIO_MATCH_H

#include "levio_types.h"
#include "levio_cfg.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Bidirectional brute-force matcher on ORB descriptors.
 *
 * O(n_a * n_b) Hamming-distance comparisons with mutual nearest-neighbour
 * (cross-check) filtering.
 *
 * @param a        Feature set A (e.g., current frame).
 * @param na       Number of features in A.
 * @param b        Feature set B (e.g., previous frame or world-point set).
 * @param nb       Number of features in B.
 * @param out      Output match array (caller-allocated, capacity max_out).
 * @param max_out  Maximum number of matches to return.
 * @return         Number of matches found, or -1 on error.
 */
int levio_match_bidir(const levio_feat_t *a, int na,
                      const levio_feat_t *b, int nb,
                      levio_match_t *out, int max_out);

/**
 * @brief Nearest-neighbour ratio-test matcher (Lowe's ratio test).
 *
 * Accepts match a_i → b_j only if dist(a_i, b_j) < ratio * dist(a_i, b_k)
 * where b_k is the second-nearest neighbour.
 *
 * @param a        Feature set A.
 * @param na       Number of features in A.
 * @param b        Feature set B.
 * @param nb       Number of features in B.
 * @param ratio    Distance ratio threshold (e.g., LEVIO_MATCH_RATIO_THR).
 * @param out      Output match array (caller-allocated, capacity max_out).
 * @param max_out  Maximum number of matches to return.
 * @return         Number of matches found.
 */
int levio_match_ratio(const levio_feat_t *a, int na,
                      const levio_feat_t *b, int nb,
                      float ratio,
                      levio_match_t *out, int max_out);

#ifdef __cplusplus
}
#endif

#endif /* LEVIO_MATCH_H */
