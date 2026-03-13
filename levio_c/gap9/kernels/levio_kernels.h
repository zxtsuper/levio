/**
 * @file levio_kernels.h
 * @brief GAP9 cluster kernel declarations for LEVIO parallel processing.
 *
 * These kernels run on the 8 Processing Elements (PE) of the GAP9 Cluster.
 * On desktop / non-GAP9 builds (when LEVIO_USE_GAP9_CLUSTER is not defined)
 * single-threaded fallback implementations are provided automatically.
 *
 * Parallel sections:
 *   1) Descriptor matching   – Hamming-distance brute-force over N×M pairs.
 *   2) RANSAC inlier count   – per-hypothesis reprojection error evaluation.
 *
 * Usage (FC side, GAP9):
 * @code
 *   levio_match_args_t args = { feats_a, n_a, feats_b, n_b, out, max_out };
 *   levio_cl_match_ratio_fork(&args);  // enqueues + waits
 * @endcode
 */
#ifndef LEVIO_KERNELS_H
#define LEVIO_KERNELS_H

#include "levio_types.h"
#include "levio_camera.h"

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------
 * Descriptor matching kernel
 * -------------------------------------------------------------------------- */

/**
 * @brief Arguments for the parallel matching kernel.
 *
 * FC populates this structure and passes a pointer to the cluster task.
 */
typedef struct {
    const levio_feat_t *feats_a;   /**< Query feature set A */
    int                 n_a;       /**< Size of A */
    const levio_feat_t *feats_b;   /**< Reference feature set B */
    int                 n_b;       /**< Size of B */
    levio_match_t      *out;       /**< Output match array */
    int                 max_out;   /**< Capacity of out[] */
    float               ratio;     /**< Lowe ratio threshold */
    int                 n_matches; /**< Written by kernel: number of matches */
} levio_match_args_t;

/**
 * @brief PE kernel entry: ratio-test matching (parallel over A's features).
 *
 * Partition features_a evenly across available PEs.
 * Each PE writes to its own region of out[].  After the fork, the FC
 * compacts the per-PE results into the final match list.
 *
 * @param args  Pointer to levio_match_args_t.
 */
void levio_cl_match_kernel(void *args);

/**
 * @brief FC-side helper: fork cluster team for matching, wait for completion.
 *
 * On non-GAP9 builds this simply calls the single-threaded matcher.
 *
 * @param args  Pointer to levio_match_args_t (updated n_matches on return).
 */
void levio_cl_match_ratio_fork(levio_match_args_t *args);

/* --------------------------------------------------------------------------
 * RANSAC inlier-counting kernel
 * -------------------------------------------------------------------------- */

/**
 * @brief Arguments for the parallel RANSAC inlier-count kernel.
 */
typedef struct {
    const float        *Rcw;       /**< Rotation 3×3 (row-major) */
    const vec3f_t      *tcw;       /**< Translation */
    const levio_K_t    *K;         /**< Camera intrinsics */
    const vec3f_t      *Pw;        /**< World points array */
    const vec2f_t      *px;        /**< 2D observations array */
    int                 n;         /**< Number of correspondences */
    float               thr2;      /**< Reprojection error threshold² */
    int                 n_inliers; /**< Written by kernel: total inlier count */
} levio_inlier_args_t;

/**
 * @brief PE kernel entry: count inliers for a given pose hypothesis.
 *
 * Each PE evaluates a stripe of the n correspondences and accumulates a
 * local count.  The FC sums local counts after the fork.
 *
 * @param args  Pointer to levio_inlier_args_t.
 */
void levio_cl_inlier_kernel(void *args);

/**
 * @brief FC-side helper: fork cluster team for inlier counting.
 *
 * @param args  Pointer to levio_inlier_args_t (updated n_inliers on return).
 */
void levio_cl_inlier_count_fork(levio_inlier_args_t *args);

#ifdef __cplusplus
}
#endif

#endif /* LEVIO_KERNELS_H */
