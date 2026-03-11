/**
 * @file levio_triangulate.c
 * @brief Two-view triangulation stub (DLT).
 *
 * TODO: Implement a 4×4 SVD to complete levio_triangulate_dlt().
 */
#include "levio_triangulate.h"
#include "levio_math.h"
#include <math.h>
#include <string.h>

int levio_triangulate_dlt(const float R1[9], vec3f_t t1,
                           const float R2[9], vec3f_t t2,
                           const levio_K_t *K,
                           vec2f_t px1, vec2f_t px2,
                           vec3f_t *Pw)
{
    /* TODO: Build 4×4 DLT matrix A from cross-product form
     *         p × (P * x) = 0
     *       for each view, yielding two equations per view.
     *       Solve for the homogeneous world point as the last
     *       right singular vector of A (4×4 SVD).
     *       Dehomogenise: Pw = X[0:3] / X[3].
     *       Check positive depth in both cameras.
     */
    (void)R1; (void)t1; (void)R2; (void)t2; (void)K;
    (void)px1; (void)px2;

    Pw->x = Pw->y = Pw->z = 0.0f;
    return -1; /* not yet implemented */
}

int levio_triangulate_batch(const float R1[9], vec3f_t t1,
                             const float R2[9], vec3f_t t2,
                             const levio_K_t *K,
                             const levio_match_t *matches, int n_matches,
                             const levio_feat_t *feats1,
                             const levio_feat_t *feats2,
                             levio_landmark_t *lms_out)
{
    int n_ok = 0;
    int i;

    for (i = 0; i < n_matches; ++i) {
        int ia = matches[i].idx_a;
        int ib = matches[i].idx_b;
        vec3f_t Pw;
        if (levio_triangulate_dlt(R1, t1, R2, t2, K,
                                   feats1[ia].px, feats2[ib].px,
                                   &Pw) == 0) {
            lms_out[n_ok].Pw        = Pw;
            lms_out[n_ok].valid     = 1;
            lms_out[n_ok].obs_count = 2;
            /* Copy descriptor from second frame feature */
            int desc_idx;
            for (desc_idx = 0; desc_idx < LEVIO_DESC_BYTES; ++desc_idx)
                lms_out[n_ok].desc[desc_idx] = feats2[ib].desc[desc_idx];
            ++n_ok;
        }
    }
    return n_ok;
}
