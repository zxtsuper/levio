/**
 * @file levio_ransac_8pt.c
 * @brief 8-point essential matrix + RANSAC stub.
 *
 * TODO: Implement levio_compute_essential() using a minimal 8×9 null-space
 *       solver (SVD or QR), and the full R,t recovery (4-way decomposition
 *       + cheirality test).
 */
#include "levio_ransac_8pt.h"
#include "levio_math.h"
#include <string.h>
#include <math.h>

int levio_compute_essential(const vec3f_t n_pts[8], const vec3f_t n_cur[8],
                             float E_out[9])
{
    /* TODO: Build 8×9 coefficient matrix A from the epipolar constraint
     *         x'^T E x = 0
     *       Solve Ae = 0 via SVD (last right singular vector).
     *       Enforce rank-2 constraint on the resulting E.
     */
    (void)n_pts;
    (void)n_cur;
    memset(E_out, 0, 9 * sizeof(float));
    return -1; /* not yet implemented */
}

int levio_ransac_8pt(const vec2f_t *px1, const vec2f_t *px2, int n,
                     const levio_K_t *K, levio_8pt_result_t *out)
{
    /* TODO: Full RANSAC loop:
     *  1) Randomly sample 8 correspondences.
     *  2) Normalise pixel coordinates using K.
     *  3) Call levio_compute_essential() to get E.
     *  4) Decompose E → (R1, R2) × (t, -t) (4 candidates).
     *  5) Disambiguate via positive-depth cheirality check.
     *  6) Count inliers: symmetric epipolar distance < threshold.
     *  7) Keep best hypothesis.
     */
    (void)px1; (void)px2; (void)n; (void)K;

    out->n_inliers = 0;
    out->valid     = 0;
    memset(out->R, 0, sizeof(out->R));
    out->R[0] = out->R[4] = out->R[8] = 1.0f; /* identity fallback */
    out->t.x = out->t.y = out->t.z = 0.0f;

    if (n < 8)
        return -1;

    /* TODO: implement */
    return -1;
}
