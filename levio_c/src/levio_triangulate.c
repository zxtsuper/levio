/**
 * @file levio_triangulate.c
 * @brief Two-view triangulation – midpoint method (closed form, no SVD).
 *
 * For each correspondence, the 3-D point is found as the midpoint of the
 * common perpendicular of the two camera rays.  The method degenerates
 * gracefully (returns -1) when the rays are (near-)parallel or the
 * reconstructed point is behind either camera.
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
    /* -----------------------------------------------------------------------
     * Midpoint triangulation
     *
     * Camera i: X_ci = Ri * Xw + ti   (world → camera frame)
     * Camera centre in world: Ci = -Ri^T * ti
     * Ray direction in world: di = Ri^T * unproject(K, pxi)   (unit vector)
     *
     * Find s, t that minimise |C1+s*d1 - C2-t*d2|^2
     * 2×2 linear system:
     *   [d1·d1  -d1·d2] [s]   [d1·(C2-C1)]
     *   [d1·d2  -d2·d2] [t] = [d2·(C2-C1)]
     * --------------------------------------------------------------------- */
    mat3f_t R1m, R2m, R1t, R2t;
    int i;

    for (i = 0; i < 9; ++i) {
        R1m.m[i] = R1[i];
        R2m.m[i] = R2[i];
    }
    R1t = levio_mat3_transpose(R1m);
    R2t = levio_mat3_transpose(R2m);

    /* Camera centres in world */
    vec3f_t C1 = levio_mat3_mul_vec3(R1t, levio_vec3_scale(t1, -1.0f));
    vec3f_t C2 = levio_mat3_mul_vec3(R2t, levio_vec3_scale(t2, -1.0f));

    /* Unprojected rays (in camera frame), then rotate to world frame */
    vec3f_t ray1_c = {(px1.x - K->cx) / K->fx,
                      (px1.y - K->cy) / K->fy, 1.0f};
    vec3f_t ray2_c = {(px2.x - K->cx) / K->fx,
                      (px2.y - K->cy) / K->fy, 1.0f};

    vec3f_t d1 = levio_vec3_normalize(levio_mat3_mul_vec3(R1t, ray1_c));
    vec3f_t d2 = levio_vec3_normalize(levio_mat3_mul_vec3(R2t, ray2_c));

    /* Solve 2×2 system */
    float d1d1 = levio_vec3_dot(d1, d1); /* ≈ 1 */
    float d1d2 = levio_vec3_dot(d1, d2);
    float d2d2 = levio_vec3_dot(d2, d2); /* ≈ 1 */

    float det = d1d1 * d2d2 - d1d2 * d1d2;
    if (fabsf(det) < 1e-8f) {
        /* Rays nearly parallel – degenerate triangulation */
        Pw->x = Pw->y = Pw->z = 0.0f;
        return -1;
    }

    vec3f_t diff = levio_vec3_sub(C2, C1);
    float b1 = levio_vec3_dot(d1, diff);
    float b2 = levio_vec3_dot(d2, diff);

    float inv_det = 1.0f / det;
    float s = (b1 * d2d2 - b2 * d1d2) * inv_det;
    float t_param = (b1 * d1d2 - b2 * d1d1) * inv_det;

    /* Closest points on each ray */
    vec3f_t P1 = levio_vec3_add(C1, levio_vec3_scale(d1, s));
    vec3f_t P2 = levio_vec3_add(C2, levio_vec3_scale(d2, t_param));

    /* Midpoint */
    Pw->x = (P1.x + P2.x) * 0.5f;
    Pw->y = (P1.y + P2.y) * 0.5f;
    Pw->z = (P1.z + P2.z) * 0.5f;

    /* Check positive depth in both cameras */
    vec3f_t Pc1 = levio_vec3_add(levio_mat3_mul_vec3(R1m, *Pw), t1);
    vec3f_t Pc2 = levio_vec3_add(levio_mat3_mul_vec3(R2m, *Pw), t2);
    if (Pc1.z <= 0.0f || Pc2.z <= 0.0f) {
        Pw->x = Pw->y = Pw->z = 0.0f;
        return -1;
    }

    /* Sanity: reject if residual distance between rays is too large */
    vec3f_t gap = levio_vec3_sub(P1, P2);
    if (levio_vec3_norm(gap) > 10.0f) {
        Pw->x = Pw->y = Pw->z = 0.0f;
        return -1;
    }

    return 0;
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
