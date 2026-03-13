/**
 * @file levio_ransac_8pt.c
 * @brief Normalised 8-point essential matrix estimator + RANSAC.
 *
 * Algorithm:
 *   1) Build 8×9 epipolar constraint matrix A.
 *   2) Find null vector of A via smallest eigenvector of A^T*A (9×9).
 *   3) Enforce rank-2 on the resulting E via 3×3 SVD.
 *   4) Decompose E → 4 candidate (R, t) pairs.
 *   5) Disambiguate via cheirality (positive depth) check.
 *   RANSAC loop wraps steps 1-5 over random 8-subsets.
 */
#include "levio_ransac_8pt.h"
#include "levio_math.h"
#include "levio_camera.h"
#include <string.h>
#include <math.h>
#include <stdint.h>

/* -------------------------------------------------------------------------
 * Helpers
 * --------------------------------------------------------------------- */

/* Sampson epipolar distance (squared) for a single correspondence.
 * x1, x2 are normalised homogeneous (z=1) image coordinates. */
static float sampson_dist2(const float E[9],
                            float u1, float v1,
                            float u2, float v2)
{
    /* Ex  = E  * [u1,v1,1]^T */
    float Ex[3], Etx[3];
    Ex[0] = E[0]*u1 + E[1]*v1 + E[2];
    Ex[1] = E[3]*u1 + E[4]*v1 + E[5];
    Ex[2] = E[6]*u1 + E[7]*v1 + E[8];

    /* E^T x2 */
    Etx[0] = E[0]*u2 + E[3]*v2 + E[6];
    Etx[1] = E[1]*u2 + E[4]*v2 + E[7];
    Etx[2] = E[2]*u2 + E[5]*v2 + E[8];

    float num = E[0]*u1*u2 + E[1]*v1*u2 + E[2]*u2
              + E[3]*u1*v2 + E[4]*v1*v2 + E[5]*v2
              + E[6]*u1    + E[7]*v1    + E[8];
    num = num * num;

    float den = Ex[0]*Ex[0] + Ex[1]*Ex[1]
              + Etx[0]*Etx[0] + Etx[1]*Etx[1];

    if (den < 1e-15f)
        return 1e30f;
    return num / den;
}

/* Simple cheirality check: given R, t and one normalised pair, check
 * that the triangulated point is in front of both cameras. */
static int cheirality_ok(const float R[9], const float t[3],
                          float u1, float v1, float u2, float v2)
{
    /* Camera 1: identity. Camera 2: R, t.
     * Triangulate via midpoint (inline, using the midpoint formula). */
    float d1[3] = {u1, v1, 1.0f};
    float n1 = sqrtf(d1[0]*d1[0] + d1[1]*d1[1] + d1[2]*d1[2]);
    d1[0] /= n1; d1[1] /= n1; d1[2] /= n1;

    /* d2 in camera-1 world = R^T * [u2,v2,1]^T */
    float d2c[3] = {u2, v2, 1.0f};
    float nd2 = sqrtf(d2c[0]*d2c[0]+d2c[1]*d2c[1]+d2c[2]*d2c[2]);
    d2c[0] /= nd2; d2c[1] /= nd2; d2c[2] /= nd2;

    float d2[3];
    d2[0] = R[0]*d2c[0]+R[3]*d2c[1]+R[6]*d2c[2];
    d2[1] = R[1]*d2c[0]+R[4]*d2c[1]+R[7]*d2c[2];
    d2[2] = R[2]*d2c[0]+R[5]*d2c[1]+R[8]*d2c[2];

    /* Camera-2 centre in world coords: C2 = -R^T * t */
    float C2[3];
    C2[0] = -(R[0]*t[0]+R[3]*t[1]+R[6]*t[2]);
    C2[1] = -(R[1]*t[0]+R[4]*t[1]+R[7]*t[2]);
    C2[2] = -(R[2]*t[0]+R[5]*t[1]+R[8]*t[2]);

    float d1d1 = d1[0]*d1[0]+d1[1]*d1[1]+d1[2]*d1[2];
    float d1d2 = d1[0]*d2[0]+d1[1]*d2[1]+d1[2]*d2[2];
    float d2d2 = d2[0]*d2[0]+d2[1]*d2[1]+d2[2]*d2[2];

    float det = d1d1*d2d2 - d1d2*d1d2;
    if (fabsf(det) < 1e-8f)
        return 0;

    float b1 = C2[0]*d1[0]+C2[1]*d1[1]+C2[2]*d1[2];
    float b2 = C2[0]*d2[0]+C2[1]*d2[1]+C2[2]*d2[2];

    float s = (b1*d2d2 - b2*d1d2) / det;
    float tpar = (b1*d1d2 - b2*d1d1) / det;

    if (s < 0.0f || tpar < 0.0f)
        return 0;

    /* Point in world: P ≈ s*d1 */
    /* Check depth in camera 2: z_cam2 = R[2,:] * P + t[2] */
    float Px = d1[0]*s, Py = d1[1]*s, Pz = d1[2]*s;
    float z2 = R[2]*Px + R[5]*Py + R[8]*Pz + t[2];
    return (Pz > 0.0f && z2 > 0.0f) ? 1 : 0;
}

/* -------------------------------------------------------------------------
 * levio_compute_essential
 * --------------------------------------------------------------------- */

int levio_compute_essential(const vec3f_t n_pts[8], const vec3f_t n_cur[8],
                             float E_out[9])
{
    /* Build 8×9 epipolar constraint matrix A.
     * Row i: [u'u, u'v, u', v'u, v'v, v', u, v, 1]
     * where (u,v) = n_pts[i], (u',v') = n_cur[i] */
    float A[8 * 9];
    float AtA[9 * 9];
    int i, j, k;

    for (i = 0; i < 8; ++i) {
        float u  = n_pts[i].x, v  = n_pts[i].y;
        float up = n_cur[i].x, vp = n_cur[i].y;
        A[i*9+0] = up*u;  A[i*9+1] = up*v;  A[i*9+2] = up;
        A[i*9+3] = vp*u;  A[i*9+4] = vp*v;  A[i*9+5] = vp;
        A[i*9+6] = u;     A[i*9+7] = v;     A[i*9+8] = 1.0f;
    }

    /* A^T * A (9×9 symmetric) */
    for (i = 0; i < 9; ++i)
        for (j = 0; j < 9; ++j) {
            float s = 0.0f;
            for (k = 0; k < 8; ++k)
                s += A[k*9+i] * A[k*9+j];
            AtA[i*9+j] = s;
        }

    /* Find smallest eigenvector of AtA (9×9) */
    float V9[9*9];
    levio_sym_eig(AtA, V9, 9);

    /* Find index of smallest eigenvalue */
    int min_idx = 0;
    float min_ev = AtA[0];
    for (i = 1; i < 9; ++i) {
        float ev = AtA[i*9+i];
        if (ev < min_ev) { min_ev = ev; min_idx = i; }
    }

    /* Extract null vector → raw E */
    float E_raw[9];
    for (i = 0; i < 9; ++i)
        E_raw[i] = V9[i*9+min_idx];

    /* Enforce rank-2 constraint via SVD: set smallest singular value to 0,
     * average the two larger ones to enforce s1 = s2 (essential matrix). */
    float Ue[9], se[3], Vte[9];
    levio_svd3(E_raw, Ue, se, Vte);

    float sm = (se[0] + se[1]) * 0.5f;

    /* E = U * diag(sm,sm,0) * Vt (row-major multiply) */
    for (i = 0; i < 3; ++i)
        for (j = 0; j < 3; ++j) {
            float val = 0.0f;
            for (k = 0; k < 3; ++k) {
                float sv = (k == 2) ? 0.0f : sm;
                val += Ue[i*3+k] * sv * Vte[k*3+j];
            }
            E_out[i*3+j] = val;
        }

    return 0;
}

/* -------------------------------------------------------------------------
 * levio_ransac_8pt – full RANSAC loop
 * --------------------------------------------------------------------- */

int levio_ransac_8pt(const vec2f_t *px1, const vec2f_t *px2, int n,
                     const levio_K_t *K, levio_8pt_result_t *out)
{
    int i, iter;

    out->n_inliers = 0;
    out->valid     = 0;
    memset(out->R, 0, sizeof(out->R));
    out->R[0] = out->R[4] = out->R[8] = 1.0f;
    out->t.x = out->t.y = out->t.z = 0.0f;

    if (n < 8)
        return -1;

    /* -----------------------------------------------------------------
     * Normalise pixel coords to improve numerical conditioning.
     * T: translate by centroid, scale so mean distance to origin = sqrt(2).
     * ------------------------------------------------------------- */
    float cx1 = 0.0f, cy1 = 0.0f, cx2 = 0.0f, cy2 = 0.0f;
    for (i = 0; i < n; ++i) {
        cx1 += px1[i].x; cy1 += px1[i].y;
        cx2 += px2[i].x; cy2 += px2[i].y;
    }
    cx1 /= n; cy1 /= n; cx2 /= n; cy2 /= n;

    float sc1 = 0.0f, sc2 = 0.0f;
    for (i = 0; i < n; ++i) {
        float dx1 = px1[i].x - cx1, dy1 = px1[i].y - cy1;
        float dx2 = px2[i].x - cx2, dy2 = px2[i].y - cy2;
        sc1 += sqrtf(dx1*dx1 + dy1*dy1);
        sc2 += sqrtf(dx2*dx2 + dy2*dy2);
    }
    sc1 = (sc1 < 1e-8f) ? 1.0f : (1.41421356f * n / sc1);
    sc2 = (sc2 < 1e-8f) ? 1.0f : (1.41421356f * n / sc2);

    /* Normalised coordinates (heap-free: use static buffer) */
    /* Use camera intrinsics-based normalisation when building epipolar matrix */
    float ifx = 1.0f / K->fx, ify = 1.0f / K->fy;

    /* Convert sampson threshold from pixels to normalised coordinates */
    float thr_norm = LEVIO_RANSAC_REPROJ_THR * LEVIO_RANSAC_REPROJ_THR
                     * (ifx * ifx + ify * ify) * 0.5f;

    /* W matrix for E decomposition: W = [[0,-1,0],[1,0,0],[0,0,1]] */
    static const float W[9] = {0,-1,0, 1,0,0, 0,0,1};
    static const float Wt[9] = {0,1,0, -1,0,0, 0,0,1};

    /* RANSAC state */
    int   best_inliers = 0;
    float best_E[9];
    memset(best_E, 0, sizeof(best_E));
    best_E[0] = best_E[4] = best_E[8] = 0.0f;

    uint32_t rng = 12345u;

    for (iter = 0; iter < LEVIO_RANSAC_MAX_ITER; ++iter) {
        /* Sample 8 distinct indices */
        int idx[8];
        int cnt = 0, tries = 0;
        while (cnt < 8 && tries < 1000) {
            int r = (int)(levio_rand_next(&rng) % (uint32_t)n);
            int dup = 0;
            int di;
            for (di = 0; di < cnt; ++di)
                if (idx[di] == r) { dup = 1; break; }
            if (!dup) idx[cnt++] = r;
            ++tries;
        }
        if (cnt < 8) continue;

        /* Build normalised point pairs */
        vec3f_t np1[8], np2[8];
        for (i = 0; i < 8; ++i) {
            int id = idx[i];
            np1[i].x = (px1[id].x - K->cx) * ifx;
            np1[i].y = (px1[id].y - K->cy) * ify;
            np1[i].z = 1.0f;
            np2[i].x = (px2[id].x - K->cx) * ifx;
            np2[i].y = (px2[id].y - K->cy) * ify;
            np2[i].z = 1.0f;
        }

        /* Estimate E */
        float E_hyp[9];
        if (levio_compute_essential(np1, np2, E_hyp) != 0)
            continue;

        /* Count inliers using Sampson distance */
        int n_in = 0;
        for (i = 0; i < n; ++i) {
            float u1 = (px1[i].x - K->cx) * ifx;
            float v1 = (px1[i].y - K->cy) * ify;
            float u2 = (px2[i].x - K->cx) * ifx;
            float v2 = (px2[i].y - K->cy) * ify;
            if (sampson_dist2(E_hyp, u1, v1, u2, v2) < thr_norm)
                ++n_in;
        }

        if (n_in > best_inliers) {
            best_inliers = n_in;
            memcpy(best_E, E_hyp, 9 * sizeof(float));
        }
    }

    if (best_inliers < 8) {
        out->n_inliers = best_inliers;
        return -1;
    }

    /* -----------------------------------------------------------------
     * Decompose best E → (R, t)
     * E = U * Sigma * Vt
     * Two rotation candidates: R1 = U*W*Vt, R2 = U*W^T*Vt
     * Two translation candidates: t = U[:,2], t = -U[:,2]
     * Disambiguate via cheirality (first inlier point).
     * ------------------------------------------------------------- */
    float Ue[9], se[3], Vte[9];
    levio_svd3(best_E, Ue, se, Vte);

    /* Ensure det(U) and det(Vt) are +1 */
    {
        float detU =  Ue[0]*(Ue[4]*Ue[8]-Ue[5]*Ue[7])
                    - Ue[1]*(Ue[3]*Ue[8]-Ue[5]*Ue[6])
                    + Ue[2]*(Ue[3]*Ue[7]-Ue[4]*Ue[6]);
        if (detU < 0.0f)
            for (i = 0; i < 9; ++i) Ue[i] = -Ue[i];

        float detV =  Vte[0]*(Vte[4]*Vte[8]-Vte[5]*Vte[7])
                    - Vte[1]*(Vte[3]*Vte[8]-Vte[5]*Vte[6])
                    + Vte[2]*(Vte[3]*Vte[7]-Vte[4]*Vte[6]);
        if (detV < 0.0f)
            for (i = 0; i < 9; ++i) Vte[i] = -Vte[i];
    }

    /* Compute U*W (or U*Wt) then multiply by Vt */
    float R_cands[4][9];
    float t_cands[4][3];

    /* Helper: multiply 3×3 A * B → C (row-major) */
#define MM3(A, B, C) \
    do { int _i,_j,_k; for(_i=0;_i<3;_i++) for(_j=0;_j<3;_j++){ \
        float _s=0; for(_k=0;_k<3;_k++) _s+=(A)[_i*3+_k]*(B)[_k*3+_j]; \
        (C)[_i*3+_j]=_s; } } while(0)

    float UW[9], UWt[9];
    MM3(Ue, W,  UW);
    MM3(Ue, Wt, UWt);
    MM3(UW,  Vte, R_cands[0]);
    MM3(UW,  Vte, R_cands[1]);
    MM3(UWt, Vte, R_cands[2]);
    MM3(UWt, Vte, R_cands[3]);
#undef MM3

    /* t = ±U[:,2] */
    for (i = 0; i < 3; ++i) {
        t_cands[0][i] =  Ue[i*3+2];
        t_cands[1][i] = -Ue[i*3+2];
        t_cands[2][i] =  Ue[i*3+2];
        t_cands[3][i] = -Ue[i*3+2];
    }

    /* Find first inlier for cheirality test */
    int test_idx = -1;
    for (i = 0; i < n; ++i) {
        float u1 = (px1[i].x - K->cx) * ifx;
        float v1 = (px1[i].y - K->cy) * ify;
        float u2 = (px2[i].x - K->cx) * ifx;
        float v2 = (px2[i].y - K->cy) * ify;
        if (sampson_dist2(best_E, u1, v1, u2, v2) < thr_norm) {
            test_idx = i;
            break;
        }
    }

    int best_cand = 0;
    if (test_idx >= 0) {
        float u1 = (px1[test_idx].x - K->cx) * ifx;
        float v1 = (px1[test_idx].y - K->cy) * ify;
        float u2 = (px2[test_idx].x - K->cx) * ifx;
        float v2 = (px2[test_idx].y - K->cy) * ify;
        int ci;
        for (ci = 0; ci < 4; ++ci) {
            if (cheirality_ok(R_cands[ci], t_cands[ci], u1, v1, u2, v2)) {
                best_cand = ci;
                break;
            }
        }
    }

    /* Fill output */
    memcpy(out->R, R_cands[best_cand], 9 * sizeof(float));
    out->t.x = t_cands[best_cand][0];
    out->t.y = t_cands[best_cand][1];
    out->t.z = t_cands[best_cand][2];
    out->n_inliers = best_inliers;
    out->valid = 1;

    return 0;
}
