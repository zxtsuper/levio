/**
 * @file levio_pnp_epnp.c
 * @brief PnP + RANSAC for 2D-3D pose estimation.
 *
 * levio_epnp_minimal() uses an iterative Gauss-Newton solver on the
 * reprojection residuals (8 equations for 6 unknowns with 4 point pairs).
 * It initialises from the centroid of observations and converges in ~10 steps.
 *
 * levio_epnp_ransac() wraps the minimal solver in a RANSAC loop.
 */
#include "levio_pnp_epnp.h"
#include "levio_math.h"
#include "levio_se3.h"
#include "levio_camera.h"
#include <string.h>
#include <math.h>
#include <stdint.h>

/* Reprojection error squared (pixels²) for one 2D-3D pair. */
static float reproj_err2(const float Rcw[9], const vec3f_t *tcw,
                          const levio_K_t *K,
                          const vec3f_t *Pw, const vec2f_t *px)
{
    float X = Rcw[0]*Pw->x + Rcw[1]*Pw->y + Rcw[2]*Pw->z + tcw->x;
    float Y = Rcw[3]*Pw->x + Rcw[4]*Pw->y + Rcw[5]*Pw->z + tcw->y;
    float Z = Rcw[6]*Pw->x + Rcw[7]*Pw->y + Rcw[8]*Pw->z + tcw->z;
    if (Z <= 0.0f) return 1e30f;
    float iz = 1.0f / Z;
    float ex = K->fx * X * iz + K->cx - px->x;
    float ey = K->fy * Y * iz + K->cy - px->y;
    return ex*ex + ey*ey;
}

/* -------------------------------------------------------------------------
 * levio_epnp_minimal – 4-point iterative Gauss-Newton PnP
 * --------------------------------------------------------------------- */

int levio_epnp_minimal(const vec3f_t Pw4[4], const vec2f_t px4[4],
                        const levio_K_t *K, float Rcw[9], vec3f_t *tcw)
{
    /* Gauss-Newton on the 8 reprojection residuals (4 pairs × 2 dims)
     * for the 6-DoF camera pose [δφ (3) | δt (3)].
     *
     * Initialisation: R = I, t chosen so that world centroid projects
     *   to the centroid of the 2D observations.
     */
    int i, gn;

    /* Compute centroids */
    float cw_x = 0, cw_y = 0, cw_z = 0;
    float c2_x = 0, c2_y = 0;
    for (i = 0; i < 4; ++i) {
        cw_x += Pw4[i].x; cw_y += Pw4[i].y; cw_z += Pw4[i].z;
        c2_x += px4[i].x; c2_y += px4[i].y;
    }
    cw_x *= 0.25f; cw_y *= 0.25f; cw_z *= 0.25f;
    c2_x *= 0.25f; c2_y *= 0.25f;

    /* Initial pose: R = I, t so that centroid maps correctly */
    float cur_R[9] = {1,0,0, 0,1,0, 0,0,1};
    float cur_t[3];
    /* Guess depth as z-centroid of world points */
    float depth = cw_z;
    if (depth < 0.5f) depth = 0.5f;
    /* t such that K*[R*cw+t] = [c2_x; c2_y] at depth */
    /* For R=I: Xc = cw + t, project: fx*(cw_x+tx)/depth + cx = c2_x */
    cur_t[0] = (c2_x - K->cx) / K->fx * depth - cw_x;
    cur_t[1] = (c2_y - K->cy) / K->fy * depth - cw_y;
    cur_t[2] = depth - cw_z;

    /* Gauss-Newton loop */
    for (gn = 0; gn < 20; ++gn) {
        /* Build J^T * J (6×6) and J^T * r (6) */
        float JtJ[36], Jtr[6];
        memset(JtJ, 0, sizeof(JtJ));
        memset(Jtr, 0, sizeof(Jtr));

        for (i = 0; i < 4; ++i) {
            /* Transform point to camera frame */
            float Xc = cur_R[0]*Pw4[i].x + cur_R[1]*Pw4[i].y + cur_R[2]*Pw4[i].z + cur_t[0];
            float Yc = cur_R[3]*Pw4[i].x + cur_R[4]*Pw4[i].y + cur_R[5]*Pw4[i].z + cur_t[1];
            float Zc = cur_R[6]*Pw4[i].x + cur_R[7]*Pw4[i].y + cur_R[8]*Pw4[i].z + cur_t[2];

            if (Zc < 1e-3f) Zc = 1e-3f;
            float iz = 1.0f / Zc;

            /* Residual */
            float rx = K->fx * Xc * iz + K->cx - px4[i].x;
            float ry = K->fy * Yc * iz + K->cy - px4[i].y;

            /* Jacobian of residual w.r.t. [δφ (rotation left-perturbation); δt]
             * ∂rx/∂δφ = -fx*[∂(Xc/Zc)/∂δφ] = -fx*[1/Zc, 0, -Xc/Zc²] * [Pc]×
             * Using levio_reproj_jacobian_pose output layout (2×6): [J_rot | J_trans]
             */
            vec3f_t Pc = {Xc, Yc, Zc};
            float J26[12];
            levio_reproj_jacobian_pose(K, Pc, J26);

            /* Accumulate J^T * J and J^T * r */
            int a, b;
            for (a = 0; a < 6; ++a) {
                Jtr[a] += J26[0*6+a] * rx + J26[1*6+a] * ry;
                for (b = 0; b < 6; ++b)
                    JtJ[a*6+b] += J26[0*6+a]*J26[0*6+b] + J26[1*6+a]*J26[1*6+b];
            }
        }

        /* Add small Levenberg damping */
        for (i = 0; i < 6; ++i)
            JtJ[i*6+i] += 1e-4f;

        /* Solve (J^T J) * delta = J^T * r */
        float delta[6];
        memcpy(delta, Jtr, sizeof(Jtr));
        if (levio_solve_chol(JtJ, delta, 6) != 0)
            break;

        /* Apply update: pose = boxplus(pose, -delta) */
        levio_se3_t T;
        for (int m = 0; m < 9; ++m) T.R.m[m] = cur_R[m];
        T.t.x = cur_t[0]; T.t.y = cur_t[1]; T.t.z = cur_t[2];

        float neg_delta[6];
        for (i = 0; i < 6; ++i) neg_delta[i] = -delta[i];
        T = levio_se3_boxplus(T, neg_delta);

        for (int m = 0; m < 9; ++m) cur_R[m] = T.R.m[m];
        cur_t[0] = T.t.x; cur_t[1] = T.t.y; cur_t[2] = T.t.z;

        /* Convergence check */
        float delta_norm = 0.0f;
        for (i = 0; i < 6; ++i) delta_norm += delta[i]*delta[i];
        if (delta_norm < 1e-10f)
            break;
    }

    /* Check that all 4 points project with positive depth */
    for (i = 0; i < 4; ++i) {
        float Zc = cur_R[6]*Pw4[i].x + cur_R[7]*Pw4[i].y + cur_R[8]*Pw4[i].z + cur_t[2];
        if (Zc <= 0.0f) return -1;
    }

    for (i = 0; i < 9; ++i) Rcw[i] = cur_R[i];
    tcw->x = cur_t[0]; tcw->y = cur_t[1]; tcw->z = cur_t[2];
    return 0;
}

/* -------------------------------------------------------------------------
 * levio_epnp_ransac
 * --------------------------------------------------------------------- */

int levio_epnp_ransac(const vec3f_t *Pw, const vec2f_t *px, int n,
                      const levio_K_t *K, levio_pnp_result_t *out)
{
    int i, iter;
    float thr2 = LEVIO_RANSAC_REPROJ_THR * LEVIO_RANSAC_REPROJ_THR;

    out->n_inliers = 0;
    out->valid     = 0;
    memset(out->Rcw, 0, sizeof(out->Rcw));
    out->Rcw[0] = out->Rcw[4] = out->Rcw[8] = 1.0f;
    out->tcw.x = out->tcw.y = out->tcw.z = 0.0f;

    if (n < 4)
        return -1;

    int best_inliers = 0;
    float best_R[9], best_t[3];
    memset(best_R, 0, 9*sizeof(float));
    best_R[0] = best_R[4] = best_R[8] = 1.0f;
    memset(best_t, 0, 3*sizeof(float));

    uint32_t rng = 98765u;

    for (iter = 0; iter < LEVIO_RANSAC_MAX_ITER; ++iter) {
        /* Sample 4 distinct indices */
        int idx[4];
        int cnt = 0, tries = 0;
        while (cnt < 4 && tries < 500) {
            int r = (int)(levio_rand_next(&rng) % (uint32_t)n);
            int dup = 0, di;
            for (di = 0; di < cnt; ++di)
                if (idx[di] == r) { dup = 1; break; }
            if (!dup) idx[cnt++] = r;
            ++tries;
        }
        if (cnt < 4) continue;

        vec3f_t Pw4[4];
        vec2f_t px4[4];
        for (i = 0; i < 4; ++i) {
            Pw4[i] = Pw[idx[i]];
            px4[i] = px[idx[i]];
        }

        float Rc[9];
        vec3f_t tc;
        if (levio_epnp_minimal(Pw4, px4, K, Rc, &tc) != 0)
            continue;

        /* Count inliers */
        int n_in = 0;
        for (i = 0; i < n; ++i) {
            if (reproj_err2(Rc, &tc, K, &Pw[i], &px[i]) < thr2)
                ++n_in;
        }

        if (n_in > best_inliers) {
            best_inliers = n_in;
            memcpy(best_R, Rc, 9*sizeof(float));
            best_t[0] = tc.x; best_t[1] = tc.y; best_t[2] = tc.z;
        }
    }

    if (best_inliers < 4)
        return -1;

    /* Re-run on best inliers for refinement */
    {
        static vec3f_t inl_Pw[LEVIO_MAX_WORLD_POINTS];
        static vec2f_t inl_px[LEVIO_MAX_WORLD_POINTS];
        int n_in = 0;
        vec3f_t bt = {best_t[0], best_t[1], best_t[2]};
        for (i = 0; i < n; ++i) {
            if (reproj_err2(best_R, &bt, K, &Pw[i], &px[i]) < thr2) {
                if (n_in < LEVIO_MAX_WORLD_POINTS) {
                    inl_Pw[n_in] = Pw[i];
                    inl_px[n_in] = px[i];
                    ++n_in;
                }
            }
        }
        if (n_in >= 4) {
            /* Refine on all inliers using 4 representative ones */
            vec3f_t Pw4r[4]; vec2f_t px4r[4];
            for (i = 0; i < 4; ++i) {
                int si = i * (n_in / 4);
                Pw4r[i] = inl_Pw[si]; px4r[i] = inl_px[si];
            }
            float Rc2[9]; vec3f_t tc2;
            if (levio_epnp_minimal(Pw4r, px4r, K, Rc2, &tc2) == 0) {
                int n_in2 = 0;
                for (i = 0; i < n; ++i)
                    if (reproj_err2(Rc2, &tc2, K, &Pw[i], &px[i]) < thr2)
                        ++n_in2;
                if (n_in2 >= best_inliers) {
                    memcpy(best_R, Rc2, 9*sizeof(float));
                    best_t[0] = tc2.x; best_t[1] = tc2.y; best_t[2] = tc2.z;
                    best_inliers = n_in2;
                }
            }
        }
    }

    memcpy(out->Rcw, best_R, 9*sizeof(float));
    out->tcw.x = best_t[0];
    out->tcw.y = best_t[1];
    out->tcw.z = best_t[2];
    out->n_inliers = best_inliers;
    out->valid = 1;
    return 0;
}
