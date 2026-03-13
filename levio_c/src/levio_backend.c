/**
 * @file levio_backend.c
 * @brief Sliding-window back-end: tight-coupled BA + IMU optimizer.
 *
 * Implements a Levenberg–Marquardt (LM) optimizer with Schur complement
 * landmark elimination.  State dimension per keyframe = 15 (6 pose + 3 vel
 * + 3 gyro bias + 3 accel bias).  Landmark positions (3 per landmark) are
 * eliminated via Schur complement before solving the reduced system.
 */
#include "levio_backend.h"
#include "levio_factors.h"
#include "levio_math.h"
#include "levio_se3.h"
#include <string.h>

/* Internal per-landmark Hessian block (3×3) and gradient (3).
 * We keep one such block per active landmark, and also accumulate
 * the H_pl cross-blocks for the Schur complement.
 */

void levio_backend_init(levio_backend_t *be, const levio_K_t *K,
                         vec3f_t g_w, const levio_imu_noise_t *imu_noise)
{
    memset(be, 0, sizeof(*be));
    be->K         = *K;
    be->g_w       = g_w;
    be->imu_noise = *imu_noise;
    be->n_kf      = 0;
    be->win_head  = 0;
    be->n_lms     = 0;
    be->n_obs     = 0;
    be->lm_lambda = LEVIO_LM_LAMBDA_INIT;
    be->prior.dim = 0;
}

int levio_backend_add_keyframe(levio_backend_t *be,
                                const levio_keyframe_t *kf,
                                const levio_preint_t *preint,
                                const levio_landmark_t *lms, int n_lms,
                                const levio_obs_t *obs, int n_obs)
{
    /* If window is full, marginalise the oldest keyframe first */
    if (be->n_kf >= LEVIO_WINDOW_SIZE) {
        if (levio_backend_marginalise(be) != 0)
            return -1;
    }

    /* Insert keyframe into circular buffer */
    int slot = (be->win_head + be->n_kf) % LEVIO_WINDOW_SIZE;
    be->win[slot] = *kf;
    be->win[slot].win_idx = (int8_t)slot;

    if (preint && be->n_kf > 0) {
        be->preint[slot] = *preint;
    }

    ++be->n_kf;

    /* Add landmarks */
    int i;
    for (i = 0; i < n_lms && be->n_lms < LEVIO_MAX_WORLD_POINTS; ++i)
        be->lms[be->n_lms++] = lms[i];

    /* Add observations */
    for (i = 0; i < n_obs && be->n_obs < LEVIO_MAX_OBS; ++i)
        be->obs[be->n_obs++] = obs[i];

    return 0;
}

/* --------------------------------------------------------------------------
 * Backend LM optimizer
 * -------------------------------------------------------------------------- */

/*
 * Helper: extract SE3 [R|t] for a keyframe slot into a flat 6-vector
 * (so3 axis-angle + translation).  Used only conceptually; actual state
 * update uses levio_se3_boxplus.
 */

int levio_backend_optimize(levio_backend_t *be, int n_iter)
{
    if (be->n_kf < 2)
        return 0; /* not enough keyframes to optimise */

    int iter, i, j, k;
    int nkf = be->n_kf;
    int dim  = nkf * LEVIO_KF_DIM; /* total state dimension */

    if (dim > LEVIO_RSYS_DIM)
        return -1;

    /* -----------------------------------------------------------------
     * Allocate normal equation blocks on the stack.
     * H_pp: dim × dim  (pose/vel/bias Hessian)
     * b_pp: dim        (gradient)
     * ----------------------------------------------------------------- */

    /* We use the pre-allocated be->H_pp and be->b_pp arrays */
    float *H_pp = be->H_pp;
    float *b_pp = be->b_pp;

    for (iter = 0; iter < n_iter; ++iter) {
        memset(H_pp, 0, sizeof(float) * dim * dim);
        memset(b_pp, 0, sizeof(float) * dim);

        /* -----------------------------------------------------------
         * Accumulate visual reprojection factors
         * For each observation: add J^T * J to the pose block and the
         * landmark block, plus the cross terms.  Then Schur-eliminate
         * the landmark block.
         * --------------------------------------------------------- */

        /* Per-landmark accumulators (stack, small fixed size) */
        static float H_ll[LEVIO_MAX_WORLD_POINTS][9];  /* 3×3 per lm */
        static float b_ll[LEVIO_MAX_WORLD_POINTS][3];  /* 3 per lm */
        /* H_pl: for each observation, a 15×3 cross block.
         * We store contributions indexed by (kf_slot, lm_id). */
        static float H_pl[LEVIO_MAX_OBS][LEVIO_KF_DIM * 3]; /* 15×3 per obs */
        /* Pair (obs_idx → kf_slot, lm_id) is in be->obs[] */

        memset(H_ll, 0, sizeof(H_ll));
        memset(b_ll, 0, sizeof(b_ll));

        uint32_t obs_idx;
        for (obs_idx = 0; obs_idx < be->n_obs; ++obs_idx) {
            const levio_obs_t *ob = &be->obs[obs_idx];
            int lm_id = ob->lm_id;
            int kf_slot = ob->kf_idx;

            if (lm_id < 0 || lm_id >= (int)be->n_lms) continue;
            if (!be->lms[lm_id].valid) continue;

            /* Find pose index in window */
            int pose_idx = -1;
            for (i = 0; i < nkf; ++i) {
                int slot = (be->win_head + i) % LEVIO_WINDOW_SIZE;
                if (slot == kf_slot) { pose_idx = i; break; }
            }
            if (pose_idx < 0) continue;

            const levio_keyframe_t *kf = &be->win[kf_slot];
            float res[2], J_pose[12], J_point[6];

            if (levio_factor_visual(&be->K,
                                     kf->Rwb.m, kf->pwb,
                                     be->lms[lm_id].Pw, ob->obs,
                                     res, J_pose, J_point) != 0)
                continue;

            /* Accumulate H_pp block for this keyframe (15×15 sub-block) */
            /* J_pose is 2×6 (rot + trans), state is 15 but only first 6 */
            int base = pose_idx * LEVIO_KF_DIM;
            for (i = 0; i < 6; ++i)
                for (j = 0; j < 6; ++j) {
                    float v = J_pose[0*6+i]*J_pose[0*6+j]
                            + J_pose[1*6+i]*J_pose[1*6+j];
                    H_pp[(base+i)*dim + (base+j)] += v;
                }

            /* Accumulate b_pp block */
            for (i = 0; i < 6; ++i) {
                b_pp[base+i] -= J_pose[0*6+i]*res[0] + J_pose[1*6+i]*res[1];
            }

            /* Accumulate H_ll block (3×3) */
            for (i = 0; i < 3; ++i)
                for (j = 0; j < 3; ++j)
                    H_ll[lm_id][i*3+j] += J_point[0*3+i]*J_point[0*3+j]
                                         + J_point[1*3+i]*J_point[1*3+j];

            /* Accumulate b_ll */
            for (i = 0; i < 3; ++i)
                b_ll[lm_id][i] -= J_point[0*3+i]*res[0]
                                 + J_point[1*3+i]*res[1];

            /* Accumulate H_pl cross block (6×3, at pose_idx offset) */
            if (obs_idx < LEVIO_MAX_OBS) {
                memset(H_pl[obs_idx], 0, sizeof(H_pl[obs_idx]));
                for (i = 0; i < 6; ++i)
                    for (j = 0; j < 3; ++j)
                        H_pl[obs_idx][i*3+j] =
                            J_pose[0*6+i]*J_point[0*3+j]
                          + J_pose[1*6+i]*J_point[1*3+j];
            }
        }

        /* -----------------------------------------------------------
         * Accumulate IMU factors between consecutive keyframes
         * --------------------------------------------------------- */
        for (i = 1; i < nkf; ++i) {
            int slot_i = (be->win_head + i - 1) % LEVIO_WINDOW_SIZE;
            int slot_j = (be->win_head + i)     % LEVIO_WINDOW_SIZE;

            float res_imu[LEVIO_PREINT_DIM];
            levio_factor_imu(&be->preint[slot_j],
                              &be->win[slot_i], &be->win[slot_j],
                              be->g_w, res_imu, NULL, NULL);

            /* Add residual squared as a simple scalar cost contribution to
             * the gradient (simplified: only diagonal damping effect).
             * A full implementation would compute 9×15 Jacobians. */
            int base_i = (i-1) * LEVIO_KF_DIM;
            int base_j = i     * LEVIO_KF_DIM;
            /* Add a small information contribution along velocity/bias dims
             * (simplified – avoids under-constrained system) */
            float imu_info = 0.1f;
            for (k = 6; k < LEVIO_KF_DIM; ++k) {
                H_pp[(base_i+k)*dim+(base_i+k)] += imu_info;
                H_pp[(base_j+k)*dim+(base_j+k)] += imu_info;
                /* Gradient from IMU residual (v,bg,ba components) */
                b_pp[base_i+k] -= imu_info * res_imu[k-6 + 3];
                b_pp[base_j+k] -= imu_info * res_imu[k-6 + 3];
            }
            (void)base_i; (void)base_j;
        }

        /* -----------------------------------------------------------
         * Schur complement: eliminate landmark blocks
         * H_pp* = H_pp - sum_lm H_pl_lm * H_ll_lm^{-1} * H_lp_lm
         * b_pp* = b_pp - sum_lm H_pl_lm * H_ll_lm^{-1} * b_ll_lm
         * --------------------------------------------------------- */
        for (i = 0; i < (int)be->n_lms; ++i) {
            if (!be->lms[i].valid) continue;

            /* Invert 3×3 H_ll block */
            mat3f_t Hll_m;
            memcpy(Hll_m.m, H_ll[i], 9*sizeof(float));

            /* Add small damping to ensure invertibility */
            Hll_m.m[0] += 1e-6f;
            Hll_m.m[4] += 1e-6f;
            Hll_m.m[8] += 1e-6f;

            mat3f_t Hll_inv = levio_mat3_inv(Hll_m);

            /* Find observations for this landmark */
            for (obs_idx = 0; obs_idx < be->n_obs; ++obs_idx) {
                const levio_obs_t *ob_p = &be->obs[obs_idx];
                if (ob_p->lm_id != i) continue;
                if (obs_idx >= LEVIO_MAX_OBS) continue;

                int kf_slot_p = ob_p->kf_idx;
                int pose_p = -1;
                for (k = 0; k < nkf; ++k) {
                    int slot = (be->win_head + k) % LEVIO_WINDOW_SIZE;
                    if (slot == kf_slot_p) { pose_p = k; break; }
                }
                if (pose_p < 0) continue;

                /* H_pl_p * H_ll^{-1}: (6×3) * (3×3) → (6×3) */
                float Jt[18]; /* 6×3 */
                memcpy(Jt, H_pl[obs_idx], 18*sizeof(float));

                float JtHllInv[18]; /* 6×3 */
                levio_matmul(Jt, Hll_inv.m, JtHllInv, 6, 3, 3);

                /* Update H_pp and b_pp for all pairs of observations */
                uint32_t obs_q;
                for (obs_q = obs_idx; obs_q < be->n_obs; ++obs_q) {
                    const levio_obs_t *ob_q = &be->obs[obs_q];
                    if (ob_q->lm_id != i) continue;
                    if (obs_q >= LEVIO_MAX_OBS) continue;

                    int kf_slot_q = ob_q->kf_idx;
                    int pose_q = -1;
                    for (k = 0; k < nkf; ++k) {
                        int slot = (be->win_head + k) % LEVIO_WINDOW_SIZE;
                        if (slot == kf_slot_q) { pose_q = k; break; }
                    }
                    if (pose_q < 0) continue;

                    float Jq[18];
                    memcpy(Jq, H_pl[obs_q], 18*sizeof(float));

                    /* Schur block: JtHllInv (6×3) * Jq^T (3×6) → 6×6 */
                    int bp = pose_p * LEVIO_KF_DIM;
                    int bq = pose_q * LEVIO_KF_DIM;
                    {
                        int r2, c2, m2;
                        for (r2 = 0; r2 < 6; ++r2)
                            for (c2 = 0; c2 < 6; ++c2) {
                                float v = 0.0f;
                                for (m2 = 0; m2 < 3; ++m2)
                                    v += JtHllInv[r2*3+m2] * Jq[c2*3+m2];
                                H_pp[(bp+r2)*dim+(bq+c2)] -= v;
                                if (obs_q != obs_idx)
                                    H_pp[(bq+c2)*dim+(bp+r2)] -= v;
                            }
                    }
                }

                /* Update b_pp: -= JtHllInv (6×3) * b_ll (3) */
                int bp2 = pose_p * LEVIO_KF_DIM;
                {
                    int r2, m2;
                    for (r2 = 0; r2 < 6; ++r2) {
                        float v = 0.0f;
                        for (m2 = 0; m2 < 3; ++m2)
                            v += JtHllInv[r2*3+m2] * b_ll[i][m2];
                        b_pp[bp2+r2] += v;
                    }
                }
                (void)i;
            }
        }

        /* -----------------------------------------------------------
         * Add LM damping: H_pp* += lambda * I
         * --------------------------------------------------------- */
        for (i = 0; i < dim; ++i)
            H_pp[i*dim+i] += be->lm_lambda;

        /* Also add prior information if available */
        if (be->prior.dim > 0 && be->prior.dim <= dim) {
            int pd = be->prior.dim;
            for (i = 0; i < pd; ++i)
                for (j = 0; j < pd; ++j)
                    H_pp[i*dim+j] += be->prior.H[i*pd+j];
            for (i = 0; i < pd; ++i)
                b_pp[i] += be->prior.b[i];
        }

        /* -----------------------------------------------------------
         * Solve reduced system: H_pp * delta_x = b_pp  (Cholesky)
         * --------------------------------------------------------- */
        static float H_copy[LEVIO_RSYS_DIM * LEVIO_RSYS_DIM];
        static float delta_x[LEVIO_RSYS_DIM];

        memcpy(H_copy, H_pp, sizeof(float) * dim * dim);
        memcpy(delta_x, b_pp, sizeof(float) * dim);

        if (levio_solve_chol(H_copy, delta_x, dim) != 0) {
            /* Cholesky failed: increase damping and retry */
            be->lm_lambda *= LEVIO_LM_LAMBDA_FACTOR;
            continue;
        }

        /* -----------------------------------------------------------
         * Apply increments to pose/velocity/bias states
         * --------------------------------------------------------- */
        float cost_before = 0.0f, cost_after = 0.0f;
        /* Quick cost estimate (sum of squared reprojection errors) */
        for (obs_idx = 0; obs_idx < be->n_obs; ++obs_idx) {
            const levio_obs_t *ob = &be->obs[obs_idx];
            int lm_id = ob->lm_id;
            int kf_slot_c = ob->kf_idx;
            if (lm_id < 0 || lm_id >= (int)be->n_lms) continue;
            if (!be->lms[lm_id].valid) continue;
            const levio_keyframe_t *kf = &be->win[kf_slot_c];
            float res[2];
            if (levio_factor_visual(&be->K, kf->Rwb.m, kf->pwb,
                                     be->lms[lm_id].Pw, ob->obs,
                                     res, NULL, NULL) == 0)
                cost_before += res[0]*res[0] + res[1]*res[1];
        }

        /* Save original states */
        static levio_keyframe_t kf_backup[LEVIO_WINDOW_SIZE];
        memcpy(kf_backup, be->win, sizeof(be->win));

        /* Apply box-plus update */
        for (i = 0; i < nkf; ++i) {
            int slot = (be->win_head + i) % LEVIO_WINDOW_SIZE;
            int base = i * LEVIO_KF_DIM;
            const float *dx = delta_x + base;

            /* SE3 boxplus: [δφ(0:3), δt(3:6)] */
            levio_se3_t T;
            T.R = be->win[slot].Rwb;
            T.t = be->win[slot].pwb;
            T = levio_se3_boxplus(T, dx);
            be->win[slot].Rwb = T.R;
            be->win[slot].pwb = T.t;

            /* Velocity update */
            be->win[slot].vwb.x += dx[6];
            be->win[slot].vwb.y += dx[7];
            be->win[slot].vwb.z += dx[8];

            /* Bias update */
            be->win[slot].bg.x += dx[9];
            be->win[slot].bg.y += dx[10];
            be->win[slot].bg.z += dx[11];
            be->win[slot].ba.x += dx[12];
            be->win[slot].ba.y += dx[13];
            be->win[slot].ba.z += dx[14];
        }

        /* Back-substitute to update landmark positions:
         * δx_l = H_ll^{-1} * (b_ll - H_lp * δx_p)
         */
        for (i = 0; i < (int)be->n_lms; ++i) {
            if (!be->lms[i].valid) continue;

            mat3f_t Hll_m2;
            memcpy(Hll_m2.m, H_ll[i], 9*sizeof(float));
            Hll_m2.m[0] += 1e-6f; Hll_m2.m[4] += 1e-6f; Hll_m2.m[8] += 1e-6f;
            mat3f_t Hll_inv2 = levio_mat3_inv(Hll_m2);

            /* b_ll_adjusted = b_ll - sum_obs H_lp * delta_x_p */
            float b_adj[3];
            b_adj[0] = b_ll[i][0];
            b_adj[1] = b_ll[i][1];
            b_adj[2] = b_ll[i][2];

            for (obs_idx = 0; obs_idx < be->n_obs; ++obs_idx) {
                if (be->obs[obs_idx].lm_id != i) continue;
                if (obs_idx >= LEVIO_MAX_OBS) continue;
                int kf_slot_bs = be->obs[obs_idx].kf_idx;
                int pose_bs = -1;
                for (k = 0; k < nkf; ++k) {
                    if ((be->win_head + k) % LEVIO_WINDOW_SIZE == kf_slot_bs) {
                        pose_bs = k; break;
                    }
                }
                if (pose_bs < 0) continue;
                int bp3 = pose_bs * LEVIO_KF_DIM;
                /* H_lp (3×6) = H_pl^T (6×3)^T */
                for (k = 0; k < 3; ++k)
                    for (j = 0; j < 6; ++j)
                        b_adj[k] -= H_pl[obs_idx][j*3+k] * delta_x[bp3+j];
            }

            /* δl = H_ll^{-1} * b_adj */
            vec3f_t ba_v = {b_adj[0], b_adj[1], b_adj[2]};
            vec3f_t dl = levio_mat3_mul_vec3(Hll_inv2, ba_v);
            be->lms[i].Pw.x += dl.x;
            be->lms[i].Pw.y += dl.y;
            be->lms[i].Pw.z += dl.z;
        }

        /* Compute cost after update */
        for (obs_idx = 0; obs_idx < be->n_obs; ++obs_idx) {
            const levio_obs_t *ob = &be->obs[obs_idx];
            int lm_id = ob->lm_id;
            int kf_slot_ca = ob->kf_idx;
            if (lm_id < 0 || lm_id >= (int)be->n_lms) continue;
            if (!be->lms[lm_id].valid) continue;
            const levio_keyframe_t *kf = &be->win[kf_slot_ca];
            float res[2];
            if (levio_factor_visual(&be->K, kf->Rwb.m, kf->pwb,
                                     be->lms[lm_id].Pw, ob->obs,
                                     res, NULL, NULL) == 0)
                cost_after += res[0]*res[0] + res[1]*res[1];
        }

        /* LM damping update */
        if (cost_after < cost_before) {
            be->lm_lambda /= LEVIO_LM_LAMBDA_FACTOR;
        } else {
            /* Reject step: restore states */
            memcpy(be->win, kf_backup, sizeof(be->win));
            be->lm_lambda *= LEVIO_LM_LAMBDA_FACTOR;
        }
    }

    return 0;
}

int levio_backend_marginalise(levio_backend_t *be)
{
    if (be->n_kf == 0)
        return -1;

    /* For now, drop oldest frame without building a full marginalisation prior.
     * A proper implementation would compute the Schur complement of the oldest
     * pose block and fold it into be->prior. */
    be->win_head = (be->win_head + 1) % LEVIO_WINDOW_SIZE;
    --be->n_kf;
    return 0;
}

void levio_backend_get_pose(const levio_backend_t *be, levio_pose_t *pose)
{
    if (be->n_kf == 0) {
        pose->valid = 0;
        return;
    }
    /* Most recent keyframe is at win_head + n_kf - 1 (mod WINDOW_SIZE) */
    int slot = (be->win_head + be->n_kf - 1) % LEVIO_WINDOW_SIZE;
    pose->Rcw  = be->win[slot].Rwb;
    pose->tcw  = be->win[slot].pwb;
    pose->valid = 1;
}
