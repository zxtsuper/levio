/**
 * @file levio_backend.c
 * @brief Sliding-window back-end: tight-coupled BA + IMU optimizer stub.
 *
 * TODO: Implement full Hessian construction, Schur complement elimination
 *       (equations 5–6 of the paper), and LM iteration.
 */
#include "levio_backend.h"
#include "levio_factors.h"
#include "levio_math.h"
#include <string.h>

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

int levio_backend_optimize(levio_backend_t *be, int n_iter)
{
    /* TODO: Full LM optimisation implementing equations (5)–(6):
     *
     *  For each iteration k:
     *  1) Build block Hessian:
     *       [H_pp  H_pl] [δx_p]   [b_p]
     *       [H_lp  H_ll] [δx_l] = [b_l]
     *     from all visual factors (levio_factor_visual) and
     *     IMU factors (levio_factor_imu) + prior.
     *
     *  2) Schur complement (eq. 5 → eq. 6):
     *       H_pp* = H_pp - H_pl * H_ll^{-1} * H_lp   ... (5a)
     *       b_p*  = b_p  - H_pl * H_ll^{-1} * b_l     ... (5b)
     *     Add LM damping: H_pp* += λ I
     *     Solve: δx_p = (H_pp*)^{-1} b_p*             ... (6)
     *
     *  3) Back-substitute for landmark update:
     *       δx_l = H_ll^{-1} (b_l - H_lp δx_p)
     *
     *  4) Apply increments to all states.
     *  5) Update LM damping λ based on cost ratio.
     */
    int iter;
    for (iter = 0; iter < n_iter; ++iter) {
        /* TODO: single LM step */
        (void)be;
    }
    return 0;
}

int levio_backend_marginalise(levio_backend_t *be)
{
    /* TODO: Marginalise the oldest keyframe.
     *  1) Identify all factors connected to win[win_head].
     *  2) Linearise those factors at the current estimate.
     *  3) Compute the Schur complement of the oldest pose/vel/bias block.
     *  4) Fold result into be->prior (Λ += Schur complement).
     *  5) Remove oldest keyframe from the window.
     */
    if (be->n_kf == 0)
        return -1;

    /* For now just drop the oldest frame without building a prior.
     * TODO: replace with proper marginalisation. */
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
