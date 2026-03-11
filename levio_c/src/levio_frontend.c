/**
 * @file levio_frontend.c
 * @brief Visual odometry front-end: orchestrates the §3.2 pipeline.
 */
#include "levio_frontend.h"
#include "levio_orb.h"
#include "levio_match.h"
#include "levio_ransac_8pt.h"
#include "levio_pnp_epnp.h"
#include "levio_triangulate.h"
#include "levio_math.h"
#include <string.h>
#include <math.h>

/* Temporary work buffers (avoid stack blowout on embedded targets) */
static levio_feat_t  s_cur_feats[LEVIO_MAX_FRAME_FEATS];
static levio_match_t s_matches_ff[LEVIO_MAX_FRAME_FEATS]; /* frame-to-frame */
static levio_match_t s_matches_fw[LEVIO_MAX_WORLD_POINTS]; /* frame-to-world  */
static levio_landmark_t s_new_lms[LEVIO_MAX_FRAME_FEATS];

void levio_frontend_init(levio_frontend_t *fe, const levio_K_t *K)
{
    memset(fe, 0, sizeof(*fe));
    fe->K          = *K;
    fe->n_prev_feats = 0;
    fe->has_kf     = 0;
    fe->n_lms      = 0;
    fe->frame_count = 0;
    fe->cur_pose.valid = 0;
}

float levio_mean_parallax(const levio_feat_t *feats_a,
                           const levio_feat_t *feats_b,
                           const levio_match_t *matches, int n_matches)
{
    float sum = 0.0f;
    int i;
    if (n_matches <= 0) return 0.0f;
    for (i = 0; i < n_matches; ++i) {
        float dx = feats_a[matches[i].idx_a].px.x - feats_b[matches[i].idx_b].px.x;
        float dy = feats_a[matches[i].idx_a].px.y - feats_b[matches[i].idx_b].px.y;
        sum += sqrtf(dx * dx + dy * dy);
    }
    return sum / (float)n_matches;
}

int levio_frontend_process(levio_frontend_t *fe,
                            double t,
                            const uint8_t *gray, int w, int h, int stride,
                            levio_frontend_result_t *out)
{
    memset(out, 0, sizeof(*out));

    /* -----------------------------------------------------------------------
     * Segment 1: ORB extraction (paper §3.2)
     * --------------------------------------------------------------------- */
    int n_cur = levio_orb_extract(gray, w, h, stride,
                                   s_cur_feats, LEVIO_MAX_FRAME_FEATS);
    if (n_cur <= 0)
        return -1;

    /* -----------------------------------------------------------------------
     * Segment 2: Bidirectional matching (paper §3.2)
     * Frame-to-frame match (against previous frame)
     * --------------------------------------------------------------------- */
    int n_ff = 0;
    if (fe->n_prev_feats > 0) {
        n_ff = levio_match_bidir(s_cur_feats, n_cur,
                                  fe->prev_feats, fe->n_prev_feats,
                                  s_matches_ff, LEVIO_MAX_FRAME_FEATS);
    }

    /* Frame-to-world match (against known landmarks) */
    int n_fw = 0;
    if (fe->n_lms > 0) {
        /* Build a temporary feature array from landmark descriptors */
        /* For simplicity we re-use levio_feat_t with landmark descriptors */
        /* TODO: a dedicated world-point descriptor array would be cleaner */
        static levio_feat_t s_lm_feats[LEVIO_MAX_WORLD_POINTS];
        int i;
        int n_valid_lms = 0;
        for (i = 0; i < fe->n_lms; ++i) {
            if (fe->lms[i].valid) {
                int byte_idx;
                for (byte_idx = 0; byte_idx < LEVIO_DESC_BYTES; ++byte_idx)
                    s_lm_feats[n_valid_lms].desc[byte_idx] = fe->lms[i].desc[byte_idx];
                s_lm_feats[n_valid_lms].lm_id = (int16_t)i;
                ++n_valid_lms;
            }
        }
        n_fw = levio_match_bidir(s_cur_feats, n_cur,
                                  s_lm_feats, n_valid_lms,
                                  s_matches_fw, LEVIO_MAX_WORLD_POINTS);
    }

    /* -----------------------------------------------------------------------
     * Segments 3 & 4: Pose estimation with PnP / 8-pt fallback (paper §3.2)
     * --------------------------------------------------------------------- */
    if (fe->frame_count == 0 || n_fw < LEVIO_PNP_MIN_INLIERS) {
        /* Segment 3: 8-point RANSAC fallback */
        if (n_ff >= 8 && fe->has_kf) {
            static vec2f_t s_px1[LEVIO_MAX_FRAME_FEATS];
            static vec2f_t s_px2[LEVIO_MAX_FRAME_FEATS];
            int i;
            for (i = 0; i < n_ff; ++i) {
                s_px1[i] = fe->prev_feats[s_matches_ff[i].idx_b].px;
                s_px2[i] = s_cur_feats [s_matches_ff[i].idx_a].px;
            }
            levio_8pt_result_t r8;
            levio_ransac_8pt(s_px1, s_px2, n_ff, &fe->K, &r8);
            /* TODO: apply relative pose to fe->cur_pose when r8.valid */
            out->used_8pt = 1;
        }
        /* First frame: mark current pose as valid identity */
        if (fe->frame_count == 0) {
            mat3f_t I = { {1,0,0, 0,1,0, 0,0,1} };
            fe->cur_pose.Rcw = I;
            fe->cur_pose.tcw.x = fe->cur_pose.tcw.y = fe->cur_pose.tcw.z = 0.0f;
            fe->cur_pose.valid = 1;
        }
    } else {
        /* Segment 4: EPnP RANSAC (primary path) */
        static vec3f_t s_Pw[LEVIO_MAX_WORLD_POINTS];
        static vec2f_t s_obs[LEVIO_MAX_WORLD_POINTS];
        int i;
        for (i = 0; i < n_fw; ++i) {
            int lm_idx = s_matches_fw[i].idx_b;
            s_Pw[i]  = fe->lms[lm_idx].Pw;
            s_obs[i] = s_cur_feats[s_matches_fw[i].idx_a].px;
        }
        levio_pnp_result_t rpnp;
        if (levio_epnp_ransac(s_Pw, s_obs, n_fw, &fe->K, &rpnp) == 0
            && rpnp.valid) {
            int elem;
            for (elem = 0; elem < 9; ++elem)
                fe->cur_pose.Rcw.m[elem] = rpnp.Rcw[elem];
            fe->cur_pose.tcw   = rpnp.tcw;
            fe->cur_pose.valid = 1;
        }
    }

    /* -----------------------------------------------------------------------
     * Segment 5: Keyframe selection (paper §3.2 – parallax threshold)
     * --------------------------------------------------------------------- */
    float parallax = levio_mean_parallax(s_cur_feats, fe->prev_feats,
                                          s_matches_ff, n_ff);
    int make_kf = (!fe->has_kf) ||
                  (parallax > LEVIO_KF_PARALLAX_PX) ||
                  (fe->frame_count == 0);

    if (make_kf) {
        /* -----------------------------------------------------------------------
         * Segment 6: Triangulate new world points (paper §3.2)
         * --------------------------------------------------------------------- */
        if (fe->has_kf && n_ff >= 8) {
            int n_new = levio_triangulate_batch(
                fe->last_kf.Rwb.m, fe->last_kf.pwb,
                fe->cur_pose.Rcw.m, fe->cur_pose.tcw,
                &fe->K,
                s_matches_ff, n_ff,
                fe->prev_feats, s_cur_feats,
                s_new_lms);

            /* Add triangulated points to the world map */
            int i;
            for (i = 0; i < n_new && fe->n_lms < LEVIO_MAX_WORLD_POINTS; ++i)
                fe->lms[fe->n_lms++] = s_new_lms[i];
        }

        /* Save current frame as new keyframe */
        fe->last_kf.t      = t;
        fe->last_kf.Rwb    = fe->cur_pose.Rcw;
        fe->last_kf.pwb    = fe->cur_pose.tcw;
        fe->last_kf.n_feats = (uint16_t)n_cur;
        int cp;
        for (cp = 0; cp < n_cur; ++cp)
            fe->last_kf.feats[cp] = s_cur_feats[cp];
        fe->has_kf = 1;
        out->is_keyframe = 1;
    }

    /* Update previous frame features for next iteration */
    fe->n_prev_feats = (uint16_t)n_cur;
    int cp;
    for (cp = 0; cp < n_cur; ++cp)
        fe->prev_feats[cp] = s_cur_feats[cp];

    out->pose = fe->cur_pose;
    ++fe->frame_count;
    return 0;
}
