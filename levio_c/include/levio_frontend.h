/**
 * @file levio_frontend.h
 * @brief Visual odometry front-end: orchestrates the six segments of §3.2.
 *
 * Pipeline (paper §3.2):
 *   Segment 1: ORB extraction           (levio_orb.h)
 *   Segment 2: Bidirectional matching   (levio_match.h)
 *   Segment 3: 8-point RANSAC           (levio_ransac_8pt.h)  ← fallback
 *   Segment 4: EPnP RANSAC              (levio_pnp_epnp.h)   ← primary
 *   Segment 5: Keyframe selection       (parallax threshold §3.2)
 *   Segment 6: Triangulation & tracking (levio_triangulate.h)
 *
 * Fallback rule: if 2D-3D matches < LEVIO_PNP_MIN_INLIERS, skip segment 4
 * and use segment 3 (8-pt) to estimate a relative frame-to-frame pose.
 */
#ifndef LEVIO_FRONTEND_H
#define LEVIO_FRONTEND_H

#include "levio_types.h"
#include "levio_camera.h"
#include "levio_cfg.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Persistent state of the front-end across frames. */
typedef struct {
    levio_K_t    K;               /**< camera intrinsics */

    /* Previous frame state (for frame-to-frame matching) */
    levio_feat_t prev_feats[LEVIO_MAX_FRAME_FEATS];
    uint16_t     n_prev_feats;

    /* Last keyframe (for keyframe selection and triangulation) */
    levio_keyframe_t last_kf;
    uint8_t          has_kf;      /**< 1 if a keyframe exists */

    /* World landmarks */
    levio_landmark_t lms[LEVIO_MAX_WORLD_POINTS];
    uint16_t         n_lms;

    /* Current estimated pose */
    levio_pose_t cur_pose;

    uint32_t frame_count;
} levio_frontend_t;

/** Front-end output produced for each image frame. */
typedef struct {
    levio_pose_t pose;          /**< estimated camera pose */
    uint8_t      is_keyframe;   /**< 1 if this frame was selected as keyframe */
    uint8_t      used_8pt;      /**< 1 if 8-pt fallback was triggered */
} levio_frontend_result_t;

/**
 * @brief Initialise the front-end state.
 *
 * @param fe   Front-end state (caller-allocated).
 * @param K    Camera intrinsics.
 */
void levio_frontend_init(levio_frontend_t *fe, const levio_K_t *K);

/**
 * @brief Process one image frame through the full VO pipeline (§3.2).
 *
 * @param fe     Front-end state (updated in place).
 * @param t      Frame timestamp [s].
 * @param gray   Greyscale image buffer (LEVIO_IMG_W × LEVIO_IMG_H, row-major).
 * @param w      Image width.
 * @param h      Image height.
 * @param stride Row stride in bytes.
 * @param out    Output pose estimate and keyframe flag.
 * @return       0 on success, negative on failure.
 */
int levio_frontend_process(levio_frontend_t *fe,
                            double t,
                            const uint8_t *gray, int w, int h, int stride,
                            levio_frontend_result_t *out);

/**
 * @brief Compute mean parallax between two feature sets using matched pairs.
 *
 * Used for keyframe selection in segment 5 (§3.2).
 *
 * @param feats_a   Features from frame A.
 * @param feats_b   Features from frame B.
 * @param matches   Matched pairs.
 * @param n_matches Number of matches.
 * @return          Mean parallax in pixels.
 */
float levio_mean_parallax(const levio_feat_t *feats_a,
                           const levio_feat_t *feats_b,
                           const levio_match_t *matches, int n_matches);

#ifdef __cplusplus
}
#endif

#endif /* LEVIO_FRONTEND_H */
