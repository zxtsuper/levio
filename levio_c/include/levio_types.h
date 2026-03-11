/**
 * @file levio_types.h
 * @brief Core data types for LEVIO embedded VIO.
 *
 * All structures are designed for static allocation (no heap) to match
 * the GAP9 embedded target (paper §3.4).
 */
#ifndef LEVIO_TYPES_H
#define LEVIO_TYPES_H

#include <stdint.h>
#include <stddef.h>
#include "levio_cfg.h"

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------
 * Basic vector / matrix types (column-major float storage)
 * -------------------------------------------------------------------------- */
typedef struct { float x, y; }       vec2f_t;
typedef struct { float x, y, z; }    vec3f_t;
typedef struct { float x, y, z, w; } vec4f_t;

/** 3×3 matrix stored row-major: m[row*3 + col] */
typedef struct { float m[9]; }  mat3f_t;

/** 4×4 matrix stored row-major: m[row*4 + col] */
typedef struct { float m[16]; } mat4f_t;

/* --------------------------------------------------------------------------
 * Camera intrinsics
 * -------------------------------------------------------------------------- */
typedef struct {
    float fx, fy;   /**< focal lengths [px] */
    float cx, cy;   /**< principal point [px] */
} levio_K_t;

/* --------------------------------------------------------------------------
 * IMU sample
 * -------------------------------------------------------------------------- */
typedef struct {
    double  t;           /**< timestamp [s] */
    vec3f_t acc;         /**< accelerometer [m/s²] */
    vec3f_t gyro;        /**< gyroscope [rad/s] */
} levio_imu_sample_t;

/* --------------------------------------------------------------------------
 * ORB feature (2-D pixel + descriptor, optional landmark association)
 * -------------------------------------------------------------------------- */
typedef struct {
    vec2f_t  px;                    /**< pixel coordinates */
    uint8_t  desc[LEVIO_DESC_BYTES]; /**< 256-bit ORB descriptor */
    int16_t  lm_id;                 /**< world-point id, -1 if unmatched */
    float    angle;                 /**< orientation [rad] */
    uint8_t  octave;                /**< detection octave */
} levio_feat_t;

/* --------------------------------------------------------------------------
 * World landmark (3-D point + representative descriptor)
 * Paper §3.2: world stores up to LEVIO_MAX_WORLD_POINTS descriptors
 * -------------------------------------------------------------------------- */
typedef struct {
    vec3f_t  Pw;                     /**< 3-D world position */
    uint8_t  desc[LEVIO_DESC_BYTES]; /**< representative descriptor */
    uint16_t obs_count;              /**< number of observations */
    uint8_t  valid;                  /**< 1 = active, 0 = culled */
} levio_landmark_t;

/* --------------------------------------------------------------------------
 * Keyframe state (pose + velocity + IMU bias)
 * Paper §3.3: each node stores {R, p, v, bg, ba}
 * -------------------------------------------------------------------------- */
typedef struct {
    double   t;            /**< timestamp [s] */

    /* Pose: body-in-world (R_wb, p_wb) */
    mat3f_t  Rwb;          /**< rotation matrix, world←body */
    vec3f_t  pwb;          /**< translation, world←body */

    /* Velocity and IMU bias (§3.3 tight-coupled states) */
    vec3f_t  vwb;          /**< velocity in world frame */
    vec3f_t  bg;           /**< gyroscope bias */
    vec3f_t  ba;           /**< accelerometer bias */

    /* Front-end observations */
    uint16_t   n_feats;
    levio_feat_t feats[LEVIO_MAX_FRAME_FEATS];

    /* Keyframe index in sliding window (-1 = invalid) */
    int8_t   win_idx;
} levio_keyframe_t;

/* --------------------------------------------------------------------------
 * Match pair (for brute-force matcher output)
 * -------------------------------------------------------------------------- */
typedef struct {
    int16_t  idx_a;   /**< index in set A */
    int16_t  idx_b;   /**< index in set B */
    uint16_t dist;    /**< Hamming distance */
} levio_match_t;

/* --------------------------------------------------------------------------
 * Estimated pose output from frontend
 * -------------------------------------------------------------------------- */
typedef struct {
    mat3f_t  Rcw;   /**< rotation world→camera */
    vec3f_t  tcw;   /**< translation world→camera */
    int      valid; /**< 1 if pose was successfully estimated */
} levio_pose_t;

#ifdef __cplusplus
}
#endif

#endif /* LEVIO_TYPES_H */
