/**
 * @file levio.h
 * @brief Top-level LEVIO VIO API.
 *
 * Provides the two entry points required to drive LEVIO:
 *   levio_push_imu()   – feed high-rate IMU data
 *   levio_push_image() – feed a camera frame (triggers front-end + back-end)
 *
 * Typical usage:
 * @code
 *   levio_t vio;
 *   levio_K_t K = {458.654f, 457.296f, 367.215f, 248.375f}; // EuRoC cam0
 *   levio_init(&vio, &K);
 *
 *   // IMU loop (200 Hz):
 *   levio_push_imu(&vio, t, ax, ay, az, gx, gy, gz);
 *
 *   // Camera loop (20 Hz):
 *   levio_pose_t p = levio_push_image(&vio, t, gray, W, H, stride);
 *   if (p.valid) { ... use pose ... }
 * @endcode
 */
#ifndef LEVIO_H
#define LEVIO_H

#include <stdint.h>
#include "levio_cfg.h"
#include "levio_types.h"
#include "levio_frontend.h"
#include "levio_backend.h"
#include "levio_ringbuf.h"
#include "levio_imu_preint.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Top-level VIO context (statically allocated). */
typedef struct {
    levio_frontend_t  fe;           /**< visual front-end state */
    levio_backend_t   be;           /**< optimisation back-end state */

    levio_ringbuf_t   imu_buf;      /**< ring buffer for incoming IMU data */
    levio_preint_t    cur_preint;   /**< pre-integration accumulator */

    double            last_img_t;   /**< timestamp of previous image */
    uint8_t           initialized;  /**< 1 after first keyframe is established */
} levio_t;

/**
 * @brief Initialise the VIO system.
 *
 * @param vio  System context (caller-allocated).
 * @param K    Camera intrinsics.
 * @param g_w  Gravity vector in world frame (e.g., {0, 0, -9.81}).
 */
void levio_init(levio_t *vio, const levio_K_t *K, vec3f_t g_w);

/**
 * @brief Push one IMU measurement into the system.
 *
 * Should be called at the IMU rate (e.g., 200 Hz for EuRoC).
 * Data is stored in the ring buffer and consumed during the next
 * levio_push_image() call.
 *
 * @param vio  System context.
 * @param t    Timestamp [s].
 * @param ax   Accelerometer X [m/s²].
 * @param ay   Accelerometer Y [m/s²].
 * @param az   Accelerometer Z [m/s²].
 * @param gx   Gyroscope X [rad/s].
 * @param gy   Gyroscope Y [rad/s].
 * @param gz   Gyroscope Z [rad/s].
 */
void levio_push_imu(levio_t *vio, double t,
                    float ax, float ay, float az,
                    float gx, float gy, float gz);

/**
 * @brief Process one camera frame.
 *
 * Drains the IMU ring buffer (integrating into cur_preint), runs the
 * visual front-end, and triggers the back-end optimizer on new keyframes.
 *
 * @param vio     System context.
 * @param t       Image timestamp [s].
 * @param gray    Greyscale image buffer (w × h, row-major, 1 byte/pixel).
 * @param w       Image width  (should equal LEVIO_IMG_W).
 * @param h       Image height (should equal LEVIO_IMG_H).
 * @param stride  Row stride in bytes.
 * @return        Current pose estimate; pose.valid == 0 if not yet initialised.
 */
levio_pose_t levio_push_image(levio_t *vio, double t,
                               const uint8_t *gray, int w, int h, int stride);

/**
 * @brief Retrieve the latest pose estimate without processing new data.
 *
 * @param vio  System context.
 * @return     Most recent pose estimate.
 */
levio_pose_t levio_get_pose(const levio_t *vio);

#ifdef __cplusplus
}
#endif

#endif /* LEVIO_H */
