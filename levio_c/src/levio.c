/**
 * @file levio.c
 * @brief Top-level LEVIO VIO implementation.
 *
 * Wires the front-end and back-end together:
 *   - IMU data → ring buffer → pre-integration on keyframe trigger
 *   - Image   → front-end (6 segments of §3.2)
 *   - On keyframe: pre-integration result + visual observations → back-end
 *   - Back-end: LM optimisation of sliding window (§3.3)
 */
#include "levio.h"
#include "levio_math.h"
#include <string.h>

void levio_init(levio_t *vio, const levio_K_t *K, vec3f_t g_w)
{
    memset(vio, 0, sizeof(*vio));

    /* Default EuRoC-like IMU noise parameters (placeholder – replace with
     * calibration values from the IMU data sheet). */
    levio_imu_noise_t noise;
    noise.sigma_gyro_c   = 1.6968e-4f;
    noise.sigma_accel_c  = 2.0000e-3f;
    noise.sigma_gyro_rw  = 1.9393e-5f;
    noise.sigma_accel_rw = 3.0000e-3f;

    levio_frontend_init(&vio->fe, K);
    levio_backend_init(&vio->be, K, g_w, &noise);
    levio_ringbuf_init(&vio->imu_buf);

    vec3f_t bg0 = {0.0f, 0.0f, 0.0f};
    vec3f_t ba0 = {0.0f, 0.0f, 0.0f};
    levio_preint_reset(&vio->cur_preint, bg0, ba0);

    vio->last_img_t   = -1.0;
    vio->initialized  = 0;
}

void levio_push_imu(levio_t *vio, double t,
                    float ax, float ay, float az,
                    float gx, float gy, float gz)
{
    levio_imu_sample_t s;
    s.t       = t;
    s.acc.x   = ax; s.acc.y   = ay; s.acc.z   = az;
    s.gyro.x  = gx; s.gyro.y  = gy; s.gyro.z  = gz;
    /* Discard if buffer full (oldest data lost – acceptable for real-time) */
    levio_ringbuf_push(&vio->imu_buf, &s);
}

levio_pose_t levio_push_image(levio_t *vio, double t,
                               const uint8_t *gray, int w, int h, int stride)
{
    levio_pose_t invalid_pose;
    memset(&invalid_pose, 0, sizeof(invalid_pose));

    /* -----------------------------------------------------------------------
     * Drain IMU buffer: integrate all samples with timestamp <= t
     * --------------------------------------------------------------------- */
    levio_imu_sample_t s;
    while (levio_ringbuf_peek(&vio->imu_buf, &s) == 0 && s.t <= t) {
        levio_ringbuf_pop(&vio->imu_buf, &s);

        if (vio->last_img_t > 0.0) {
            float dt = (float)(s.t - vio->last_img_t);
            if (dt > 0.0f && dt < 1.0f) {
                levio_preint_push(&vio->cur_preint, dt,
                                   s.acc, s.gyro, NULL);
            }
        }
    }

    /* -----------------------------------------------------------------------
     * Visual front-end (§3.2)
     * --------------------------------------------------------------------- */
    levio_frontend_result_t fe_out;
    if (levio_frontend_process(&vio->fe, t, gray, w, h, stride, &fe_out) != 0)
        return invalid_pose;

    /* -----------------------------------------------------------------------
     * On keyframe: feed pre-integration + observations to back-end (§3.3)
     * --------------------------------------------------------------------- */
    if (fe_out.is_keyframe && vio->fe.has_kf) {
        const levio_keyframe_t *kf = &vio->fe.last_kf;

        /* Currently no observation array from frontend – TODO: wire up */
        levio_backend_add_keyframe(&vio->be, kf,
                                    &vio->cur_preint,
                                    NULL, 0,
                                    NULL, 0);

        /* Trigger LM optimisation */
        levio_backend_optimize(&vio->be, LEVIO_LM_MAX_ITER);

        /* Reset pre-integration for next window */
        vec3f_t bg = kf->bg;
        vec3f_t ba = kf->ba;
        levio_preint_reset(&vio->cur_preint, bg, ba);

        vio->initialized = 1;
    }

    vio->last_img_t = t;

    levio_pose_t out;
    levio_backend_get_pose(&vio->be, &out);
    if (!out.valid)
        out = fe_out.pose;  /* fall back to front-end estimate */
    return out;
}

levio_pose_t levio_get_pose(const levio_t *vio)
{
    levio_pose_t p;
    levio_backend_get_pose(&vio->be, &p);
    if (!p.valid)
        p = vio->fe.cur_pose;
    return p;
}
