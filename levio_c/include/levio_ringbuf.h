/**
 * @file levio_ringbuf.h
 * @brief Lock-free ring buffer for IMU samples.
 *
 * Stores LEVIO_IMU_BUF_SIZE IMU samples in a statically allocated circular
 * buffer.  Thread safety: designed for single-producer / single-consumer use
 * (push from ISR / sensor task, pop from VIO pipeline).
 */
#ifndef LEVIO_RINGBUF_H
#define LEVIO_RINGBUF_H

#include <stdint.h>
#include "levio_cfg.h"
#include "levio_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    levio_imu_sample_t buf[LEVIO_IMU_BUF_SIZE];
    volatile uint32_t  head; /**< next write index */
    volatile uint32_t  tail; /**< next read index  */
} levio_ringbuf_t;

/** Initialise (zero) the ring buffer. */
void levio_ringbuf_init(levio_ringbuf_t *rb);

/** Push one IMU sample.  Returns 0 on success, -1 if full. */
int  levio_ringbuf_push(levio_ringbuf_t *rb, const levio_imu_sample_t *s);

/** Pop one IMU sample.  Returns 0 on success, -1 if empty. */
int  levio_ringbuf_pop(levio_ringbuf_t *rb, levio_imu_sample_t *s);

/** Peek at next sample without consuming it.  Returns 0 on success. */
int  levio_ringbuf_peek(const levio_ringbuf_t *rb, levio_imu_sample_t *s);

/** Number of samples currently available. */
uint32_t levio_ringbuf_size(const levio_ringbuf_t *rb);

/** 1 if empty, 0 otherwise. */
int  levio_ringbuf_empty(const levio_ringbuf_t *rb);

/** 1 if full, 0 otherwise. */
int  levio_ringbuf_full(const levio_ringbuf_t *rb);

#ifdef __cplusplus
}
#endif

#endif /* LEVIO_RINGBUF_H */
