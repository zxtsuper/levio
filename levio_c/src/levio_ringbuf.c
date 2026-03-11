/**
 * @file levio_ringbuf.c
 * @brief IMU ring buffer implementation.
 */
#include "levio_ringbuf.h"
#include <string.h>

void levio_ringbuf_init(levio_ringbuf_t *rb)
{
    memset(rb->buf, 0, sizeof(rb->buf));
    rb->head = 0;
    rb->tail = 0;
}

int levio_ringbuf_push(levio_ringbuf_t *rb, const levio_imu_sample_t *s)
{
    uint32_t next = (rb->head + 1u) % LEVIO_IMU_BUF_SIZE;
    if (next == rb->tail)
        return -1;  /* full */
    rb->buf[rb->head] = *s;
    rb->head = next;
    return 0;
}

int levio_ringbuf_pop(levio_ringbuf_t *rb, levio_imu_sample_t *s)
{
    if (rb->tail == rb->head)
        return -1;  /* empty */
    *s = rb->buf[rb->tail];
    rb->tail = (rb->tail + 1u) % LEVIO_IMU_BUF_SIZE;
    return 0;
}

int levio_ringbuf_peek(const levio_ringbuf_t *rb, levio_imu_sample_t *s)
{
    if (rb->tail == rb->head)
        return -1;  /* empty */
    *s = rb->buf[rb->tail];
    return 0;
}

uint32_t levio_ringbuf_size(const levio_ringbuf_t *rb)
{
    return (rb->head - rb->tail + LEVIO_IMU_BUF_SIZE) % LEVIO_IMU_BUF_SIZE;
}

int levio_ringbuf_empty(const levio_ringbuf_t *rb)
{
    return rb->tail == rb->head;
}

int levio_ringbuf_full(const levio_ringbuf_t *rb)
{
    return ((rb->head + 1u) % LEVIO_IMU_BUF_SIZE) == rb->tail;
}
