/**
 * @file levio_orb.c
 * @brief ORB feature extraction (stub) and Hamming distance (implemented).
 *
 * The Hamming distance implementation is complete and tested.
 * levio_orb_extract() is a stub placeholder that returns grid-sampled
 * keypoints with zeroed descriptors.
 *
 * TODO: Replace levio_orb_extract() with a full Oriented FAST + Rotated
 *       BRIEF implementation (or wrap an embedded ORB library).
 */
#include "levio_orb.h"
#include "levio_cfg.h"
#include <string.h>
#include <stdint.h>

/* --------------------------------------------------------------------------
 * Hamming distance
 * -------------------------------------------------------------------------- */

uint32_t levio_orb_hamming(const uint8_t *a, const uint8_t *b)
{
    uint32_t dist = 0;
    int i;
    for (i = 0; i < LEVIO_DESC_BYTES; ++i) {
        uint8_t xv = a[i] ^ b[i];
        /* Brian Kernighan bit-count */
        while (xv) {
            ++dist;
            xv &= (uint8_t)(xv - 1u);
        }
    }
    return dist;
}

/* --------------------------------------------------------------------------
 * ORB extraction stub
 * -------------------------------------------------------------------------- */

int levio_orb_extract(const uint8_t *gray, int w, int h, int stride,
                      levio_feat_t *out_feats, int max_feats)
{
    /* TODO: Implement Oriented FAST keypoint detection followed by
     *       Rotated BRIEF descriptor computation.
     *
     * Current placeholder: distribute features on a uniform grid so that
     * the rest of the pipeline (matching, RANSAC, triangulation) can be
     * exercised without a real detector.
     */
    int cols = 20;
    int rows = 15;
    int n = 0;
    int r, c;

    if (!gray || w <= 0 || h <= 0 || !out_feats || max_feats <= 0)
        return -1;

    for (r = 0; r < rows && n < max_feats; ++r) {
        for (c = 0; c < cols && n < max_feats; ++c) {
            float px_x = (c + 0.5f) * ((float)w  / cols);
            float px_y = (r + 0.5f) * ((float)h  / rows);

            /* Suppress points outside image bounds */
            if (px_x < 1.0f || px_x >= (float)(w - 1) ||
                px_y < 1.0f || px_y >= (float)(h - 1))
                continue;

            out_feats[n].px.x    = px_x;
            out_feats[n].px.y    = px_y;
            out_feats[n].angle   = 0.0f;
            out_feats[n].octave  = 0;
            out_feats[n].lm_id   = -1;
            /* Descriptor: sample pixel value into first byte, zero the rest */
            int xi = (int)px_x;
            int yi = (int)px_y;
            memset(out_feats[n].desc, 0, LEVIO_DESC_BYTES);
            out_feats[n].desc[0] = gray[yi * stride + xi];
            ++n;
        }
    }
    return n;
}
