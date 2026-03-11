/**
 * @file levio_orb.h
 * @brief ORB feature extraction interface for LEVIO.
 *
 * Paper §3.2 (first segment): "ORB descriptor extraction" on each incoming
 * frame, up to LEVIO_MAX_FRAME_FEATS (700) features.
 *
 * The full ORB pipeline (Oriented FAST + Rotated BRIEF) is complex; this
 * header provides the API.  The current src/levio_orb.c implements a
 * simplified placeholder (FAST corner detection + zero descriptors) so the
 * rest of the pipeline compiles and links on desktop.
 *
 * TODO: replace levio_orb_extract() body with a full ORB implementation
 *       or wrap an existing embedded ORB library (e.g., the one used on
 *       the GAP9 target).
 */
#ifndef LEVIO_ORB_H
#define LEVIO_ORB_H

#include <stdint.h>
#include "levio_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Extract ORB features from a greyscale image.
 *
 * @param gray       Pointer to LEVIO_IMG_W × LEVIO_IMG_H greyscale buffer
 *                   (row-major, 1 byte per pixel).
 * @param w          Image width  (should equal LEVIO_IMG_W).
 * @param h          Image height (should equal LEVIO_IMG_H).
 * @param stride     Row stride in bytes (usually == w).
 * @param out_feats  Output feature array (caller-allocated, capacity max_feats).
 * @param max_feats  Maximum number of features to extract.
 * @return           Number of features extracted (≥ 0), or -1 on error.
 */
int levio_orb_extract(const uint8_t *gray, int w, int h, int stride,
                      levio_feat_t *out_feats, int max_feats);

/**
 * @brief Compute the Hamming distance between two 256-bit ORB descriptors.
 *
 * Uses popcount (POPCNT instruction where available, otherwise bit-twiddling).
 *
 * @param a  Pointer to first  descriptor (LEVIO_DESC_BYTES bytes).
 * @param b  Pointer to second descriptor (LEVIO_DESC_BYTES bytes).
 * @return   Hamming distance in [0, 256].
 */
uint32_t levio_orb_hamming(const uint8_t *a, const uint8_t *b);

#ifdef __cplusplus
}
#endif

#endif /* LEVIO_ORB_H */
