/**
 * @file levio_match.c
 * @brief Bidirectional brute-force ORB descriptor matcher.
 *
 * Paper §3.2 (second segment): bidirectional matching with Hamming distance.
 */
#include "levio_match.h"
#include "levio_orb.h"
#include <stdint.h>

/* Internal: find index of nearest neighbour in set b for descriptor a_desc.
 * Returns index and writes distance into *out_dist.
 * Returns -1 if nb == 0. */
static int find_nn(const uint8_t *a_desc,
                   const levio_feat_t *b, int nb,
                   uint32_t *out_dist)
{
    int best_idx = -1;
    uint32_t best_dist = UINT32_MAX;
    int i;
    for (i = 0; i < nb; ++i) {
        uint32_t d = levio_orb_hamming(a_desc, b[i].desc);
        if (d < best_dist) {
            best_dist = d;
            best_idx  = i;
        }
    }
    *out_dist = best_dist;
    return best_idx;
}

int levio_match_bidir(const levio_feat_t *a, int na,
                      const levio_feat_t *b, int nb,
                      levio_match_t *out, int max_out)
{
    int n_matches = 0;
    int i;

    if (!a || na <= 0 || !b || nb <= 0 || !out || max_out <= 0)
        return 0;

    for (i = 0; i < na && n_matches < max_out; ++i) {
        /* Forward: a[i] → nearest in b */
        uint32_t d_fwd;
        int j_nn = find_nn(a[i].desc, b, nb, &d_fwd);
        if (j_nn < 0)
            continue;

        /* Backward: b[j_nn] → nearest in a */
        uint32_t d_bwd;
        int i_check = find_nn(b[j_nn].desc, a, na, &d_bwd);

        /* Accept only mutual nearest neighbours (cross-check) */
        if (i_check == i) {
            out[n_matches].idx_a = (int16_t)i;
            out[n_matches].idx_b = (int16_t)j_nn;
            out[n_matches].dist  = (uint16_t)d_fwd;
            ++n_matches;
        }
    }
    return n_matches;
}

int levio_match_ratio(const levio_feat_t *a, int na,
                      const levio_feat_t *b, int nb,
                      float ratio,
                      levio_match_t *out, int max_out)
{
    int n_matches = 0;
    int i, j;

    if (!a || na <= 0 || !b || nb <= 0 || !out || max_out <= 0)
        return 0;

    for (i = 0; i < na && n_matches < max_out; ++i) {
        uint32_t best1 = UINT32_MAX, best2 = UINT32_MAX;
        int best_idx = -1;

        for (j = 0; j < nb; ++j) {
            uint32_t d = levio_orb_hamming(a[i].desc, b[j].desc);
            if (d < best1) {
                best2 = best1;
                best1 = d;
                best_idx = j;
            } else if (d < best2) {
                best2 = d;
            }
        }

        if (best_idx >= 0 && best2 > 0 &&
            (float)best1 < ratio * (float)best2) {
            out[n_matches].idx_a = (int16_t)i;
            out[n_matches].idx_b = (int16_t)best_idx;
            out[n_matches].dist  = (uint16_t)best1;
            ++n_matches;
        }
    }
    return n_matches;
}
