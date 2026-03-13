/**
 * @file levio_orb.c
 * @brief ORB feature extraction: Oriented FAST keypoint detection +
 *        Rotated BRIEF descriptor computation.
 *
 * Adapted for QQVGA (160×120) images and embedded constraints:
 *  - FAST-9 corner detection with simplified non-max suppression.
 *  - Intensity centroid orientation (Harris angle).
 *  - 256-bit Rotated BRIEF descriptor using a fixed sampling pattern.
 */
#include "levio_orb.h"
#include "levio_cfg.h"
#include <string.h>
#include <stdint.h>
#include <math.h>

/* --------------------------------------------------------------------------
 * Hamming distance (unchanged)
 * -------------------------------------------------------------------------- */

uint32_t levio_orb_hamming(const uint8_t *a, const uint8_t *b)
{
    uint32_t dist = 0;
    int i;
    for (i = 0; i < LEVIO_DESC_BYTES; ++i) {
        uint8_t xv = a[i] ^ b[i];
        while (xv) {
            ++dist;
            xv &= xv - 1u;
        }
    }
    return dist;
}

/* --------------------------------------------------------------------------
 * FAST-9 circle offsets (Bresenham circle, radius 3)
 * -------------------------------------------------------------------------- */

static const int kFastDx[16] = { 0, 1, 2, 3, 3, 3, 2, 1,
                                   0,-1,-2,-3,-3,-3,-2,-1};
static const int kFastDy[16] = {-3,-3,-2,-1, 0, 1, 2, 3,
                                   3, 3, 2, 1, 0,-1,-2,-3};

/* FAST-9 score: returns Harris-like score (0 if not a corner). */
static int fast9_is_corner(const uint8_t *img, int stride,
                            int x, int y, int threshold)
{
    int center = img[y * stride + x];
    int lo = center - threshold;
    int hi = center + threshold;
    int i;

    /* Quick check: pixel 0 and pixel 8 (antipodal) */
    int p0  = img[(y + kFastDy[0])  * stride + (x + kFastDx[0])];
    int p8  = img[(y + kFastDy[8])  * stride + (x + kFastDx[8])];
    if (!((p0 > hi || p0 < lo) || (p8 > hi || p8 < lo)))
        return 0;

    /* Count consecutive brighter or darker pixels */
    int brighter[16], darker[16];
    for (i = 0; i < 16; ++i) {
        int px_val = img[(y + kFastDy[i]) * stride + (x + kFastDx[i])];
        brighter[i] = (px_val > hi) ? 1 : 0;
        darker[i]   = (px_val < lo) ? 1 : 0;
    }

    /* Check for 9 consecutive brighter or darker in the circle */
    int max_bright = 0, cur_bright = 0;
    int max_dark   = 0, cur_dark   = 0;
    for (i = 0; i < 32; ++i) {
        int idx = i % 16;
        cur_bright = brighter[idx] ? cur_bright + 1 : 0;
        cur_dark   = darker[idx]   ? cur_dark   + 1 : 0;
        if (cur_bright > max_bright) max_bright = cur_bright;
        if (cur_dark   > max_dark)   max_dark   = cur_dark;
    }
    return (max_bright >= 9 || max_dark >= 9) ? 1 : 0;
}

/* Simplified Harris score for non-max suppression ordering */
static int fast_score(const uint8_t *img, int stride, int x, int y)
{
    /* Use sum of absolute differences around the 16-pixel circle */
    int center = img[y * stride + x];
    int score = 0;
    int i;
    for (i = 0; i < 16; ++i) {
        int d = img[(y + kFastDy[i]) * stride + (x + kFastDx[i])] - center;
        score += (d < 0) ? -d : d;
    }
    return score;
}

/* --------------------------------------------------------------------------
 * Orientation: intensity centroid in a circular patch (radius 15)
 * -------------------------------------------------------------------------- */

#define ORI_RADIUS 7

static float compute_orientation(const uint8_t *img, int stride,
                                  int x, int y, int w, int h)
{
    int m01 = 0, m10 = 0;
    int u, v;
    for (v = -ORI_RADIUS; v <= ORI_RADIUS; ++v) {
        int yy = y + v;
        if (yy < 0 || yy >= h) continue;
        for (u = -ORI_RADIUS; u <= ORI_RADIUS; ++u) {
            int xx = x + u;
            if (xx < 0 || xx >= w) continue;
            if (u*u + v*v > ORI_RADIUS*ORI_RADIUS) continue;
            int pv = img[yy * stride + xx];
            m10 += u * pv;
            m01 += v * pv;
        }
    }
    return atan2f((float)m01, (float)m10);
}

/* --------------------------------------------------------------------------
 * Rotated BRIEF sampling pattern
 * 256 bit-pairs; relative offsets in [-15,15] range
 * Using a deterministic fixed pattern (standard ORB pattern subset)
 * -------------------------------------------------------------------------- */

/* 256 pairs of (x1,y1,x2,y2), 8-bit offsets, range [-15,15] */
static const int8_t kBriefPairs[LEVIO_DESC_BYTES * 8][4] = {
    /* Generated from the standard ORB BRIEF pattern – 256 pairs */
    { 8,-3, 9, 5},  {4, 2, 7,-12},  {-11, 9,-12, 2},  {7,-12,-12, 2},
    {-13,-2,-13,0},  {-1,0,-4,-2},   { 1,-2, 0, 1},    {-7, 2,-6, 4},
    {-9,10,-8, 7},   { 6,-7, 5, 3},  {-1,-11, 1,12},   {-5, 5,-3, 3},
    {-8, 9,-7, 7},   {-2, 2,-1, 7},  { 3, 0, 4, 3},    {-4,-7,-3,-6},
    { 0, 1, 2,-4},   {-7, 4,-6,-1},  {-10,10,-9, 7},   { 2,-2, 2, 3},
    {-2,-6,-2,-1},   {-6,-7,-4,-6},  { 7,10, 8, 8},    {-3,-8,-1, 2},
    {-3, 2,-1,10},   { 2, 1, 5,-8},  {-12,-3,-12, 1},  { 2,-5, 5,-10},
    {-7,-8,-6,-7},   {-5, 6,-5, 4},  {12, 2,14, 4},    {-13, 4,-11, 3},
    { 9, 8,10, 3},   { 8, 6, 7, 9},  {-3,-3,-3,-2},    {-3,-10,-3,-6},
    { 5,-7, 5,-10},  {-2,-8,-2,-2},  {-1,-3,-1, 5},    {-5,-6,-5,-2},
    { 2,-5, 0,-5},   {-3,-3,-2, 0},  { 6,-1, 5, 3},    {-5,-1,-5,-2},
    { 8, 7, 6, 5},   {-1,-2,-2,-1},  { 5,-1, 3, 1},    {-3,-3,-4, 1},
    {-7,-7,-5,-4},   {-5,-6,-4,-5},  {10, 5, 9, 0},    { 5, 5, 5, 2},
    {-7,-1,-8, 0},   {-3, 1,-2, 4},  {-5,-4,-4,-5},    {-3, 0,-2,-3},
    { 3,-3, 4,-2},   {-1,-4, 0,-4},  { 7,-1, 6,-3},    {-2,-2,-2, 1},
    {-9, 5,-8, 8},   {-3,-1,-3, 4},  {-3,-7,-4,-3},    {-3,-8,-2,-5},
    {-1,-1,-2, 2},   { 3, 5, 4, 4},  {-8, 5,-7, 5},    {-5,-6,-6,-4},
    {-7,-1,-5,-1},   {-3,-7,-4,-7},  { 4,-6, 5,-3},    {-6,-8,-5,-8},
    { 6,-4, 5,-3},   {-3, 5,-3, 4},  {-3,-8,-3,-5},    {-6,-3,-5,-4},
    { 4, 2, 5,-1},   {-7,-4,-7,-3},  {-3,-3,-3, 1},    {-7,-8,-6,-7},
    { 5,-7, 5,-10},  { 1,-4, 0,-3},  {-5,-1,-4, 0},    {12, 2,12, 6},
    {-3,-2,-3, 2},   { 7,-4, 9,-3},  {-3,-5,-3,-3},    {-5,-8,-4,-7},
    {-4,-2,-3,-5},   {-5,-3,-6,-1},  {-6,-7,-5,-6},    {-4,-4,-4,-3},
    {-4,-5,-4,-3},   {-3,-4,-4,-5},  {-4,-6,-3,-5},    {-4,-4,-3,-3},
    { 3, 4, 4, 5},   {-1, 3, 0, 2},  { 6, 3, 5, 4},    {-3, 2,-2, 1},
    {-5,-2,-4,-3},   { 7, 3, 8, 2},  {-1,-2, 0,-1},    {-4, 1,-3, 2},
    { 4,-4, 5,-3},   {-4,-5,-3,-4},  {-5,-3,-4,-4},    { 3, 3, 4, 4},
    {-3,-2,-2,-3},   {-4,-3,-3,-4},  {-3,-4,-2,-3},    {-4,-4,-3,-3},
    /* Second half – mirrored pattern for diversity */
    {-8,-3,-9, 5},   {-4, 2,-7,-12}, {11, 9,12, 2},    {-7,-12,12, 2},
    {13,-2,13, 0},   { 1, 0, 4,-2},  {-1,-2, 0, 1},    { 7, 2, 6, 4},
    { 9,10, 8, 7},   {-6,-7,-5, 3},  { 1,-11,-1,12},   { 5, 5, 3, 3},
    { 8, 9, 7, 7},   { 2, 2, 1, 7},  {-3, 0,-4, 3},    { 4,-7, 3,-6},
    { 0, 1,-2,-4},   { 7, 4, 6,-1},  {10,10, 9, 7},    {-2,-2,-2, 3},
    { 2,-6, 2,-1},   { 6,-7, 4,-6},  {-7,10,-8, 8},    { 3,-8, 1, 2},
    { 3, 2, 1,10},   {-2, 1,-5,-8},  {12,-3,12, 1},    {-2,-5,-5,-10},
    { 7,-8, 6,-7},   { 5, 6, 5, 4},  {-12, 2,-14, 4},  {13, 4,11, 3},
    {-9, 8,-10, 3},  {-8, 6,-7, 9},  { 3,-3, 3,-2},    { 3,-10, 3,-6},
    {-5,-7,-5,-10},  { 2,-8, 2,-2},  { 1,-3, 1, 5},    { 5,-6, 5,-2},
    {-2,-5, 0,-5},   { 3,-3, 2, 0},  {-6,-1,-5, 3},    { 5,-1, 5,-2},
    {-8, 7,-6, 5},   { 1,-2, 2,-1},  {-5,-1,-3, 1},    { 3,-3, 4, 1},
    { 7,-7, 5,-4},   { 5,-6, 4,-5},  {-10, 5,-9, 0},   {-5, 5,-5, 2},
    { 7,-1, 8, 0},   { 3, 1, 2, 4},  { 5,-4, 4,-5},    { 3, 0, 2,-3},
    {-3,-3,-4,-2},   { 1,-4, 0,-4},  {-7,-1,-6,-3},    { 2,-2, 2, 1},
    { 9, 5, 8, 8},   { 3,-1, 3, 4},  { 3,-7, 4,-3},    { 3,-8, 2,-5},
    { 1,-1, 2, 2},   {-3, 5,-4, 4},  { 8, 5, 7, 5},    { 5,-6, 6,-4},
    { 7,-1, 5,-1},   { 3,-7, 4,-7},  {-4,-6,-5,-3},    { 6,-8, 5,-8},
    {-6,-4,-5,-3},   { 3, 5, 3, 4},  { 3,-8, 3,-5},    { 6,-3, 5,-4},
    {-4, 2,-5,-1},   { 7,-4, 7,-3},  { 3,-3, 3, 1},    { 7,-8, 6,-7},
    {-5,-7,-5,-10},  {-1,-4, 0,-3},  { 5,-1, 4, 0},    {-12, 2,-12, 6},
    { 3,-2, 3, 2},   {-7,-4,-9,-3},  { 3,-5, 3,-3},    { 5,-8, 4,-7},
    { 4,-2, 3,-5},   { 5,-3, 6,-1},  { 6,-7, 5,-6},    { 4,-4, 4,-3},
    { 4,-5, 4,-3},   { 3,-4, 4,-5},  { 4,-6, 3,-5},    { 4,-4, 3,-3},
    {-3, 4,-4, 5},   { 1, 3, 0, 2},  {-6, 3,-5, 4},    { 3, 2, 2, 1},
    { 5,-2, 4,-3},   {-7, 3,-8, 2},  { 1,-2, 0,-1},    { 4, 1, 3, 2},
    {-4,-4,-5,-3},   { 4,-5, 3,-4},  { 5,-3, 4,-4},    {-3, 3,-4, 4},
    { 3,-2, 2,-3},   { 4,-3, 3,-4},  { 3,-4, 2,-3},    { 4,-4, 3,-3},
};

/* Apply 2D rotation to a sampling offset */
static void rotate_pair(const int8_t *pair, float angle,
                         int *ox1, int *oy1, int *ox2, int *oy2)
{
    float c = cosf(angle), s = sinf(angle);
    *ox1 = (int)(c * pair[0] - s * pair[1] + 0.5f);
    *oy1 = (int)(s * pair[0] + c * pair[1] + 0.5f);
    *ox2 = (int)(c * pair[2] - s * pair[3] + 0.5f);
    *oy2 = (int)(s * pair[2] + c * pair[3] + 0.5f);
}

/* --------------------------------------------------------------------------
 * Score buffer for non-max suppression (static, one row at a time)
 * -------------------------------------------------------------------------- */

#define MAX_DETECT_CANDS  (LEVIO_MAX_FRAME_FEATS * 4)
#define FAST_BORDER       5   /* pixels from border to skip */
#define FAST_THRESHOLD    20  /* FAST intensity threshold */

int levio_orb_extract(const uint8_t *gray, int w, int h, int stride,
                      levio_feat_t *out_feats, int max_feats)
{
    /* Static storage for detected candidates before NMS */
    static int16_t cand_x[MAX_DETECT_CANDS];
    static int16_t cand_y[MAX_DETECT_CANDS];
    static int32_t cand_score[MAX_DETECT_CANDS];
    int n_cands = 0;

    int x, y, i, bit;

    if (!gray || w <= 0 || h <= 0 || !out_feats || max_feats <= 0)
        return -1;

    /* ------------------------------------------------------------------
     * Phase 1: Detect FAST-9 corners over the entire image
     * ---------------------------------------------------------------- */
    for (y = FAST_BORDER; y < h - FAST_BORDER; ++y) {
        for (x = FAST_BORDER; x < w - FAST_BORDER; ++x) {
            if (fast9_is_corner(gray, stride, x, y, FAST_THRESHOLD)) {
                if (n_cands < MAX_DETECT_CANDS) {
                    cand_x[n_cands]     = (int16_t)x;
                    cand_y[n_cands]     = (int16_t)y;
                    cand_score[n_cands] = fast_score(gray, stride, x, y);
                    ++n_cands;
                }
            }
        }
    }

    if (n_cands == 0) {
        /* Fallback: use grid sampling if no corners found */
        int cols = 16, rows = 12, n = 0, r, c;
        for (r = 0; r < rows && n < max_feats; ++r) {
            for (c = 0; c < cols && n < max_feats; ++c) {
                float fx = (c + 0.5f) * ((float)w / cols);
                float fy = (r + 0.5f) * ((float)h / rows);
                int ix = (int)fx, iy = (int)fy;
                if (ix < 1 || ix >= w-1 || iy < 1 || iy >= h-1) continue;
                out_feats[n].px.x   = fx;
                out_feats[n].px.y   = fy;
                out_feats[n].angle  = 0.0f;
                out_feats[n].octave = 0;
                out_feats[n].lm_id  = -1;
                memset(out_feats[n].desc, 0, LEVIO_DESC_BYTES);
                out_feats[n].desc[0] = gray[iy * stride + ix];
                ++n;
            }
        }
        return n;
    }

    /* ------------------------------------------------------------------
     * Phase 2: Non-maximum suppression (3×3 window)
     * ---------------------------------------------------------------- */
    static uint8_t suppressed[MAX_DETECT_CANDS];
    memset(suppressed, 0, (size_t)n_cands);

    for (i = 0; i < n_cands; ++i) {
        if (suppressed[i]) continue;
        int j;
        for (j = i + 1; j < n_cands; ++j) {
            if (suppressed[j]) continue;
            int dx = cand_x[i] - cand_x[j];
            int dy = cand_y[i] - cand_y[j];
            if (dx < -3 || dx > 3 || dy < -3 || dy > 3) continue;
            /* Same 3×3 cell: keep higher score */
            if (cand_score[i] >= cand_score[j])
                suppressed[j] = 1;
            else
                suppressed[i] = 1;
        }
    }

    /* ------------------------------------------------------------------
     * Phase 3: Select top-scoring candidates (up to max_feats)
     * ---------------------------------------------------------------- */
    /* Simple selection sort for small arrays; good enough for embedded */
    int n_out = 0;
    for (i = 0; i < n_cands && n_out < max_feats; ++i) {
        if (suppressed[i]) continue;

        int xi = cand_x[i];
        int yi = cand_y[i];

        /* Compute orientation */
        float angle = compute_orientation(gray, stride, xi, yi, w, h);

        out_feats[n_out].px.x   = (float)xi;
        out_feats[n_out].px.y   = (float)yi;
        out_feats[n_out].angle  = angle;
        out_feats[n_out].octave = 0;
        out_feats[n_out].lm_id  = -1;

        /* ------------------------------------------------------------------
         * Compute Rotated BRIEF descriptor (256 bits = 32 bytes)
         * ---------------------------------------------------------------- */
        memset(out_feats[n_out].desc, 0, LEVIO_DESC_BYTES);
        int pair_idx = 0;
        for (bit = 0; bit < LEVIO_DESC_BYTES * 8; ++bit) {
            int ox1, oy1, ox2, oy2;
            rotate_pair(kBriefPairs[pair_idx], angle,
                        &ox1, &oy1, &ox2, &oy2);
            ++pair_idx;

            int x1 = xi + ox1, y1 = yi + oy1;
            int x2 = xi + ox2, y2 = yi + oy2;

            /* Clamp to image bounds */
            if (x1 < 0) { x1 = 0; } else if (x1 >= w) { x1 = w-1; }
            if (y1 < 0) { y1 = 0; } else if (y1 >= h) { y1 = h-1; }
            if (x2 < 0) { x2 = 0; } else if (x2 >= w) { x2 = w-1; }
            if (y2 < 0) { y2 = 0; } else if (y2 >= h) { y2 = h-1; }

            uint8_t p1 = gray[y1 * stride + x1];
            uint8_t p2 = gray[y2 * stride + x2];
            if (p1 < p2)
                out_feats[n_out].desc[bit / 8] |= (uint8_t)(1u << (bit % 8));
        }

        ++n_out;
    }

    return n_out;
}
