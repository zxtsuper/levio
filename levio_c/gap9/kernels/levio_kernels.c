/**
 * @file levio_kernels.c
 * @brief GAP9 cluster PE kernels for LEVIO parallel processing.
 *
 * Compile-time switch:
 *   #define LEVIO_USE_GAP9_CLUSTER   – use pi_cl_team_fork() / pi_core_id()
 *                                      (requires GAP SDK / PMSIS headers)
 *   (undefined)                      – single-threaded fallback
 *
 * The fallback path is compiled on any C99-compliant host and passes the
 * existing desktop unit-tests without modification.
 */
#include "levio_kernels.h"
#include "levio_orb.h"   /* levio_orb_hamming() */
#include <string.h>
#include <stdint.h>
#include <math.h>

#ifdef LEVIO_USE_GAP9_CLUSTER
#  include "pmsis.h"
#  define NB_PE  8
#  define CL_CORE_ID()  pi_core_id()
#  define CL_NUM_CORES() NB_PE
#  define CL_TEAM_FORK(nb, fn, args)  pi_cl_team_fork((nb), (fn), (args))
#else
/* Desktop fallback: single-threaded, core-id = 0, num_cores = 1 */
#  define NB_PE  1
#  define CL_CORE_ID()   0
#  define CL_NUM_CORES() 1
#  define CL_TEAM_FORK(nb, fn, args)  (fn)(args)
#endif

/* ==========================================================================
 * Matching kernel
 * ========================================================================== */

/*
 * Per-PE work buffer for the ratio-test match output.
 * Each PE writes its private results into a slice of the shared output array.
 * The FC compacts them afterwards.
 *
 * Layout: out[pe * (max_out / NB_PE) + local_idx]
 * The global match count is updated atomically (or via sum reduction on FC).
 */

/* Shared scratch: per-PE match counts (written by PEs, read by FC). */
static volatile int s_pe_match_counts[NB_PE];

void levio_cl_match_kernel(void *arg)
{
    levio_match_args_t *a = (levio_match_args_t *)arg;

    int core_id  = CL_CORE_ID();
    int n_cores  = CL_NUM_CORES();

    /* Partition A across PEs (stride = n_cores) */
    int chunk = (a->n_a + n_cores - 1) / n_cores;
    int start = core_id * chunk;
    int end   = start + chunk;
    if (end > a->n_a) end = a->n_a;

    /* Each PE writes to its own slice of out[] to avoid concurrent writes */
    int pe_cap   = a->max_out / n_cores;
    levio_match_t *pe_out = a->out + core_id * pe_cap;
    int pe_n = 0;

    int i, j;
    for (i = start; i < end && pe_n < pe_cap; ++i) {
        /* Lowe ratio test: find best and second-best in B */
        uint32_t best1 = UINT32_MAX, best2 = UINT32_MAX;
        int best_idx = -1;

        for (j = 0; j < a->n_b; ++j) {
            uint32_t d = levio_orb_hamming(a->feats_a[i].desc,
                                            a->feats_b[j].desc);
            if (d < best1) {
                best2 = best1;
                best1 = d;
                best_idx = j;
            } else if (d < best2) {
                best2 = d;
            }
        }

        if (best_idx >= 0 && best2 > 0 &&
            (float)best1 < a->ratio * (float)best2) {
            pe_out[pe_n].idx_a = (int16_t)i;
            pe_out[pe_n].idx_b = (int16_t)best_idx;
            pe_out[pe_n].dist  = (uint16_t)best1;
            ++pe_n;
        }
    }

    s_pe_match_counts[core_id] = pe_n;
}

void levio_cl_match_ratio_fork(levio_match_args_t *args)
{
    int i;
    /* Reset per-PE counters */
    for (i = 0; i < NB_PE; ++i)
        s_pe_match_counts[i] = 0;

    CL_TEAM_FORK(NB_PE, levio_cl_match_kernel, (void *)args);

    /* Compact per-PE slices into a contiguous output array */
    int n_cores = CL_NUM_CORES();
    int pe_cap  = args->max_out / n_cores;
    int total   = 0;

    for (i = 0; i < n_cores; ++i) {
        int pe_n  = s_pe_match_counts[i];
        levio_match_t *pe_out  = args->out + i * pe_cap;
        levio_match_t *dst_out = args->out + total;

        /* Only move if the slice is not already in place */
        if (i > 0 && pe_n > 0)
            memmove(dst_out, pe_out, (size_t)pe_n * sizeof(levio_match_t));

        total += pe_n;
        if (total >= args->max_out) {
            total = args->max_out;
            break;
        }
    }

    args->n_matches = total;
}

/* ==========================================================================
 * RANSAC inlier-counting kernel
 * ========================================================================== */

static volatile int s_pe_inlier_counts[NB_PE];

void levio_cl_inlier_kernel(void *arg)
{
    levio_inlier_args_t *a = (levio_inlier_args_t *)arg;

    int core_id = CL_CORE_ID();
    int n_cores = CL_NUM_CORES();

    /* Stride over points */
    int local_count = 0;
    int i;

    for (i = core_id; i < a->n; i += n_cores) {
        float X = a->Rcw[0]*a->Pw[i].x + a->Rcw[1]*a->Pw[i].y
                + a->Rcw[2]*a->Pw[i].z + a->tcw->x;
        float Y = a->Rcw[3]*a->Pw[i].x + a->Rcw[4]*a->Pw[i].y
                + a->Rcw[5]*a->Pw[i].z + a->tcw->y;
        float Z = a->Rcw[6]*a->Pw[i].x + a->Rcw[7]*a->Pw[i].y
                + a->Rcw[8]*a->Pw[i].z + a->tcw->z;

        if (Z <= 0.0f) continue;

        float iz = 1.0f / Z;
        float ex = a->K->fx * X * iz + a->K->cx - a->px[i].x;
        float ey = a->K->fy * Y * iz + a->K->cy - a->px[i].y;

        if (ex*ex + ey*ey < a->thr2)
            ++local_count;
    }

    s_pe_inlier_counts[core_id] = local_count;
}

void levio_cl_inlier_count_fork(levio_inlier_args_t *args)
{
    int i;
    for (i = 0; i < NB_PE; ++i)
        s_pe_inlier_counts[i] = 0;

    CL_TEAM_FORK(NB_PE, levio_cl_inlier_kernel, (void *)args);

    /* Sum per-PE inlier counts */
    int total = 0;
    for (i = 0; i < CL_NUM_CORES(); ++i)
        total += s_pe_inlier_counts[i];

    args->n_inliers = total;
}
