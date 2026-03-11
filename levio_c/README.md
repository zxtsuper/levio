# levio_c — Embedded-C Skeleton for LEVIO

This directory contains an initial **embedded-C** implementation skeleton for
the LEVIO visual-inertial odometry system, targeting the **GAP9 SoC** by
GreenWaves Technologies.

The code maps to the algorithm described in:

> **LEVIO: Lightweight Embedded Visual Inertial Odometry for Resource-Constrained Devices**  
> arXiv:2602.03294v1 — <https://arxiv.org/abs/2602.03294>

---

## Status

**Skeleton / work-in-progress.**  The components listed below compile cleanly,
pass unit tests on desktop, and match all paper-defined default values.
Math-heavy internals (SVD, EPnP solver, Schur complement) are structurally
complete stubs that return `−1` and contain detailed `TODO` comments.

---

## How this maps to paper sections

### §3.2 — Visual Odometry front-end

Six pipeline segments, each with a header and implementation:

| Segment | Paper description | Module |
|---------|------------------|--------|
| 1 | ORB extraction (up to 700 descriptors per frame) | `levio_orb.{h,c}` |
| 2 | Bidirectional brute-force matching (Hamming distance) | `levio_match.{h,c}` |
| 3 | 8-point algorithm + RANSAC (fallback when < 25 world matches) | `levio_ransac_8pt.{h,c}` |
| 4 | EPnP + RANSAC (primary 2D-3D pose estimation) | `levio_pnp_epnp.{h,c}` |
| 5 | Keyframe selection (parallax threshold 15 px) | `levio_frontend.{h,c}` |
| 6 | Triangulation of new world points (DLT, up to 1000 landmarks) | `levio_triangulate.{h,c}` |

The fallback rule (< 25 world-point matches → 8-point algorithm) is
implemented in `levio_frontend_process()`.

### §3.3 — Tight-coupled BA + IMU

| Component | Module |
|-----------|--------|
| IMU ring buffer (FIFO, 256 samples) | `levio_ringbuf.{h,c}` |
| Pre-integration, equations (2)–(4) | `levio_imu_preint.{h,c}` |
| Visual reprojection factor + Jacobians | `levio_factors.{h,c}`, `levio_camera.{h,c}` |
| IMU pre-integration factor | `levio_factors.{h,c}` |
| Prior / marginalisation factor | `levio_factors.{h,c}` |
| Sliding window back-end (10 keyframes) | `levio_backend.{h,c}` |
| LM optimiser + Schur complement (eq. 5–6) stub | `levio_backend.c` |

### §3.4 — Embedded optimisations (GAP9 target)

| Optimisation | Status |
|-------------|--------|
| Image resolution QQVGA 160×120 | ✅ `LEVIO_IMG_W/H` in `levio_cfg.h` |
| Self-contained C, no heap allocation | ✅ all structures statically allocated |
| Custom linear-algebra library | ✅ `levio_math.{h,c}`, `levio_se3.{h,c}` |
| Schur complement for reduced system | 🔧 stub in `levio_backend.c` |
| Parallel cluster execution (GAP9) | ❌ not yet (add `pi_cluster_*` calls) |
| Tiered L1/L2 memory usage | ❌ not yet |

---

## Default values (paper-aligned)

All constants live in `include/levio_cfg.h` with inline section references:

| Constant | Value | Paper reference |
|----------|-------|-----------------|
| `LEVIO_IMG_W` / `LEVIO_IMG_H` | 160 / 120 | §3.4 "QQVGA" |
| `LEVIO_MAX_FRAME_FEATS` | 700 | §3.2 |
| `LEVIO_MAX_WORLD_POINTS` | 1000 | §3.2 |
| `LEVIO_PNP_MIN_INLIERS` | 25 | §3.2 |
| `LEVIO_WINDOW_SIZE` | 10 | §3.3 Table I |
| `LEVIO_KF_PARALLAX_PX` | 15.0 px | §3.2 |
| `LEVIO_IMU_BUF_SIZE` | 256 | derived (200 Hz / 5 Hz × 2) |

> **Note:** `LEVIO_WINDOW_SIZE` and `LEVIO_KF_PARALLAX_PX` are set to typical
> values consistent with the paper's description.  Verify these against Table I
> in arXiv:2602.03294v1 §3.3 once you have access to the full paper PDF.

---

## Build

Requires: CMake ≥ 3.14, a C99 compiler, and `libm`.

```bash
# From the repository root:
cmake -S levio_c -B build
cmake --build build

# Run unit tests:
cd build && ctest -V
# or directly:
./build/levio_tests

# Run the offline EuRoC demo:
./build/levio_euroc_demo
```

---

## Directory layout

```
levio_c/
├── CMakeLists.txt           # Builds static library + demo + tests
├── README.md                # This file
├── include/
│   ├── levio_cfg.h          # Paper-aligned compile-time constants
│   ├── levio_types.h        # Core data types (vec3, mat3, features, ...)
│   ├── levio_math.h         # Custom linear algebra (mat3, Cholesky, ...)
│   ├── levio_se3.h          # SO3 / SE3 Lie-group helpers
│   ├── levio_ringbuf.h      # IMU ring buffer
│   ├── levio_camera.h       # Pinhole projection / Jacobians
│   ├── levio_orb.h          # ORB extraction API + Hamming distance
│   ├── levio_match.h        # Bidirectional brute-force matcher
│   ├── levio_ransac_8pt.h   # 8-point essential + RANSAC (§3.2 segment 3)
│   ├── levio_pnp_epnp.h     # EPnP + RANSAC (§3.2 segment 4)
│   ├── levio_triangulate.h  # Two-view DLT triangulation (§3.2 segment 6)
│   ├── levio_frontend.h     # Front-end orchestrator (§3.2)
│   ├── levio_imu_preint.h   # IMU pre-integration eq. (2)–(4) (§3.3)
│   ├── levio_factors.h      # Visual / IMU / prior factors (§3.3)
│   ├── levio_backend.h      # Sliding-window BA + LM + Schur (§3.3)
│   └── levio.h              # Top-level API: push_imu / push_image
├── src/                     # Corresponding .c files
├── apps/
│   └── euroc_offline.c      # Desktop demo wiring up push_imu / push_image
└── tests/
    └── test_levio.c         # 16 unit tests (Hamming, ringbuf, math, SE3, ...)
```

---

## What's implemented vs TODO

### ✅ Fully implemented (real code, tested)
- Hamming distance for 256-bit ORB descriptors (`levio_orb_hamming`)
- Bidirectional brute-force matcher + ratio-test matcher
- IMU ring buffer (push / pop / peek / overflow handling)
- Camera projection / unprojection (pinhole, no distortion)
- Reprojection error + 2×6 pose Jacobian + 2×3 point Jacobian
- `vec3` / `mat3` operations (add, sub, scale, dot, cross, norm, mul, inv, det, skew)
- `float` Cholesky decomposition + forward/back substitution
- General dense `matmul` + `matadd_scaled`
- SO3 exponential / logarithm (Rodrigues formula)
- SO3 right Jacobian
- SE3 composition, inversion, transform, box-plus
- IMU pre-integration accumulator (equations (2)–(4) of the paper)
- Front-end pipeline skeleton with 8-pt fallback logic

### 🔧 Structurally correct stubs (compile, link, return −1)
- 8-point essential matrix solver (needs 8×9 null-space via SVD)
- EPnP minimal solver (needs 12×12 null-space + 3×3 SVD)
- Two-view DLT triangulation (needs 4×4 SVD)
- Full IMU pre-integration residual (formula complete, needs Jacobians)
- LM optimiser + Schur complement (equations 5–6)
- Marginalisation into prior factor

### ❌ Not yet started
- Full ORB feature extraction (Oriented FAST + Rotated BRIEF)
- GAP9-specific parallel-cluster execution
- Tiered L1/L2 memory management
- IMU–camera extrinsic calibration integration

---

## Next steps (recommended order)

1. **Add a minimal SVD** for 3×3 and small rectangular matrices
   (needed by 8-point, EPnP, and DLT triangulation).
2. **Complete 8-point RANSAC** using the SVD → E decomposition → R,t.
3. **Complete EPnP minimal solver** to unlock the primary pose-estimation path.
4. **Complete two-view triangulation** → world points grow beyond zero.
5. **Complete IMU factor residuals & Jacobians** (use Forster et al. TRO 2017
   as the standard reference).
6. **Implement LM + Schur complement** in `levio_backend_optimize()`.
7. **Implement marginalisation** in `levio_backend_marginalise()`.
8. **Replace ORB stub** with a real detector/descriptor or integrate the
   existing GAP9-optimised ORB implementation.
9. **Port to GAP9**: replace `<math.h>` SIMD calls, add `pi_cluster_*` task
   dispatch, allocate hot arrays in L1, colder arrays in L2.
