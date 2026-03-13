# levio_c — Embedded-C Implementation of LEVIO

This directory contains the **embedded-C** implementation of the LEVIO
visual-inertial odometry system, targeting the **GAP9 SoC** by GreenWaves
Technologies with PMSIS cluster parallel execution.

The code maps to the algorithm described in:

> **LEVIO: Lightweight Embedded Visual Inertial Odometry for Resource-Constrained Devices**  
> arXiv:2602.03294v1 — <https://arxiv.org/abs/2602.03294>

---

## Status

All major algorithms are **implemented** (not stubs).  The following
table summarises what is complete and what is ongoing.

---

## How this maps to paper sections

### §3.2 — Visual Odometry front-end

| Segment | Paper description | Module | Status |
|---------|------------------|--------|--------|
| 1 | ORB extraction (Oriented FAST + Rotated BRIEF) | `levio_orb.{h,c}` | ✅ |
| 2 | Bidirectional brute-force matching (Hamming) | `levio_match.{h,c}` | ✅ |
| 3 | 8-point algorithm + RANSAC (normalised; Sampson distance) | `levio_ransac_8pt.{h,c}` | ✅ |
| 4 | Iterative GN-based PnP + RANSAC (4-point minimal solver) | `levio_pnp_epnp.{h,c}` | ✅ |
| 5 | Keyframe selection (parallax threshold 15 px) | `levio_frontend.{h,c}` | ✅ |
| 6 | Two-view triangulation (midpoint method) | `levio_triangulate.{h,c}` | ✅ |

### §3.3 — Tight-coupled BA + IMU

| Component | Module | Status |
|-----------|--------|--------|
| IMU ring buffer (FIFO, 256 samples) | `levio_ringbuf.{h,c}` | ✅ |
| Pre-integration, equations (2)–(4) | `levio_imu_preint.{h,c}` | ✅ |
| IMU residual r_R / r_v / r_p | `levio_imu_preint.c` | ✅ |
| Visual reprojection factor + Jacobians | `levio_factors.{h,c}` | ✅ |
| Sliding window LM optimizer + Schur complement (eq. 5–6) | `levio_backend.c` | ✅ |
| Marginalisation (simplified drop) | `levio_backend.c` | ✅ basic |

### §3.4 — Embedded optimisations (GAP9 target)

| Optimisation | Status |
|-------------|--------|
| Image resolution QQVGA 160×120 | ✅ `LEVIO_IMG_W/H` in `levio_cfg.h` |
| Self-contained C99, no heap allocation | ✅ all structures statically allocated |
| Custom linear-algebra library | ✅ `levio_math.{h,c}`, `levio_se3.{h,c}` |
| Jacobi SVD (3×3) + symmetric eigensolver (up to 12×12) | ✅ |
| Schur complement for reduced system + Cholesky solve | ✅ |
| **Parallel cluster matching (GAP9 PE cores)** | ✅ `gap9/kernels/levio_kernels.{c,h}` |
| **Parallel RANSAC inlier counting (GAP9 PE cores)** | ✅ `gap9/kernels/levio_kernels.{c,h}` |
| Tiered L1/L2 memory usage | ❌ not yet |

---

## Default values (paper-aligned)

All constants live in `include/levio_cfg.h`:

| Constant | Value | Paper reference |
|----------|-------|-----------------|
| `LEVIO_IMG_W` / `LEVIO_IMG_H` | 160 / 120 | §3.4 "QQVGA" |
| `LEVIO_MAX_FRAME_FEATS` | 700 | §3.2 |
| `LEVIO_MAX_WORLD_POINTS` | 1000 | §3.2 |
| `LEVIO_PNP_MIN_INLIERS` | 25 | §3.2 |
| `LEVIO_WINDOW_SIZE` | 10 | §3.3 Table I |
| `LEVIO_KF_PARALLAX_PX` | 15.0 px | §3.2 |
| `LEVIO_RANSAC_MAX_ITER` | 100 | §3.2 |
| `LEVIO_RANSAC_REPROJ_THR` | 2.0 px | §3.2 |

---

## Desktop build

Requires: CMake ≥ 3.14, a C99 compiler, and `libm`.

```bash
# From the repository root:
cmake -S levio_c -B build
cmake --build build

# Run unit tests (23 tests):
cd build && ctest -V
# or directly:
./build/levio_tests

# Run the offline EuRoC demo:
./build/levio_euroc_demo
```

---

## GAP9 / PMSIS build

### Prerequisites

1. Install the [GAP SDK](https://github.com/GreenWaves-Technologies/gap_sdk)
   and set `GAP_SDK_HOME`:

   ```bash
   source <gap_sdk_root>/configs/gap9_evk.sh
   export GAP_SDK_HOME=$(dirname $(which riscv32-unknown-elf-gcc))/../..
   ```

2. Verify the toolchain is on `PATH`:

   ```bash
   riscv32-unknown-elf-gcc --version
   ```

### Cross-compile for GAP9 board

```bash
mkdir build_gap9 && cd build_gap9
cmake -DCMAKE_TOOLCHAIN_FILE=../cmake/toolchains/gap9-pmsis.cmake \
      -DGAP_SDK_HOME=$GAP_SDK_HOME \
      -DLEVIO_USE_CLUSTER=ON \
      -DCMAKE_BUILD_TYPE=Release \
      ../levio_c/gap9
make -j4
```

This produces `levio_gap9_demo`.

### Run on board

```bash
# Connect the GAP9 EVK board, then:
gapy --target=gap9_evk --exec-prepare --exec run
```

### Run on GVSOC simulator

```bash
gapy --target=gap9 --exec-prepare --exec run
```

### Desktop simulation (no GAP SDK required)

Use the fallback build to verify pipeline logic on a PC:

```bash
mkdir build_gap9_sim && cd build_gap9_sim
cmake -DLEVIO_USE_CLUSTER=OFF -DCMAKE_BUILD_TYPE=Debug \
      ../levio_c/gap9
make && ./levio_gap9_demo
```

---

## GAP9 FC / PE architecture

```
GAP9 SoC
├── Fabric Controller (FC) — single core, runs:
│   ├── levio_push_imu()          IMU ring buffer ingestion
│   ├── levio_orb_extract()        FAST corner detection + Rotated BRIEF
│   ├── levio_frontend_process()   Pose estimation (PnP / 8pt)
│   ├── levio_backend_optimize()   LM + Schur complement
│   └── levio_gap9_main()          Main loop / keyframe logic
│
└── Cluster (8 × PE cores) — spawned by FC via pi_cluster_send_task_to_cl()
    ├── levio_cl_match_kernel()    Parallel Hamming-distance descriptor matching
    │   └── Each PE handles a stripe of feats_a; ratio-test filtering
    └── levio_cl_inlier_kernel()   Parallel RANSAC inlier counting
        └── Each PE evaluates stride subset of point correspondences
```

### Enabling / disabling cluster parallelism

| CMake option | Effect |
|---|---|
| `-DLEVIO_USE_CLUSTER=ON` | Defines `LEVIO_USE_GAP9_CLUSTER=1`; uses `pi_cl_team_fork` |
| `-DLEVIO_USE_CLUSTER=OFF` (default) | Single-threaded fallback; cluster calls become no-ops |

The fallback is transparent: `levio_cl_match_ratio_fork()` and
`levio_cl_inlier_count_fork()` call the kernel function directly without
forking.  All desktop unit tests pass in either mode.

---

## Directory layout

```
levio_c/
├── CMakeLists.txt           # Desktop build; LEVIO_TARGET=GAP9 delegates to gap9/
├── README.md                # This file
├── include/                 # All API headers
│   ├── levio_cfg.h          # Paper-aligned compile-time constants
│   ├── levio_math.h         # Linear algebra: mat3, SVD, Jacobi eigensolver
│   ├── levio_se3.h          # SO3 / SE3 Lie-group helpers
│   ├── levio_orb.h          # ORB extraction + Hamming distance
│   ├── levio_ransac_8pt.h   # 8-point essential + RANSAC
│   ├── levio_pnp_epnp.h     # Iterative PnP + RANSAC
│   ├── levio_triangulate.h  # Midpoint triangulation
│   ├── levio_imu_preint.h   # IMU pre-integration + residual
│   ├── levio_factors.h      # Visual / IMU / prior factors
│   ├── levio_backend.h      # Sliding-window LM optimizer
│   └── levio.h              # Top-level API
├── src/                     # Implementation files
├── apps/
│   └── euroc_offline.c      # EuRoC MH01 offline demo
├── tests/
│   └── test_levio.c         # 23 unit tests (all pass)
└── gap9/                    # GAP9 / PMSIS target
    ├── CMakeLists.txt        # GAP9 standalone build entry
    ├── main.c                # FC entry: cluster open, pipeline loop
    └── kernels/
        ├── levio_kernels.h   # PE kernel API declarations
        └── levio_kernels.c   # Parallel matching + RANSAC inlier count
```

Also:
```
cmake/
└── toolchains/
    └── gap9-pmsis.cmake      # CMake toolchain for GAP9 cross-compilation
```
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
