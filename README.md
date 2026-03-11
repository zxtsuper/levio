# LEVIO: Lightweight Embedded Visual Inertial Odometry for Resource-Constrained Devices

> **Status:** Code will be released by **early March**  
> **Paper:** *LEVIO: Lightweight Embedded Visual Inertial Odometry for Resource-Constrained Devices*  
> (Accepted at IEEE Sensors Journal)

---

## 📄 Overview

This repository will contain the official implementation of **LEVIO**, a lightweight embedded Visual-Inertial Odometry (VIO) system designed for **resource-constrained devices** such as microcontrollers and ultra low-power Systems on Chip (SoCs).

The release will include:
- **An optimized implementation for the GAP9 SoC** by [GreenWaves Technologies](https://github.com/GreenWaves-Technologies)
- **A golden model** used for algorithm development and evaluation in the accompanying paper

**LEVIO** is designed with:
- **Minimal memory footprint**
- **Low computational overhead** leveraging parallelization
- **Real-time performance** on ultra-low power embedded devices

Our approach targets applications where traditional VIO solutions are too heavy to run, enabling precise motion estimation for **IoT**, **wearable devices**, **mobile robotics**, and **AR/VR on embedded systems**.

---

## 📅 Availability

The complete source code and documentation will be made publicly available in **early March**.

---

## 🛠️ Initial Embedded-C Skeleton

An initial **embedded-C skeleton** (GAP9-style) is available in [`levio_c/`](levio_c/).

This skeleton implements the LEVIO architecture from arXiv:2602.03294v1 with:
- Paper-aligned default values (QQVGA 160×120, 700 features, 1000 world points, etc.)
- Fully working: Hamming distance, bidirectional brute-force matching, IMU ring buffer,
  camera projection/unprojection, SO3/SE3 Lie-group math, Cholesky solver.
- Structurally complete stubs for: 8-point RANSAC, EPnP RANSAC, DLT triangulation,
  IMU pre-integration, LM optimiser, Schur complement.
- Builds on desktop via CMake; 16 unit tests all pass.

See [`levio_c/README.md`](levio_c/README.md) for build instructions and a
detailed map of each module to the corresponding paper section.
