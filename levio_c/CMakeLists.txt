cmake_minimum_required(VERSION 3.14)
project(levio_c C)

# Use C99 for embedded compatibility
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# --------------------------------------------------------------------------
# Target selection
# --------------------------------------------------------------------------
# Set LEVIO_TARGET=GAP9 to build a GAP9/PMSIS flavour of levio_c.
# Alternatively, configure the gap9/ subdirectory directly with the GAP9
# CMake toolchain (see cmake/toolchains/gap9-pmsis.cmake).
set(LEVIO_TARGET "DESKTOP" CACHE STRING
    "Target platform: DESKTOP (default) or GAP9")
set_property(CACHE LEVIO_TARGET PROPERTY STRINGS DESKTOP GAP9)

# --------------------------------------------------------------------------
# Cluster-parallel option (applies to both DESKTOP sim and GAP9)
# --------------------------------------------------------------------------
option(LEVIO_USE_CLUSTER "Enable GAP9 cluster parallel kernels (or desktop sim)" OFF)

# Compiler warnings (reasonable set for embedded-style C)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

# --------------------------------------------------------------------------
# Source files for the static library
# --------------------------------------------------------------------------
set(LEVIO_SOURCES
    src/levio_math.c
    src/levio_se3.c
    src/levio_ringbuf.c
    src/levio_camera.c
    src/levio_orb.c
    src/levio_match.c
    src/levio_ransac_8pt.c
    src/levio_pnp_epnp.c
    src/levio_triangulate.c
    src/levio_frontend.c
    src/levio_imu_preint.c
    src/levio_factors.c
    src/levio_backend.c
    src/levio.c
)

# --------------------------------------------------------------------------
# Static library: levio
# --------------------------------------------------------------------------
add_library(levio STATIC ${LEVIO_SOURCES})

target_include_directories(levio PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(levio PUBLIC m)

if(LEVIO_USE_CLUSTER)
    target_compile_definitions(levio PUBLIC LEVIO_USE_GAP9_CLUSTER=1)
endif()

# --------------------------------------------------------------------------
# GAP9 target: delegate to gap9/ subdirectory
# --------------------------------------------------------------------------
if(LEVIO_TARGET STREQUAL "GAP9")
    message(STATUS "LEVIO_TARGET=GAP9: including gap9/ subdirectory")
    add_subdirectory(gap9)
else()
    # --------------------------------------------------------------------------
    # Desktop builds only
    # --------------------------------------------------------------------------

    # Example application: levio_euroc_demo
    add_executable(levio_euroc_demo apps/euroc_offline.c)
    target_link_libraries(levio_euroc_demo PRIVATE levio)

    # Unit tests: levio_tests
    enable_testing()

    add_executable(levio_tests tests/test_levio.c)
    target_link_libraries(levio_tests PRIVATE levio)

    add_test(NAME levio_unit_tests COMMAND levio_tests)
endif()
