# levio_c/gap9/CMakeLists.txt
#
# Build entry for the GAP9 (PMSIS) demo.
#
# Standalone usage (cross-compile):
#   mkdir build_gap9 && cd build_gap9
#   cmake -DCMAKE_TOOLCHAIN_FILE=../../cmake/toolchains/gap9-pmsis.cmake \
#         -DGAP_SDK_HOME=$GAP_SDK_HOME \
#         -DLEVIO_USE_CLUSTER=ON \
#         -DCMAKE_BUILD_TYPE=Release \
#         ../levio_c/gap9
#
# Desktop simulation (no GAP SDK, single-threaded fallback):
#   mkdir build_gap9_sim && cd build_gap9_sim
#   cmake -DLEVIO_USE_CLUSTER=OFF \
#         -DCMAKE_BUILD_TYPE=Debug \
#         ../levio_c/gap9
#   make && ./levio_gap9_demo

cmake_minimum_required(VERSION 3.14)
project(levio_gap9 C)

# -----------------------------------------------------------------------
# Options
# -----------------------------------------------------------------------
option(LEVIO_USE_CLUSTER "Enable GAP9 cluster parallel kernels" OFF)

# -----------------------------------------------------------------------
# levio core library path (relative to this CMakeLists.txt)
# -----------------------------------------------------------------------
set(LEVIO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/..")

# Use levio_c sources directly (avoids building the desktop library first)
file(GLOB LEVIO_SRCS "${LEVIO_ROOT}/src/*.c")

add_library(levio_core STATIC ${LEVIO_SRCS})
target_include_directories(levio_core PUBLIC "${LEVIO_ROOT}/include")
target_compile_features(levio_core PRIVATE c_std_99)
target_link_libraries(levio_core PUBLIC m)

if(LEVIO_USE_CLUSTER)
    target_compile_definitions(levio_core PUBLIC LEVIO_USE_GAP9_CLUSTER=1)
endif()

# -----------------------------------------------------------------------
# Cluster kernel library
# -----------------------------------------------------------------------
add_library(levio_kernels STATIC
    "${CMAKE_CURRENT_SOURCE_DIR}/kernels/levio_kernels.c"
)
target_include_directories(levio_kernels PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/kernels"
    "${LEVIO_ROOT}/include"
)
target_link_libraries(levio_kernels PUBLIC levio_core)
if(LEVIO_USE_CLUSTER)
    target_compile_definitions(levio_kernels PUBLIC LEVIO_USE_GAP9_CLUSTER=1)
    # Add PMSIS include paths if available
    if(COMMAND target_add_pmsis_includes)
        target_add_pmsis_includes(levio_kernels)
        target_add_pmsis_includes(levio_core)
    endif()
endif()

# -----------------------------------------------------------------------
# GAP9 demo executable
# -----------------------------------------------------------------------
add_executable(levio_gap9_demo
    "${CMAKE_CURRENT_SOURCE_DIR}/main.c"
)
target_include_directories(levio_gap9_demo PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/kernels"
    "${LEVIO_ROOT}/include"
)
target_link_libraries(levio_gap9_demo PRIVATE levio_core levio_kernels m)

if(LEVIO_USE_CLUSTER)
    target_compile_definitions(levio_gap9_demo PRIVATE LEVIO_USE_GAP9_CLUSTER=1)
    if(COMMAND target_add_pmsis_includes)
        target_add_pmsis_includes(levio_gap9_demo)
    endif()
    # Link PMSIS libraries if found by the toolchain file
    if(TARGET pmsis)
        target_link_libraries(levio_gap9_demo PRIVATE pmsis)
    endif()
    if(TARGET pmsis_bsp)
        target_link_libraries(levio_gap9_demo PRIVATE pmsis_bsp)
    endif()
endif()

set_target_properties(levio_gap9_demo PROPERTIES
    OUTPUT_NAME "levio_gap9_demo"
    C_STANDARD 99
)

# -----------------------------------------------------------------------
# Summary
# -----------------------------------------------------------------------
message(STATUS "LEVIO GAP9 demo:")
message(STATUS "  Cluster parallel: ${LEVIO_USE_CLUSTER}")
message(STATUS "  Image dims:       ${LEVIO_IMG_W}x${LEVIO_IMG_H} (from levio_cfg.h)")
