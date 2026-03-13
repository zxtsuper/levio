cmake_minimum_required(VERSION 3.14)
project(levio_c C)

# Use C99 for embedded compatibility
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

find_package(ZLIB REQUIRED)

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

# --------------------------------------------------------------------------
# Example application: levio_euroc_demo
# --------------------------------------------------------------------------
add_executable(levio_euroc_demo
    apps/euroc_offline.c
    apps/euroc_dataset.c
)
target_include_directories(levio_euroc_demo PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/apps
)
target_link_libraries(levio_euroc_demo PRIVATE levio ZLIB::ZLIB)

# --------------------------------------------------------------------------
# Unit tests: levio_tests
# --------------------------------------------------------------------------
enable_testing()

add_executable(levio_tests
    tests/test_levio.c
    apps/euroc_dataset.c
)
target_include_directories(levio_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/apps
)
target_link_libraries(levio_tests PRIVATE levio ZLIB::ZLIB)

add_test(NAME levio_unit_tests COMMAND levio_tests)
