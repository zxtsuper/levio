# cmake/toolchains/gap9-pmsis.cmake
#
# CMake toolchain file for GAP9 (GreenWaves Technologies) using the GAP SDK
# and PMSIS (Parallel ultra-low-Power Micro-controller SDK) runtime.
#
# Usage:
#   cmake -DCMAKE_TOOLCHAIN_FILE=<path>/cmake/toolchains/gap9-pmsis.cmake \
#         -DGAP_SDK_HOME=/path/to/gap_sdk \
#         -DCMAKE_BUILD_TYPE=Release \
#         ..
#
# Required environment / CMake variables:
#   GAP_SDK_HOME   – root of the GAP SDK installation
#                    (also honoured as an environment variable)
#   PMSIS_OS       – PMSIS OS backend: "freertos" (default) or "pulpos"
#
# After installing the GAP SDK and sourcing its configuration script:
#   source <gap_sdk_root>/configs/gap9_evk.sh
# set GAP_SDK_HOME to $GAPY_OPENOCD_CABLE and build with CMake as above.

cmake_minimum_required(VERSION 3.14)

# Prevent CMake from trying to run executables during configuration
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR riscv)

# -----------------------------------------------------------------------
# Resolve GAP SDK location
# -----------------------------------------------------------------------
if(NOT GAP_SDK_HOME)
    if(DEFINED ENV{GAP_SDK_HOME})
        set(GAP_SDK_HOME $ENV{GAP_SDK_HOME})
    else()
        message(FATAL_ERROR
            "GAP_SDK_HOME is not set. "
            "Please either:\n"
            "  (a) source <gap_sdk>/configs/gap9_evk.sh, or\n"
            "  (b) pass -DGAP_SDK_HOME=<path> to cmake.")
    endif()
endif()

# -----------------------------------------------------------------------
# PMSIS OS selection
# -----------------------------------------------------------------------
if(NOT PMSIS_OS)
    set(PMSIS_OS "freertos")
endif()

message(STATUS "GAP SDK: ${GAP_SDK_HOME}")
message(STATUS "PMSIS OS: ${PMSIS_OS}")

# -----------------------------------------------------------------------
# Toolchain binaries (RISC-V cross-compiler from the GAP SDK)
# -----------------------------------------------------------------------
set(TOOLCHAIN_PREFIX "${GAP_SDK_HOME}/tools/gap_riscv_toolchain/bin/riscv32-unknown-elf")

set(CMAKE_C_COMPILER   "${TOOLCHAIN_PREFIX}-gcc")
set(CMAKE_CXX_COMPILER "${TOOLCHAIN_PREFIX}-g++")
set(CMAKE_ASM_COMPILER "${TOOLCHAIN_PREFIX}-gcc")
set(CMAKE_OBJCOPY      "${TOOLCHAIN_PREFIX}-objcopy")
set(CMAKE_SIZE         "${TOOLCHAIN_PREFIX}-size")

# Avoid compiler test failures in cross-compile mode
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# -----------------------------------------------------------------------
# GAP9 hardware flags (RV32IMCXgap2, 9-stage pipeline)
# -----------------------------------------------------------------------
set(GAP9_ARCH_FLAGS
    "-march=rv32imcxgap2"
    "-mPE=8"           # 8 Processing Elements in the cluster
    "-mFC=1"           # Fabric Controller
    "-D__gap9__=1"
    "-D__riscv__=1"
    "-DCONFIG_GAP9=1"
)

string(JOIN " " GAP9_ARCH_FLAGS_STR ${GAP9_ARCH_FLAGS})

# -----------------------------------------------------------------------
# PMSIS include directories
# -----------------------------------------------------------------------
set(PMSIS_INC_DIRS
    "${GAP_SDK_HOME}/rtos/pmsis/pmsis_api/include"
    "${GAP_SDK_HOME}/rtos/pmsis/pmsis_bsp/include"
    "${GAP_SDK_HOME}/rtos/${PMSIS_OS}/pmsis_implem/include"
    "${GAP_SDK_HOME}/rtos/${PMSIS_OS}/include"
    "${GAP_SDK_HOME}/rtos/pmsis/pmsis_api/include/pmsis"
)

# -----------------------------------------------------------------------
# Compiler/linker flags
# -----------------------------------------------------------------------
set(CMAKE_C_FLAGS_INIT
    "${GAP9_ARCH_FLAGS_STR} -O2 -g -ffunction-sections -fdata-sections"
    CACHE STRING "C compiler flags" FORCE)

set(CMAKE_EXE_LINKER_FLAGS_INIT
    "-Wl,--gc-sections -nostartfiles"
    CACHE STRING "Linker flags" FORCE)

# -----------------------------------------------------------------------
# Find / export PMSIS libraries for use in CMakeLists.txt
# -----------------------------------------------------------------------
find_library(PMSIS_LIB
    NAMES pmsis
    HINTS "${GAP_SDK_HOME}/rtos/${PMSIS_OS}/lib"
          "${GAP_SDK_HOME}/libs"
    NO_DEFAULT_PATH
)

find_library(PMSIS_BSP_LIB
    NAMES bsp
    HINTS "${GAP_SDK_HOME}/rtos/pmsis/pmsis_bsp/lib"
          "${GAP_SDK_HOME}/libs"
    NO_DEFAULT_PATH
)

# Expose as imported targets so CMakeLists files can simply link them
if(PMSIS_LIB)
    add_library(pmsis STATIC IMPORTED GLOBAL)
    set_target_properties(pmsis PROPERTIES IMPORTED_LOCATION "${PMSIS_LIB}")
endif()

if(PMSIS_BSP_LIB)
    add_library(pmsis_bsp STATIC IMPORTED GLOBAL)
    set_target_properties(pmsis_bsp PROPERTIES IMPORTED_LOCATION "${PMSIS_BSP_LIB}")
endif()

# Helper function: add GAP9 PMSIS include dirs to a target
function(target_add_pmsis_includes TARGET)
    target_include_directories(${TARGET} PRIVATE ${PMSIS_INC_DIRS})
endfunction()
