# CMakeLists.txt
cmake_minimum_required(VERSION 3.22)

project(
    BBTC
    VERSION 0.1.0
    DESCRIPTION "BMAGS' Ballistic Trajectory Calculator"
    HOMEPAGE_URL "https://github.com/BMAGS6/BBTC"
    LANGUAGES C
)

set(BBTC_BUILD_TESTS_DEFAULT OFF)

if(PROJECT_IS_TOP_LEVEL)
    set(BBTC_BUILD_TESTS_DEFAULT ON)
endif()

option(
    BBTC_BUILD_TESTS
    "Build the BBTC test suite"
    "${BBTC_BUILD_TESTS_DEFAULT}"
)

unset(BBTC_BUILD_TESTS_DEFAULT)

add_library(
    bbtc
    STATIC
    include/bbtc/bbtc.h
    include/bbtc/diagnostics.h
    include/bbtc/internal_ballistics.h
    include/bbtc/internal_ballistics/geometry.h
    include/bbtc/internal_ballistics/loading_state.h
    include/bbtc/internal_ballistics/projectile.h
    include/bbtc/internal_ballistics/propellant_charge.h
    include/bbtc/precision.h
    include/bbtc/status.h
    src/diagnostics.c
    src/internal/geometry.c
    src/internal/loading_state.c
    src/internal/projectile.c
    src/internal/propellant_charge.c
    src/precision.c
    src/status.c
)

add_library(bbtc::bbtc ALIAS bbtc)

target_include_directories(
    bbtc
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>"
)

target_compile_features(
    bbtc
    PUBLIC
        c_std_23
)

set_target_properties(
    bbtc
    PROPERTIES
        C_EXTENSIONS OFF
        C_STANDARD 23
        C_STANDARD_REQUIRED ON
)

if(BBTC_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
