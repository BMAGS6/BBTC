# Independent VVUQ-only library. No dependency on the production BBTC target.
add_library(bbtc_vv_exact_dyadic STATIC exact_dyadic.c exact_dyadic.h)

target_include_directories(bbtc_vv_exact_dyadic PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_features(bbtc_vv_exact_dyadic PUBLIC c_std_23)
set_target_properties(bbtc_vv_exact_dyadic PROPERTIES
    C_STANDARD 23 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)

# The proof assumes contraction-free arithmetic and correct observation of fenv.
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(bbtc_vv_exact_dyadic PRIVATE
        -frounding-math -ffp-contract=off -fno-fast-math)
endif()

if(UNIX)
    target_link_libraries(bbtc_vv_exact_dyadic PUBLIC m)
endif()

if(BBTC_VVUQ_BUILD_TESTS)
    enable_testing()
    add_subdirectory("${PROJECT_SOURCE_DIR}/tests/validation" "${PROJECT_BINARY_DIR}/vv-validation-tests")
endif()

# B1b.3b is intentionally a private decision-preparation foundation only.
# The contractual bbtc_vv_compare_* entry points are not exported or linked
# until B1b.3c supplies correctly rounded scalar diagnostic metrics.
# This library depends exclusively on the exact dyadic kernel, never bbtc::bbtc.
add_library(bbtc_vv_comparison_foundation STATIC
    comparison.c comparison.h comparison_internal.h)
target_link_libraries(bbtc_vv_comparison_foundation PUBLIC bbtc_vv_exact_dyadic)
target_include_directories(bbtc_vv_comparison_foundation PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_features(bbtc_vv_comparison_foundation PUBLIC c_std_23)
set_target_properties(bbtc_vv_comparison_foundation PROPERTIES
    C_STANDARD 23 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(bbtc_vv_comparison_foundation PRIVATE
        -frounding-math -ffp-contract=off -fno-fast-math)
endif()

# VV0.1-B1b.3c-1: exact mathematical error construction, deliberately without
# any output rounding or complete comparison entrypoints. Developer-only.
# The only dependency is the independently qualified exact-dyadic kernel.
add_library(bbtc_vv_exact_metric STATIC exact_metric.c exact_metric.h)
target_include_directories(bbtc_vv_exact_metric PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(bbtc_vv_exact_metric PUBLIC bbtc_vv_exact_dyadic)
target_compile_features(bbtc_vv_exact_metric PUBLIC c_std_23)
set_target_properties(bbtc_vv_exact_metric PROPERTIES
    C_STANDARD 23 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(bbtc_vv_exact_metric PRIVATE
        -frounding-math -ffp-contract=off -fno-fast-math)
endif()

# VV0.1-B1b.3c-2: integer-exact nearest-even conversion. Deliberately
# developer-internal: no final bbtc_vv_compare_* function is exported here.
# Consumers of this target can also obtain its exact_metric_t dependency.
add_library(bbtc_vv_exact_round STATIC exact_round.c exact_round.h)
target_link_libraries(bbtc_vv_exact_round PUBLIC bbtc_vv_exact_metric)
target_include_directories(bbtc_vv_exact_round PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_features(bbtc_vv_exact_round PUBLIC c_std_23)
set_target_properties(bbtc_vv_exact_round PROPERTIES
    C_STANDARD 23 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(bbtc_vv_exact_round PRIVATE
        -frounding-math -ffp-contract=off -fno-fast-math)
endif()
