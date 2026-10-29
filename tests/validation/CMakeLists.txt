# Deliberate failure if developer tests were requested without cmocka.
find_package(PkgConfig REQUIRED)
pkg_check_modules(CMOCKA REQUIRED IMPORTED_TARGET cmocka)

add_executable(bbtc_vv_test_exact_dyadic test_exact_dyadic.c)
target_link_libraries(bbtc_vv_test_exact_dyadic PRIVATE
    bbtc_vv_exact_dyadic PkgConfig::CMOCKA)
target_compile_features(bbtc_vv_test_exact_dyadic PRIVATE c_std_23)
set_target_properties(bbtc_vv_test_exact_dyadic PROPERTIES
    C_STANDARD 23 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(bbtc_vv_test_exact_dyadic PRIVATE
        -frounding-math -ffp-contract=off -fno-fast-math)
endif()

add_test(NAME bbtc.vv.exact_dyadic COMMAND bbtc_vv_test_exact_dyadic)

# This is deliberately an *opt-in* differential oracle. The ordinary CTest
# suite needs only cmocka and the C compiler; it must not acquire a mandatory
# Python dependency. Enabling the oracle means failure to find Python is fatal,
# not a silently skipped check. The C driver links ONLY the VVUQ kernel.
set(BBTC_VVUQ_ORACLE_CASES 2000 CACHE STRING
    "Seeded random exact-dyadic cases per precision for CTest's optional oracle")

if(BBTC_VVUQ_ORACLE_TESTS)
    find_package(Python3 REQUIRED COMPONENTS Interpreter)

    add_executable(bbtc_vv_dyadic_oracle_driver dyadic_oracle_driver.c)
    target_link_libraries(bbtc_vv_dyadic_oracle_driver PRIVATE bbtc_vv_exact_dyadic)
    target_compile_features(bbtc_vv_dyadic_oracle_driver PRIVATE c_std_23)
    set_target_properties(bbtc_vv_dyadic_oracle_driver PROPERTIES
        C_STANDARD 23 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)
    if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(bbtc_vv_dyadic_oracle_driver PRIVATE
            -frounding-math -ffp-contract=off -fno-fast-math)
    endif()

    add_test(NAME bbtc.vv.exact_dyadic.oracle
        COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/test_exact_dyadic_oracle.py"
            --driver "$<TARGET_FILE:bbtc_vv_dyadic_oracle_driver>"
            --per-family "${BBTC_VVUQ_ORACLE_CASES}"
            --seed 0xBB7C2026)
    set_tests_properties(bbtc.vv.exact_dyadic.oracle PROPERTIES TIMEOUT 180)
endif()

# VV0.1-B1b.3b: test the deliberately *internal* decision-preparation layer.
# This target does not call, declare, or imply availability of the final
# metric-producing bbtc_vv_compare_* API.
add_executable(bbtc_vv_test_comparison_foundation test_comparison_foundation.c)
target_link_libraries(bbtc_vv_test_comparison_foundation PRIVATE
    bbtc_vv_comparison_foundation PkgConfig::CMOCKA)
target_compile_features(bbtc_vv_test_comparison_foundation PRIVATE c_std_23)
set_target_properties(bbtc_vv_test_comparison_foundation PROPERTIES
    C_STANDARD 23 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(bbtc_vv_test_comparison_foundation PRIVATE
        -frounding-math -ffp-contract=off -fno-fast-math)
endif()

add_test(NAME bbtc.vv.comparison_foundation
    COMMAND bbtc_vv_test_comparison_foundation)

# VV0.1-B1b.3c-1: the numerator/denominator output is an *exact* rational,
# not a rounded scalar diagnostic or the final bbtc_vv_compare_* interface.
add_executable(bbtc_vv_test_exact_metric test_exact_metric.c)
target_link_libraries(bbtc_vv_test_exact_metric PRIVATE
    bbtc_vv_exact_metric PkgConfig::CMOCKA)
target_compile_features(bbtc_vv_test_exact_metric PRIVATE c_std_23)
set_target_properties(bbtc_vv_test_exact_metric PROPERTIES
    C_STANDARD 23 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(bbtc_vv_test_exact_metric PRIVATE
        -frounding-math -ffp-contract=off -fno-fast-math)
endif()
add_test(NAME bbtc.vv.exact_metric COMMAND bbtc_vv_test_exact_metric)

# Extend the *existing* explicit oracle opt-in; Python is never required by
# ordinary BBTC builds, and its absence fails loudly when requested.
set(BBTC_VVUQ_METRIC_ORACLE_CASES 2000 CACHE STRING
    "Random exact metric cases per scalar family for the optional oracle")
if(BBTC_VVUQ_ORACLE_TESTS)
    add_executable(bbtc_vv_metric_oracle_driver exact_metric_oracle_driver.c)
    target_link_libraries(bbtc_vv_metric_oracle_driver PRIVATE bbtc_vv_exact_metric)
    target_compile_features(bbtc_vv_metric_oracle_driver PRIVATE c_std_23)
    set_target_properties(bbtc_vv_metric_oracle_driver PROPERTIES
        C_STANDARD 23 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)
    if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(bbtc_vv_metric_oracle_driver PRIVATE
            -frounding-math -ffp-contract=off -fno-fast-math)
    endif()
    add_test(NAME bbtc.vv.exact_metric.oracle
        COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/test_exact_metric_oracle.py"
            --driver "$<TARGET_FILE:bbtc_vv_metric_oracle_driver>"
            --per-family "${BBTC_VVUQ_METRIC_ORACLE_CASES}"
            --seed 0xC023C1)
    set_tests_properties(bbtc.vv.exact_metric.oracle PROPERTIES TIMEOUT 180)
endif()
