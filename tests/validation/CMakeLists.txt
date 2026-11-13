# Deliberate failure if developer tests were requested without cmocka.
find_package(PkgConfig REQUIRED)
pkg_check_modules(CMOCKA REQUIRED IMPORTED_TARGET cmocka)

# VV0.1-B2b.3a: first independent M-01..M-04 mathematical reference matrix.
# The runner is intentionally a developer-only cmocka executable. It obtains
# observations through the production-facing B2 adapter, and its three EXACT
# decisions per case are delegated to the separately linkable B1 comparator.
# No production target or public BBTC API depends on this runner.
add_executable(bbtc_vv_test_virial_reference_matrix test_virial_reference_matrix.c)
target_link_libraries(bbtc_vv_test_virial_reference_matrix PRIVATE
    bbtc_vv_virial_adapter bbtc_vv_virial_profile bbtc_vv_virial_control
    bbtc_vv_comparison PkgConfig::CMOCKA)
target_compile_features(bbtc_vv_test_virial_reference_matrix PRIVATE c_std_23)
set_target_properties(bbtc_vv_test_virial_reference_matrix PROPERTIES
    C_STANDARD 23 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(bbtc_vv_test_virial_reference_matrix PRIVATE
        -frounding-math -ffp-contract=off -fno-fast-math)
endif()
add_test(NAME bbtc.vv.virial_reference_matrix
    COMMAND bbtc_vv_test_virial_reference_matrix)

# VV0.1-B2b.4a Stage 2: independently challenge each native EXACT reference
# with both adjacent floating-point values and signed-zero equivalence.
# This test is developer-only and MUST NOT be linked from bbtc::bbtc.
add_executable(bbtc_vv_test_virial_exact_neighbors test_virial_exact_neighbors.c)
target_link_libraries(bbtc_vv_test_virial_exact_neighbors PRIVATE
    bbtc_vv_comparison bbtc_vv_virial_profile bbtc_vv_virial_control
    PkgConfig::CMOCKA)
target_compile_features(bbtc_vv_test_virial_exact_neighbors PRIVATE c_std_23)
set_target_properties(bbtc_vv_test_virial_exact_neighbors PROPERTIES
    C_STANDARD 23 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(bbtc_vv_test_virial_exact_neighbors PRIVATE
        -frounding-math -ffp-contract=off -fno-fast-math)
endif()
add_test(NAME bbtc.vv.virial_exact_neighbors
    COMMAND bbtc_vv_test_virial_exact_neighbors)

# VV0.1-B2b.4b Stage 2: real x87/MXCSR changes in isolated POSIX children.
# The child restores both control words and communicates observations by pipe.
# This is a developer-only host-specific test; bbtc::bbtc never links it.
if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND
   CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$" AND
   CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    add_executable(bbtc_vv_test_virial_live_fp_rejection test_virial_live_fp_rejection.c)
    target_link_libraries(bbtc_vv_test_virial_live_fp_rejection PRIVATE
        bbtc_vv_virial_control bbtc_vv_virial_profile PkgConfig::CMOCKA)
    target_compile_features(bbtc_vv_test_virial_live_fp_rejection PRIVATE c_std_23)
    set_target_properties(bbtc_vv_test_virial_live_fp_rejection PROPERTIES
        C_STANDARD 23 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)
    target_compile_options(bbtc_vv_test_virial_live_fp_rejection PRIVATE
        -frounding-math -ffp-contract=off -fno-fast-math)
    add_test(NAME bbtc.vv.virial_live_fp_rejection
        COMMAND bbtc_vv_test_virial_live_fp_rejection)
endif()

# VV0.1-B2b.3c.2: reject inconsistent host ABI, fenv, or production metadata.
# The profile module can return only SCREENED_UNQUALIFIED or UNSUPPORTED.
# These checks are separate from the 36 numerical reference findings.
add_executable(bbtc_vv_test_virial_fp_profile test_virial_fp_profile.c)
target_link_libraries(bbtc_vv_test_virial_fp_profile PRIVATE
    bbtc_vv_virial_profile PkgConfig::CMOCKA)
target_compile_features(bbtc_vv_test_virial_fp_profile PRIVATE c_std_23)
set_target_properties(bbtc_vv_test_virial_fp_profile PROPERTIES
    C_STANDARD 23 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(bbtc_vv_test_virial_fp_profile PRIVATE
        -frounding-math -ffp-contract=off -fno-fast-math)
endif()
add_test(NAME bbtc.vv.virial_fp_profile COMMAND bbtc_vv_test_virial_fp_profile)

# VV0.1-B2b.3c.3a: actual x87/MXCSR control capture with adverse synthetic
# register-word controls. This executable never mutates live FP registers.
# A passing test is screening evidence and cannot qualify production code.
add_executable(bbtc_vv_test_virial_fp_control test_virial_fp_control.c)
target_link_libraries(bbtc_vv_test_virial_fp_control PRIVATE
    bbtc_vv_virial_control PkgConfig::CMOCKA)
target_compile_features(bbtc_vv_test_virial_fp_control PRIVATE c_std_23)
set_target_properties(bbtc_vv_test_virial_fp_control PROPERTIES
    C_STANDARD 23 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(bbtc_vv_test_virial_fp_control PRIVATE
        -frounding-math -ffp-contract=off -fno-fast-math)
endif()
add_test(NAME bbtc.vv.virial_fp_control COMMAND bbtc_vv_test_virial_fp_control)

# VV0.1-B2b: observation-adapter smoke tests, distinct from mathematical
# reference qualification and deliberately independent of the B1 comparator.
# The matrix and its B2b.3b negative controls are tested above.
add_executable(bbtc_vv_test_virial_adapter test_virial_adapter.c)
target_link_libraries(bbtc_vv_test_virial_adapter PRIVATE
    bbtc_vv_virial_adapter PkgConfig::CMOCKA)
target_compile_features(bbtc_vv_test_virial_adapter PRIVATE c_std_23)
set_target_properties(bbtc_vv_test_virial_adapter PROPERTIES
    C_STANDARD 23 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(bbtc_vv_test_virial_adapter PRIVATE
        -frounding-math -ffp-contract=off -fno-fast-math)
endif()
add_test(NAME bbtc.vv.virial_adapter COMMAND bbtc_vv_test_virial_adapter)

# VV0.1-B2b.2b: contract/status checks are distinct from numerical
# reference comparisons and remain in their own developer-only CTest entry.
add_executable(bbtc_vv_test_virial_adapter_contract test_virial_adapter_contract.c)
target_link_libraries(bbtc_vv_test_virial_adapter_contract PRIVATE
    bbtc_vv_virial_adapter PkgConfig::CMOCKA)
target_compile_features(bbtc_vv_test_virial_adapter_contract PRIVATE c_std_23)
set_target_properties(bbtc_vv_test_virial_adapter_contract PROPERTIES
    C_STANDARD 23 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(bbtc_vv_test_virial_adapter_contract PRIVATE
        -frounding-math -ffp-contract=off -fno-fast-math)
endif()
add_test(NAME bbtc.vv.virial_adapter.contract COMMAND bbtc_vv_test_virial_adapter_contract)

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

# VV0.1-B1b.3c-2: separately prove the exact-rational-to-native conversion.
# The cmocka suite is an ordinary VVUQ developer test. Python remains opt-in.
add_executable(bbtc_vv_test_exact_round test_exact_round.c)
target_link_libraries(bbtc_vv_test_exact_round PRIVATE
    bbtc_vv_exact_round PkgConfig::CMOCKA)
target_compile_features(bbtc_vv_test_exact_round PRIVATE c_std_23)
set_target_properties(bbtc_vv_test_exact_round PROPERTIES
    C_STANDARD 23 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(bbtc_vv_test_exact_round PRIVATE
        -frounding-math -ffp-contract=off -fno-fast-math)
endif()
add_test(NAME bbtc.vv.exact_round COMMAND bbtc_vv_test_exact_round)

set(BBTC_VVUQ_ROUND_ORACLE_CASES 2000 CACHE STRING
    "Seeded exact-rational rounding cases per scalar family for opt-in oracle")
if(BBTC_VVUQ_ORACLE_TESTS)
    add_executable(bbtc_vv_round_oracle_driver exact_round_oracle_driver.c)
    target_link_libraries(bbtc_vv_round_oracle_driver PRIVATE bbtc_vv_exact_round)
    target_compile_features(bbtc_vv_round_oracle_driver PRIVATE c_std_23)
    set_target_properties(bbtc_vv_round_oracle_driver PROPERTIES
        C_STANDARD 23 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)
    if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(bbtc_vv_round_oracle_driver PRIVATE
            -frounding-math -ffp-contract=off -fno-fast-math)
    endif()
    add_test(NAME bbtc.vv.exact_round.oracle
        COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/test_exact_round_oracle.py"
            --driver "$<TARGET_FILE:bbtc_vv_round_oracle_driver>"
            --per-family "${BBTC_VVUQ_ROUND_ORACLE_CASES}"
            --seed 0xC023C2)
    set_tests_properties(bbtc.vv.exact_round.oracle PROPERTIES TIMEOUT 180)
endif()

# VV0.1-B1b.3c-3: the first complete, native-family comparison API. All
# decisions remain governed by the original exact dyadic kernel; diagnostics
# are independently constructed and rounded. No production BBTC dependency.
# B1b.3d will add a separately implemented end-to-end rational oracle.
add_executable(bbtc_vv_test_comparison_api test_comparison_api.c)
target_link_libraries(bbtc_vv_test_comparison_api PRIVATE
    bbtc_vv_comparison PkgConfig::CMOCKA)
target_compile_features(bbtc_vv_test_comparison_api PRIVATE c_std_23)
set_target_properties(bbtc_vv_test_comparison_api PROPERTIES
    C_STANDARD 23 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(bbtc_vv_test_comparison_api PRIVATE
        -frounding-math -ffp-contract=off -fno-fast-math)
endif()
add_test(NAME bbtc.vv.comparison_api COMMAND bbtc_vv_test_comparison_api)

# VV0.1-B1b.3d-1: black-box differential acceptance of the COMPLETE B1a API.
# The Python reference is written independently of the three C numerical
# engines, including validation precedence and full native metric rounding.
# Keep this behind the existing explicit oracle opt-in: a normal BBTC build
# MUST NOT acquire Python or the test-driver as a production dependency.
set(BBTC_VVUQ_API_ORACLE_CASES 2000 CACHE STRING
    "Seeded end-to-end comparison API cases per native scalar family")
if(BBTC_VVUQ_ORACLE_TESTS)
    add_executable(bbtc_vv_comparison_api_oracle_driver
        comparison_api_oracle_driver.c)
    target_link_libraries(bbtc_vv_comparison_api_oracle_driver PRIVATE
        bbtc_vv_comparison)
    target_compile_features(bbtc_vv_comparison_api_oracle_driver PRIVATE c_std_23)
    set_target_properties(bbtc_vv_comparison_api_oracle_driver PROPERTIES
        C_STANDARD 23 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)
    if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(bbtc_vv_comparison_api_oracle_driver PRIVATE
            -frounding-math -ffp-contract=off -fno-fast-math)
    endif()
    add_test(NAME bbtc.vv.comparison_api.oracle
        COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/test_comparison_api_oracle.py"
            --driver "$<TARGET_FILE:bbtc_vv_comparison_api_oracle_driver>"
            --per-family "${BBTC_VVUQ_API_ORACLE_CASES}"
            --seed 0xB1B3D)
    set_tests_properties(bbtc.vv.comparison_api.oracle PROPERTIES TIMEOUT 300)
endif()
