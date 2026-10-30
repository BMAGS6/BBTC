#!/usr/bin/env bash
# VV0.1-B1b.3d-2 -- reproducible candidate acceptance-evidence capture.
#
# PURPOSE
#   Collect the host's compiler/format profile, verify all existing B1 CTest
#   suites with both strict optimized compilers and an instrumented Clang
#   build, check actual developer API symbols, and prove that the production
#   configuration does not register or link the VVUQ developer library.
#
# BOUNDARY
#   Passing this script is NOT formal mathematical proof, physical model
#   validation, or automatic B1 acceptance. A signed-off contract audit is
#   required. The script deliberately does not edit production source.
#
# USAGE
#   bash tools/bbtc_vv/acceptance_capture.sh 2>&1 | tee \
#       "$HOME/Downloads/junk/vv0_1b3d2_acceptance_evidence.txt"
#
#   With a pipeline, ensure the invoking shell uses `set -o pipefail` to make
#   the exit status reflect failures in this script rather than only tee.
set -euo pipefail

readonly root="$(git rev-parse --show-toplevel)"
cd "$root"

readonly audit_build="build/vv0_1b3d2_acceptance"
readonly probe="tools/bbtc_vv/platform_capability_probe.c"

if [[ ! -f "$probe" ]]; then
    printf 'ERROR: Missing platform probe: %s\n' "$probe" >&2
    exit 1
fi

for tool in gcc clang cmake ctest ninja nm python3 pkg-config; do
    if ! command -v "$tool" >/dev/null 2>&1; then
        printf 'ERROR: Required tool not found: %s\n' "$tool" >&2
        exit 1
    fi
done

printf '\n=== SOURCE AND TOOLCHAIN IDENTITY ===\n'
date -u '+UTC=%Y-%m-%dT%H:%M:%SZ'
uname -a
printf 'ROOT=%s\n' "$root"
git rev-parse HEAD
git branch --show-current
git status --short --branch
gcc --version | sed -n '1p'
clang --version | sed -n '1p'
cmake --version | sed -n '1p'
ninja --version
python3 --version
pkg-config --modversion cmocka

# A13/A14: the reference program must fail on deliberately corrupted protocol
# replies. This tests oracle validation logic itself without calling the C
# kernels. It is part of acceptance capture, not a tenth CTest target, so the
# existing nine-suite registration and oracle-case totals remain unchanged.
# A failure is fatal under set -e; no numerical campaign may be accepted with
# a known blind spot in its result parser.
printf '\n=== PYTHON ORACLE INTEGRITY SELF-TEST ===\n'
PYTHONDONTWRITEBYTECODE=1 python3 tests/validation/test_oracle_integrity.py -v

# Avoid reporting the unsupported assumption that all configurations are
# interchangeable: each compiler independently emits its format profile.
readonly warn=(
    -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion
    -Wswitch-enum -Wswitch-default -Wshadow -Wformat=2 -Wundef -Werror
    -Wstrict-prototypes -Wmissing-prototypes -Wmissing-declarations
    -Wold-style-definition
)
readonly fp_options=(-frounding-math -ffp-contract=off -fno-fast-math)
readonly common_cmake=(
    -DBBTC_BUILD_TESTS=OFF
    -DBBTC_BUILD_VVUQ=ON
    -DBBTC_VVUQ_BUILD_TESTS=ON
    -DBBTC_VVUQ_ORACLE_TESTS=ON
    -DBBTC_VVUQ_ORACLE_CASES=100000
    -DBBTC_VVUQ_METRIC_ORACLE_CASES=10000
    -DBBTC_VVUQ_ROUND_ORACLE_CASES=10000
    -DBBTC_VVUQ_API_ORACLE_CASES=10000
)

# GCC and Clang must be checked independently; the primary target is not
# certified just because one compiler happens to accept the source.
for compiler in gcc clang; do
    build="$audit_build/strict_$compiler"
    printf '\n=== HOST PROFILE: %s ===\n' "$compiler"
    mkdir -p "$build"
    "$compiler" -std=c23 "${warn[@]}" "${fp_options[@]}" \
        "$probe" -lm -o "$build/vv_platform_probe"
    "$build/vv_platform_probe"

    printf '\n=== VVUQ STRICT BUILD AND TEST: %s ===\n' "$compiler"
    cmake -S . -B "$build" -G Ninja \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DCMAKE_C_COMPILER="$compiler" \
        -DCMAKE_C_FLAGS="${warn[*]}" \
        "${common_cmake[@]}"
    cmake --build "$build"
    ctest --test-dir "$build" --show-only | grep -F 'Total Tests: 9'
    ctest --test-dir "$build" --output-on-failure

    # nm checks *definitions* in the developer archive, not just declarations
    # in comparison.h. Avoid certifying a header-only symbol that cannot link.
    printf '\n=== DEVELOPER API SYMBOLS: %s ===\n' "$compiler"
    archive="$build/tools/bbtc_vv/libbbtc_vv_comparison.a"
    test -s "$archive"
    for symbol in \
        bbtc_vv_compare_float \
        bbtc_vv_compare_double \
        bbtc_vv_compare_long_double; do
        if ! nm -g --defined-only "$archive" | \
            grep -Eq "[[:space:]]T[[:space:]]${symbol}$"; then
            printf 'ERROR: Missing defined API symbol: %s\n' "$symbol" >&2
            exit 1
        fi
        printf 'DEVELOPER_API_DEFINED=%s\n' "$symbol"
    done
done

printf '\n=== CLANG DEBUG ASAN/UBSAN ===\n'
build="$audit_build/sanitized_clang"
mkdir -p "$build"
readonly san=(-fsanitize=address,undefined -fno-omit-frame-pointer)
clang -std=c23 "${warn[@]}" "${fp_options[@]}" "${san[@]}" \
    "$probe" -lm -o "$build/vv_platform_probe"
ASAN_OPTIONS='detect_leaks=1:halt_on_error=1' \
UBSAN_OPTIONS='halt_on_error=1:print_stacktrace=1' \
    "$build/vv_platform_probe"
cmake -S . -B "$build" -G Ninja \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_C_COMPILER=clang \
    -DCMAKE_C_FLAGS="${warn[*]} ${san[*]}" \
    -DCMAKE_EXE_LINKER_FLAGS=-fsanitize=address,undefined \
    "${common_cmake[@]}"
cmake --build "$build"
ctest --test-dir "$build" --show-only | grep -F 'Total Tests: 9'
ASAN_OPTIONS='detect_leaks=1:halt_on_error=1' \
UBSAN_OPTIONS='halt_on_error=1:print_stacktrace=1' \
    ctest --test-dir "$build" --output-on-failure

printf '\n=== PRODUCTION-ONLY BUILD SEPARATION ===\n'
build="$audit_build/production_clang"
cmake -S . -B "$build" -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_C_COMPILER=clang \
    -DBBTC_BUILD_TESTS=OFF \
    -DBBTC_BUILD_VVUQ=OFF \
    -DBBTC_VVUQ_BUILD_TESTS=OFF \
    -DBBTC_VVUQ_ORACLE_TESTS=OFF
cmake --build "$build"
test -s "$build/libbbtc.a"

if ninja -C "$build" -t targets all | grep -E 'bbtc_vv_|vv-validation-tests'; then
    echo 'ERROR: Production build unexpectedly contains VVUQ targets.' >&2
    exit 1
fi
if nm -g --defined-only "$build/libbbtc.a" | grep -E 'bbtc_vv_'; then
    echo 'ERROR: Production library defines VVUQ implementation symbols.' >&2
    exit 1
fi
printf 'PRODUCTION_VVUQ_TARGETS=0\nPRODUCTION_DEFINED_VVUQ_SYMBOLS=0\n'

printf '\n=== SOURCE INTEGRITY AND WHITESPACE ===\n'
sha256sum \
    tools/bbtc_vv/exact_dyadic.c \
    tools/bbtc_vv/exact_metric.c \
    tools/bbtc_vv/exact_round.c \
    tools/bbtc_vv/comparison.c \
    tools/bbtc_vv/comparison_api.c \
    tests/validation/test_exact_dyadic_oracle.py \
    tests/validation/test_exact_metric_oracle.py \
    tests/validation/test_exact_round_oracle.py \
    tests/validation/test_comparison_api_oracle.py \
    tests/validation/test_oracle_integrity.py \
    tests/validation/dyadic_oracle_driver.c \
    tests/validation/exact_metric_oracle_driver.c \
    tests/validation/exact_round_oracle_driver.c \
    tests/validation/comparison_api_oracle_driver.c \
    tests/validation/CMakeLists.txt \
    tools/bbtc_vv/acceptance_capture.sh

git diff --check
git diff --cached --check
printf '\n=== EVIDENCE CAPTURE COMPLETE; MANUAL ACCEPTANCE REVIEW STILL REQUIRED ===\n'
