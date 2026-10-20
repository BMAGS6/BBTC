# VV0.1-B1b.2 — Independent Oracle and FENV Hardening

**Status:** proposed verification hardening patch. This document records the
implemented test design and commands, **not** acceptance of the entire B1b
comparison API or any scientific/physical validation.

## Rationale and trust boundaries

The exact dyadic C23 core is intended to answer an exact arithmetic question:
for the exact binary numbers presented at its inputs, does the selected
comparison pass? The result must be independent of rounded subtraction,
overflowed products, native diagnostic metrics, or caller rounding mode.

This patch introduces an **independent** reference calculation with Python
`fractions.Fraction`. It does not call the kernel's integer-limb implementation
or translate its internal sign-comparison algorithm into Python. Inputs are
constructed independently from binary32, binary64, or x87 format parameters.
Each input is sent as a hex dyadic (for example, `0x800000p-23`), which is
exactly representable in the selected family. The C driver parses through the
matching native C conversion function, then calls the private kernel. No
Python binary64-to-`long double` conversion is involved.

The oracle and C driver are **test-only** tools. Neither is installed with
BBTC, linked into the production library, or a physical reference dataset.
The driver rejects unsupported primary-format characteristics instead of
quietly degrading coverage; its startup check is not a proof of all x87 ABI
characteristics. Noncanonical x87 encodings are outside its generated corpus.

## Reproducible commands

Run from the separate B1b worktree, not the BBTC production worktree:

```sh
cd ~/Projects/BBTC_VVUQ_B1b || exit 1

cmake -S . -B build/vv0_1b2_oracle_gcc -G Ninja \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++ \
    -DBBTC_BUILD_TESTS=OFF \
    -DBBTC_BUILD_VVUQ=ON \
    -DBBTC_VVUQ_BUILD_TESTS=ON \
    -DBBTC_VVUQ_ORACLE_TESTS=ON \
    -DBBTC_VVUQ_ORACLE_CASES=100000

cmake --build build/vv0_1b2_oracle_gcc
ctest --test-dir build/vv0_1b2_oracle_gcc --output-on-failure -V
```

Repeat with `clang` / `clang++` and a separate build directory. CMake
registers the existing cmocka executable as one test and the independently
calculated differential campaign as one additional test. The campaign uses
**100,000 seeded randomized comparisons plus 18 directed vectors per scalar
family**, or **300,054** cases total **per compiler**. The seed is
`0xBB7C2026`; the Python interpreter version is printed for replay.
Changing `BBTC_VVUQ_ORACLE_CASES` changes the statistical coverage only;
directed cases remain mandatory. The script streams bounded batches to
avoid linear memory growth for large test campaigns.

For a direct rerun of a particular oracle executable, use:

```sh
python3 tests/validation/test_exact_dyadic_oracle.py \
    --driver build/vv0_1b2_oracle_gcc/vv-validation-tests/bbtc_vv_dyadic_oracle_driver \
    --per-family 100000 --seed 0xBB7C2026
```

The exact executable path is determined by CMake and may vary if the build
layout is changed; using `ctest -V` is the recommended reproducible option.
The runner's `--batch-size` controls memory use only and does not change the
random sequence. A mismatch prints a complete replayable input line with
family, method, reference, observation, and tolerance.

To check that Python discovery cannot be silently skipped when explicitly
requested, test that configuring with `-DBBTC_VVUQ_ORACLE_TESTS=ON` and
`-DBBTC_VVUQ_BUILD_TESTS=OFF` **fails**, without modifying existing builds.
The ordinary build continues to have no Python/cmocka requirement.

## Test coverage and limitations

- Three native scalar families (binary32, binary64, x87 80-bit extended
  precision with 16-byte storage on the original target).
- All three B1 initial relations: Exact, Absolute, Reference-relative.
- Directed cases for signed zero, exact tolerance equality, reference-relative
  asymmetry, zero reference, subnormal boundaries, maximum-finite inputs,
  adverse rounding at `2^p+1`, negative tolerance and failure precedence.
- Seeded cases covering zero, subnormal, minimum-normal, maximum-normal,
  small and large exponents, opposite signs, exact equality, and zero or
  negative tolerance.
- C mock tests cover four rounding modes, individual sticky exception flags,
  combined flags, valid and rejected numeric operations, and invalid methods.
- All random tests are deterministic given the Python version, seed,
  and case count; they are not a completeness or correctness proof.

**Not verified by these tests:** signaling-NaN trapping behavior, callers
with unmasked traps, hostile FTZ/DAZ modes, unusual long-double ABI layouts,
noncanonical x87 representations, or general portability to binary128.
The comparison kernel's exactness proof and these gaps remain subject to
independent review before B1b acceptance. Metric calculation, public
`bbtc_vv_compare_*` entry points, certified bounds, reference provenance,
and scientific validation remain future phases.

## Authoring-environment checks (not user-host validation)

For this proposed patch, the authoring environment used GCC 14.2 and Clang
17-compatible tooling to compile the production-independent driver with
strict C23 warnings and `-Werror`. A local cmocka-compatible **shim** (not
real cmocka) executed the expanded tests under GCC/Clang at `-O0`/`-O2`.
The independently generated Python corpus of 300,054 comparisons **per compiler** matched the
C kernel under both GCC and Clang in the local environment. These are preliminary checks,
not a substitute for the user's GCC/Clang/cmocka runs or a proof of the
complete exact-arithmetic domain.
