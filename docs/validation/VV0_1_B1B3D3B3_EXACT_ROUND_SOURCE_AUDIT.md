# VV0.1-B1b.3d-3b-3 — Exact Binary Rounding: Source Proof Audit

**Disposition:** CONDITIONAL source-review finding for A05. **The complete B1 comparison core is not yet accepted.** Primary-host qualification and the later whole-API audit remain required. Do not rewrite the earlier B1b.3c-2 proof note as though its original deferred milestones were still current.

**Reviewed implementation:** `tools/bbtc_vv/exact_round.c` (qualified native-family version after B1b.3d-3a capability hardening), `exact_round.h`, and the existing `test_exact_round.c`. The patch associated with this review changes only tests and documentation, not the numerical algorithm or production library. Preserve the exact Git commit and test transcript before closing A05.

**Governing specification:** `VVUQ_COMPARISON_CORE_CONTRACT.md` v0.1.0, `VV0_1_B1B3_DIAGNOSTIC_DESIGN.md`, `VV0_1_B1B3C2_EXACT_ROUNDING_PROOF.md`, and the A05 row of `VV0_1_B1B3D2_ACCEPTANCE_AUDIT.md`. This is software numerical verification, **not** physical or empirical validation.

## 1. Mathematical contract and classification

For a valid private exact metric, let

    N = sum(word[i] * 2^(64*i), i=0..used-1)
    X = (N * 2^numerator_exponent) /
        (denominator * 2^denominator_exponent), N >= 0, denominator > 0.

The result must be the closest nonnegative representable binary-family scalar to X, breaking exact midpoint ties toward the **even significand**. This result is independent of the caller's rounding mode. Every decision is made on exact integers before constructing a native scalar.

- `FINITE_EXACT`: X is representable in the target native family, including zero.
- `FINITE_ROUNDED`: X is positive, at most the largest finite scalar, but needs a nonzero rounded result.
- `EXCEEDS_RANGE`: **mathematical** X is greater than the largest finite native scalar. Publish the +0 sentinel, irrespective of what the hardware might round to.
- `ROUNDED_TO_ZERO`: X is positive and within mathematical range, but round-to-nearest-even yields zero. Publish +0.
- An invalid metric representation must return `INVALID_ARGUMENT` and clear both outputs.

Exact PASS/FAIL findings are owned by the dyadic decision kernel and must never depend on this diagnostic or its classification.

## 2. Exact comparison of an arbitrary rational with a dyadic candidate

For a candidate `m*2^k`, `compare_candidate` compares

    N*2^E  ?  (denominator*m)*2^(F+k).

The product of two unsigned 64-bit integers is less than 2^128 and is represented in two 64-bit limbs. Its implementation uses four 32-by-32 partial products and checked-width carry decomposition: no nonportable `__int128` or native FP multiplication is involved.

For the midpoint `((2*m+1)/2)*2^k`, `compare_midpoint` instead compares

    N*2^(E+1)  ?  denominator*(2*m+1)*2^(F+k).

With `denominator <= 2^64-1` and `m <= 2^64-1`, the right-hand integer is strictly smaller than `2^129` and fits in **three** 64-bit limbs. `midpoint_multiplier` builds it as `2*(denominator*m)+denominator`, with a carry propagated as far as the third limb. The 129-bit directed test explicitly exercises the extremal 65-by-64 multiplication, not a truncated 64-bit stand-in.

`compare_views` determines the most significant absolute bit of each shifted unsigned view; if those differ, the comparison follows immediately. Otherwise, `shifted_digit` aligns the views on an abstract common exponent and compares words from high to low. Its shift remainder is within `[0,63]`, and it evaluates a `64-shift` right shift only when `shift != 0`. Only declared limbs are dereferenced; absent words behave as zero. No giant integer needs to be materialized.

### Preconditions and bounds

`representation_ok` rejects a zero denominator, `used > 520`, a zero highest declared word, and any nonzero word beyond `used`. The externally reachable private metric structure admits signed 32-bit exponents; each view's bit length is at most `520*64=33280`. Combining these values into 64-bit bit positions cannot overflow, including inputs with extreme signed 32-bit exponents. This is deliberately stronger than relying solely on the narrower range of values emitted by `exact_metric.c`.

These assertions concern **field values**. They do not claim that C struct padding bytes have been initialized or may safely be compared with `memcmp`.

## 3. Range-first nearest-even selection

For target precision p and C exponent limits `MIN_EXP` and `MAX_EXP`:

    maximum_finite = (2^p-1)*2^(MAX_EXP-p)
    subnormal_quantum = 2^(MIN_EXP-p).

`select_rounding` first compares mathematical X with `maximum_finite` exactly. If X exceeds it, `EXCEEDS_RANGE` is returned **before** choosing a finite coefficient. This is stricter than merely checking the hardware's overflow flag: values just over maximum finite may still round *to* maximum finite.

For X positive and in-range, let `n` and `d` be the exact bit lengths of its numerator and denominator, respectively, and define

    c = n - d + E - F.

The binary-log floor lies in `{c-1,c}`. One exact comparison against `2^c` selects the correct binade `b=floor(log2(X))`. The candidate quantum is `2^(b-p+1)` for the normal region or the fixed subnormal quantum near zero. The implementation binary-searches `[0, 2^p-1]` for the floor coefficient, with `uint64_t` midpoint arithmetic avoiding overflow even when p=64 and `high=UINT64_MAX`.

After checking exact equality to that coefficient, `compare_midpoint` selects the lower or upper coefficient. At an exact halfway point the low coefficient parity breaks the tie. When the selected result would be coefficient `2^p`, the implementation normalizes it to coefficient `2^(p-1)` and increments the quantum exponent, without attempting to store `2^64` in `uint64_t`. A positive result rounding to a zero coefficient receives `ROUNDED_TO_ZERO`.

## 4. Native conversion and floating-point environment

Only after selecting a **mathematically representable** coefficient and exponent does the converter call the corresponding `scalbnf`, `scalbn`, or `scalbnl`. The selected coefficient has at most p bits and is exactly representable in its native family. Its power-of-two scaling is an exact representable result, so the caller's rounding direction cannot influence its value.

The entrypoint clears its accessible outputs before validation, rejects unsupported native formats/evaluation modes, then holds/restores the caller environment around the mathematical selection and final native conversion. The outer whole-API environmental and failure-path review is still an **OPEN** A09/A07 obligation; do not use this component audit to claim that every possible host or unmasked-trap configuration is certified.

The verified initial host profile is native binary32/binary64 and padded x87 `long double`, `FLT_EVAL_METHOD=0`, four recognized rounding modes, and configured strict FP compiler behavior. The B1b.3d-3a scope hardening supersedes the older proof document's permissive reference to binary64-backed `long double`.

## 5. New focused regression cases

The ten new cmocka tests, in addition to the existing 15, cover:

1. **Three per-family parity tests:** `(2^p+1)*2^-p` ties downward to `1`, while `(2^p+3)*2^-p` ties upward to `1+2^(2-p)`. These prove the code distinguishes an **even lower** from an **odd lower** coefficient.
2. **Three per-family binade-carry tests:** the exact midpoint immediately below `2`, plus immediately neighboring values, exercises the overflow-safe normalization of a `2^p` coefficient into the next binade.
3. **Three per-family maximum-finite tests:** `MAX - ULP/4` rounds to `MAX`, whereas `MAX + ULP/4` is `EXCEEDS_RANGE` despite lying close enough that hardware could return a finite scalar.
4. **One full-width x87 test:** use `A=2^64`, `d=A-1`, and `N=d*(2*(A-1)+1)=2*A^2-3*A+1` with limbs `[1,A-3,1]`. The constructed rational `X=N*2^-64/d` lies exactly halfway below `2`. The expected even result is `2`; this tests the 129-bit midpoint multiplier and p=64 binade carry simultaneously.

Input construction uses integer shifts strictly smaller than 64 or explicit separate limbs. Expected values are derived from mathematical identities or `nextafter*` adjacency, never calculated using the rounding implementation itself. All test functions and helper macros carry Doxygen explanations of the proof obligation.

The existing tests and independent Python `fractions.Fraction` oracle still cover normal/subnormal transitions, positive-to-zero underflow, ordinary non-dyadic fractions, malformed input, fenv preservation, and broad reproducible random inputs. These new directed checks are **not** claims of exhaustive testing.

## 6. Required qualification and decision

Reproduce the *full* existing `tools/bbtc_vv/acceptance_capture.sh` campaign after applying the patch, using the actual primary-host CMake/cmocka integration:

- Strict optimized GCC and Clang: all nine registered CTest suites pass.
- Clang Debug ASan/UBSan: all nine registered suites pass.
- The exact-round cmocka suite reports **25/25** cases in each build.
- Four independent Python oracles remain passing, with no changed seeds or unexplained skipped suites.
- Developer API symbols remain defined; production-only BBTC has zero VVUQ targets and defined symbols.
- Staged Git diff passes whitespace checks and includes only the intended test and audit document.
- Preserve the full transcript, compiler/format profile, hash values, and final source commit.

**Provisional A05 finding:** the reviewed integer comparison and coefficient-selection algorithms satisfy the stated nearest-even and representability policy on the qualified native families, *conditional on the source argument above and successful host qualification*. This is not a machine-checked formal proof and does not independently certify whole-API correctness, unmasked exceptions, physical uncertainty, or ballistic predictions.

**Do not mark B1 accepted:** A02, A06–A14 require the separate contract-to-source closure review and a final disposition document. No production BBTC physics or public ABI is modified in this increment.
