# BBTC VVUQ — B1b.3 Diagnostic Metric Implementation Decision

| Metadata | Value |
| --- | --- |
| Project phase | VV0.1-B1b.3 |
| Document status | **Accepted — implementation pending** |
| Baseline commit | `17e1c4d` |
| Governing contract | `docs/validation/VVUQ_COMPARISON_CORE_CONTRACT.md` v0.1.0 |
| Scope | Internal numerical comparison API and scalar diagnostic metrics |

## 1. Decision: deterministic diagnostic rounding

For each supported native scalar family (`float`, `double`, `long double`), a successful Absolute or Reference-relative comparison SHALL return the correctly rounded native-family diagnostic under **round-to-nearest, ties-to-even**, independently of the caller's active floating-point rounding mode.

This selects the diagnostic rounding rule deferred by B1a for the B1b.3 implementation. The mathematical PASS/FAIL finding SHALL continue to be determined solely by the independently qualified exact-dyadic decision kernel; no diagnostic value may change that finding.

The implementation SHALL preserve the caller's rounding mode and sticky floating-point exception flags on all supported normal returns, including invalid-input paths. The ambient rounding mode is an input to environmental qualification but SHALL NOT affect either the exact finding or the diagnostic's numerical value.

## 2. Exact mathematical metrics

Let `r`, `o`, and `t` denote the exact real values represented by the native-type arguments, after validation under the governing contract.

- **Exact mode:** No diagnostic computed. Set `metric_state = BBTC_VV_METRIC_NOT_COMPUTED` and `metric = +0`.
- **Absolute mode:** The mathematical metric is `D = |o - r|` (units of the compared quantity).
- **Reference-relative mode:** The mathematical metric is `Q = |o - r| / |r|` (dimensionless, and defined only when `r != 0`).

Unlike the comparison criterion, `Q` may be a non-dyadic rational even if all arguments are dyadic. In general, native floating-point division is insufficient to *prove* correct rounding under all supported environments and supported compiler settings. The diagnostic algorithm must perform correctly rounded rational-to-binary conversion, or provide a separately reviewed equivalent proof.

## 3. Result classifications

On successful Absolute or Reference-relative comparison, classify the **exact mathematical metric** first, then encode a native-family diagnostic if possible:

| Exact mathematical situation | Required state | Returned metric |
| --- | --- | --- |
| Exactly zero, or a finite positive number exactly representable in the native family | `FINITE_EXACT` | Exact native value, including `+0` |
| Positive finite mathematical value within native finite range, not exactly representable, rounding to a finite nonzero native value | `FINITE_ROUNDED` | Correct nearest-even value |
| Mathematical value exceeds the native family's maximum finite value | `EXCEEDS_RANGE` | `+0` sentinel; do not return infinity |
| Strictly positive mathematical value whose nearest-even representation is zero | `ROUNDED_TO_ZERO` | `+0` sentinel |
| Algorithm cannot defensibly compute or classify the metric while the exact finding remains available | `UNRESOLVED` | `+0` sentinel; this is exceptional, not the normal implementation path |

The boundary between zero and the least positive subnormal requires exact ties-to-even treatment: a metric equal to **one-half of the least positive subnormal** rounds to positive zero; values above that boundary round to a nonzero value. The exact `FINITE_EXACT` label must never be assigned merely because a rounded computation happened to equal a finite native value.

A mathematically positive metric exactly equal to the maximum finite native value is `FINITE_EXACT`; an exact metric larger than that value is `EXCEEDS_RANGE`, even if an IEEE nearest-even conversion would otherwise round to the maximum finite value rather than infinity.

## 4. Correct-rounding construction and proof obligations

The implementation SHOULD reuse exact dyadic decomposition, but it MUST keep diagnostic arithmetic separate from exact-decision arithmetic. An acceptable constructive strategy is:

1. Represent `D` as an exact nonnegative integer multiplied by a power of two, without overflowing a native scalar.
2. For Relative mode, represent `Q` as an exact positive integer numerator and denominator with a power-of-two exponent offset. Cancel common factors only when mathematically exact.
3. Locate the correctly rounded binary exponent and significand using exact integer comparisons and quotient/remainder operations. For halfway cases, compare exactly `2 * remainder` against the divisor, and choose the even candidate on equality.
4. Handle subnormal rounding, carry into a new exponent, exact zero, and mathematical overflow as separate branches; do not infer these merely from floating-point exceptions.
5. Convert a proven representable output significand/exponent to the **native** scalar family, with an explicit correctness argument for that final conversion under each supported environment. No `long double`-to-`double` narrowing is allowed.
6. Classify the result from exact integer arithmetic, independently of the final native value.

No mandatory heap allocation is permitted. Any bounded fixed-array arithmetic must carry reviewed width/exponent bounds and checked indexing, as in B1b.2. If a representable exact metric is classified as `UNRESOLVED` routinely, the implementation has not met this milestone.

## 5. Full comparison API behavior

Use the exact signatures, public-to-VVUQ-internal enumerations, policy types, and result types accepted in B1a, implemented in `tools/bbtc_vv/comparison.h` and `comparison.c`.

Input checking and failure output precedence MUST exactly follow B1a section 5. On non-success return with non-null output, clear `finding`, `metric_state`, and `metric` to their zero-valued canonical representation. Within supported environments, numeric NaN classification takes precedence over infinity, followed by finite argument/domain rules. On successful comparisons, `finding` is **always** `PASS` or `FAIL`, never `INDETERMINATE`.

The wrapper must preserve `fenv` around the entire operation, not merely rely on the nested kernel to preserve its own environment. Failure to restore state is a numerical failure, not success. Do not mask unsupported unmasked traps by claiming restoration of a state after a trap occurs.

## 6. Independent acceptance tests

Use all three native scalar families. Add cmocka tests covering:

- Structural precedence: null output; null policy; invalid/unknown method; canonical clearing on every failure.
- NaN-before-infinity numeric precedence; negative, signed-zero, and nonfinite tolerances; reference-relative zero domain failure.
- Exact accepted comparisons from B1b.2, including the `2^p + 1` adversarial FAIL case.
- Exact-zero versus underflow-to-zero diagnostics; precisely representable positive values versus rounded-positive values.
- Nearest-even halfway ties including even/odd significands, normal/subnormal transition, and midpoint-to-zero.
- Extreme absolute differences exceeding `*_MAX`, and relative ratios that overflow or underflow the native diagnostic domain.
- Identical results for every supported incoming rounding mode, with unchanged pre-set individual and combined exception flags after success and failure.
- Native `long double` scenarios with exponent range beyond binary64, with no narrowing.

Add an independently implemented Python `fractions.Fraction` oracle for **both diagnostic classification and correctly rounded bit-pattern/value**, not only the PASS/FAIL finding. The oracle must not reuse the production rounding logic or assume that converting Python `float` can represent native x87 values.

Run GCC/Clang Debug and optimized strict `-Werror` builds, ASan/UBSan, native cmocka, differential oracle testing, and a VVUQ-only CMake build. Preserve the standard BBTC build without a new runtime dependency.

## 7. Deferred scope and gate

Certified numerical-error intervals, ULP distance, physical reference cases, and BBTC model adapters remain outside B1b.3. The B1b.3 implementation is not accepted solely because the B1b.2 exact kernel passes its previous tests; the new diagnostic arithmetic requires separate correctness evidence.

**Implementation acceptance gate:** This rounding decision is accepted; the implementation must satisfy the fixed B1a API, and diagnostics must receive independently reproducible tests before B1b.3 is committed as complete.
