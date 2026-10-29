# VV0.1-B1b.3d-1 — Independent End-to-End Comparison Oracle

**Status:** implementation increment; independent host qualification and review pending.

**Base:** VV0.1-B1b.3c-3 complete comparison API, atop commit `28dbd74`.

**Governing contracts:** `VVUQ_COMPARISON_CORE_CONTRACT.md` (B1a v0.1.0) and
`VV0_1_B1B3_DIAGNOSTIC_DESIGN.md` (nearest-even diagnostic decision).

## 1. Scope and reason for independence

B1b.3c previously qualified the exact decision, exact rational magnitude, and
nearest-even converter as separate modules and exercised the integrated API
with directed cmocka tests. This checkpoint adds a **black-box differential
oracle for the complete, user-callable, developer-only VVUQ comparison API**.

The independent Python model predicts all four meaningful outputs:

1. **Operation status**: structural validation first, then NaN ahead of
   infinity, then tolerance constraints and relative zero-reference domain.
2. **Exact mathematical finding**: PASS/FAIL for exact equality or the accepted
   inclusive Absolute/Reference-relative threshold inequalities; no comparison
   between rounded diagnostic and tolerance is ever used.
3. **Metric state**: no metric in Exact mode; otherwise exact representability,
   nonzero rounded result, magnitude above native maximum, or positive metric
   rounded to zero.
4. **Rounded native-family metric**: exact rational nearest-even rounding for
   binary32, binary64, and qualified x87 binary80 (`long double`).

Reference calculations use Python's arbitrary-precision `Fraction`. Neither the
C conversion routines nor any previous component-oracle Python module are
imported, called, or used to compute expected values. The oracle is auditable
from the accepted mathematical definitions.

**This is a mathematical API verification tool, NOT a ballistics reference
dataset and NOT empirical/physical validation.**

## 2. Precise binary interchange, including extended precision

`comparison_api_oracle_driver.c` is a deliberately tiny test *transport*.
Input is an ASCII record per API call:

```
FAMILY METHOD NULL_POLICY NULL_RESULT ROUNDING FLAGS REFERENCE OBSERVATION TOLERANCE
```

- `FAMILY`: `F` native float, `D` native double, `L` native long double.
- `METHOD`: exact contract enumerator as unsigned decimal integer (0..255).
- `NULL_POLICY`, `NULL_RESULT`: independent Boolean flags (0 or 1).
- `ROUNDING`: ordinal 0 nearest, 1 upward, 2 downward, 3 toward zero.
- `FLAGS`: ordinal 0 none, 1 inexact, 2 invalid|underflow,
  3 overflow|divide-by-zero. The driver resolves platform-specific FE masks.
- Native operands: exactly representable hexadecimal floating literals,
  including signed zeros and all supported normal/subnormal magnitudes;
  exceptional tests also use `nan` and `inf` tokens.

Each output contains operation status, finding, metric state, a native hex
floating-point diagnostic, and an environment-preserved indicator.
Native `float` is promoted to `double` ONLY when printing `%a`; that promotion
is exact on the qualified binary32/binary64 target. Native `long double` is
printed directly via `%La`, **without narrowing to Python float or C double**.

The driver parses input BEFORE installing the chosen incoming FP environment.
It records the caller rounding mode and sticky flags immediately before the
API call, then compares both immediately after return. Printing the result
comes strictly after the environment check and restoration. Parsing errors,
unsupported environment setup, or failed driver actions terminate the test
harness with an error, rather than being misreported as an API status.

A null-result request cannot be inspected through the null pointer, so the
driver emits a separate zero-valued transport placeholder. No validation
claim about inaccessible output storage is inferred from that placeholder.

## 3. Independent mathematical oracle

All finite input values are decoded to exact rationals. The oracle uses the
same mathematical definition as the accepted contract, **not the production
C algorithm**:

- Exact: equality of finite represented values (including signed-zero
  equality), with exact zero tolerance.
- Absolute: `abs(o - r) <= t`, metric `abs(o-r)`.
- Reference-relative: nonzero reference, `abs(o-r) <= t * abs(r)`, metric
  `abs(o-r) / abs(r)`. No rounded division controls PASS/FAIL.
- Nearest-even: determine the exact binary binade using rational comparison,
  select the native quantum (including subnormal regime), then perform one
  arbitrary-precision integer quotient/remainder. Halfway remainders round to
  an even significand. Classify exact magnitude before any rounding.

The oracle does not claim to model unqualified decimal, binary128, binary64-
backed `long double`, altered excess-precision semantics, or unmasked traps.

## 4. Directed and randomized coverage

The mandatory directed corpus includes the original `2^p + 1` false-PASS
adversary for Absolute and relative modes; values at and across the maximal
finite boundary; subnormal and smallest-normal represented operands; signed
zero; negative references; exact and rounded relative diagnostics; reference
zero; null pointers; invalid enumerators; invalid and negative tolerances; and
NaN precedence over infinity. Every directed scenario is repeated under all
four incoming rounding directions and all four exception-flag presets.

A deterministically seeded supplementary corpus draws exactly representable
normal and subnormal inputs from the full supported exponent ranges, together
with domain-invalid and exceptional inputs. The option
`BBTC_VVUQ_API_ORACLE_CASES` controls random samples per family; fixed directed
cases are added. The test reports seed, native family, number of checked cases,
and detailed failing input/expected/observed tuples for reproducibility.

For each case the oracle compares the actual API status, exact finding,
classification, and rounded native metric, **and** demands preservation of the
caller environment. Every failure return must publish the canonical cleared
finding/state/+0 metric. If the mathematical finding is sound but a diagnostic
exceeds the native finite range, SUCCESS and an exact finding are still
required, accompanied by EXCEEDS_RANGE and a +0 sentinel.

## 5. Acceptance criteria and exclusions

- Strict optimized GCC and Clang builds must pass the new ninth CTest entry
  `bbtc.vv.comparison_api.oracle` and all eight previously registered tests.
- Clang ASan/UBSan must pass, including the new full-API oracle driver.
- The previously qualified cmocka suites and three independent component
  oracles must continue passing unchanged.
- Production-only builds must remain decoupled from Python, cmocka, and the
  developer-only VVUQ library. New code must not modify production physics.
- The exact native ABI and compiler/format capability assumptions from the
  prior proof documents remain in force; no new platform certification.
- This checkpoint does not establish complete formal verification or scientific
  model validation. Remaining B1b.3d acceptance work includes final audit,
  production-build separation checks, Git hygiene, and a contract-conformance
  acceptance record before the B1 implementation can be called accepted.

## 6. Files and dependency structure

New developer-only files:

- `tests/validation/comparison_api_oracle_driver.c`: opaque C API transport.
- `tests/validation/test_comparison_api_oracle.py`: independent rational model.
- `docs/validation/VV0_1_B1B3D1_END_TO_END_ORACLE.md`: design and checks.

Only `tests/validation/CMakeLists.txt` is modified. The new driver links
`bbtc_vv_comparison` but **not** the production `bbtc::bbtc` target. The new
Python dependency is gated behind `BBTC_VVUQ_ORACLE_TESTS`, so normal CMake
configurations do not require Python.
