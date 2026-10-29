# VV0.1-B1b.3c-3 — Complete Native Comparison API Integration

**Status:** implementation increment; awaiting independent B1b.3d end-to-end qualification.

**Base:** commit `28dbd74` (`VV0.1-B1b.3c-2: implement exact rational rounding`).

**Governing specifications:** accepted `VVUQ_COMPARISON_CORE_CONTRACT.md` v0.1.0 and `VV0_1_B1B3_DIAGNOSTIC_DESIGN.md`.

## 1. Scope and mathematical roles

This checkpoint provides callable, developer-only `bbtc_vv_compare_float`,
`bbtc_vv_compare_double`, and `bbtc_vv_compare_long_double` entry points.
Each accepts a native reference, native observation, method/tolerance policy,
and an output struct containing operation status (as the return value),
mathematical finding, metric state, and native diagnostic scalar.

The pipeline is deliberately one-way:

1. `bbtc_vv_prepare_*` validates arguments, maps statuses, and delegates the
   **exact mathematical PASS/FAIL** to `bbtc_vv_dyadic_compare_*`.
2. For EXACT only, no metric exists. Return NOT_COMPUTED/+0 with the exact
   finding, without constructing or rounding a rational.
3. For ABSOLUTE and REFERENCE_RELATIVE, `bbtc_vv_metric_exact_*` constructs
   the **exact rational** absolute error or relative error from native inputs.
4. `bbtc_vv_metric_round_*` independently classifies the exact rational and
   chooses the nearest-even **native scalar** without using native arithmetic
   to decide where the rounding boundary lies.
5. Only after the caller's floating-point environment is restored do the
   operations assign the completed finding, state, and diagnostic to the result.

The diagnostic is never fed back into the exact comparison. Thus the decisive
adversarial case `r=2^p`, `o=-1`, `t=2^p` remains FAIL even though the rounded
absolute diagnostic is `2^p` and the rounded relative diagnostic is `1`.

## 2. Validation and output guarantees

- The output pointer is checked before any write. A valid output is cleared
  in full before any policy validation (even an invalid/unknown method).
- Null policies and invalid method enumerators fail structurally without
  numerical inspection. The already tested preparation layer handles numeric
  NaN-before-infinity precedence, tolerance sign/zero, and relative zero
  reference. Unsupported floating-point formats/environments fail closed.
- Every unsuccessful return leaves `finding=NOT_EVALUATED`,
  `metric_state=NOT_COMPUTED`, `metric=+0`. Only SUCCESS may publish PASS/FAIL.
  `INDETERMINATE` is not a permitted B1 successful finding.
- `EXCEEDS_RANGE` and `ROUNDED_TO_ZERO` are successful metric states using +0
  sentinels, not execution failures or reasons to alter the exact finding.
- Unexpected arithmetic failures after an exact finding do **not** return
  misleading partial success. They fail the complete operation with its
  canonical cleared output and status `NUMERICAL_FAILURE`, except that an
  explicitly unsupported FP environment propagates its environment status.
- The outer fenv guard encloses preparation, rational construction, and
  rounding. The nested kernels also save/restore their own fenv state.
  The outer guard intentionally restores the caller's original rounding
  direction and sticky flags before publishing success.

Canonical clearing concerns **field values**, not C struct padding bytes.
No assumption is made about the object representation of padding.

## 3. Build and API exposure

- Existing CMake target `bbtc_vv_comparison_foundation` remains unchanged,
  preserving all of its regression tests and its internal preparation symbols.
- A NEW developer CMake static target `bbtc_vv_comparison` exports the complete
  API in `comparison_api.c`. It links `bbtc_vv_comparison_foundation` and
  `bbtc_vv_exact_round`, which transitively supply the exact metric and exact
  dyadic dependencies. No production `bbtc::bbtc` dependency exists.
- `comparison_internal.h` and existing preparation functions remain available
  for their dedicated regression tests, not as replacements for full results.
- No BBTC installed public header or production model ABI is modified.
  This is numerical comparison machinery, not a physics validation claim.

## 4. Verification requirements

1. New `bbtc.vv.comparison_api` cmocka suite, 15 tests, exercises the full
   three-family API, statuses, metric states, extreme magnitudes, argument
   precedence, and fenv preservation across supported rounding modes.
2. Continue the existing 7 VVUQ suites (including 3 independent Python
   exact-rational oracles). **Eight** CTest entries should be registered.
3. GCC/Clang strict optimized builds with `-Werror` and contraction disabled.
4. Clang ASan/UBSan on a Debug build; normal production-only build remains
   free of a VVUQ dependency.
5. Confirm the three native API symbols are present in the developer library
   and no production header was changed.
6. B1b.3d must add an **independent end-to-end** exact-rational oracle for
   policy validation, exact finding, metric-state classification and native
   diagnostic. Component-level agreement alone is not final acceptance.

## 5. Deferred capabilities

B1b.3d is required before claiming the complete comparison API accepted.
Certified interval enclosures, ULP comparisons, scientific uncertainty
budgets, empirical reference datasets, BBTC model adapters, and reporting are
outside this implementation and require separate contract checkpoints.

Supported numerical-family assumptions remain those qualified by the existing
kernels: native binary32, binary64, and x87 extended `long double` on the
primary GNU/Linux environment; no fast-math, unmasked traps, or unreviewed
noncanonical formats. Nothing in this layer expands those support claims.
