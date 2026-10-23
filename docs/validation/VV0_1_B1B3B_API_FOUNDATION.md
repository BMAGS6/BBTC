# VV0.1-B1b.3b — Comparison API Foundation and Verification Plan

**Status:** implementation increment; not the completed comparison API.

**Baseline:** VV0.1-B1b.2 kernel (`17e1c4d`) and accepted B1b.3a
nearest-even diagnostic design.

## Scope and separation

This increment deliberately adds only the **internal decision-preparation
layer**. It makes the accepted comparison **data model** available to the
separate developer-only VVUQ build, establishes method and status translations,
and verifies the argument-precedence and floating-environment guarantees that
the ultimate `bbtc_vv_compare_*()` entry points will need.

The accepted contract requires successful Absolute/relative results to carry
**correctly rounded and faithfully classified diagnostics**. Merely returning
an exact PASS/FAIL and setting a dummy metric or `UNRESOLVED` would violate the
spirit—and potentially the letter—of that contract. B1b.3b therefore does **not**
export or define the final `bbtc_vv_compare_float()`, `..._double()`, or
`..._long_double()` functions. Those are scheduled for B1b.3c, together with
the metric backend. This is an intentional review gate, not a missing feature
masquerading as a completed public-facing implementation.

The accepted enums, policy structures, and result structures are declared in
`tools/bbtc_vv/comparison.h`. Internal callable decision preparation resides in
`comparison_internal.h` and `comparison.c`. These interfaces are developer-only,
and nothing is installed as an `include/bbtc/` header.

## Data flow and safety properties

1. Check output pointer without writing through null. When present, clear the
   finding to `BBTC_VV_FINDING_NOT_EVALUATED` before other validation.
2. Check policy pointer and switch over the accepted comparison method values.
   Neither step reads or classifies a floating-point argument.
3. Use `feholdexcept()` to capture the incoming environment and mask exceptions
   during numeric inspection. The existing exact dyadic kernel performs NaN
   before infinity classification, then tolerance and relative-domain checks.
4. Translate the kernel's execution status into the accepted VVUQ status
   enumeration. No non-success condition may return a mathematical finding.
5. Restore the incoming floating-point environment using `fesetenv()` before
   returning. Restoration failure overrides success and returns
   `BBTC_VV_STATUS_NUMERICAL_FAILURE` with cleared finding.
6. Only on completed successful execution convert the Boolean dyadic decision
   into `PASS` or `FAIL`. There is no `INDETERMINATE` for initial B1 methods.

The kernel remains solely responsible for the mathematical truth of the exact
inequality. The wrapper does not recompute it with rounded floating-point
arithmetic or derive it from a future numerical-error metric.

The normal-return preservation guarantee depends on qualified GCC/Clang build
flags and supported `<fenv.h>` behavior. Enabled unmasked traps, unexpected
floating-point formats, noncanonical x87 encodings, and fast-math assumptions
are not newly certified by this increment. Tests cannot prove universality.

## B1b.3b acceptance checklist

- [ ] Independently compile and run the new 10-case cmocka suite on GCC and Clang.
- [ ] Retain the existing dyadic and Python exact-rational regression suites.
- [ ] Verify structural output clearing, invalid methods, numeric precedence,
      and exact adversarial decisions on all three native scalar families.
- [ ] Verify success/failure preservation of rounding mode and sticky flags.
- [ ] Repeat optimized strict `-Werror` and Clang ASan/UBSan builds.
- [ ] Verify a VVUQ-only CMake test build without the production test suite.
- [ ] Verify a production-only build never requires VVUQ dependencies.
- [ ] Confirm that `bbtc_vv_compare_*` final symbols are NOT exported by this
      checkpoint; they must be implemented in B1b.3c after metric qualification.

## Next increment: B1b.3c

Implement the correctly rounded Absolute and Reference-relative diagnostic
backend using exact integer/rational arithmetic. Prove representability and
round-to-nearest/ties-to-even for binary32, binary64, and qualified x87; test
normal/subnormal boundaries, mathematical overflow, and exact halfway ties.
Then expose the full accepted `bbtc_vv_compare_*` signatures in `comparison.h`,
implement their complete result semantics, and run a separate independent
Python `fractions.Fraction` diagnostic oracle before accepting the milestone.

No physical reference dataset, scientific uncertainty model, or firearm safety
claim is provided or implied by this implementation increment.
