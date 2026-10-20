# VV0.1-B1b.2 — Exact Dyadic Decision Kernel: Engineering Notes

**Status:** experimental implementation increment; acceptance pending on the
user's GCC 16 / Clang 23 host, complete cmocka tests, supported-environment
qualification, and broader independent review. This document is an
implementation proof outline, not an empirical validation record or a claim
that the complete VV0.1-B1b comparison interface has been delivered.

## Scope

`tools/bbtc_vv/exact_dyadic.{h,c}` provides exact **Boolean decisions**, not
numeric-error diagnostics, uncertainty budgets, reference credibility, unit
checking, or physical conclusions. The three supported relations are:

- Exact: `reference == observation` (numerical signed zeros compare equal).
- Absolute: `|observation - reference| <= tolerance`.
- Reference-relative:
  `|observation - reference| <= tolerance * |reference|`, with nonzero reference.

All finite inputs are interpreted as **exact represented real numbers**. The
inequalities are inclusive at equality. Tolerances must be finite and
nonnegative; Exact requires tolerance equal to zero. A successfully executed
operation yields an exact Boolean; there is no indeterminate case in these
three methods.

## Proof outline

### 1. Exact representation

For a radix-2 floating-point number with `p` significand bits, `frexp` gives
`x = f * 2^e`, where `0.5 <= |f| < 1` for a nonzero finite value.
Multiplication by `2^p` produces an exact integral significand `M`, satisfying
`0 < M < 2^p`. Therefore `x = sign * M * 2^(e-p)` as an equality of real
numbers. The implementation verifies `p <= 64` and never converts the
`long double` input through `double`.

For supported binary arithmetic, scaling the normalized fraction by `2^p`
is exact and in the finite normal range. The resulting integer fits in
`uint64_t`, including the maximum 64-bit x87 significand. No rounded
subtraction or approximate reference computation is involved.

`x == 0` becomes a zero atom. For exact equality, C floating-point `==`
is used only after numeric classification and environment checks; oppositely
signed zeros therefore compare equal, as the contract requires.

### 2. Exact relative-threshold multiplication

If tolerance and reference have integer significands `M_t` and `M_r`, then

`T = tolerance * abs(reference) = (M_t * M_r) * 2^(E_t + E_r)`.

The integer product fits in **128 bits** because both factors are strictly
less than `2^64`. `multiply_u64()` uses four ordinary 32-bit partial
products and explicit carry propagation. Each partial product fits into a
64-bit integer. The largest intermediate middle-word sum is less than
`3 * 2^32`, so it, too, fits in 64 bits. The result is a pair of unsigned
64-bit words. Neither native `__int128` nor floating-point multiplication
is required.

### 3. Exact signed inequalities

For a nonnegative exact threshold `T`,

`|o-r| <= T` **if and only if** both `(o-r-T) <= 0` and `(r-o-T) <= 0`.

Each difference is the signed sum of at most three exact dyadic terms. Using
a common exponent origin, the implementation adds positive terms to one
nonnegative big integer, and negative terms (by absolute magnitude) to
another. It does not subtract magnitudes or normalize through any floating
arithmetic. The difference has the exact sign of `positive - negative`,
which is determined by comparing unsigned words from most significant to
least significant. Carry propagation is exact in base `2^64`.

For both inequalities, acceptance at equality follows naturally from
`sign <= 0`, not from a rounded metric. The **returned Boolean is unaffected
by the rounding mode** because the decision path after decomposition uses
only exact integer arithmetic.

### 4. Proven fixed-width bounds

For a supported floating-point family with `p <= 64`, the smallest
subnormal is at exponent `MIN_EXP - p` (C `<float.h>` notation). After `frexp`
and conversion to a `p`-bit integer significand, the smallest possible atom
exponent is `MIN_EXP - 2p + 1`.

For the widest initially enabled x87 format (`p=64`, `MIN_EXP=-16381`),
this is `-16508`; the exponent of a product of two such atoms is therefore
at least `-33016`.

A single nonzero finite x87 value is strictly less in magnitude than
`2^16384`. A product of two is strictly below `2^32768`, and a sum of three
nonnegative terms of that size is strictly below `2^32770`.

The fixed accumulator covers bit positions `[-33152, 33152)`: **1036 x
64-bit words**, leaving headroom at both ends. These bounds cover the
binary32, binary64, and x87 cases allowed by the format gate. Every array
access and any carry running out of range is additionally checked.

This is intentionally *not* a general arbitrary-precision integer library.
Enabling binary128 or other formats requires reviewing the decomposition,
word size, and exponent bounds and adding corresponding independent tests.

### 5. Control-flow and error semantics

A nonnull output Boolean is cleared before checking the comparison method.
Input NaN takes precedence over infinity inside the supported numerical
validation layer; nonfinite inputs cannot produce a successful comparison.
Reference-relative mode rejects a zero reference without substituting an
absolute comparison. Unsupported format or floating environment returns a
separate status; internal bound failures never masquerade as `PASS`.

### 6. Floating-point-environment isolation

All floating-point inspection and decomposition is performed between a
successful `feholdexcept()` and a `fesetenv()` restoring the prior state.
The incoming rounding mode and sticky exception flags are restored across
supported normal return paths, even if decomposition raises an inexact or
underflow flag temporarily. Failure to restore yields an explicit internal
failure, not successful completion.

The implementation requires rounding-aware compilation and prohibits
fast-math. The execution guarantee is currently **restricted** to verified
masked-exception environments. Portable ISO C has no general-purpose API
for discovering whether all floating-point traps are unmasked; unmasked-trap
configurations are not yet certified or deliberately exercised. The initial
runtime subnormal round-trip check is a defensive probe, not complete
qualification of every hostile or misconfigured floating-point environment.
Such broader capability handling must be resolved before declaring B1b fully
accepted on additional platforms.

## Independent testing and remaining work

The C cmocka suite checks equality, exact thresholds, relative asymmetry,
nearby representables, extremes, subnormals, NaN/Inf precedence, output
clearing, and incoming fenv state under all four supported rounding modes.

The following independent engineering checks were performed while preparing
this *prototype* in the authoring environment, not on the user's host:

- GCC 14 and Clang 17, optimization `-O0` / `-O2`, strict warning set.
- Sanitizer runs (`-fsanitize=address`, `-fsanitize=undefined`) on a
  **standalone local assertion harness** (a cmocka-compatible test shim,
  not the actual cmocka library).
- An independent Python `fractions.Fraction` exact-rational oracle:
  36,000 float comparisons, 36,000 double comparisons, and 6,000 x87
  long-double comparisons with hexadecimal round-trip input representations.

These checks do **not** replace executing and reviewing the real cmocka
suite under the user's GCC 16.2.1 / Clang 23.1.1 and cmocka 2.0.2,
including sanitizer builds. There are no physical-model claims.

**Next increment:** implement the accepted `comparison.h` API (policies,
operation statuses, results), add precisely classified scalar diagnostics
and their rounding rule, extend test coverage, and perform final supported
platform qualification. The exact dyadic component alone does not satisfy
all VV0.1-B1b acceptance criteria.

## Hardening review: fixed-width arithmetic details (B1b.2 revision)

The implementation uses three separately verifiable exactness arguments. This
supplement expands the earlier proof outline, with explicit correspondence to
functions in `tools/bbtc_vv/exact_dyadic.c`:

1. **64-bit significands from each native family.** `NAME##_atom` uses
   `frexp*` to obtain a binary fraction, then multiplies that normalized value
   by precisely `2^p` using `scalbn*`. All inputs reaching this function are
   finite. For a supported binary family with at most 64 significant bits,
   the result is a nonnegative integral significand at most `2^64-1`, so
   casting it to `uint64_t` is defined. The scaling result is normal and
   finite because its magnitude is in `[2^(p-1),2^p)`; no subnormal rounding
   can occur during this scale step. The frexp integer exponent is bounded
   by native `MIN_EXP`/`MAX_EXP`, so storing the resulting exponent in
   `int32_t` is safe on the formats explicitly gated by the code.
2. **Exact 64-by-64 multiplication.** `multiply_u64` decomposes both factors
   into two base-`2^32` words, forms four exact `uint64_t` partial products,
   and propagates the cross-word carry. Each product of two 32-bit digits
   is less than `2^64`. The middle sum is below `3*2^32`, and the final
   high word represents the high half of a product mathematically below
   `2^128`. No signed-overflow assumption, compiler-specific 128-bit type,
   or floating-point operation is used.
3. **Exact sign comparisons with bounded bit grids.** `add_dyadic` converts
   every nonnegative dyadic term into base-`2^64` digits aligned to bit
   `BBTC_VV_ORIGIN_BIT == -33152`. It adds the low and high portions of each
   shifted 64-bit source word using `add_word`, which propagates carry until
   exhausted and rejects carry outside the allocated span. `compare_big`
   scans from the most significant digit down. `signed_difference_minus_threshold`
   separates positive and negative terms: comparing their exact integer sums
   is mathematically equivalent to taking the sign of the three-term signed
   expression. `compare_absolute_threshold` enforces **both** half-space
   inequalities, so inclusive threshold equality is correct.

### Envelope arithmetic and checked failure

| Quantity | Conservative bound for initially enabled x87 | Why |
| --- | --- | --- |
| Lowest atom grid bit | `-16508` | `MIN_EXP - 2*p + 1` with `p=64` |
| Lowest threshold grid bit | `-33016` | Two lowest exponents added |
| Upper bit of any single finite value | strictly below `16384` | x87 normal maximum |
| Upper bit of two-value exact product | strictly below `32768` | Multiply two finite values |
| Upper bit of three-term nonnegative sum | strictly below `32770` | Sum less than `3*2^32768` |
| Reserved grid positions | `[-33152,33152)` | `1036` unsigned 64-bit words |

The lower bound provides 136 bits of reserve below the smallest possible
product bit; the upper bound provides over 380 bits of reserve above the
largest possible sum. A zero-valued dyadic term does not occupy any grid
bits, and its exponent is irrelevant. `add_dyadic` and `add_word` check the
array boundaries and signal internal failure if a future format violates
this domain; they never silently truncate a nonzero source word.

This bound is **not** universal to all C23 binary formats. In particular,
IEEE binary128 has 113 bits of significand and is deliberately rejected by
this implementation. C23 compilation alone is not evidence of format support.

### Independent reproducibility and environment limitations

The hardening increment adds `tests/validation/dyadic_oracle_driver.c` as a
*transport*, and `test_exact_dyadic_oracle.py` as the independent oracle. The
Python code derives inputs from format-specific integer fields and computes
outcomes with `fractions.Fraction`. It never evaluates expectations by
calling C's `frexp`, `scalbn`, `fabs`, or the integer-grid algorithm above.
The transport supplies exactly representable **hexadecimal binary** tokens to
`strtof`, `strtod`, and `strtold`; no Python binary64 intermediate is used
for the long-double cases. Every C return status and Boolean is compared;
any mismatch or missing response fails the test and prints the replay input.

The new cmocka matrix exercises all three native families, four supported
rounding modes, each common sticky exception flag individually, flag
combinations, success returns, NaN input rejection, and invalid-method early
return. Results from masked-exception configurations do **not** validate
caller-enabled unmasked floating-point traps, signaling NaNs, x87
noncanonical encodings, or processors where different floating-point units
have inconsistent FTZ/DAZ state. Full platform support still requires a
separate qualification review and real cmocka execution on the user's host.

The independent oracle is sampling evidence, not a proof. Python and the C
kernel could conceivably share an incorrect assumption about supported
formats; the input-encoding rules and exponent bounds therefore also need
review. See `VV0_1_B1B2_HARDENING.md` for exact invocation, seed,
configuration, and the distinction between local authoring checks and user
machine results.
