# VV0.1-B1b.3d-3b-2 — Exact-Metric Arithmetic Source and Bounds Audit

**Disposition:** conditional source-level finding supporting **A04** of the
B1b.3d-2 acceptance matrix; not a declaration that B1 is accepted.
Qualification of the new tests on the primary Arch GNU/Linux host is still
required. A05 (rounding) and the remaining whole-API obligations are separate.

**Numerical implementation reviewed:** `tools/bbtc_vv/exact_metric.c` (Git blob
`b835f4927301578b854bcd6672fede8ff735944f`), plus `exact_metric.h` (blob
`179b9158b5d7de6afd97a5a138df1ea4206fdff1`). These files are **not**
modified by this increment. Record the *actual* host commit after qualifying
and committing the associated test source; do not infer it from this document.

**Authority:** `VVUQ_COMPARISON_CORE_CONTRACT.md` v0.1.0,
`VV0_1_B1B3C1_EXACT_METRIC_PROOF.md`,
`VV0_1_B1B3D2_ACCEPTANCE_AUDIT.md`, and the previously qualified B1b.3d-3a
native-format gate. This document describes **software numerical properties**,
not empirical verification of any physical model or reference dataset.

## 1. Mathematical and representational obligations

Let `r` and `o` be *finite, admissible and exactly represented* native scalars.
The private helper supports precisely two metric methods:

- Absolute: `D = |o-r|`.
- Reference-relative: `Q = |o-r| / |r|`, requiring `r != 0`.

All operations in the formulae above denote **real-number operations on the
represented inputs**. They are not native rounded subtraction or division.

`bbtc_vv_exact_metric_t` represents a nonnegative exact rational:

    N = sum(word[i] * 2^(64*i), i = 0 .. used-1)
    metric = (N * 2^numerator_exponent) /
             (denominator * 2^denominator_exponent)

A successful output satisfies these invariants:

1. `0 <= used <= 520`; `word[used-1] != 0` when `used > 0`.
2. `word[i] == 0` for all `i >= used`. Limb order, not byte order, is little-endian.
3. `denominator != 0`. For absolute mode it is exactly `1 * 2^0`.
4. For relative mode the denominator is the **magnitude of the reference**,
   including its exact power of two; changing the sign of `r` must not change it.
5. An exact-zero numerator has `used=0`, `numerator_exponent=0`, and a **valid
   nonzero denominator**. The representation need not be in reduced form.
6. On failure all *field values* are zero. C structure padding bytes are not
   guaranteed to have any particular contents and must not be compared with
   `memcmp`.

The large fixed buffer is an internal representation, not a claim that its
exact rational can be converted into the original native type without range
loss. Native scalar rounding belongs to B1b.3c-2, audited separately.

## 2. Source-level decomposition argument

`atom_float`, `atom_double`, and `atom_long_double` use their own typed
`fabs*`, `frexp*`, and `scalbn*` operations. For any nonzero supported input:

    |x| = f * 2^e,   1/2 <= f < 1
    p = native significand precision, p in {24,53,64}
    m = f * 2^p,   an exact integer with 2^(p-1) <= m < 2^p
    x = sign(x) * m * 2^(e-p).

Thus `m` is representable as `uint64_t`, **including `p=64`**: the upper
bound is strict, so converting `m` never requires representing `2^64`.
`frexp*` normalization of a positive subnormal is required to behave just
like normalization of any other nonzero finite represented scalar. The
qualified format gate and its `TRUE_MIN` round-trip probe supply the execution
assumptions for this argument; arbitrary environments are not certified.

A zero atom has a zero mantissa, regardless of its original sign. Its
exponent cannot influence the origin chosen in `form_difference`.

For the broadest supported x87 binary80 format:

    p = 64; MIN_EXP = -16381; MAX_EXP = 16384.
    least subnormal = 2^(MIN_EXP-p) = 2^-16445.
    frexp exponent of least subnormal = MIN_EXP-p+1 = -16444.
    smallest atom exponent = MIN_EXP-2p+1 = -16508.
    largest atom exponent  = MAX_EXP-p = 16320.

Hence every nonzero atom exponent is inside `[-16508,16320]` and the
maximum difference between two atom exponents is **32,828**. Both endpoint
values and their difference fit comfortably into `int32_t`. Binary32 and
binary64 have smaller exponent envelopes.

## 3. Proof of the fixed 520-limb capacity (A04)

`form_difference` selects the **minimum exponent among nonzero operands** as
its common origin `E`. No term is ever shifted negatively. A nonzero aligned
significand is strictly smaller than `2^64` before shifting, so a term whose
atom exponent differs from `E` by `d` occupies fewer than `d+64` bits.

The worst possible exponent displacement is `d <= 32828`, implying that
any one aligned operand has magnitude `< 2^(32828+64) = 2^32892`.
The nonnegative sum of two aligned magnitudes is then `< 2^32893`.
The magnitude of their difference cannot exceed that sum. Therefore the
integer numerator's **bit length is at most 32,893**.

The representation has exactly:

    520 * 64 = 33,280 bits
    33,280 - 32,893 = 387 bits of unused theoretical headroom.

This is a conservative bound: it does not assume that the two extremal atom
exponents can occur simultaneously with arbitrary mantissa patterns.

A validated `uint32_t used` index can represent all 520 limbs; a signed
32-bit exponent can represent every selected origin and denominator exponent.
No heap growth, allocation failure, or hidden truncation is needed to
represent any of the admitted differences.

## 4. Review of the C23 unsigned arithmetic

### 4.1 `add_digit` and `add_shifted`

`add_digit` adds a `uint64_t` digit and propagates a one-bit carry. Unsigned
addition is defined modulo `2^64` in ISO C. `new < prior` detects the carry.
It checks the index before dereferencing a limb and returns `false` if a
nonzero carry would escape the array.

`add_shifted` decomposes its bit offset into `base=offset/64` and
`shift=offset%64`. Consequently the left shift count is in `[0,63]`.
Its right shift is attempted *only if `shift != 0`*, so the right-shift count
`64-shift` is in `[1,63]` (never 64). The delegated `add_digit` checks any
last-limb or carried write before access. A shift of zero magnitude succeeds
without attempting to inspect an out-of-range exponent.

### 4.2 `compare_magnitude` and `subtract_magnitude`

`compare_magnitude` visits limbs from most to least significant. That is the
exact ordering for base-`2^64` unsigned integers.

`subtract_magnitude` is called only when the selected left operand is known
to be at least the right operand. For every limb it evaluates:

    partial = original - smaller_word        (mod 2^64)
    first   = original < smaller_word
    result  = partial - incoming_borrow       (mod 2^64)
    second  = partial < incoming_borrow
    outgoing_borrow = first OR second.

This two-stage borrow test remains correct even when adding the incoming
borrow to `smaller_word` would overflow an unsigned word. The final borrow
must be zero under the caller's magnitude-order precondition; an unexpected
nonzero borrow becomes an internal numerical failure.

### 4.3 Signed cases and canonicalization

When `reference` and `observation` have different signs, the absolute
difference is their magnitude **sum**; `form_difference` initializes a copy
of the first aligned magnitude and adds the second. When signs agree, the
absolute difference is the larger magnitude **minus** the smaller. This
holds even when either original sign was the sign of zero, since zero has
magnitude zero in the atom representation.

`form_difference` recomputes `used` by scanning down from the most
significant limb. If the integer is zero, it resets the numerator exponent
to zero. The result begins as a cleared compound literal; no operation
writes into limbs beyond the final highest nonzero limb. Consequently unused
limbs stay zero, including on reuse after a previous successful call.

The opposite-sign maximal-finite case can have a **mathematical magnitude
larger than the largest finite input**. Its exact rational is still valid;
no overflow classification belongs in this helper.

## 5. Reference-relative denominator argument

The reference is first validated as finite and, in reference-relative mode,
nonzero. Its unsigned native significand is `r.mantissa > 0`, and
`r.exponent` is its exact native integer exponent. The helper writes:

    denominator          = r.mantissa;
    denominator_exponent = r.exponent.

The quotient therefore equals `D/|r|` precisely. Neither the sign nor the
magnitude of the observation is substituted for the reference. A ratio such
as `1/3` stays rational rather than being truncated to a dyadic scalar.

## 6. Invalid inputs and floating-point environment

The helper checks for a null output, clears all output fields, then checks the
method before numeric examination. An outer `feholdexcept` encloses the
previously qualified dyadic validation kernel and the exact construction.
Only validated finite scalars reach `atom_*` or `form_difference`.

If dyadic validation rejects the input, if the relative reference is zero,
if a range/carry/borrow check fails, or if `fesetenv` fails, the returned
status is unsuccessful and the output fields are cleared again. Unknown
dyadic status codes fail closed as `NUMERICAL_FAILURE`. Nested fenv
save/restore is intentional; the final whole-API environmental audit (A09)
remains a separate obligation.

Scope exclusions remain in effect: unmasked traps, unfamiliar floating
representations, fast-math, unverified excess precision and noncanonical x87
encodings are **not** newly supported by this source review.

## 7. Focused regression tests introduced here

The new directed cmocka tests (10 tests; three tests per family plus one
reuse/error test) assert *integer identities* rather than computing expected
answers with this kernel or native rounded arithmetic:

1. **Long borrow:** for every family, compare `1` to its least positive
   subnormal `epsilon=2^q`. After factoring out the normalized atom exponent,
   the exact numerator is `2^A-2^(p-1)`, so **every bit** in the interval
   `[p-1,A)` is set. The test checks all 520 words, including the zero tail.
2. **Opposite finite maxima:** `|M-(-M)|=2M`, with
   `M=(2^p-1)*2^(MAX_EXP-p)`. The test checks the exact coefficient and its
   carry into the second limb for x87 `p=64`.
3. **Negative reference at subnormal scale:** `|-epsilon-0|/|-epsilon|=1`
   is tested by independently checking the exact numerator, its exponent,
   the positive reference-magnitude denominator and the denominator exponent.
   An immediately reused result then checks a zero relative numerator against
   a nonzero maximal-finite reference.
4. **Failure clearing:** a previously populated multi-limb result is reused
   for an invalid method. Every numerator word and all metadata fields must
   be zeroed. Padding bytes are intentionally not compared.

These are supplemental focused cases. The prior independent Python
`fractions.Fraction` metric and full-API oracles remain the broad
*differential* check. Running the same reproducible random corpus through
multiple compilers is useful cross-configuration evidence, not new distinct
mathematical inputs or an exhaustive proof.

## 8. Conditional audit finding and acceptance gate

**A04 source finding:** the exact-metric operations and the 520-limb buffer
satisfy the stated mathematical capacity and rational-representation
requirements *under the qualified binary32/binary64/x87 and compiler/fenv
assumptions*, subject to successful reproduction of the test campaign.

The acceptance-capture gate for this increment requires:

- Strict optimized GCC and Clang qualification, all nine CTest entries.
- Clang ASan/UBSan qualification, all nine CTest entries.
- The exact-metric cmocka suite passing **23/23 cases** in each build.
- All existing independent mathematical oracles passing unchanged.
- Production-only build still defining no developer VVUQ symbols or targets.
- Clean whitespace, exact file list, reproducible source/toolchain evidence.

The implementation source and public production library remain unchanged by
this increment. Do not infer closure of A05 (correct rounding), A07 (policy
precedence), A09 (all FP environment paths), A13 (oracle independence), or
final B1 acceptance. The next planned audit is **B1b.3d-3b-3** (exact binary
rounding and classification).
