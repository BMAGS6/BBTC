# VV0.1-B1b.3c-1 — Exact Error Magnitudes: Construction and Proof Obligations

**Status:** prototype implementation; requires primary GCC/Clang/cmocka qualification
**Scope:** exact absolute differences and exact reference-relative ratios, **not** native rounding, metric-state classification, or the final `bbtc_vv_compare_*` API.

## 1. Mathematical contract

For finite, admissible native scalar values `r` and `o`, define

- `D = |o-r|`, the exact absolute error;
- `Q = |o-r|/|r|` for `r != 0`, the exact reference-relative error.

Each input is the **exact represented dyadic value**. The code does not
interpret an original decimal token or a physically meaningful uncertainty.
It does not change the exact PASS/FAIL kernel and never computes a rounded
native subtraction/division to construct either error.

The internal `bbtc_vv_exact_metric_t` stores

```
                  used-1
metric = (2^E *  SUM  word[i] * 2^(64*i)) / (denominator * 2^F)
                    i=0
```

with a nonnegative numerator, `denominator > 0`, integral signed binary
exponents `E,F`, and little-endian **limb order**, independent of host byte
endianness. Absolute mode sets `(denominator,F)=(1,0)`; reference-relative
mode uses the unsigned normalized reference mantissa/exponent. No attempt is
made to reduce the rational. All unused words are zero; `used=0` means exact
zero with a nonzero denominator on successful returns.

## 2. Binary scalar decomposition

Under the existing B1b.2 supported-format gate, for every nonzero finite
native scalar `x`, `frexp*` produces `fraction` with `1/2 <= |fraction| < 1`
and integer `frexp_exponent`. Since `p = *_MANT_DIG <= 64`, the quantity
`fraction * 2^p` is exactly an integer `m` with `2^(p-1) <= m < 2^p`.
The exact represented scalar is `(-1)^sign * m * 2^(frexp_exponent-p)`.
No integer cast is performed until that conversion is exactly representable
in `uint64_t`. In particular, `m` is **strictly less** than `2^64` even when
`p=64`. Zero is represented separately, with a zero mantissa.

This operation relies on the qualified binary format, gradual subnormal
support and compiler/fenv assumptions already stated for B1b.2. Both native
`float` and `long double` use their own libm operations; there is no narrowing
to `double`.

## 3. Finite bit-capacity bound

For the most expansive supported family, x87 binary80 with `p=64`,
`MIN_EXP=-16381`, `MAX_EXP=16384`, normalizing even the least subnormal gives
an integer mantissa exponent of no less than

`MIN_EXP - 2p + 1 = -16508`.

At the upper end the exponent is at most `MAX_EXP-p = 16320`.
Thus a pair of nonzero atoms has an exponent-anchor displacement at most
`16320-(-16508)=32828` bits. A 64-bit significand occupies at most 64
further bits. Summing two nonnegative terms adds at most one carry bit.
Therefore the aligned magnitude occupies no more than **32893 bits**.
The fixed 520-limb (33,280-bit) buffer contains it with at least 387 bits
of spare headroom. Binary32 and binary64 fit narrower intervals. The current
format gate forbids binary128, decimal, and unfamiliar representations.

All shifts have counts in `[0,63]`, never the width of an integer type.
Digit carry, signed-free subtraction borrow, and range of every destination
index are checked. A violation returns NUMERICAL_FAILURE with zeroed output
instead of silently truncating a significant bit. `int32_t` easily covers the
finite input exponent ranges and their differences on supported families.

## 4. Exact absolute difference

After aligning each nonzero operand to the smaller binary exponent `E`, its
magnitude is an exact unsigned big integer. The sign distinction is simple:

- If signs differ, add the magnitudes: `|o-r|=|o|+|r|`.
- If signs agree, subtract the lesser unsigned magnitude from the greater.

Unsigned limb arithmetic and carry/borrow propagation are exact. The
mathematical difference is `N*2^E` even when `|o-r|` is larger than
`LDBL_MAX` or smaller than any native normal scalar. For both zero operands,
`N=0` and the exponent anchor is canonicalized to zero. Oppositely signed
zeros contribute zero magnitudes and remain mathematically equal.

## 5. Exact relative difference

For nonzero reference `r`, its decomposition yields the exact positive
denominator `m_r*2^e_r`. Preserve the absolute error numerator and record
`(denominator, denominator_exponent)=(m_r,e_r)`. Then

`Q = (N*2^E)/(m_r*2^e_r)`

is an exact rational. The denominator integer fits 64 bits, even if the
mathematical quotient is not dyadic (e.g. `1/3`). There is **no** native
floating-point division, no overflow, and no underflow of the exact rational.

## 6. Environment and error behavior

The pointer is checked and output zeroed before any floating input inspection.
The mode must be ABSOLUTE or REFERENCE_RELATIVE; EXACT mode has no diagnostic
and is rejected at this private helper level. With the environment held,
B1b.2's already-tested kernel checks for supported families, NaN precedence,
and finite inputs. Only then does the metric code perform native-family
decomposition and unsigned arithmetic. A zero reference in relative mode is
OUTSIDE_DOMAIN. Every failure leaves an entirely zero output; success has a
strictly positive denominator even if its numerator is zero.

The outer `feholdexcept`/`fesetenv` protects the full operation. Unmasked
traps, noncanonical extended encodings, unexpected C environments, and unsafe
fast-math settings remain outside the certified domain. The helper does not
claim to inspect caller trap masks or add support beyond B1b.2.

## 7. Verification and limitations

A reproducible independent Python `fractions.Fraction` oracle should
reconstruct exact values from separate integer-generated hexadecimal dyadic
operands and compare each to the returned `(N,E,m_r,e_r)` rational. Its
arithmetic must not copy this implementation's limb operations.

Required tests include equal values, oppositely signed zero, very distant
exponents, finite extremes whose difference exceeds the native range,
non-dyadic relative ratios, negative references, all four rounding modes,
preexisting sticky flags, structural and numeric errors, and supported
subnormals. Run strict optimized GCC/Clang, sanitizers, and the existing
B1b.2 differential oracle. Agreement on sampled cases is evidence, **not**
a formal mathematical proof.

**Deferred:** exact nearest-even rounding, `FINITE_EXACT` / `FINITE_ROUNDED` /
`EXCEEDS_RANGE` / `ROUNDED_TO_ZERO` / `UNRESOLVED` classification, final result
struct population and callable `bbtc_vv_compare_*` functions. No unfinished
metric is presented to consumers as a complete diagnostic.
