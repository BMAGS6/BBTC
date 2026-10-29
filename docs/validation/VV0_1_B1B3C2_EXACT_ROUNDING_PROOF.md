# VV0.1-B1b.3c-2 — Correctly Rounded Binary Diagnostics: Construction and Proof

**Status:** implementation candidate; primary GCC/Clang/cmocka qualification pending.

**Scope:** independently verifiable conversion of an exact nonnegative rational
metric into the *native* `float`, `double`, or qualified x87 `long double` type,
including a fidelity classification. No BBTC production header, external
ballistics model, physical reference dataset, or complete `bbtc_vv_compare_*`
entrypoint is changed by this increment.

## 1. Contract and independent decisions

Let the B1b.3c-1 representation be

```
           N × 2^E
    X =  --------- ,    N >= 0, d > 0.
           d × 2^F
```

`N` is a little-endian 520-limb, base-`2^64` unsigned integer. The exponents
`E` and `F` are signed integers. The representation need not be reduced. The
converter must not use a floating-point subtraction, division, multiplication,
`nextafter`, or native cast to decide which representable value is correct.
All classification decisions are made on mathematical `X` using exact integer
comparisons. The exact PASS/FAIL finding remains the exclusive responsibility
of B1b.2 and is never modified here.

The accepted rule is **round to nearest, ties to even**, without regard to the
caller's incoming rounding mode. `EXCEEDS_RANGE` means the *mathematical* value
is larger than the target's largest finite value, even if an ordinary floating
conversion could round down to that finite maximum. `ROUNDED_TO_ZERO` is a
strictly positive X whose nearest-even representation is zero. All such
sentinel results encode +0 rather than an invented infinity or negative zero.

## 2. Exact comparison without overflowing an intermediate

To compare `X` with `m * 2^k`, cross-multiply using only nonnegative integers:

```
    compare(N * 2^E, (d*m) * 2^(F+k)).
```

The coefficient `m` is at most 64 bits. Its product with a 64-bit denominator
fits in 128 bits, computed with four portable 32-bit partial products, as in
the accepted B1b.2 dyadic kernel. For a midpoint comparison, use

```
    compare(N * 2^(E+1), d*(2*m+1) * 2^(F+k)).
```

Here `2*m+1` may require 65 bits; its product with `d` occupies at most 129
bits, which fits in three 64-bit words. The implementation forms this product
as `2*(d*m)+d`, propagating the carry into the third word without any
`__int128` extension.

The integer comparison treats both integers as *views* of limb arrays with an
independent signed base-2 exponent. It compares their most-significant binary
power first; if equal, it compares virtual 64-bit digits in descending order.
The virtual-digit routine never shifts an integer by 64, never dereferences an
out-of-range limb, and never materializes an enormous product. Thus numbers
outside the representable range of any native floating-point type are safe.

For valid B1b.3c-1 inputs, the numerator contains no more than 33,280 bits;
`E` and `F` and the target conversion exponents are within the established
x87 finite/subnormal envelope. All absolute bit-position calculations and
grid offsets are performed in 64-bit integers. The maximal virtual grid spans
well under 100,000 bit positions, so its word indexes also fit `uint64_t`.
The converter independently rejects a zero denominator, invalid `used` count,
nonzero unused words, and a zero highest declared limb. It cannot interpret a
malformed representation as an otherwise successful numerical result.

## 3. Native binary format and finite overflow

For a native family of `p` significand bits, C exponent limits `MIN_EXP` and
`MAX_EXP`, the largest finite mathematical member is exactly

```
    (2^p - 1) × 2^(MAX_EXP - p).
```

The smallest positive subnormal quantum is

```
    q = 2^(MIN_EXP - p).
```

We first compare X with the largest finite value **exactly**; if larger,
classification is `EXCEEDS_RANGE` and the scalar sentinel is +0. In
particular, the algorithm does not infer `EXCEEDS_RANGE` from an IEEE infinity
or the runtime `FE_OVERFLOW` flag.

## 4. Binade and rounding quantum

For nonzero, within-range X, let `n=bit_length(N)`, `d_bits=bit_length(d)`, and

```
    c = n - d_bits + E - F.
```

Since `2^(n-1) <= N < 2^n` and `2^(d_bits-1) <= d < 2^d_bits`, X satisfies
`2^(c-1) < X < 2^(c+1)`. Therefore `floor(log2(X))` is either `c-1` or `c`;
an *exact integer comparison* to `2^c` identifies which. Denote this
exponent by `b`.

For normal-range results, the coefficient quantum exponent is `b-p+1`.
For subnormal results, it is fixed at `MIN_EXP-p`. The fixed subnormal quantum
also handles mathematical X smaller than half of the least subnormal.

The floor integer coefficient `m=floor(X / 2^quantum)` is found by binary
search on unsigned coefficients in `[0, 2^p-1]`. This range includes the
possible subnormal coefficients and all normal significands. A `uint64_t`
midpoint using division and modulo of `high-low` avoids overflow even for
`p=64` where the upper endpoint is `UINT64_MAX`.

An equality check `X == m*2^quantum` proves `FINITE_EXACT`. Otherwise, an
exact midpoint comparison against `(m+1/2)*2^quantum` selects m or m+1:

- X below midpoint: m.
- X above midpoint: m+1.
- X exactly at midpoint: the even coefficient, determined by m's low bit.

If rounding makes a `p+1`-bit coefficient `2^p`, normalize it to `2^(p-1)`
with its quantum exponent incremented by one. This handles a carry into the
next binary binade even for `p=64`, without storing 2^64 in `uint64_t`.
A nonzero X rounding to coefficient zero is `ROUNDED_TO_ZERO`; any other
inexact finite result is `FINITE_ROUNDED`.

## 5. Final native conversion and fenv isolation

Only once the exact integer algorithm has selected an allowed coefficient
and exponent do we construct the result using the matching native family:

```
    scalbnf((float)coefficient, exponent);
    scalbn((double)coefficient, exponent);
    scalbnl((long double)coefficient, exponent);
```

The coefficient has at most p bits and is *exactly representable* in its
corresponding p-bit binary floating type (including all uint64 values for
qualified p=64 x87). Scaling an exactly represented integer by a power of two
produces the previously proved representable normal/subnormal target value;
there is no rounding-sensitive intermediate conversion. These conversions
occur inside an outer `feholdexcept`/`fesetenv` pair. The incoming rounding
mode and sticky exception flags are restored before the function returns.

The converter's environment gate recognizes the same binary32, binary64, and
binary80 (or verified binary64-backed long double) families as B1b.2. It
rejects unsafe fast-math builds and relies on the same configured GCC/Clang
`-frounding-math -ffp-contract=off -fno-fast-math` assumptions. Runtime traps,
noncanonical extended encodings, and novel platforms are **not** certified.

## 6. Acceptance tests and independent oracle

The cmocka tests cover positive exact zero, exact integers, exact dyadic
fractions, the non-dyadic rational 1/3, both directions of halfway rounding,
positive-to-zero underflow, the last subnormal/first normal midpoint, the
largest finite value, mathematical overflow, native x87 preservation, and
unmodified caller rounding modes and sticky flags on success/failure.

The optional Python oracle constructs a `fractions.Fraction` from integer
numerator/denominator and binary exponents, then performs a **different**
algorithm: one Python arbitrary-precision `divmod` and exact remainder-based
nearest-even decision. It parses C's `%a` / `%La` output into a `Fraction`
without narrowing `long double` through Python `float`. The corpus includes
mandatory boundary cases and independently seeded randomized integers and
exponents over the x87 envelope. Test failures print the full seed and case
for reproduction.

Strict optimized GCC/Clang, sanitizer builds, cmocka, and the Python oracle on
the primary host are required before an acceptance commit. Passing finite
sampled tests does **not** imply exhaustive correctness, portability to
binary128/decimal formats, or validation of a ballistic prediction.

## 7. Explicitly deferred

The full `bbtc_vv_compare_float`, `_double`, and `_long_double` wrappers,
structural policy/result validation integration, caller-visible complete
metric/result assembly, and full B1b.3d verification campaign remain
**deferred to B1b.3c-3 and B1b.3d**. This is a composable numerical component,
not an incomplete API masquerading as a finished one. The separate future
certified-error-interval checkpoint remains independent.
