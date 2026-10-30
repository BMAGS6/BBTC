# VV0.1-B1b.3d-3b-1 — Exact Dyadic Decision: Source-Level Proof Audit

**Disposition:** conditional source-review finding for A01 and A03; **not** a B1
acceptance declaration. The host qualification must be repeated after staging
this checkpoint. A02, A04–A14, and the final disposition remain open in
`VV0_1_B1B3D2_ACCEPTANCE_AUDIT.md`.

**Reviewed source:** `tools/bbtc_vv/exact_dyadic.c` after B1b.3d-3a capability
hardening (source blob should be checked against the actual reviewed commit).
This document records reasoning about the implementation as written, rather
than assuming that a green CTest run constitutes an exhaustive proof.

## 1. Accepted mathematical problem (A01)

The accepted B1a contract defines exact represented-value comparisons:

- **Exact:** `r == o` as mathematical finite numbers, not bit patterns;
  opposite signs of zero compare equal. Tolerance must be zero.
- **Absolute:** `|o-r| <= t` for a nonnegative finite tolerance `t`.
- **Reference-relative:** `|o-r| <= t*|r|` for `r != 0` and nonnegative `t`.

The comparison is inclusive. These are not measurement-uncertainty tests, nor
do they establish any ballistic model's physical correctness.

For all `T >= 0`, the following identity is elementary and exact:

    |o-r| <= T  <=>  (o-r-T <= 0) AND (r-o-T <= 0).

`compare_absolute_threshold` implements the right side through two calls to
`signed_difference_minus_threshold`, stopping early when the first inequality
is false. `signed_difference_minus_threshold` accumulates positive and
negative dyadic terms independently and compares their *nonnegative integers*.
There is no rounding involved in choosing the sign. A zero sign is admitted,
which implements the inclusive inequality without an epsilon fudge factor.

The exact method uses C's native equality on *already validated finite*
native scalars. C numerical equality does not distinguish +0 and -0; on the
qualified binary formats, this correctly matches the specified mathematics.

**Review condition:** these statements apply only after the native-family
format gate and successful floating-environment isolation, not to arbitrary
floating-point objects or unmasked-trap execution environments.

## 2. Exact binary decomposition

For a supported native scalar with `p <= 64` significant binary bits,
`frexp*(fabs*(x), &e)` represents every nonzero finite `x` as

    |x| = f * 2^e,     1/2 <= f < 1.

`scalbn*(f, p)` yields an **exact** integer `m` in `[2^(p-1), 2^p)`.
Every binary significand bit fits in `uint64_t` even when `p=64`, because
`m <= 2^64-1`. The integer exponent stored in `bbtc_vv_atom_t` is `e-p`:

    x = sign(x) * m * 2^(e-p).

The normalized significand is integral and its scale operation cannot lose a
bit on the qualified format. Signed zero is represented with zero mantissa;
its sign does not affect any nonzero magnitude.

For x87 `long double` (`p=64`, `MIN_EXP=-16381`, `MAX_EXP=16384`), the
smallest positive subnormal has `e = MIN_EXP-p+1 = -16444`, so its atom's
integer exponent is `-16508`. The largest finite scalar has `e=16384`, so
its atom's integer exponent is `16320`.

    -16508 <= atom exponent <= 16320.

The narrower binary32/binary64 families lie inside this interval.
All these exponents and their pairwise sums fit `int32_t`.

## 3. Portable 64-by-64 product

`multiply_u64(a,b)` splits each factor into 32-bit digits. It forms
`p00`, `p01`, `p10`, `p11` with `uint64_t`, then computes

    mid  = high32(p00) + low32(p01) + low32(p10)
    low  = low32(p00) + 2^32 * low32(mid)
    high = p11 + high32(p01) + high32(p10) + high32(mid).

Here `mid <= 3*(2^32-1)`, so it cannot overflow `uint64_t`. Every partial
32-by-32 product is below `2^64`. The result exactly represents a product
smaller than `2^128`, and the high digit fits 64 bits because the mathematical
product of two 64-bit unsigned integers is smaller than `2^128`.

No nonstandard `__int128`, native floating multiplication, or undefined signed
integer overflow is required to construct a relative threshold.

## 4. Fixed-grid capacity proof (A03)

`BBTC_VV_ORIGIN_BIT=-33152`, `BBTC_VV_END_BIT=33152`, and
`BBTC_VV_WORDS=1036` describe exactly 66,304 binary positions.
A nonzero scalar term has an integer exponent no smaller than `-16508` and a
significand occupying at most 64 bits.

A relative threshold multiplies two such atoms. Hence:

    minimum nonzero threshold exponent = -16508 + (-16508) = -33016
    maximum threshold exponent         =  16320 +  16320  =  32640.

A product occupies at most 128 bits. Thus its highest possibly occupied
binary position is `< 32640+128 = 32768`. A scalar term is strictly less
than `2^16384`. A complete signed-side accumulator includes at most three
nonnegative magnitudes (two scalar atoms and one threshold), so its sum is
strictly less than

    2^32768 + 2*2^16384 < 2^32770.

The supported term positions are therefore contained in
`[-33016, 32770)`, **strictly inside** the allocated `[-33152, 33152)`
grid. There are at least 136 unused bit positions below the lowest possible
threshold exponent and more than 382 above the highest possible sum bound.

`add_dyadic` separates every shifted 64-bit word into a low part and a high
part. Shift counts are the remainder modulo 64, so are in `[0,63]`. The
`source[i] >> (64-shift)` expression executes **only** when `shift != 0`;
therefore its count is in `[1,63]`, never 64. Each partial addition goes
through `add_word`, which propagates unsigned carry with a bounds check
before accessing an indexed word. The maximum-sum proof implies that carry
cannot leave the allocated grid on an admissible, qualified input; the guard
still fails closed if the assumptions are somehow violated.

`compare_big` examines the highest limb first, yielding the exact unsigned
ordering of the two aligned integer magnitudes. Consequently its output
agrees with the sign of each signed inequality.

## 5. Directed boundary checks added in this increment

`test_exact_dyadic.c` now contains three additional cmocka test functions,
one per native family, each testing six exact identities. The cases include:

- Threshold `epsilon*epsilon`, where `epsilon` is the least subnormal, to
  exercise the minimum admissible relative-product exponent.
- `epsilon` versus `-epsilon` at relative tolerance `2` (inclusive PASS) and
  relative tolerance `epsilon` (FAIL).
- `M` versus `-M` at relative tolerances `nextafter(2,0)` (FAIL), `2`
  (inclusive PASS), and `M` (PASS despite the mathematical product `M*M`
  exceeding the scalar type's finite range).

These checks are *directed regression evidence*, not a substitute for the
source argument above. The existing independently implemented Python
`fractions.Fraction` oracle remains responsible for broad differential
verification under the qualified native formats.

## 6. Source-level disposition and remaining obligations

**A01:** source argument supports exact inclusive comparison for the admitted
finite native domain, subject to the platform/fenv assumptions and successful
internal arithmetic. The new test suite must pass before recording closure.

**A03:** the fixed-grid bit range is sufficiently large for every supported
scalar and relative threshold, with checked unsigned limb operations. The
new tests exercise extremal products; the proof depends on the stated `p<=64`
and x87 exponent limits enforced by the format gates.

**A02 remains OPEN** for a later whole-API success/failure-path review, even
though the dyadic kernel returns an exact Boolean for admitted inputs. The
other A04–A14 obligations also remain under review. Do not mark B1 accepted
based solely on this report.

Reproduce on the primary qualified host with `acceptance_capture.sh` and
retain the transcript and exact source Git commit. No production BBTC source,
public header, or physics implementation is changed by this increment.
