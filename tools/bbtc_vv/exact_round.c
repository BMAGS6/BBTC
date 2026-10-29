/**
 * @file exact_round.c
 * @brief Provably nearest-even native-binary conversion of an exact rational.
 *
 * @details
 * The accepted exact_metric.h representation is X=N*2^E/(d*2^F), N>=0,
 * d>0. All decisions compare N*2^E to integer multiples of d*2^F using
 * exact 64-bit limb arithmetic; neither native subtraction nor division is
 * permitted to choose a result. The output is assembled only AFTER an exact
 * integer decision proves it representable in the target family.
 *
 * The proof and supported exponent envelope are documented in
 * docs/validation/VV0_1_B1B3C2_EXACT_ROUNDING_PROOF.md.
 *
 * There is no dynamic allocation. Unsigned multiplication uses portable
 * 32x32->64 partial products. The maximum exact comparison multiplier is
 * (2*(2^64-1)+1)*(2^64-1) < 2^129, so three uint64_t words suffice.
 *
 * Decimal conversions, excess precision in unqualified environments,
 * unmasked traps, unsafe math transformations, and noncanonical formats are
 * deliberately out of scope. A sanitized and independently reproduced test
 * campaign is a separate acceptance requirement.
 */
#include "exact_round.h"

#include <fenv.h>
#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#if defined(__FAST_MATH__)
    #error "BBTC VVUQ rational rounding cannot use -ffast-math"
#endif
#if defined(__FINITE_MATH_ONLY__)
    #if __FINITE_MATH_ONLY__ != 0
        #error "BBTC VVUQ rational rounding requires finite-math-only disabled"
    #endif
#endif
#if defined(__clang__)
    #pragma STDC FENV_ACCESS ON
#endif

_Static_assert(sizeof(uint64_t) * 8 == 64, "64-bit integer limbs required");
_Static_assert(BBTC_VV_METRIC_LIMBS >= 515, "x87 difference needs >=515 limbs");

/** @brief Up to 129 exact multiplier bits, least-significant word first. */
typedef struct bbtc_vv_round_multiplier_t
{
    uint64_t word[3];
    size_t used;
} bbtc_vv_round_multiplier_t;

/** @brief A read-only integer word sequence positioned at an absolute bit. */
typedef struct bbtc_vv_round_view_t
{
    const uint64_t* word;
    size_t used;
    int64_t exponent;
} bbtc_vv_round_view_t;

/**
 * @brief Return the bit length of a nonzero native unsigned 64-bit limb.
 * The loop has at most 64 iterations and does not assume compiler intrinsics.
 */
static unsigned bit_length_u64(uint64_t x)
{
    unsigned count = 0;
    while (x != 0)
    {
        ++count;
        x >>= 1;
    }
    return count;
}

/** @brief Exact bit length of a normalized little-endian integer. */
static int64_t view_bit_length(const bbtc_vv_round_view_t* view)
{
    if (view->used == 0)
        return 0;
    return (int64_t)(view->used - 1) * INT64_C(64) +
        (int64_t)bit_length_u64(view->word[view->used - 1]);
}

/**
 * @brief Exact 64-by-64 unsigned multiplication without __int128.
 *
 * Each 32x32 partial product fits in uint64_t. The largest middle term is
 * below 3*2^32, so the shift and carry computations cannot overflow.
 */
static bbtc_vv_round_multiplier_t multiply_u64(uint64_t a, uint64_t b)
{
    const uint64_t a0 = (uint32_t)a;
    const uint64_t a1 = a >> 32;
    const uint64_t b0 = (uint32_t)b;
    const uint64_t b1 = b >> 32;
    const uint64_t p00 = a0 * b0;
    const uint64_t p01 = a0 * b1;
    const uint64_t p10 = a1 * b0;
    const uint64_t p11 = a1 * b1;
    const uint64_t mid = (p00 >> 32) + (uint32_t)p01 + (uint32_t)p10;
    bbtc_vv_round_multiplier_t product =
    {
        .word =
        {
            (mid << 32) | (uint32_t)p00,
            p11 + (p01 >> 32) + (p10 >> 32) + (mid >> 32),
            0
        },
        .used = 2
    };
    if (product.word[1] == 0)
        product.used = 1;
    return product;
}

/**
 * @brief Construct d*(2*m+1) exactly, including the 129th possible bit.
 * This multiplier is used solely to compare against the halfway threshold.
 */
static bbtc_vv_round_multiplier_t midpoint_multiplier(uint64_t d, uint64_t m)
{
    const bbtc_vv_round_multiplier_t product = multiply_u64(d, m);
    bbtc_vv_round_multiplier_t midpoint =
    {
        .word =
        {
            product.word[0] << 1,
            (product.word[1] << 1) | (product.word[0] >> 63),
            product.word[1] >> 63
        },
        .used = 3
    };
    const uint64_t previous = midpoint.word[0];
    midpoint.word[0] += d;
    if (midpoint.word[0] < previous)
    {
        const uint64_t high_previous = midpoint.word[1];
        ++midpoint.word[1];
        if (midpoint.word[1] < high_previous)
            ++midpoint.word[2];
    }
    while (midpoint.used > 1 && midpoint.word[midpoint.used - 1] == 0)
        --midpoint.used;
    return midpoint;
}

/**
 * @brief Read the 64-bit digit at a shifted, virtual bit-grid position.
 *
 * The integer's exponent is interpreted relative to a shared binary origin.
 * The caller guarantees a nonnegative shift. The two source digits, when
 * present, are bitwise combined; all out-of-range digits are zero. No
 * pointer arithmetic is performed outside the declared source extent.
 */
static uint64_t shifted_digit(
    const bbtc_vv_round_view_t* view,
    uint64_t shift_bits,
    uint64_t index
)
{
    const uint64_t base = shift_bits / UINT64_C(64);
    const unsigned shift = (unsigned)(shift_bits % UINT64_C(64));
    if (index < base)
        return 0;
    const uint64_t offset = index - base;
    uint64_t digit = 0;
    if (offset < (uint64_t)view->used)
        digit = view->word[(size_t)offset] << shift;
    if (shift != 0 && offset > 0 &&
        offset - 1 < (uint64_t)view->used)
        digit |= view->word[(size_t)(offset - 1)] >> (64U - shift);
    return digit;
}

/**
 * @brief Compare two exact integers multiplied by independent powers of two.
 *
 * First compare their absolute most-significant bit positions. Different
 * positions prove the relation immediately. If equal, align both views to
 * the lower exponent and compare unsigned 64-bit grid words top-down.
 * All exponent and index arithmetic is int64_t/uint64_t to avoid signed
 * overflow for the documented inputs and defensively reject no valid cases.
 */
static int compare_views(
    bbtc_vv_round_view_t left,
    bbtc_vv_round_view_t right
)
{
    if (left.used == 0 || right.used == 0)
        return left.used == right.used ? 0 : (left.used == 0 ? -1 : 1);

    const int64_t top_left = left.exponent + view_bit_length(&left) - 1;
    const int64_t top_right = right.exponent + view_bit_length(&right) - 1;
    if (top_left != top_right)
        return top_left < top_right ? -1 : 1;

    const int64_t origin = left.exponent < right.exponent ?
        left.exponent : right.exponent;
    const uint64_t shift_left = (uint64_t)(left.exponent - origin);
    const uint64_t shift_right = (uint64_t)(right.exponent - origin);
    const uint64_t last = (uint64_t)(top_left - origin) / UINT64_C(64);
    for (uint64_t cursor = last + 1; cursor != 0; --cursor)
    {
        const uint64_t i = cursor - 1;
        const uint64_t a = shifted_digit(&left, shift_left, i);
        const uint64_t b = shifted_digit(&right, shift_right, i);
        if (a != b)
            return a < b ? -1 : 1;
    }
    return 0;
}

/**
 * @brief Compare exact X=N*2^E/(d*2^F) with m*2^k.
 *
 * This cross-multiplies only nonnegative integers:
 *     compare N*2^E versus (d*m)*2^(F+k).
 * The multiplication is exact up to 128 bits, with no native floating point.
 */
static int compare_candidate(
    const bbtc_vv_exact_metric_t* exact,
    uint64_t coefficient,
    int64_t exponent
)
{
    const bbtc_vv_round_multiplier_t product =
        multiply_u64(exact->denominator, coefficient);
    const bbtc_vv_round_view_t numerator =
        {exact->word, exact->used, exact->numerator_exponent};
    const bbtc_vv_round_view_t denominator =
        {product.word, coefficient == 0 ? 0 : product.used,
         (int64_t)exact->denominator_exponent + exponent};
    return compare_views(numerator, denominator);
}

/**
 * @brief Compare 2*X with (2*m+1)*2^k for nearest-even selection.
 * The exact midpoint is represented by the three-limb multiplier, not by
 * an inexact floating-point average (which can overflow or lose bits).
 */
static int compare_midpoint(
    const bbtc_vv_exact_metric_t* exact,
    uint64_t coefficient,
    int64_t exponent
)
{
    const bbtc_vv_round_multiplier_t product =
        midpoint_multiplier(exact->denominator, coefficient);
    const bbtc_vv_round_view_t numerator =
        {exact->word, exact->used, (int64_t)exact->numerator_exponent + 1};
    const bbtc_vv_round_view_t denominator =
        {product.word, product.used,
         (int64_t)exact->denominator_exponent + exponent};
    return compare_views(numerator, denominator);
}

/**
 * @brief Reject malformed exact metrics instead of trusting used/denominator.
 * Requiring zero unused words prevents hidden significant limbs from being
 * ignored. Only the canonical positive denominator and nonnegative numerator
 * representation is accepted; no numeric claim follows an invalid record.
 */
static bool representation_ok(const bbtc_vv_exact_metric_t* exact)
{
    if (exact->denominator == 0 || exact->used > BBTC_VV_METRIC_LIMBS)
        return false;
    if (exact->used != 0 && exact->word[exact->used - 1] == 0)
        return false;
    for (size_t i = exact->used; i < BBTC_VV_METRIC_LIMBS; ++i)
    {
        if (exact->word[i] != 0)
            return false;
    }
    return true;
}

/**
 * @brief Exact scalar rounding decision (integer only, independent of fenv).
 *
 * e = floor(log2 X) is established from bit lengths and at most one exact
 * comparison with 2^e. Normal values use a binary quantum 2^(e-p+1);
 * subnormal values always use 2^(MIN_EXP-p). Binary search finds the floor
 * coefficient. Comparing 2X to the exact midpoint chooses the neighboring
 * coefficient, resolving ties in favor of the even low bit.
 *
 * The exact mathematical range test is performed BEFORE coefficient search:
 * X>MAX_FINITE is EXCEEDS_RANGE even if nearest-even could round to MAX.
 * No native conversion occurs in this function.
 */
static bbtc_vv_metric_state_e select_rounding(
    const bbtc_vv_exact_metric_t* exact,
    unsigned precision,
    int minimum_exponent,
    int maximum_exponent,
    uint64_t* out_coefficient,
    int* out_exponent
)
{
    *out_coefficient = 0;
    *out_exponent = 0;
    if (exact->used == 0)
        return BBTC_VV_METRIC_FINITE_EXACT;

    const uint64_t max_coefficient = precision == 64 ? UINT64_MAX :
        (UINT64_C(1) << precision) - UINT64_C(1);
    const int64_t max_quantum = (int64_t)maximum_exponent - (int64_t)precision;
    if (compare_candidate(exact, max_coefficient, max_quantum) > 0)
        return BBTC_VV_METRIC_EXCEEDS_RANGE;

    const bbtc_vv_round_view_t numerator =
        {exact->word, exact->used, exact->numerator_exponent};
    const uint64_t denominator_word[] = {exact->denominator};
    const bbtc_vv_round_view_t denominator =
        {denominator_word, 1, exact->denominator_exponent};
    const int64_t candidate = view_bit_length(&numerator) -
        view_bit_length(&denominator) +
        (int64_t)exact->numerator_exponent -
        (int64_t)exact->denominator_exponent;
    const int64_t floor_exponent = compare_candidate(exact, 1, candidate) < 0 ?
        candidate - 1 : candidate;
    const int64_t normal_floor = (int64_t)minimum_exponent - 1;
    int64_t quantum = floor_exponent < normal_floor ?
        (int64_t)minimum_exponent - (int64_t)precision :
        floor_exponent - (int64_t)precision + 1;

    /* The largest possible native coefficient always fits uint64_t. */
    uint64_t low = 0;
    uint64_t high = max_coefficient;
    while (low < high)
    {
        const uint64_t delta = high - low;
        const uint64_t middle = low + delta / 2 + delta % 2;
        if (compare_candidate(exact, middle, quantum) >= 0)
            low = middle;
        else
            high = middle - 1;
    }

    /* Exactly representable results are classified before rounding. */
    if (compare_candidate(exact, low, quantum) == 0)
    {
        *out_coefficient = low;
        *out_exponent = (int)quantum;
        return BBTC_VV_METRIC_FINITE_EXACT;
    }

    /* Compare the true rational with the exact integer half-ULP boundary. */
    const int midpoint_sign = compare_midpoint(exact, low, quantum);
    if (midpoint_sign > 0 || (midpoint_sign == 0 && (low & 1U) != 0))
    {
        if (low == max_coefficient)
        {
            /* Carry from 2^p to 2^(p-1) at the next binary quantum. */
            low = UINT64_C(1) << (precision - 1);
            ++quantum;
        }
        else
        {
            ++low;
        }
    }

    if (low == 0)
        return BBTC_VV_METRIC_ROUNDED_TO_ZERO;
    *out_coefficient = low;
    *out_exponent = (int)quantum;
    return BBTC_VV_METRIC_FINITE_ROUNDED;
}

/**
 * @brief Generate one native-family entrypoint with a real native conversion.
 *
 * feholdexcept/fesetenv surround the entire operation, including the final
 * scalbn* call. The cast from uint64_t is exact: the selected coefficient is
 * at most p bits and p <= native significand precision. Multiplication by an
 * exact power of two yields a proved representable normal/subnormal result,
 * so changing the incoming rounding direction cannot change that value.
 * fenv changes made by libm are discarded before returning the metric.
 */
#define DEFINE_METRIC_ROUND(NAME, TYPE, SUFFIX, PRECISION, MIN_EXP, MAX_EXP, FORMAT_OK) \
    bbtc_vv_operation_status_e bbtc_vv_metric_round_##NAME(                     \
        const bbtc_vv_exact_metric_t* exact,                                    \
        bbtc_vv_metric_state_e* out_state, TYPE* out_value                      \
    )                                                                           \
    {                                                                           \
        if (out_state != NULL)                                                   \
            *out_state = BBTC_VV_METRIC_NOT_COMPUTED;                            \
        if (out_value != NULL)                                                   \
            *out_value = (TYPE)0;                                                \
        if (exact == NULL || out_state == NULL || out_value == NULL)             \
            return BBTC_VV_STATUS_INVALID_ARGUMENT;                             \
        if (!(FORMAT_OK) || FLT_RADIX != 2 || (PRECISION) > 64)                  \
            return BBTC_VV_STATUS_UNSUPPORTED_FP_ENVIRONMENT;                   \
        fenv_t incoming;                                                         \
        if (feholdexcept(&incoming) != 0)                                        \
            return BBTC_VV_STATUS_UNSUPPORTED_FP_ENVIRONMENT;                   \
        bbtc_vv_operation_status_e status = BBTC_VV_STATUS_SUCCESS;             \
        bbtc_vv_metric_state_e state = BBTC_VV_METRIC_NOT_COMPUTED;              \
        TYPE value = (TYPE)0;                                                    \
        const int mode = fegetround();                                           \
        if (mode != FE_TONEAREST && mode != FE_UPWARD &&                         \
            mode != FE_DOWNWARD && mode != FE_TOWARDZERO)                        \
            status = BBTC_VV_STATUS_UNSUPPORTED_FP_ENVIRONMENT;                 \
        else if (!representation_ok(exact))                                      \
            status = BBTC_VV_STATUS_INVALID_ARGUMENT;                           \
        else                                                                     \
        {                                                                        \
            uint64_t coefficient = 0;                                           \
            int exponent = 0;                                                    \
            state = select_rounding(exact, PRECISION, MIN_EXP, MAX_EXP,          \
                &coefficient, &exponent);                                        \
            if (state == BBTC_VV_METRIC_FINITE_EXACT ||                          \
                state == BBTC_VV_METRIC_FINITE_ROUNDED)                          \
                value = scalbn##SUFFIX((TYPE)coefficient, exponent);            \
        }                                                                        \
        if (fesetenv(&incoming) != 0)                                            \
            status = BBTC_VV_STATUS_NUMERICAL_FAILURE;                          \
        if (status == BBTC_VV_STATUS_SUCCESS)                                    \
        {                                                                        \
            *out_state = state;                                                  \
            *out_value = value;                                                  \
        }                                                                        \
        return status;                                                           \
    }

DEFINE_METRIC_ROUND(float, float, f, FLT_MANT_DIG, FLT_MIN_EXP, FLT_MAX_EXP,
    (FLT_MANT_DIG == 24 && FLT_MIN_EXP == -125 && FLT_MAX_EXP == 128))
DEFINE_METRIC_ROUND(double, double, , DBL_MANT_DIG, DBL_MIN_EXP, DBL_MAX_EXP,
    (DBL_MANT_DIG == 53 && DBL_MIN_EXP == -1021 && DBL_MAX_EXP == 1024))
DEFINE_METRIC_ROUND(long_double, long double, l,
    LDBL_MANT_DIG, LDBL_MIN_EXP, LDBL_MAX_EXP,
    ((LDBL_MANT_DIG == 64 && LDBL_MIN_EXP == -16381 && LDBL_MAX_EXP == 16384 &&
      sizeof(long double) == 16) ||
     (LDBL_MANT_DIG == 53 && LDBL_MIN_EXP == -1021 && LDBL_MAX_EXP == 1024)))

#undef DEFINE_METRIC_ROUND
