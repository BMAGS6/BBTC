/**
 * @file exact_dyadic.c
 * @brief Exact, allocation-free binary-rational threshold decisions.
 *
 * @details
 * Algorithmic argument (review target VV0.1-B1b.2):
 *
 * Every supported finite input has an exact dyadic representation
 *     x = (-1)^sign * significand * 2^exponent,
 * where 0 <= significand < 2^64. A relative threshold is also dyadic:
 *     tolerance * |reference| = (M_t * M_r) * 2^(E_t + E_r).
 * The integer product has at most 128 significant bits and is calculated
 * exactly with four portable 32x32->64 multiplications.
 *
 * To test |o-r| <= T, test BOTH signed inequalities:
 *     o - r - T <= 0, and r - o - T <= 0.
 * Each is the sum of no more than three signed dyadic terms. The positive
 * and negative terms are accumulated independently in fixed-width arrays of
 * base-2^64 digits after alignment to one common binary exponent. No
 * floating-point subtraction, multiplication, division, or rounding enters
 * either Boolean decision. Comparing the two nonnegative integers determines
 * the exact sign, including inclusive equality at the threshold.
 *
 * The supported exponent envelope is initially limited to binary32,
 * binary64 and x87 extended binary80 (16-byte storage). The bit-buffer range
 * [-33152, 33152) includes every bit of every admissible input or product,
 * with spare carry headroom. Smallest possible normalized-frexp exponent
 * after scaling to a p-bit integer is MIN_EXP - 2p + 1. For x87 p=64 and
 * MIN_EXP=-16381, a two-input product begins no lower than -33016.
 * A product of two finite x87 values is strictly below 2^32768; adding up
 * to three terms remains far below bit 33152. Array operations also perform
 * explicit range/overflow checks; any breach is INTERNAL_FAILURE.
 *
 * The decomposition uses frexp/scalbn with exact powers of two, plus conversion
 * of an exactly integral significand to uint64_t. Format eligibility and
 * subnormal round-trip probes are deliberately conservative. A full
 * platform qualification, including unmasked-trap handling, remains separate.
 *
 * fenv isolation: feholdexcept() captures current state and masks exceptions
 * during validation and exact decomposition; fesetenv() restores the original
 * rounding mode and sticky flags on all supported normal return paths.
 * Failed environment restoration is surfaced as INTERNAL_FAILURE, never a
 * successful comparison. Build with rounding-aware, non-fast-math options.
 */
#include "exact_dyadic.h"

#include <fenv.h>
#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>

/* Unsafe finite-math assumptions would invalidate NaN/input classification.
 * Reject such builds instead of silently compiling weaker semantics. */
#if defined(__FAST_MATH__)
    #error "BBTC VVUQ exact kernel does not support -ffast-math"
#endif

#if defined(__FINITE_MATH_ONLY__)
    #if __FINITE_MATH_ONLY__ != 0
        #error "BBTC VVUQ exact kernel requires finite-math-only to be disabled"
    #endif
#endif

#if defined(__clang__)
    #pragma STDC FENV_ACCESS ON
#endif

/** @brief Minimum exponent position of the finite, fixed-width accumulator. */
#define BBTC_VV_ORIGIN_BIT (-33152)

/** @brief Exclusive upper exponent position of the accumulator. */
#define BBTC_VV_END_BIT (33152)

/** @brief Count of uint64_t words in the exact integer accumulator. */
#define BBTC_VV_WORDS ((BBTC_VV_END_BIT - BBTC_VV_ORIGIN_BIT) / 64)

_Static_assert(BBTC_VV_WORDS == 1036, "unexpected dyadic buffer size");
_Static_assert(sizeof(uint64_t) * 8 == 64, "64-bit words required");

/** @brief Magnitude of a nonnegative unsigned integer on the common grid. */
typedef struct bbtc_vv_big_t
{
    uint64_t word[BBTC_VV_WORDS];
} bbtc_vv_big_t;

/** @brief Exact signed magnitude with at most 64 significand bits. */
typedef struct bbtc_vv_atom_t
{
    uint64_t mantissa;
    int32_t exponent;
    bool negative;
} bbtc_vv_atom_t;

/** @brief Exact 128-bit unsigned product, stored in two portable words. */
typedef struct bbtc_vv_product_t
{
    uint64_t low;
    uint64_t high;
} bbtc_vv_product_t;

/**
 * @brief Multiply two 64-bit integers without uint128_t or compiler extensions.
 *
 * Every partial product is at most (2^32-1)^2, so uint64_t holds it. The
 * middle-word sum is at most 3(2^32-1), also exactly representable.
 */
static bbtc_vv_product_t multiply_u64(uint64_t a, uint64_t b)
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

    return (bbtc_vv_product_t)
    {
        .low = (mid << 32) | (uint32_t)p00,
        .high = p11 + (p01 >> 32) + (p10 >> 32) + (mid >> 32)
    };
}

/**
 * @brief Add one uint64_t digit and propagate any carry without wraparound.
 * @return False only if carry would escape the fixed buffer.
 */
static bool add_word(bbtc_vv_big_t* const big, size_t position, uint64_t digit)
{
    while (digit != 0)
    {
        if (position >= BBTC_VV_WORDS)
            return false;

        const uint64_t old = big->word[position];
        big->word[position] += digit;
        digit = big->word[position] < old ? 1 : 0;
        ++position;
    }

    return true;
}

/**
 * @brief Add an exact 128-bit dyadic term to a nonnegative accumulator.
 *
 * Splitting each shifted source word into low and high portions avoids any
 * shift by 64 (undefined behavior in C). All range checks occur before an
 * array access. A zero term is always harmless, irrespective of exponent.
 */
static bool add_dyadic(
    bbtc_vv_big_t* const big,
    bbtc_vv_product_t magnitude,
    int32_t exponent
)
{
    if (magnitude.low == 0 && magnitude.high == 0)
        return true;

    const int64_t offset = (int64_t)exponent - BBTC_VV_ORIGIN_BIT;
    if (offset < 0 || offset >= BBTC_VV_END_BIT - BBTC_VV_ORIGIN_BIT)
        return false;

    const size_t base = (size_t)(offset / 64);
    const unsigned shift = (unsigned)(offset % 64);
    const uint64_t source[2] = {magnitude.low, magnitude.high};

    for (size_t i = 0; i < 2; ++i)
    {
        if (source[i] == 0)
            continue;

        if (i > BBTC_VV_WORDS - base - 1)
            return false;

        if (!add_word(big, base + i, source[i] << shift))
            return false;

        if (shift != 0 && !add_word(big, base + i + 1, source[i] >> (64 - shift)))
            return false;
    }

    return true;
}

/** @brief Compare exact nonnegative integers from most-significant word down. */
static int compare_big(const bbtc_vv_big_t* a, const bbtc_vv_big_t* b)
{
    for (size_t i = BBTC_VV_WORDS; i != 0; --i)
    {
        const size_t k = i - 1;
        if (a->word[k] > b->word[k])
            return 1;
        if (a->word[k] < b->word[k])
            return -1;
    }
    return 0;
}

/**
 * @brief Determine the exact sign of u-v-threshold.
 *
 * An atom's sign is represented separately from its unsigned mantissa.
 * The threshold is always nonnegative; u and v may have either sign.
 * Returned sign is -1, 0 or +1; a failed range check is reported separately.
 */
static bool signed_difference_minus_threshold(
    bbtc_vv_atom_t u,
    bbtc_vv_atom_t v,
    bbtc_vv_product_t threshold,
    int32_t threshold_exponent,
    int* const out_sign
)
{
    bbtc_vv_big_t positive = {0};
    bbtc_vv_big_t negative = {0};
    const bbtc_vv_product_t umag = {.low = u.mantissa, .high = 0};
    const bbtc_vv_product_t vmag = {.low = v.mantissa, .high = 0};

    if (!add_dyadic(u.negative ? &negative : &positive, umag, u.exponent))
        return false;
    if (!add_dyadic(v.negative ? &positive : &negative, vmag, v.exponent))
        return false;
    if (!add_dyadic(&negative, threshold, threshold_exponent))
        return false;

    *out_sign = compare_big(&positive, &negative);
    return true;
}

/**
 * @brief Test |observation-reference| <= threshold exactly.
 *
 * Two signed half-space comparisons are logically equivalent to the stated
 * absolute-value relation, without ever forming a rounded difference.
 */
static bool compare_absolute_threshold(
    bbtc_vv_atom_t reference,
    bbtc_vv_atom_t observation,
    bbtc_vv_product_t threshold,
    int32_t threshold_exponent,
    bool* const out_pass
)
{
    int sign = 0;
    if (!signed_difference_minus_threshold(
        observation, reference, threshold, threshold_exponent, &sign
    ))
        return false;

    if (sign > 0)
    {
        *out_pass = false;
        return true;
    }

    if (!signed_difference_minus_threshold(
        reference, observation, threshold, threshold_exponent, &sign
    ))
        return false;

    *out_pass = sign <= 0;
    return true;
}

/** @brief Structural method validation, performed before floating-point work. */
static bool valid_method(bbtc_vv_dyadic_method_e method)
{
    switch (method)
    {
        case BBTC_VV_DYADIC_EXACT:
        case BBTC_VV_DYADIC_ABSOLUTE:
        case BBTC_VV_DYADIC_REFERENCE_RELATIVE:
            return true;
        default:
            return false;
    }
}

/**
 * @brief Define three independently typed entry points without narrowing.
 *
 * The `SUFFIX`-qualified libm functions are used in every scalar operation.
 * The generated code has the same control flow, but `float`, `double`, and
 * `long double` arithmetic is never routed through a different scalar family.
 */
#define BBTC_VV_DEFINE_FAMILY(NAME, TYPE, SUFFIX, MANT_DIG, MIN_EXP, MAX_EXP, FORMAT_OK, TRUE_MIN) \
    static bool NAME##_environment_ok(void)                                                   \
    {                                                                                         \
        if (!(FORMAT_OK) || FLT_RADIX != 2)                                                   \
            return false;                                                                     \
        if ((MANT_DIG) > 64 || (MIN_EXP) < -16381 || (MAX_EXP) > 16384)                       \
            return false;                                                                     \
        const int round = fegetround();                                                        \
        if (round != FE_TONEAREST && round != FE_DOWNWARD &&                                  \
            round != FE_UPWARD && round != FE_TOWARDZERO)                                     \
            return false;                                                                     \
        volatile TYPE smallest = (TRUE_MIN);                                                  \
        int exp = 0;                                                                          \
        TYPE normalized = frexp##SUFFIX(smallest, &exp);                                       \
        if (normalized == (TYPE)0)                                                            \
            return false;                                                                     \
        volatile TYPE restored = scalbn##SUFFIX(normalized, exp);                             \
        return restored == smallest && restored != (TYPE)0;                                   \
    }                                                                                         \
    static bbtc_vv_atom_t NAME##_atom(TYPE value)                                             \
    {                                                                                         \
        if (value == (TYPE)0)                                                                 \
            return (bbtc_vv_atom_t){0};                                                       \
        int exp = 0;                                                                          \
        const TYPE normalized = frexp##SUFFIX(fabs##SUFFIX(value), &exp);                      \
        const TYPE integer = scalbn##SUFFIX(normalized, (MANT_DIG));                          \
        return (bbtc_vv_atom_t)                                                               \
        {                                                                                     \
            .mantissa = (uint64_t)integer,                                                    \
            .exponent = exp - (MANT_DIG),                                                     \
            .negative = signbit(value) != 0                                                   \
        };                                                                                    \
    }                                                                                         \
    bbtc_vv_dyadic_status_e bbtc_vv_dyadic_compare_##NAME(                                  \
        bbtc_vv_dyadic_method_e method, TYPE reference, TYPE observation,                    \
        TYPE tolerance, bool* out_pass                                                        \
    )                                                                                         \
    {                                                                                         \
        if (out_pass == NULL)                                                                 \
            return BBTC_VV_DYADIC_INVALID_ARGUMENT;                                          \
        *out_pass = false;                                                                    \
        if (!valid_method(method))                                                            \
            return BBTC_VV_DYADIC_INVALID_ARGUMENT;                                          \
        if (!(FORMAT_OK) || FLT_RADIX != 2 || (MANT_DIG) > 64 ||                              \
            (MIN_EXP) < -16381 || (MAX_EXP) > 16384)                                          \
            return BBTC_VV_DYADIC_UNSUPPORTED_ENVIRONMENT;                                   \
        fenv_t original;                                                                      \
        if (feholdexcept(&original) != 0)                                                     \
            return BBTC_VV_DYADIC_UNSUPPORTED_ENVIRONMENT;                                   \
        bbtc_vv_dyadic_status_e status = BBTC_VV_DYADIC_SUCCESS;                              \
        bool answer = false;                                                                  \
        do                                                                                    \
        {                                                                                     \
            if (!NAME##_environment_ok())                                                     \
            {                                                                                 \
                status = BBTC_VV_DYADIC_UNSUPPORTED_ENVIRONMENT;                              \
                break;                                                                        \
            }                                                                                 \
            if (isnan(reference) || isnan(observation) || isnan(tolerance))                    \
            {                                                                                 \
                status = BBTC_VV_DYADIC_NAN_INPUT;                                            \
                break;                                                                        \
            }                                                                                 \
            if (!isfinite(reference) || !isfinite(observation) || !isfinite(tolerance))        \
            {                                                                                 \
                status = BBTC_VV_DYADIC_NONFINITE_INPUT;                                      \
                break;                                                                        \
            }                                                                                 \
            if (method == BBTC_VV_DYADIC_EXACT)                                               \
            {                                                                                 \
                if (tolerance != (TYPE)0)                                                     \
                    status = BBTC_VV_DYADIC_INVALID_ARGUMENT;                                \
                else                                                                          \
                    answer = reference == observation;                                       \
                break;                                                                        \
            }                                                                                 \
            if (tolerance < (TYPE)0)                                                          \
            {                                                                                 \
                status = BBTC_VV_DYADIC_INVALID_ARGUMENT;                                    \
                break;                                                                        \
            }                                                                                 \
            if (method == BBTC_VV_DYADIC_REFERENCE_RELATIVE && reference == (TYPE)0)          \
            {                                                                                 \
                status = BBTC_VV_DYADIC_OUTSIDE_DOMAIN;                                      \
                break;                                                                        \
            }                                                                                 \
            const bbtc_vv_atom_t r = NAME##_atom(reference);                                 \
            const bbtc_vv_atom_t o = NAME##_atom(observation);                               \
            const bbtc_vv_atom_t t = NAME##_atom(tolerance);                                 \
            bbtc_vv_product_t threshold = {.low = t.mantissa, .high = 0};                     \
            int32_t threshold_exp = t.exponent;                                               \
            if (method == BBTC_VV_DYADIC_REFERENCE_RELATIVE)                                 \
            {                                                                                 \
                threshold = multiply_u64(t.mantissa, r.mantissa);                             \
                threshold_exp = t.exponent + r.exponent;                                     \
            }                                                                                 \
            if (!compare_absolute_threshold(r, o, threshold, threshold_exp, &answer))          \
                status = BBTC_VV_DYADIC_INTERNAL_FAILURE;                                    \
        } while (0);                                                                          \
        if (fesetenv(&original) != 0)                                                         \
            return BBTC_VV_DYADIC_INTERNAL_FAILURE;                                          \
        if (status == BBTC_VV_DYADIC_SUCCESS)                                                 \
            *out_pass = answer;                                                               \
        return status;                                                                        \
    }

BBTC_VV_DEFINE_FAMILY(
    float, float, f, FLT_MANT_DIG, FLT_MIN_EXP, FLT_MAX_EXP,
    (FLT_MANT_DIG == 24 && FLT_MIN_EXP == -125 && FLT_MAX_EXP == 128), FLT_TRUE_MIN
)

BBTC_VV_DEFINE_FAMILY(
    double, double, , DBL_MANT_DIG, DBL_MIN_EXP, DBL_MAX_EXP,
    (DBL_MANT_DIG == 53 && DBL_MIN_EXP == -1021 && DBL_MAX_EXP == 1024), DBL_TRUE_MIN
)

BBTC_VV_DEFINE_FAMILY(
    long_double, long double, l, LDBL_MANT_DIG, LDBL_MIN_EXP, LDBL_MAX_EXP,
    ((LDBL_MANT_DIG == 64 && LDBL_MIN_EXP == -16381 && LDBL_MAX_EXP == 16384 &&
      sizeof(long double) == 16) ||
     (LDBL_MANT_DIG == 53 && LDBL_MIN_EXP == -1021 && LDBL_MAX_EXP == 1024)), LDBL_TRUE_MIN
)

#undef BBTC_VV_DEFINE_FAMILY
