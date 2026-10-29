/**
 * @file exact_metric.c
 * @brief Exact, bounded integer/rational error construction (no rounding).
 *
 * @details
 * The exact quantity |observation-reference| is accumulated in base-2^64
 * unsigned magnitude words. An exponent anchor aligns the two native input
 * significands, so even two finite values whose mathematical difference
 * exceeds the native family's maximum finite value are represented exactly.
 *
 * Reference-relative error records that same exact numerator divided by the
 * absolute *dyadic* reference, as one nonzero 64-bit denominator significand
 * and its signed binary exponent. Neither subtraction nor division is done
 * using the native floating-point type. The denominator is intentionally
 * not reduced: no rounding occurs and correctness does not depend on gcd.
 *
 * Input decomposition is performed only AFTER the previously qualified exact
 * dyadic decision kernel accepts the same two finite arguments and certifies
 * the active native family/environment. The outer feholdexcept/fesetenv pair
 * preserves caller state across the entire operation, including failures.
 *
 * Invariants and exponent/limb capacity are proven in
 * docs/validation/VV0_1_B1B3C1_EXACT_METRIC_PROOF.md. This increment does NOT
 * implement the final bbtc_vv_compare_* API or diagnostic rounding.
 */
#include "exact_metric.h"
#include "exact_dyadic.h"

#include <fenv.h>
#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#if defined(__FAST_MATH__)
    #error "BBTC VVUQ exact metric forbids -ffast-math"
#endif
#if defined(__FINITE_MATH_ONLY__)
    #if __FINITE_MATH_ONLY__ != 0
        #error "BBTC VVUQ exact metric requires NaN and infinity semantics"
    #endif
#endif
#if defined(__clang__)
    #pragma STDC FENV_ACCESS ON
#endif

_Static_assert(BBTC_VV_METRIC_LIMBS >= 515,
    "fixed exact-metric integer lacks x87 exponent headroom");
_Static_assert(sizeof(uint64_t) * 8 == 64, "64-bit integer limbs required");

/** @brief Nonnegative 64-bit native significand and exact power of two. */
typedef struct bbtc_vv_metric_atom_t
{
    uint64_t mantissa;
    int32_t exponent;
    bool negative;
} bbtc_vv_metric_atom_t;

/** @brief Local unsigned working magnitude, little-endian by *word order*. */
typedef struct bbtc_vv_metric_big_t
{
    uint64_t word[BBTC_VV_METRIC_LIMBS];
} bbtc_vv_metric_big_t;

/**
 * @brief Checked addition of a single digit with carry propagation.
 *
 * Native uint64_t wraparound is defined modulo 2^64. A comparison against
 * the old value detects the carry without wider-than-64-bit arithmetic.
 */
static bool add_digit(bbtc_vv_metric_big_t* magnitude, size_t index, uint64_t digit)
{
    while (digit != 0)
    {
        if (index >= BBTC_VV_METRIC_LIMBS)
            return false;
        const uint64_t prior = magnitude->word[index];
        magnitude->word[index] += digit;
        digit = (magnitude->word[index] < prior) ? UINT64_C(1) : UINT64_C(0);
        ++index;
    }
    return true;
}

/**
 * @brief Place a 64-bit mantissa at the requested nonnegative bit offset.
 *
 * The shift is always in [0,63], never 64 (a shift by the type width would
 * be undefined behavior). The low and high pieces are added separately.
 * Both range checks are made by add_digit before dereferencing an array.
 */
static bool add_shifted(
    bbtc_vv_metric_big_t* magnitude, uint64_t mantissa, int32_t offset
)
{
    if (mantissa == 0)
        return true;
    if (offset < 0)
        return false;
    const size_t base = (size_t)offset / 64;
    const unsigned shift = (unsigned)((uint32_t)offset % 64U);
    if (base >= BBTC_VV_METRIC_LIMBS)
        return false;
    if (!add_digit(magnitude, base, mantissa << shift))
        return false;
    if (shift != 0 && !add_digit(magnitude, base + 1,
        mantissa >> (64U - shift)))
        return false;
    return true;
}

/** @brief Compare two aligned nonnegative big integers exactly. */
static int compare_magnitude(
    const bbtc_vv_metric_big_t* a, const bbtc_vv_metric_big_t* b
)
{
    for (size_t i = BBTC_VV_METRIC_LIMBS; i != 0; --i)
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
 * @brief Replace larger with larger-smaller, propagating unsigned borrow.
 *
 * Preconditions: larger >= smaller and both magnitudes are aligned.
 * The two-step subtraction distinguishes subtraction borrow from input
 * borrow; neither signed overflow nor wider integers are necessary.
 */
static bool subtract_magnitude(
    bbtc_vv_metric_big_t* larger, const bbtc_vv_metric_big_t* smaller
)
{
    uint64_t borrow = 0;
    for (size_t i = 0; i < BBTC_VV_METRIC_LIMBS; ++i)
    {
        const uint64_t original = larger->word[i];
        const uint64_t partial = original - smaller->word[i];
        const uint64_t first = (original < smaller->word[i]) ? 1U : 0U;
        larger->word[i] = partial - borrow;
        const uint64_t second = (partial < borrow) ? 1U : 0U;
        borrow = first | second;
    }
    return borrow == 0;
}

/**
 * @brief Convert two native atoms into a canonical unsigned difference.
 *
 * For opposite signs, |o-r| = |o|+|r|. For identical signs, it is the
 * difference of the larger and smaller unsigned aligned integers. The
 * *signs of zero* are immaterial, because each zero atom's magnitude is 0.
 * A zero atom does not affect the exponent anchor.
 */
static bool form_difference(
    bbtc_vv_metric_atom_t reference,
    bbtc_vv_metric_atom_t observation,
    bbtc_vv_exact_metric_t* result
)
{
    if (reference.mantissa == 0 && observation.mantissa == 0)
    {
        result->numerator_exponent = 0;
        return true;
    }
    int32_t origin = reference.mantissa != 0 ? reference.exponent : observation.exponent;
    if (observation.mantissa != 0 && observation.exponent < origin)
        origin = observation.exponent;

    bbtc_vv_metric_big_t rm = {0};
    bbtc_vv_metric_big_t om = {0};
    if (!add_shifted(&rm, reference.mantissa, reference.exponent - origin) ||
        !add_shifted(&om, observation.mantissa, observation.exponent - origin))
        return false;

    if (reference.negative != observation.negative)
    {
        /* In a signed difference opposite signs always add magnitudes. */
        bbtc_vv_metric_big_t sum = rm;
        if (!add_shifted(&sum, observation.mantissa, observation.exponent - origin))
            return false;
        for (size_t i = 0; i < BBTC_VV_METRIC_LIMBS; ++i)
            result->word[i] = sum.word[i];
    }
    else
    {
        if (compare_magnitude(&rm, &om) >= 0)
        {
            if (!subtract_magnitude(&rm, &om))
                return false;
            for (size_t i = 0; i < BBTC_VV_METRIC_LIMBS; ++i)
                result->word[i] = rm.word[i];
        }
        else
        {
            if (!subtract_magnitude(&om, &rm))
                return false;
            for (size_t i = 0; i < BBTC_VV_METRIC_LIMBS; ++i)
                result->word[i] = om.word[i];
        }
    }
    result->numerator_exponent = origin;
    for (size_t i = BBTC_VV_METRIC_LIMBS; i != 0; --i)
    {
        if (result->word[i - 1] != 0)
        {
            result->used = (uint32_t)i;
            return true;
        }
    }
    result->numerator_exponent = 0;
    return true;
}

/**
 * @brief Map the previously certified dyadic kernel's status vocabulary.
 *
 * Unknown codes fail closed; they must never masquerade as SUCCESS.
 */
static bbtc_vv_operation_status_e status_from_dyadic(bbtc_vv_dyadic_status_e status)
{
    switch (status)
    {
        case BBTC_VV_DYADIC_SUCCESS:
            return BBTC_VV_STATUS_SUCCESS;
        case BBTC_VV_DYADIC_INVALID_ARGUMENT:
            return BBTC_VV_STATUS_INVALID_ARGUMENT;
        case BBTC_VV_DYADIC_NAN_INPUT:
            return BBTC_VV_STATUS_NAN_INPUT;
        case BBTC_VV_DYADIC_NONFINITE_INPUT:
            return BBTC_VV_STATUS_NONFINITE_INPUT;
        case BBTC_VV_DYADIC_OUTSIDE_DOMAIN:
            return BBTC_VV_STATUS_OUTSIDE_DOMAIN;
        case BBTC_VV_DYADIC_UNSUPPORTED_ENVIRONMENT:
            return BBTC_VV_STATUS_UNSUPPORTED_FP_ENVIRONMENT;
        case BBTC_VV_DYADIC_INTERNAL_FAILURE:
        default:
            return BBTC_VV_STATUS_NUMERICAL_FAILURE;
    }
}

/**
 * @brief Instantiate native-family operations without precision narrowing.
 *
 * Only the already-qualified dyadic kernel classifies floating inputs.
 * `frexp*` normalizes each nonzero finite value to [0.5,1), and `scalbn*`
 * scales it by exactly p binary digits. The result is an exact integer in
 * [2^(p-1), 2^p), safely convertible to uint64_t (p <= 64).
 *
 * All possible subnormal decompositions are included in the proof bounds.
 * This family macro is kept solely for the three native libm entrypoints;
 * all large-integer operations above remain ordinary reviewable C functions.
 */
#define DEFINE_EXACT_METRIC(NAME, TYPE, SUFFIX, PRECISION)                         \
    static bbtc_vv_metric_atom_t atom_##NAME(TYPE value)                           \
    {                                                                              \
        if (value == (TYPE)0)                                                       \
            return (bbtc_vv_metric_atom_t){0};                                     \
        int exponent = 0;                                                          \
        const TYPE fraction = frexp##SUFFIX(fabs##SUFFIX(value), &exponent);        \
        const TYPE scaled = scalbn##SUFFIX(fraction, (PRECISION));                 \
        return (bbtc_vv_metric_atom_t){                                             \
            .mantissa = (uint64_t)scaled,                                          \
            .exponent = exponent - (PRECISION),                                    \
            .negative = signbit(value) != 0                                        \
        };                                                                         \
    }                                                                              \
    bbtc_vv_operation_status_e bbtc_vv_metric_exact_##NAME(                        \
        bbtc_vv_comparison_method_e method, TYPE reference, TYPE observation,      \
        bbtc_vv_exact_metric_t* out_metric                                         \
    )                                                                              \
    {                                                                              \
        if (out_metric == NULL)                                                     \
            return BBTC_VV_STATUS_INVALID_ARGUMENT;                               \
        *out_metric = (bbtc_vv_exact_metric_t){0};                                  \
        if (method != BBTC_VV_COMPARISON_ABSOLUTE &&                                \
            method != BBTC_VV_COMPARISON_REFERENCE_RELATIVE)                        \
            return BBTC_VV_STATUS_INVALID_ARGUMENT;                               \
        fenv_t incoming;                                                           \
        if (feholdexcept(&incoming) != 0)                                          \
            return BBTC_VV_STATUS_UNSUPPORTED_FP_ENVIRONMENT;                     \
        bool ignored = false;                                                      \
        const bbtc_vv_dyadic_status_e dyadic_status =                              \
            bbtc_vv_dyadic_compare_##NAME(                                         \
                BBTC_VV_DYADIC_ABSOLUTE, reference, observation, (TYPE)0, &ignored \
            );                                                                     \
        bbtc_vv_operation_status_e status = status_from_dyadic(dyadic_status);     \
        if (status == BBTC_VV_STATUS_SUCCESS &&                                   \
            method == BBTC_VV_COMPARISON_REFERENCE_RELATIVE &&                     \
            reference == (TYPE)0)                                                   \
            status = BBTC_VV_STATUS_OUTSIDE_DOMAIN;                                \
        if (status == BBTC_VV_STATUS_SUCCESS)                                      \
        {                                                                          \
            const bbtc_vv_metric_atom_t r = atom_##NAME(reference);                \
            const bbtc_vv_metric_atom_t o = atom_##NAME(observation);              \
            if (!form_difference(r, o, out_metric))                                \
                status = BBTC_VV_STATUS_NUMERICAL_FAILURE;                        \
            else if (method == BBTC_VV_COMPARISON_REFERENCE_RELATIVE)              \
            {                                                                      \
                out_metric->denominator = r.mantissa;                              \
                out_metric->denominator_exponent = r.exponent;                     \
            }                                                                      \
            else                                                                   \
            {                                                                      \
                out_metric->denominator = 1;                                       \
                out_metric->denominator_exponent = 0;                              \
            }                                                                      \
        }                                                                          \
        if (fesetenv(&incoming) != 0)                                              \
            status = BBTC_VV_STATUS_NUMERICAL_FAILURE;                            \
        if (status != BBTC_VV_STATUS_SUCCESS)                                     \
            *out_metric = (bbtc_vv_exact_metric_t){0};                             \
        return status;                                                             \
    }

DEFINE_EXACT_METRIC(float, float, f, FLT_MANT_DIG)
DEFINE_EXACT_METRIC(double, double, , DBL_MANT_DIG)
DEFINE_EXACT_METRIC(long_double, long double, l, LDBL_MANT_DIG)

#undef DEFINE_EXACT_METRIC
