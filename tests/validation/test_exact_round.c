/**
 * @file test_exact_round.c
 * @brief Adversarial cmocka tests for exact-rational nearest-even conversion.
 *
 * @details
 * Tests explicitly construct exact rational inputs, independently of
 * exact_metric.c's difference builder. This isolates the converter and its
 * representability classification. The optional Python Fraction oracle adds
 * broad randomized and extreme-magnitude differential verification.
 *
 * All supported scalar families are exercised without narrowing long double.
 * Caller rounding modes and preexisting sticky flags are tested across
 * success and failure paths. No scientific validation claim follows.
 */
#include "exact_round.h"

#include <fenv.h>
#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <setjmp.h>
#include <stdarg.h>
#include <cmocka.h>

/** @brief Construct a small exact rational N*2^E/(d*2^F). */
static bbtc_vv_exact_metric_t rational(
    uint64_t numerator, int32_t exponent,
    uint64_t denominator, int32_t denominator_exponent
)
{
    bbtc_vv_exact_metric_t result = {0};
    result.word[0] = numerator;
    result.used = numerator != 0 ? 1U : 0U;
    result.numerator_exponent = exponent;
    result.denominator = denominator;
    result.denominator_exponent = denominator_exponent;
    return result;
}

/**
 * @brief Cover 0, exactly representable fractions, inexact thirds, and ties.
 * The tests use exact rational identities and native expected literals;
 * never compute the expected value using the function under test.
 */
#define DEFINE_ROUND_TESTS(NAME, TYPE, PRECISION, MINEXP, MAXEXP, SCALBN, TRUE_MIN, MAX_FINITE) \
    static void test_##NAME##_arithmetic(void** state)                                 \
    {                                                                                 \
        (void)state;                                                                  \
        bbtc_vv_metric_state_e kind = BBTC_VV_METRIC_NOT_COMPUTED;                    \
        TYPE rounded = (TYPE)0;                                                      \
        bbtc_vv_exact_metric_t x = rational(0, 0, 1, 0);                             \
        assert_int_equal(bbtc_vv_metric_round_##NAME(&x, &kind, &rounded),            \
            BBTC_VV_STATUS_SUCCESS);                                                \
        assert_int_equal(kind, BBTC_VV_METRIC_FINITE_EXACT);                         \
        assert_true(rounded == (TYPE)0);                                             \
        x = rational(3, 0, 3, 0);                                                    \
        assert_int_equal(bbtc_vv_metric_round_##NAME(&x, &kind, &rounded),            \
            BBTC_VV_STATUS_SUCCESS);                                                \
        assert_int_equal(kind, BBTC_VV_METRIC_FINITE_EXACT);                         \
        assert_true(rounded == (TYPE)1);                                             \
        x = rational(1, 0, 3, 0);                                                    \
        assert_int_equal(bbtc_vv_metric_round_##NAME(&x, &kind, &rounded),            \
            BBTC_VV_STATUS_SUCCESS);                                                \
        assert_int_equal(kind, BBTC_VV_METRIC_FINITE_ROUNDED);                       \
        assert_true(rounded > (TYPE)0 && rounded < (TYPE)1);                         \
        x = rational(1, 0, 2, 0);                                                    \
        assert_int_equal(bbtc_vv_metric_round_##NAME(&x, &kind, &rounded),            \
            BBTC_VV_STATUS_SUCCESS);                                                \
        assert_int_equal(kind, BBTC_VV_METRIC_FINITE_EXACT);                         \
        assert_true(rounded == (TYPE)0.5);                                           \
    }                                                                                 \
    static void test_##NAME##_subnormal(void** state)                                  \
    {                                                                                 \
        (void)state;                                                                  \
        const int q = (MINEXP) - (PRECISION);                                         \
        bbtc_vv_metric_state_e kind = BBTC_VV_METRIC_NOT_COMPUTED;                    \
        TYPE rounded = (TYPE)0;                                                      \
        bbtc_vv_exact_metric_t x = rational(1, q - 1, 1, 0);                         \
        assert_int_equal(bbtc_vv_metric_round_##NAME(&x, &kind, &rounded),            \
            BBTC_VV_STATUS_SUCCESS);                                                \
        assert_int_equal(kind, BBTC_VV_METRIC_ROUNDED_TO_ZERO);                      \
        assert_true(rounded == (TYPE)0 && !signbit(rounded));                         \
        x = rational(3, q - 1, 1, 0);                                                \
        assert_int_equal(bbtc_vv_metric_round_##NAME(&x, &kind, &rounded),            \
            BBTC_VV_STATUS_SUCCESS);                                                \
        assert_int_equal(kind, BBTC_VV_METRIC_FINITE_ROUNDED);                       \
        assert_true(rounded == SCALBN((TYPE)2, q));                                  \
        x = rational(1, q, 1, 0);                                                    \
        assert_int_equal(bbtc_vv_metric_round_##NAME(&x, &kind, &rounded),            \
            BBTC_VV_STATUS_SUCCESS);                                                \
        assert_int_equal(kind, BBTC_VV_METRIC_FINITE_EXACT);                         \
        assert_true(rounded == (TRUE_MIN));                                          \
        /* Exactly halfway between greatest subnormal and least normal. */          \
        x = rational(UINT64_MAX >> (64U - (PRECISION)), q - 1, 1, 0);               \
        assert_int_equal(bbtc_vv_metric_round_##NAME(&x, &kind, &rounded),            \
            BBTC_VV_STATUS_SUCCESS);                                                \
        assert_int_equal(kind, BBTC_VV_METRIC_FINITE_ROUNDED);                       \
        assert_true(rounded == SCALBN((TYPE)1, (MINEXP) - 1));                       \
    }                                                                                 \
    static void test_##NAME##_limits(void** state)                                     \
    {                                                                                 \
        (void)state;                                                                  \
        bbtc_vv_metric_state_e kind = BBTC_VV_METRIC_NOT_COMPUTED;                    \
        TYPE rounded = (TYPE)0;                                                      \
        const uint64_t maximum = UINT64_MAX >> (64U - (PRECISION));                  \
        bbtc_vv_exact_metric_t x = rational(maximum,                            \
            (MAXEXP) - (PRECISION), 1, 0);                                           \
        assert_int_equal(bbtc_vv_metric_round_##NAME(&x, &kind, &rounded),            \
            BBTC_VV_STATUS_SUCCESS);                                                \
        assert_int_equal(kind, BBTC_VV_METRIC_FINITE_EXACT);                         \
        assert_true(rounded == (MAX_FINITE));                                        \
        x = rational(1, (MAXEXP), 1, 0);                                             \
        assert_int_equal(bbtc_vv_metric_round_##NAME(&x, &kind, &rounded),            \
            BBTC_VV_STATUS_SUCCESS);                                                \
        assert_int_equal(kind, BBTC_VV_METRIC_EXCEEDS_RANGE);                        \
        assert_true(rounded == (TYPE)0);                                             \
        /* 2^p+1 is an exact tie at 2^p: the lower even scalar wins. */              \
        x = rational(1, 0, 1, 0);                                                    \
        x.word[0] = 1;                                                              \
        x.word[1] = 1;                                                              \
        x.used = 2;                                                                 \
        x.numerator_exponent = -(PRECISION);                                         \
        /* N = 2^p + 1; handle p==64 without any shift by 64. */                     \
        x.word[0] = (PRECISION) == 64 ? 1 :                                          \
            (UINT64_C(1) << ((PRECISION) % 64)) + 1;                                 \
        x.word[1] = (PRECISION) == 64 ? 1 : 0;                                       \
        x.used = (PRECISION) == 64 ? 2 : 1;                                         \
        assert_int_equal(bbtc_vv_metric_round_##NAME(&x, &kind, &rounded),            \
            BBTC_VV_STATUS_SUCCESS);                                                \
        assert_int_equal(kind, BBTC_VV_METRIC_FINITE_ROUNDED);                       \
        assert_true(rounded == (TYPE)1);                                             \
    }                                                                                 \
    static void test_##NAME##_state(void** state)                                      \
    {                                                                                 \
        (void)state;                                                                  \
        const int modes[] = {FE_TONEAREST, FE_UPWARD, FE_DOWNWARD, FE_TOWARDZERO};      \
        fenv_t saved;                                                                 \
        assert_int_equal(fegetenv(&saved), 0);                                       \
        const bbtc_vv_exact_metric_t x = rational(1, 0, 3, 0);                       \
        TYPE reference = (TYPE)0;                                                   \
        for (size_t i = 0; i < 4; ++i)                                               \
        {                                                                             \
            assert_int_equal(fesetround(modes[i]), 0);                               \
            assert_int_equal(feclearexcept(FE_ALL_EXCEPT), 0);                       \
            assert_int_equal(feraiseexcept(FE_INVALID | FE_INEXACT), 0);             \
            const int before = fetestexcept(FE_ALL_EXCEPT);                         \
            bbtc_vv_metric_state_e kind = BBTC_VV_METRIC_NOT_COMPUTED;                \
            TYPE rounded = (TYPE)0;                                                 \
            assert_int_equal(bbtc_vv_metric_round_##NAME(&x, &kind, &rounded),        \
                BBTC_VV_STATUS_SUCCESS);                                            \
            assert_int_equal(kind, BBTC_VV_METRIC_FINITE_ROUNDED);                   \
            assert_int_equal(fegetround(), modes[i]);                               \
            assert_int_equal(fetestexcept(FE_ALL_EXCEPT), before);                  \
            if (i == 0) reference = rounded;                                        \
            else assert_true(rounded == reference);                                 \
            bbtc_vv_exact_metric_t invalid = x;                                    \
            invalid.denominator = 0;                                                \
            assert_int_equal(bbtc_vv_metric_round_##NAME(                           \
                &invalid, &kind, &rounded), BBTC_VV_STATUS_INVALID_ARGUMENT);         \
            assert_int_equal(kind, BBTC_VV_METRIC_NOT_COMPUTED);                    \
            assert_true(rounded == (TYPE)0);                                         \
            assert_int_equal(fegetround(), modes[i]);                               \
            assert_int_equal(fetestexcept(FE_ALL_EXCEPT), before);                  \
        }                                                                             \
        assert_int_equal(fesetenv(&saved), 0);                                       \
    }

DEFINE_ROUND_TESTS(float, float, FLT_MANT_DIG, FLT_MIN_EXP, FLT_MAX_EXP,
    scalbnf, FLT_TRUE_MIN, FLT_MAX)
DEFINE_ROUND_TESTS(double, double, DBL_MANT_DIG, DBL_MIN_EXP, DBL_MAX_EXP,
    scalbn, DBL_TRUE_MIN, DBL_MAX)
DEFINE_ROUND_TESTS(long_double, long double, LDBL_MANT_DIG, LDBL_MIN_EXP, LDBL_MAX_EXP,
    scalbnl, LDBL_TRUE_MIN, LDBL_MAX)

/** @brief Check required failure-output clearing for missing pointers. */
static void test_structural_failures(void** state)
{
    (void)state;
    const bbtc_vv_exact_metric_t exact = rational(1, 0, 1, 0);
    bbtc_vv_metric_state_e kind = BBTC_VV_METRIC_FINITE_EXACT;
    double output = 42.0;
    assert_int_equal(bbtc_vv_metric_round_double(NULL, &kind, &output),
        BBTC_VV_STATUS_INVALID_ARGUMENT);
    assert_int_equal(kind, BBTC_VV_METRIC_NOT_COMPUTED);
    assert_true(output == 0.0);
    assert_int_equal(bbtc_vv_metric_round_double(&exact, NULL, &output),
        BBTC_VV_STATUS_INVALID_ARGUMENT);
    assert_true(output == 0.0);
    assert_int_equal(bbtc_vv_metric_round_double(&exact, &kind, NULL),
        BBTC_VV_STATUS_INVALID_ARGUMENT);
}

/** @brief Malformed limb metadata must never escape as a successful metric. */
static void test_malformed_representation(void** state)
{
    (void)state;
    bbtc_vv_exact_metric_t x = rational(1, 0, 1, 0);
    bbtc_vv_metric_state_e kind = BBTC_VV_METRIC_FINITE_EXACT;
    double value = 9.0;

    x.used = BBTC_VV_METRIC_LIMBS + 1U;
    assert_int_equal(bbtc_vv_metric_round_double(&x, &kind, &value),
        BBTC_VV_STATUS_INVALID_ARGUMENT);
    assert_int_equal(kind, BBTC_VV_METRIC_NOT_COMPUTED);
    assert_true(value == 0.0);

    x = rational(1, 0, 1, 0);
    x.used = 2;
    assert_int_equal(bbtc_vv_metric_round_double(&x, &kind, &value),
        BBTC_VV_STATUS_INVALID_ARGUMENT);

    x = rational(1, 0, 1, 0);
    x.word[1] = 1;
    assert_int_equal(bbtc_vv_metric_round_double(&x, &kind, &value),
        BBTC_VV_STATUS_INVALID_ARGUMENT);
}

/** @brief x87 keeps an exactly representable exponent outside binary64. */
static void test_long_double_extended_exponent(void** state)
{
    (void)state;
    if (LDBL_MANT_DIG != 64 || LDBL_MAX_EXP != 16384)
        return; /* This family is not qualified for extended-range testing. */

    bbtc_vv_exact_metric_t x = rational(1, 5000, 1, 0);
    bbtc_vv_metric_state_e kind = BBTC_VV_METRIC_NOT_COMPUTED;
    long double value = 0;
    assert_int_equal(bbtc_vv_metric_round_long_double(&x, &kind, &value),
        BBTC_VV_STATUS_SUCCESS);
    assert_int_equal(kind, BBTC_VV_METRIC_FINITE_EXACT);
    assert_true(value == scalbnl(1.0L, 5000));
}

/**
 * @brief Exercise both parities of exactly halfway normal significands.
 *
 * @details
 * Let p denote the target precision. Values in [1,2) are spaced by
 * ULP=2^(1-p). The exact rational (2^p+1)/2^p is halfway between 1 and
 * its successor: the lower coefficient is even and must win. The exact
 * rational (2^p+3)/2^p is halfway between the first and second successors:
 * the lower coefficient is odd, so the upper (even) coefficient must win.
 *
 * For p=64, the numerator needs a second 64-bit limb. This test builds
 * that limb explicitly rather than invoking undefined shifts by 64.
 * Expected results are exact dyadic identities, not outputs of the kernel.
 */
#define DEFINE_ROUND_PARITY_AUDIT(NAME, TYPE, PRECISION, SCALBN)                         \
    static void test_##NAME##_midpoint_parity_audit(void** state)                     \
    {                                                                                  \
        (void)state;                                                                    \
        const bool extended = (PRECISION) == 64;                                       \
        const uint64_t one = UINT64_C(1) << ((PRECISION) % 64);                        \
        bbtc_vv_exact_metric_t x = rational(extended ? 1U : one + 1U,                 \
            -(PRECISION), 1, 0);                                                        \
        if (extended)                                                                  \
        {                                                                              \
            x.word[1] = 1;                                                              \
            x.used = 2;                                                                 \
        }                                                                              \
        bbtc_vv_metric_state_e kind = BBTC_VV_METRIC_NOT_COMPUTED;                    \
        TYPE rounded = (TYPE)-1;                                                        \
        assert_int_equal(bbtc_vv_metric_round_##NAME(&x, &kind, &rounded),             \
            BBTC_VV_STATUS_SUCCESS);                                                    \
        assert_int_equal(kind, BBTC_VV_METRIC_FINITE_ROUNDED);                          \
        assert_true(rounded == (TYPE)1);                                                \
        x.word[0] = extended ? 3U : one + 3U;                                          \
        assert_int_equal(bbtc_vv_metric_round_##NAME(&x, &kind, &rounded),             \
            BBTC_VV_STATUS_SUCCESS);                                                    \
        assert_int_equal(kind, BBTC_VV_METRIC_FINITE_ROUNDED);                          \
        assert_true(rounded == (TYPE)1 + SCALBN((TYPE)1, 2 - (PRECISION)));             \
    }

DEFINE_ROUND_PARITY_AUDIT(float, float, FLT_MANT_DIG, scalbnf)
DEFINE_ROUND_PARITY_AUDIT(double, double, DBL_MANT_DIG, scalbn)
DEFINE_ROUND_PARITY_AUDIT(long_double, long double, LDBL_MANT_DIG, scalbnl)
#undef DEFINE_ROUND_PARITY_AUDIT

/**
 * @brief Check carry into the next binade at a half-ULP decision.
 *
 * @details
 * The mathematical midpoint immediately below 2 is 2-2^(-p).
 * A value one quarter of that local ULP below it must round downward
 * to the predecessor of 2; the exact midpoint and a value just above
 * it must round to 2. For p=64 the numerator occupies two limbs.
 *
 * This specifically exercises `low == max_coefficient` in the rounding
 * algorithm. The expected predecessor is obtained using the native family's
 * exact adjacent-value operation, not by rerunning the algorithm under test.
 */
#define DEFINE_ROUND_BINADE_AUDIT(NAME, TYPE, PRECISION, NEXTAFTER)                    \
    static void test_##NAME##_binade_carry_audit(void** state)                       \
    {                                                                                 \
        (void)state;                                                                   \
        const bool extended = (PRECISION) == 64;                                      \
        const uint64_t four = UINT64_C(1) << (((PRECISION) + 2) % 64);                \
        bbtc_vv_exact_metric_t x = rational(extended ? UINT64_MAX - 2U : four - 3U,  \
            -(PRECISION) - 1, 1, 0);                                                   \
        if (extended)                                                                 \
        {                                                                             \
            x.word[1] = 3;                                                             \
            x.used = 2;                                                                \
        }                                                                             \
        bbtc_vv_metric_state_e kind = BBTC_VV_METRIC_NOT_COMPUTED;                   \
        TYPE rounded = (TYPE)0;                                                       \
        assert_int_equal(bbtc_vv_metric_round_##NAME(&x, &kind, &rounded),            \
            BBTC_VV_STATUS_SUCCESS);                                                   \
        assert_int_equal(kind, BBTC_VV_METRIC_FINITE_ROUNDED);                         \
        assert_true(rounded == NEXTAFTER((TYPE)2, (TYPE)0));                           \
        x.word[0] = extended ? UINT64_MAX - 1U : four - 2U;                          \
        assert_int_equal(bbtc_vv_metric_round_##NAME(&x, &kind, &rounded),            \
            BBTC_VV_STATUS_SUCCESS);                                                   \
        assert_int_equal(kind, BBTC_VV_METRIC_FINITE_ROUNDED);                         \
        assert_true(rounded == (TYPE)2);                                               \
        x.word[0] = extended ? UINT64_MAX : four - 1U;                               \
        assert_int_equal(bbtc_vv_metric_round_##NAME(&x, &kind, &rounded),            \
            BBTC_VV_STATUS_SUCCESS);                                                   \
        assert_int_equal(kind, BBTC_VV_METRIC_FINITE_ROUNDED);                         \
        assert_true(rounded == (TYPE)2);                                               \
    }

DEFINE_ROUND_BINADE_AUDIT(float, float, FLT_MANT_DIG, nextafterf)
DEFINE_ROUND_BINADE_AUDIT(double, double, DBL_MANT_DIG, nextafter)
DEFINE_ROUND_BINADE_AUDIT(long_double, long double, LDBL_MANT_DIG, nextafterl)
#undef DEFINE_ROUND_BINADE_AUDIT

/**
 * @brief Classify values on both sides of the mathematical finite maximum.
 *
 * @details
 * If M=(2^p-1)*2^(MAX_EXP-p), then M-ULP/4 is still inside the
 * finite range and rounds to M, whereas M+ULP/4 is mathematically greater
 * than M and MUST be EXCEEDS_RANGE even if hardware might round it to M.
 * This checks *pre-round* classification and the canonical +0 sentinel.
 */
#define DEFINE_ROUND_MAX_AUDIT(NAME, TYPE, PRECISION, MAXEXP, MAX_FINITE)              \
    static void test_##NAME##_finite_max_audit(void** state)                       \
    {                                                                                \
        (void)state;                                                                  \
        const bool extended = (PRECISION) == 64;                                     \
        const uint64_t max_coeff = UINT64_MAX >> (64U - (PRECISION));               \
        bbtc_vv_exact_metric_t x = rational(extended ? UINT64_MAX - 4U :            \
            4U * max_coeff - 1U, (MAXEXP) - (PRECISION) - 2, 1, 0);                 \
        if (extended)                                                                \
        {                                                                            \
            x.word[1] = 3;                                                            \
            x.used = 2;                                                               \
        }                                                                            \
        bbtc_vv_metric_state_e kind = BBTC_VV_METRIC_NOT_COMPUTED;                  \
        TYPE rounded = (TYPE)0;                                                      \
        assert_int_equal(bbtc_vv_metric_round_##NAME(&x, &kind, &rounded),           \
            BBTC_VV_STATUS_SUCCESS);                                                  \
        assert_int_equal(kind, BBTC_VV_METRIC_FINITE_ROUNDED);                        \
        assert_true(rounded == (MAX_FINITE));                                         \
        x.word[0] = extended ? UINT64_MAX - 2U : 4U * max_coeff + 1U;               \
        assert_int_equal(bbtc_vv_metric_round_##NAME(&x, &kind, &rounded),           \
            BBTC_VV_STATUS_SUCCESS);                                                  \
        assert_int_equal(kind, BBTC_VV_METRIC_EXCEEDS_RANGE);                         \
        assert_true(rounded == (TYPE)0 && !signbit(rounded));                          \
    }

DEFINE_ROUND_MAX_AUDIT(float, float, FLT_MANT_DIG, FLT_MAX_EXP, FLT_MAX)
DEFINE_ROUND_MAX_AUDIT(double, double, DBL_MANT_DIG, DBL_MAX_EXP, DBL_MAX)
DEFINE_ROUND_MAX_AUDIT(long_double, long double, LDBL_MANT_DIG, LDBL_MAX_EXP, LDBL_MAX)
#undef DEFINE_ROUND_MAX_AUDIT

/**
 * @brief Force the midpoint comparator to multiply 65 and 64 significant bits.
 *
 * @details
 * With A=2^64, choose d=A-1, m=A-1, N=d*(2*m+1)=2*A^2-3*A+1.
 * The exact 129-bit numerator has little-endian limbs [1,A-3,1]. Set
 * X=N*2^-64/d = 2-2^-64, the exact halfway value between the greatest
 * 64-bit-precision scalar below 2 and 2. Ties-to-even must choose 2.
 *
 * No compiler-specific 128-bit type or floating-point intermediate is used
 * to construct the test input. This exercises the three-limb midpoint path.
 */
static void test_long_double_129_bit_midpoint_audit(void** state)
{
    (void)state;
    bbtc_vv_exact_metric_t x = rational(1, -64, UINT64_MAX, 0);
    x.word[1] = UINT64_MAX - UINT64_C(2);
    x.word[2] = 1;
    x.used = 3;

    bbtc_vv_metric_state_e kind = BBTC_VV_METRIC_NOT_COMPUTED;
    long double rounded = -1.0L;
    assert_int_equal(bbtc_vv_metric_round_long_double(&x, &kind, &rounded),
        BBTC_VV_STATUS_SUCCESS);
    assert_int_equal(kind, BBTC_VV_METRIC_FINITE_ROUNDED);
    assert_true(rounded == 2.0L);
}

int main(void)
{
    const struct CMUnitTest tests[] =
    {
        cmocka_unit_test(test_float_arithmetic),
        cmocka_unit_test(test_double_arithmetic),
        cmocka_unit_test(test_long_double_arithmetic),
        cmocka_unit_test(test_float_subnormal),
        cmocka_unit_test(test_double_subnormal),
        cmocka_unit_test(test_long_double_subnormal),
        cmocka_unit_test(test_float_limits),
        cmocka_unit_test(test_double_limits),
        cmocka_unit_test(test_long_double_limits),
        cmocka_unit_test(test_float_state),
        cmocka_unit_test(test_double_state),
        cmocka_unit_test(test_long_double_state),
        cmocka_unit_test(test_structural_failures),
        cmocka_unit_test(test_malformed_representation),
        cmocka_unit_test(test_long_double_extended_exponent),
        cmocka_unit_test(test_float_midpoint_parity_audit),
        cmocka_unit_test(test_double_midpoint_parity_audit),
        cmocka_unit_test(test_long_double_midpoint_parity_audit),
        cmocka_unit_test(test_float_binade_carry_audit),
        cmocka_unit_test(test_double_binade_carry_audit),
        cmocka_unit_test(test_long_double_binade_carry_audit),
        cmocka_unit_test(test_float_finite_max_audit),
        cmocka_unit_test(test_double_finite_max_audit),
        cmocka_unit_test(test_long_double_finite_max_audit),
        cmocka_unit_test(test_long_double_129_bit_midpoint_audit)
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
