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
        cmocka_unit_test(test_long_double_extended_exponent)
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
