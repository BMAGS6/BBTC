/**
 * @file test_exact_metric.c
 * @brief Deterministic boundary and state-preservation tests for exact metrics.
 *
 * @details
 * These tests assert the *rational construction* only. Native binary rounding
 * and diagnostic classifications are future B1b.3c-2 responsibilities. The
 * independent Python Fraction oracle tests much wider magnitude ranges.
 * cmocka is a test-only dependency and never links into production BBTC.
 */
#include "exact_metric.h"

#include <fenv.h>
#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <setjmp.h>
#include <stdarg.h>
#include <cmocka.h>

/** @brief Examine the exact rational for small, deliberately integral cases. */
static void assert_simple_integer(const bbtc_vv_exact_metric_t* m, uint64_t value)
{
    /* Exact integer N * 2^E / (D * 2^F). Test directly using a rational
     * identity, never the rounded floating diagnostic we have not built. */
    assert_int_equal(m->denominator, 1);
    assert_int_equal(m->denominator_exponent, 0);
    if (value == 0)
    {
        assert_int_equal(m->used, 0);
        return;
    }
    assert_true(m->numerator_exponent <= 0);
    assert_true(m->numerator_exponent >= -63);
    const unsigned shift = (unsigned)(-m->numerator_exponent);
    const uint64_t low = value << shift;
    const uint64_t high = shift == 0 ? 0 : value >> (64U - shift);
    assert_int_equal(m->used, high == 0 ? 1 : 2);
    assert_int_equal(m->word[0], low);
    if (high != 0)
        assert_int_equal(m->word[1], high);
}

/**
 * @brief Native-family checks, deliberately not converting long double via
 *        double. All constants here are exactly representable.
 */
#define DEFINE_METRIC_TESTS(NAME, TYPE, SCALBN, P, TRUE_MIN, MAX_FINITE)         \
    static void test_##NAME##_absolute(void** state)                            \
    {                                                                           \
        (void)state;                                                            \
        bbtc_vv_exact_metric_t metric = {0};                                    \
        assert_int_equal(bbtc_vv_metric_exact_##NAME(                           \
            BBTC_VV_COMPARISON_ABSOLUTE, (TYPE)1, (TYPE)3, &metric),             \
            BBTC_VV_STATUS_SUCCESS);                                           \
        assert_simple_integer(&metric, 2);                                      \
        assert_int_equal(bbtc_vv_metric_exact_##NAME(                           \
            BBTC_VV_COMPARISON_ABSOLUTE, (TYPE)1, (TYPE)1, &metric),             \
            BBTC_VV_STATUS_SUCCESS);                                           \
        assert_simple_integer(&metric, 0);                                      \
        assert_int_equal(bbtc_vv_metric_exact_##NAME(                           \
            BBTC_VV_COMPARISON_ABSOLUTE, (TYPE)0, -(TYPE)0, &metric),            \
            BBTC_VV_STATUS_SUCCESS);                                           \
        assert_simple_integer(&metric, 0);                                      \
        assert_int_equal(bbtc_vv_metric_exact_##NAME(                           \
            BBTC_VV_COMPARISON_ABSOLUTE, (MAX_FINITE), -(MAX_FINITE), &metric),  \
            BBTC_VV_STATUS_SUCCESS);                                           \
        assert_true(metric.used != 0);                                         \
        assert_int_equal(metric.denominator, 1);                               \
        assert_int_equal(metric.denominator_exponent, 0);                      \
        assert_int_equal(bbtc_vv_metric_exact_##NAME(                           \
            BBTC_VV_COMPARISON_ABSOLUTE, (TYPE)0, (TRUE_MIN), &metric),          \
            BBTC_VV_STATUS_SUCCESS);                                           \
        assert_true(metric.used != 0);                                         \
        assert_int_equal(metric.denominator, 1);                               \
    }                                                                           \
    static void test_##NAME##_relative(void** state)                            \
    {                                                                           \
        (void)state;                                                            \
        bbtc_vv_exact_metric_t metric = {0};                                    \
        assert_int_equal(bbtc_vv_metric_exact_##NAME(                           \
            BBTC_VV_COMPARISON_REFERENCE_RELATIVE, (TYPE)2, (TYPE)3, &metric),  \
            BBTC_VV_STATUS_SUCCESS);                                           \
        assert_true(metric.used != 0);                                         \
        assert_true(metric.denominator != 0);                                  \
        assert_int_equal(metric.used, 1);                                      \
        /* |3-2|/|2| = 1/2; inspect the exact integer identity. */              \
        assert_int_equal(metric.numerator_exponent, metric.denominator_exponent); \
        assert_int_equal(metric.word[0] * 2U, metric.denominator);                  \
        assert_int_equal(bbtc_vv_metric_exact_##NAME(                           \
            BBTC_VV_COMPARISON_REFERENCE_RELATIVE, (TYPE)0, (TYPE)0, &metric),  \
            BBTC_VV_STATUS_OUTSIDE_DOMAIN);                                    \
        assert_int_equal(metric.used, 0);                                      \
        assert_int_equal(metric.denominator, 0);                               \
        assert_int_equal(bbtc_vv_metric_exact_##NAME(                           \
            BBTC_VV_COMPARISON_REFERENCE_RELATIVE, (TYPE)1, (TYPE)1, &metric),  \
            BBTC_VV_STATUS_SUCCESS);                                           \
        assert_int_equal(metric.used, 0);                                      \
        assert_true(metric.denominator != 0);                                  \
    }                                                                           \
    static void test_##NAME##_invalid(void** state)                             \
    {                                                                           \
        (void)state;                                                            \
        bbtc_vv_exact_metric_t metric = {0};                                    \
        assert_int_equal(bbtc_vv_metric_exact_##NAME(                           \
            BBTC_VV_COMPARISON_ABSOLUTE, (TYPE)1, (TYPE)1, NULL),               \
            BBTC_VV_STATUS_INVALID_ARGUMENT);                                  \
        assert_int_equal(bbtc_vv_metric_exact_##NAME(                           \
            BBTC_VV_COMPARISON_EXACT, (TYPE)1, (TYPE)1, &metric),                \
            BBTC_VV_STATUS_INVALID_ARGUMENT);                                  \
        assert_int_equal(metric.used, 0);                                      \
        assert_int_equal(bbtc_vv_metric_exact_##NAME(                           \
            BBTC_VV_COMPARISON_ABSOLUTE, (TYPE)INFINITY, (TYPE)NAN, &metric),    \
            BBTC_VV_STATUS_NAN_INPUT);                                          \
        assert_int_equal(bbtc_vv_metric_exact_##NAME(                           \
            BBTC_VV_COMPARISON_ABSOLUTE, (TYPE)INFINITY, (TYPE)1, &metric),      \
            BBTC_VV_STATUS_NONFINITE_INPUT);                                    \
        assert_int_equal(metric.denominator, 0);                               \
        assert_int_equal(metric.used, 0);                                      \
    }                                                                           \
    static void test_##NAME##_environment(void** state)                         \
    {                                                                           \
        (void)state;                                                            \
        fenv_t saved;                                                           \
        assert_int_equal(fegetenv(&saved), 0);                                  \
        const int modes[] = { FE_TONEAREST, FE_DOWNWARD,                        \
                              FE_UPWARD, FE_TOWARDZERO };                       \
        const int flags[] = { 0, FE_INVALID, FE_INEXACT | FE_UNDERFLOW,         \
                              FE_DIVBYZERO | FE_OVERFLOW };                    \
        for (size_t i = 0; i < sizeof(modes)/sizeof(modes[0]); ++i)            \
        {                                                                       \
            for (size_t j = 0; j < sizeof(flags)/sizeof(flags[0]); ++j)        \
            {                                                                   \
                assert_int_equal(fesetround(modes[i]), 0);                      \
                assert_int_equal(feclearexcept(FE_ALL_EXCEPT), 0);              \
                if (flags[j] != 0)                                              \
                    assert_int_equal(feraiseexcept(flags[j]), 0);               \
                const int before = fetestexcept(FE_ALL_EXCEPT);                \
                bbtc_vv_exact_metric_t result;                                  \
                assert_int_equal(bbtc_vv_metric_exact_##NAME(                   \
                    BBTC_VV_COMPARISON_ABSOLUTE, (TYPE)3, (TYPE)1, &result),    \
                    BBTC_VV_STATUS_SUCCESS);                                   \
                assert_int_equal(fegetround(), modes[i]);                      \
                assert_int_equal(fetestexcept(FE_ALL_EXCEPT), before);         \
                assert_int_equal(bbtc_vv_metric_exact_##NAME(                   \
                    BBTC_VV_COMPARISON_ABSOLUTE, (TYPE)NAN, (TYPE)1, &result),  \
                    BBTC_VV_STATUS_NAN_INPUT);                                  \
                assert_int_equal(fegetround(), modes[i]);                      \
                assert_int_equal(fetestexcept(FE_ALL_EXCEPT), before);         \
            }                                                                   \
        }                                                                       \
        assert_int_equal(fesetenv(&saved), 0);                                  \
    }

DEFINE_METRIC_TESTS(float, float, scalbnf, FLT_MANT_DIG, FLT_TRUE_MIN, FLT_MAX)
DEFINE_METRIC_TESTS(double, double, scalbn, DBL_MANT_DIG, DBL_TRUE_MIN, DBL_MAX)
DEFINE_METRIC_TESTS(long_double, long double, scalbnl, LDBL_MANT_DIG, LDBL_TRUE_MIN, LDBL_MAX)

static void test_repr_capacity(void** state)
{
    (void)state;
    assert_true(BBTC_VV_METRIC_LIMBS * 64U >= 33280U);
    assert_true(sizeof(bbtc_vv_exact_metric_t) >= 520U * sizeof(uint64_t));
}

int main(void)
{
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_float_absolute),
        cmocka_unit_test(test_double_absolute),
        cmocka_unit_test(test_long_double_absolute),
        cmocka_unit_test(test_float_relative),
        cmocka_unit_test(test_double_relative),
        cmocka_unit_test(test_long_double_relative),
        cmocka_unit_test(test_float_invalid),
        cmocka_unit_test(test_double_invalid),
        cmocka_unit_test(test_long_double_invalid),
        cmocka_unit_test(test_float_environment),
        cmocka_unit_test(test_double_environment),
        cmocka_unit_test(test_long_double_environment),
        cmocka_unit_test(test_repr_capacity)
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
