/**
 * @file test_comparison_api.c
 * @brief B1b.3c-3 deterministic cmocka tests of the complete comparison API.
 *
 * @details
 * Each expected finding follows the exact native input values, never the
 * rounded `metric`. A deliberate adversary uses the mathematical difference
 * 2^p+1 with tolerance 2^p, which MUST FAIL although a native subtraction
 * may round to 2^p. The tests verify the full contract surface: three
 * precision families, metric states, invalid-argument precedence, canonical
 * clearing, and preservation of caller FP rounding/exception state.
 *
 * The existing three independent Python oracles continue checking arithmetic
 * engines separately. A new end-to-end differential oracle is deferred to
 * B1b.3d and is an explicit requirement before complete API acceptance.
 */
#include "comparison.h"

#include <fenv.h>
#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>

#include <setjmp.h>
#include <stdarg.h>
#include <cmocka.h>

/**
 * @brief Verify the operation-failure canonical representation, not padding.
 * All three members have specified zero-valued representations. Padding bytes
 * are deliberately not compared; C does not promise their initialization.
 */
#define ASSERT_CLEARED(result) do {                                            \
    assert_int_equal((result).finding, BBTC_VV_FINDING_NOT_EVALUATED);          \
    assert_int_equal((result).metric_state, BBTC_VV_METRIC_NOT_COMPUTED);       \
    assert_true((result).metric == 0 && !signbit((result).metric));             \
} while (0)

/**
 * @brief Native-family tests; SCALBN keeps the full precision family.
 * The user-level APIs MUST NOT shorten long double to double, including for
 * the exact absolute difference at exponents outside the binary64 range.
 */
#define DEFINE_COMPARE_TESTS(NAME, TYPE, SCALBN, P, MAX_FINITE, TRUE_MIN)     \
    static void test_##NAME##_exact(void** state)                              \
    {                                                                          \
        (void)state;                                                           \
        bbtc_vv_result_##NAME##_t r = {0};                                    \
        bbtc_vv_policy_##NAME##_t policy = {                                   \
            .method = BBTC_VV_COMPARISON_EXACT, .tolerance = -(TYPE)0          \
        };                                                                     \
        assert_int_equal(bbtc_vv_compare_##NAME(                              \
            (TYPE)0, -(TYPE)0, &policy, &r), BBTC_VV_STATUS_SUCCESS);          \
        assert_int_equal(r.finding, BBTC_VV_FINDING_PASS);                     \
        assert_int_equal(r.metric_state, BBTC_VV_METRIC_NOT_COMPUTED);         \
        assert_true(r.metric == 0 && !signbit(r.metric));                       \
        assert_int_equal(bbtc_vv_compare_##NAME(                              \
            (TYPE)1, (TYPE)2, &policy, &r), BBTC_VV_STATUS_SUCCESS);           \
        assert_int_equal(r.finding, BBTC_VV_FINDING_FAIL);                     \
        assert_int_equal(r.metric_state, BBTC_VV_METRIC_NOT_COMPUTED);         \
    }                                                                          \
    static void test_##NAME##_absolute(void** state)                           \
    {                                                                          \
        (void)state;                                                           \
        bbtc_vv_result_##NAME##_t r = {0};                                    \
        bbtc_vv_policy_##NAME##_t policy = {                                   \
            .method = BBTC_VV_COMPARISON_ABSOLUTE, .tolerance = (TYPE)1        \
        };                                                                     \
        assert_int_equal(bbtc_vv_compare_##NAME(                              \
            (TYPE)2, (TYPE)3, &policy, &r), BBTC_VV_STATUS_SUCCESS);           \
        assert_int_equal(r.finding, BBTC_VV_FINDING_PASS);                     \
        assert_int_equal(r.metric_state, BBTC_VV_METRIC_FINITE_EXACT);         \
        assert_true(r.metric == (TYPE)1);                                      \
        assert_int_equal(bbtc_vv_compare_##NAME(                              \
            (TYPE)2, (TYPE)2, &policy, &r), BBTC_VV_STATUS_SUCCESS);           \
        assert_int_equal(r.finding, BBTC_VV_FINDING_PASS);                     \
        assert_int_equal(r.metric_state, BBTC_VV_METRIC_FINITE_EXACT);         \
        assert_true(r.metric == (TYPE)0 && !signbit(r.metric));                \
        const TYPE large = SCALBN((TYPE)1, P);                                 \
        policy.tolerance = large;                                              \
        assert_int_equal(bbtc_vv_compare_##NAME(                              \
            large, -(TYPE)1, &policy, &r), BBTC_VV_STATUS_SUCCESS);            \
        /* Exact 2^p+1 > 2^p, despite a rounded metric of 2^p. */            \
        assert_int_equal(r.finding, BBTC_VV_FINDING_FAIL);                     \
        assert_int_equal(r.metric_state, BBTC_VV_METRIC_FINITE_ROUNDED);       \
        assert_true(r.metric == large);                                        \
        policy.tolerance = (MAX_FINITE);                                       \
        assert_int_equal(bbtc_vv_compare_##NAME(                              \
            (MAX_FINITE), -(MAX_FINITE), &policy, &r),                          \
            BBTC_VV_STATUS_SUCCESS);                                          \
        assert_int_equal(r.finding, BBTC_VV_FINDING_FAIL);                     \
        assert_int_equal(r.metric_state, BBTC_VV_METRIC_EXCEEDS_RANGE);        \
        assert_true(r.metric == (TYPE)0 && !signbit(r.metric));                \
    }                                                                          \
    static void test_##NAME##_relative(void** state)                           \
    {                                                                          \
        (void)state;                                                           \
        bbtc_vv_result_##NAME##_t r = {0};                                    \
        bbtc_vv_policy_##NAME##_t policy = {                                   \
            .method = BBTC_VV_COMPARISON_REFERENCE_RELATIVE,                   \
            .tolerance = (TYPE)0.5                                            \
        };                                                                     \
        assert_int_equal(bbtc_vv_compare_##NAME(                              \
            (TYPE)2, (TYPE)3, &policy, &r), BBTC_VV_STATUS_SUCCESS);           \
        assert_int_equal(r.finding, BBTC_VV_FINDING_PASS);                     \
        assert_int_equal(r.metric_state, BBTC_VV_METRIC_FINITE_EXACT);         \
        assert_true(r.metric == (TYPE)0.5);                                    \
        assert_int_equal(bbtc_vv_compare_##NAME(                              \
            (TYPE)-3, (TYPE)-2, &policy, &r), BBTC_VV_STATUS_SUCCESS);         \
        assert_int_equal(r.finding, BBTC_VV_FINDING_PASS);                     \
        assert_int_equal(r.metric_state, BBTC_VV_METRIC_FINITE_ROUNDED);       \
        assert_true(r.metric > (TYPE)0 && r.metric < (TYPE)0.5);                \
        policy.tolerance = (TYPE)1;                                            \
        const TYPE large = SCALBN((TYPE)1, P);                                 \
        assert_int_equal(bbtc_vv_compare_##NAME(                              \
            large, -(TYPE)1, &policy, &r), BBTC_VV_STATUS_SUCCESS);            \
        assert_int_equal(r.finding, BBTC_VV_FINDING_FAIL);                     \
        assert_int_equal(r.metric_state, BBTC_VV_METRIC_FINITE_ROUNDED);       \
        assert_true(r.metric == (TYPE)1);                                      \
        assert_int_equal(bbtc_vv_compare_##NAME(                              \
            (TRUE_MIN), (TYPE)1, &policy, &r), BBTC_VV_STATUS_SUCCESS);         \
        assert_int_equal(r.finding, BBTC_VV_FINDING_FAIL);                     \
        assert_int_equal(r.metric_state, BBTC_VV_METRIC_EXCEEDS_RANGE);        \
        assert_true(r.metric == (TYPE)0);                                      \
    }                                                                          \
    static void test_##NAME##_invalid(void** state)                            \
    {                                                                          \
        (void)state;                                                           \
        bbtc_vv_result_##NAME##_t r = {                                       \
            BBTC_VV_FINDING_PASS, BBTC_VV_METRIC_FINITE_EXACT, (TYPE)99        \
        };                                                                     \
        bbtc_vv_policy_##NAME##_t policy = {                                   \
            .method = BBTC_VV_COMPARISON_ABSOLUTE, .tolerance = (TYPE)1        \
        };                                                                     \
        assert_int_equal(bbtc_vv_compare_##NAME(                              \
            (TYPE)0, (TYPE)1, &policy, NULL), BBTC_VV_STATUS_INVALID_ARGUMENT);\
        assert_int_equal(bbtc_vv_compare_##NAME(                              \
            (TYPE)0, (TYPE)1, NULL, &r), BBTC_VV_STATUS_INVALID_ARGUMENT);     \
        ASSERT_CLEARED(r);                                                     \
        policy.method = (bbtc_vv_comparison_method_e)255;                      \
        r.metric = (TYPE)9;                                                    \
        assert_int_equal(bbtc_vv_compare_##NAME(                              \
            (TYPE)0, (TYPE)1, &policy, &r), BBTC_VV_STATUS_INVALID_ARGUMENT);  \
        ASSERT_CLEARED(r);                                                     \
        policy.method = BBTC_VV_COMPARISON_ABSOLUTE;                            \
        policy.tolerance = (TYPE)-1;                                           \
        assert_int_equal(bbtc_vv_compare_##NAME(                              \
            (TYPE)0, (TYPE)1, &policy, &r), BBTC_VV_STATUS_INVALID_ARGUMENT);  \
        ASSERT_CLEARED(r);                                                     \
        policy.tolerance = (TYPE)INFINITY;                                     \
        assert_int_equal(bbtc_vv_compare_##NAME(                              \
            (TYPE)NAN, (TYPE)0, &policy, &r), BBTC_VV_STATUS_NAN_INPUT);        \
        ASSERT_CLEARED(r);                                                     \
        policy.tolerance = (TYPE)1;                                            \
        assert_int_equal(bbtc_vv_compare_##NAME(                              \
            (TYPE)INFINITY, (TYPE)0, &policy, &r),                              \
            BBTC_VV_STATUS_NONFINITE_INPUT);                                   \
        ASSERT_CLEARED(r);                                                     \
        policy.method = BBTC_VV_COMPARISON_REFERENCE_RELATIVE;                 \
        assert_int_equal(bbtc_vv_compare_##NAME(                              \
            (TYPE)0, (TYPE)0, &policy, &r), BBTC_VV_STATUS_OUTSIDE_DOMAIN);    \
        ASSERT_CLEARED(r);                                                     \
        policy.method = BBTC_VV_COMPARISON_EXACT;                              \
        policy.tolerance = (TYPE)1;                                            \
        assert_int_equal(bbtc_vv_compare_##NAME(                              \
            (TYPE)1, (TYPE)1, &policy, &r), BBTC_VV_STATUS_INVALID_ARGUMENT);  \
        ASSERT_CLEARED(r);                                                     \
    }                                                                          \
    static void test_##NAME##_environment(void** state)                        \
    {                                                                          \
        (void)state;                                                           \
        fenv_t original;                                                       \
        assert_int_equal(fegetenv(&original), 0);                              \
        const int modes[] =                                                    \
            {FE_TONEAREST, FE_UPWARD, FE_DOWNWARD, FE_TOWARDZERO};             \
        const int flags[] = {0, FE_INEXACT, FE_INVALID | FE_UNDERFLOW};         \
        TYPE first_metric = (TYPE)0;                                           \
        for (size_t i = 0; i < 4; ++i)                                         \
        {                                                                      \
            for (size_t j = 0; j < 3; ++j)                                     \
            {                                                                  \
                assert_int_equal(fesetround(modes[i]), 0);                     \
                assert_int_equal(feclearexcept(FE_ALL_EXCEPT), 0);             \
                if (flags[j] != 0)                                             \
                    assert_int_equal(feraiseexcept(flags[j]), 0);              \
                const int before = fetestexcept(FE_ALL_EXCEPT);                \
                bbtc_vv_policy_##NAME##_t policy = {                           \
                    .method = BBTC_VV_COMPARISON_REFERENCE_RELATIVE,          \
                    .tolerance = (TYPE)0.5                                    \
                };                                                             \
                bbtc_vv_result_##NAME##_t r = {0};                            \
                assert_int_equal(bbtc_vv_compare_##NAME(                      \
                    (TYPE)3, (TYPE)4, &policy, &r), BBTC_VV_STATUS_SUCCESS);  \
                assert_int_equal(r.finding, BBTC_VV_FINDING_PASS);             \
                assert_int_equal(r.metric_state, BBTC_VV_METRIC_FINITE_ROUNDED);\
                if (i == 0 && j == 0)                                         \
                    first_metric = r.metric;                                   \
                else                                                           \
                    assert_true(r.metric == first_metric);                     \
                assert_int_equal(fegetround(), modes[i]);                      \
                assert_int_equal(fetestexcept(FE_ALL_EXCEPT), before);         \
                policy.tolerance = (TYPE)-1;                                   \
                assert_int_equal(bbtc_vv_compare_##NAME(                      \
                    (TYPE)3, (TYPE)4, &policy, &r),                             \
                    BBTC_VV_STATUS_INVALID_ARGUMENT);                         \
                ASSERT_CLEARED(r);                                             \
                assert_int_equal(fegetround(), modes[i]);                      \
                assert_int_equal(fetestexcept(FE_ALL_EXCEPT), before);         \
            }                                                                  \
        }                                                                      \
        assert_int_equal(fesetenv(&original), 0);                               \
    }

DEFINE_COMPARE_TESTS(float, float, scalbnf, FLT_MANT_DIG, FLT_MAX, FLT_TRUE_MIN)
DEFINE_COMPARE_TESTS(double, double, scalbn, DBL_MANT_DIG, DBL_MAX, DBL_TRUE_MIN)
DEFINE_COMPARE_TESTS(long_double, long double, scalbnl, LDBL_MANT_DIG,
    LDBL_MAX, LDBL_TRUE_MIN)

#undef DEFINE_COMPARE_TESTS
#undef ASSERT_CLEARED

int main(void)
{
    const struct CMUnitTest tests[] =
    {
        cmocka_unit_test(test_float_exact),
        cmocka_unit_test(test_double_exact),
        cmocka_unit_test(test_long_double_exact),
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
        cmocka_unit_test(test_long_double_environment)
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
