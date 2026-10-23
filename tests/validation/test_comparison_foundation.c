/**
 * @file test_comparison_foundation.c
 * @brief Deterministic cmocka checks of the B1b.3b comparison API foundation.
 *
 * @details
 * These tests exercise the internal decision preparation layer ONLY. They
 * intentionally do not link against a bbtc_vv_compare_* symbol, because a
 * correct comparison result must include a faithfully classified diagnostic
 * metric in Absolute and Reference-relative modes (B1b.3c).
 *
 * Assertions are independently derived from exact integer relationships or
 * the accepted argument-precedence contract, rather than from a recomputed
 * rounded floating-point subtraction. The B1b.2 exact-rational oracle remains
 * the independent large-scale numerical oracle for dyadic decisions.
 */
#include "comparison_internal.h"

#include <float.h>
#include <fenv.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>

#include <setjmp.h>
#include <stdarg.h>
#include <cmocka.h>

/**
 * @brief Generate family-specific contract checks without narrowing values.
 *
 * The expression (TYPE)0 or (TYPE)1 has the correct native family type.
 * The passed SCALBN is the native libm scaling operation for that family.
 * Floating-point input classification is delegated to the production kernel;
 * expected outcomes here are simple, analytically known relations.
 */
#define DEFINE_FAMILY_TESTS(NAME, TYPE, SCALBN, P)                              \
    static void test_##NAME##_decisions(void** state)                           \
    {                                                                           \
        (void)state;                                                            \
        bbtc_vv_finding_e finding = BBTC_VV_FINDING_NOT_EVALUATED;              \
        bbtc_vv_policy_##NAME##_t policy = {                                    \
            .method = BBTC_VV_COMPARISON_EXACT, .tolerance = (TYPE)0            \
        };                                                                      \
        assert_int_equal(bbtc_vv_prepare_##NAME(                                \
            (TYPE)0, -(TYPE)0, &policy, &finding), BBTC_VV_STATUS_SUCCESS);     \
        assert_int_equal(finding, BBTC_VV_FINDING_PASS);                        \
        assert_int_equal(bbtc_vv_prepare_##NAME(                                \
            (TYPE)1, (TYPE)2, &policy, &finding), BBTC_VV_STATUS_SUCCESS);      \
        assert_int_equal(finding, BBTC_VV_FINDING_FAIL);                        \
        policy.method = BBTC_VV_COMPARISON_ABSOLUTE;                            \
        policy.tolerance = (TYPE)1;                                            \
        assert_int_equal(bbtc_vv_prepare_##NAME(                                \
            (TYPE)1, (TYPE)2, &policy, &finding), BBTC_VV_STATUS_SUCCESS);      \
        assert_int_equal(finding, BBTC_VV_FINDING_PASS);                        \
        policy.method = BBTC_VV_COMPARISON_REFERENCE_RELATIVE;                  \
        policy.tolerance = (TYPE)0.5;                                          \
        assert_int_equal(bbtc_vv_prepare_##NAME(                                \
            (TYPE)2, (TYPE)3, &policy, &finding), BBTC_VV_STATUS_SUCCESS);      \
        assert_int_equal(finding, BBTC_VV_FINDING_PASS);                        \
        /* The exact distance is 2^p + 1, not the rounded 2^p. */              \
        const TYPE large = SCALBN((TYPE)1, (P));                                \
        policy.method = BBTC_VV_COMPARISON_ABSOLUTE;                            \
        policy.tolerance = large;                                              \
        assert_int_equal(bbtc_vv_prepare_##NAME(                                \
            large, -(TYPE)1, &policy, &finding), BBTC_VV_STATUS_SUCCESS);      \
        assert_int_equal(finding, BBTC_VV_FINDING_FAIL);                        \
        policy.method = BBTC_VV_COMPARISON_REFERENCE_RELATIVE;                  \
        policy.tolerance = (TYPE)1;                                            \
        assert_int_equal(bbtc_vv_prepare_##NAME(                                \
            large, -(TYPE)1, &policy, &finding), BBTC_VV_STATUS_SUCCESS);      \
        assert_int_equal(finding, BBTC_VV_FINDING_FAIL);                        \
    }                                                                           \
    static void test_##NAME##_precedence(void** state)                          \
    {                                                                           \
        (void)state;                                                            \
        bbtc_vv_finding_e finding = BBTC_VV_FINDING_PASS;                      \
        bbtc_vv_policy_##NAME##_t policy = {                                    \
            .method = BBTC_VV_COMPARISON_ABSOLUTE, .tolerance = (TYPE)1         \
        };                                                                      \
        assert_int_equal(bbtc_vv_prepare_##NAME(                                \
            (TYPE)1, (TYPE)2, &policy, NULL), BBTC_VV_STATUS_INVALID_ARGUMENT);\
        assert_int_equal(bbtc_vv_prepare_##NAME(                                \
            (TYPE)1, (TYPE)2, NULL, &finding), BBTC_VV_STATUS_INVALID_ARGUMENT);\
        assert_int_equal(finding, BBTC_VV_FINDING_NOT_EVALUATED);               \
        finding = BBTC_VV_FINDING_PASS;                                        \
        policy.method = (bbtc_vv_comparison_method_e)255;                      \
        assert_int_equal(bbtc_vv_prepare_##NAME(                                \
            (TYPE)1, (TYPE)2, &policy, &finding), BBTC_VV_STATUS_INVALID_ARGUMENT);\
        assert_int_equal(finding, BBTC_VV_FINDING_NOT_EVALUATED);               \
        policy.method = BBTC_VV_COMPARISON_ABSOLUTE;                            \
        policy.tolerance = (TYPE)-1;                                           \
        assert_int_equal(bbtc_vv_prepare_##NAME(                                \
            (TYPE)1, (TYPE)2, &policy, &finding), BBTC_VV_STATUS_INVALID_ARGUMENT);\
        assert_int_equal(finding, BBTC_VV_FINDING_NOT_EVALUATED);               \
        policy.tolerance = (TYPE)1;                                            \
        assert_int_equal(bbtc_vv_prepare_##NAME(                                \
            (TYPE)INFINITY, (TYPE)NAN, &policy, &finding), BBTC_VV_STATUS_NAN_INPUT);\
        assert_int_equal(finding, BBTC_VV_FINDING_NOT_EVALUATED);               \
        assert_int_equal(bbtc_vv_prepare_##NAME(                                \
            (TYPE)INFINITY, (TYPE)0, &policy, &finding), BBTC_VV_STATUS_NONFINITE_INPUT);\
        policy.method = BBTC_VV_COMPARISON_REFERENCE_RELATIVE;                  \
        assert_int_equal(bbtc_vv_prepare_##NAME(                                \
            (TYPE)0, (TYPE)0, &policy, &finding), BBTC_VV_STATUS_OUTSIDE_DOMAIN);\
        assert_int_equal(finding, BBTC_VV_FINDING_NOT_EVALUATED);               \
        policy.method = BBTC_VV_COMPARISON_EXACT;                               \
        policy.tolerance = (TYPE)1;                                            \
        assert_int_equal(bbtc_vv_prepare_##NAME(                                \
            (TYPE)1, (TYPE)1, &policy, &finding), BBTC_VV_STATUS_INVALID_ARGUMENT);\
        assert_int_equal(finding, BBTC_VV_FINDING_NOT_EVALUATED);               \
    }                                                                           \
    static void test_##NAME##_environment(void** state)                         \
    {                                                                           \
        (void)state;                                                            \
        fenv_t initial;                                                         \
        assert_int_equal(fegetenv(&initial), 0);                                \
        const int modes[] = {FE_TONEAREST, FE_DOWNWARD, FE_UPWARD, FE_TOWARDZERO};\
        const int flags[] = {0, FE_INEXACT, FE_INVALID | FE_UNDERFLOW};          \
        for (size_t i = 0; i < sizeof(modes) / sizeof(modes[0]); ++i)          \
        {                                                                       \
            for (size_t j = 0; j < sizeof(flags) / sizeof(flags[0]); ++j)     \
            {                                                                   \
                assert_int_equal(fesetround(modes[i]), 0);                      \
                assert_int_equal(feclearexcept(FE_ALL_EXCEPT), 0);              \
                if (flags[j] != 0)                                             \
                    assert_int_equal(feraiseexcept(flags[j]), 0);               \
                const int before = fetestexcept(FE_ALL_EXCEPT);                 \
                const int incoming_mode = fegetround();                         \
                bbtc_vv_policy_##NAME##_t policy = {                            \
                    .method = BBTC_VV_COMPARISON_ABSOLUTE,                      \
                    .tolerance = (TYPE)1                                       \
                };                                                              \
                bbtc_vv_finding_e finding = BBTC_VV_FINDING_NOT_EVALUATED;      \
                assert_int_equal(bbtc_vv_prepare_##NAME(                        \
                    (TYPE)1, (TYPE)2, &policy, &finding), BBTC_VV_STATUS_SUCCESS);\
                assert_int_equal(finding, BBTC_VV_FINDING_PASS);                \
                assert_int_equal(fegetround(), incoming_mode);                 \
                assert_int_equal(fetestexcept(FE_ALL_EXCEPT), before);          \
                policy.tolerance = (TYPE)-1;                                    \
                assert_int_equal(bbtc_vv_prepare_##NAME(                        \
                    (TYPE)1, (TYPE)2, &policy, &finding),                         \
                    BBTC_VV_STATUS_INVALID_ARGUMENT);                          \
                assert_int_equal(finding, BBTC_VV_FINDING_NOT_EVALUATED);      \
                assert_int_equal(fegetround(), incoming_mode);                 \
                assert_int_equal(fetestexcept(FE_ALL_EXCEPT), before);          \
            }                                                                   \
        }                                                                       \
        assert_int_equal(fesetenv(&initial), 0);                                \
    }

DEFINE_FAMILY_TESTS(float, float, scalbnf, FLT_MANT_DIG)
DEFINE_FAMILY_TESTS(double, double, scalbn, DBL_MANT_DIG)
DEFINE_FAMILY_TESTS(long_double, long double, scalbnl, LDBL_MANT_DIG)

#undef DEFINE_FAMILY_TESTS

/**
 * @brief Ensure the accepted data model preserves the mandated enum values.
 *
 * These are functional semantics, not external stable ABI layout promises.
 */
static void test_contract_enumerators(void** state)
{
    (void)state;
    assert_int_equal(BBTC_VV_COMPARISON_EXACT, 1);
    assert_int_equal(BBTC_VV_COMPARISON_ABSOLUTE, 2);
    assert_int_equal(BBTC_VV_COMPARISON_REFERENCE_RELATIVE, 3);
    assert_int_equal(BBTC_VV_STATUS_SUCCESS, 0);
    assert_int_equal(BBTC_VV_STATUS_NUMERICAL_FAILURE, 6);
    assert_int_equal(BBTC_VV_FINDING_NOT_EVALUATED, 0);
    assert_int_equal(BBTC_VV_FINDING_INDETERMINATE, 3);
    assert_int_equal(BBTC_VV_METRIC_NOT_COMPUTED, 0);
    assert_int_equal(BBTC_VV_METRIC_UNRESOLVED, 5);
}

int main(void)
{
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_contract_enumerators),
        cmocka_unit_test(test_float_decisions),
        cmocka_unit_test(test_double_decisions),
        cmocka_unit_test(test_long_double_decisions),
        cmocka_unit_test(test_float_precedence),
        cmocka_unit_test(test_double_precedence),
        cmocka_unit_test(test_long_double_precedence),
        cmocka_unit_test(test_float_environment),
        cmocka_unit_test(test_double_environment),
        cmocka_unit_test(test_long_double_environment)
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
