/**
 * @file test_exact_dyadic.c
 * @brief Adversarial cmocka self-tests of the private exact dyadic kernel.
 *
 * @details
 * Test expectations are derived from exact dyadic identities, not from
 * subtracting operands with the algorithm under test. Tests cover all three
 * native precision families and all supported incoming rounding modes.
 * This is a SOFTWARE verification suite, not physical ballistics validation.
 */
#include <float.h>
#include <fenv.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdarg.h>
#include <setjmp.h>

#include <cmocka.h>
#include "exact_dyadic.h"

/**
 * @brief Verify known exact mathematical outcomes in all three precisions.
 *
 * Constructs dyadic endpoints whose differences are precisely derivable.
 * The family-specific literals are not converted through `double`.
 */
#define DEFINE_BASIC_TEST(NAME, TYPE, SUFFIX, P)                                            \
    static void test_##NAME##_mathematical(void** state)                                   \
    {                                                                                      \
        (void)state;                                                                       \
        bool pass = false;                                                                 \
        bbtc_vv_dyadic_status_e status;                                                    \
        status = bbtc_vv_dyadic_compare_##NAME(                                            \
            BBTC_VV_DYADIC_EXACT, (TYPE)1, (TYPE)1, (TYPE)0, &pass);                       \
        assert_int_equal(status, BBTC_VV_DYADIC_SUCCESS);                                 \
        assert_true(pass);                                                                 \
        status = bbtc_vv_dyadic_compare_##NAME(                                            \
            BBTC_VV_DYADIC_EXACT, (TYPE)0, -(TYPE)0, (TYPE)0, &pass);                      \
        assert_int_equal(status, BBTC_VV_DYADIC_SUCCESS);                                 \
        assert_true(pass);                                                                 \
        status = bbtc_vv_dyadic_compare_##NAME(                                            \
            BBTC_VV_DYADIC_EXACT, (TYPE)1, (TYPE)2, (TYPE)0, &pass);                       \
        assert_int_equal(status, BBTC_VV_DYADIC_SUCCESS);                                 \
        assert_false(pass);                                                                \
        /* Exact absolute difference |5-2| = 3: inclusive boundary PASS. */              \
        status = bbtc_vv_dyadic_compare_##NAME(                                            \
            BBTC_VV_DYADIC_ABSOLUTE, (TYPE)2, (TYPE)5, (TYPE)3, &pass);                   \
        assert_int_equal(status, BBTC_VV_DYADIC_SUCCESS);                                 \
        assert_true(pass);                                                                 \
        status = bbtc_vv_dyadic_compare_##NAME(                                            \
            BBTC_VV_DYADIC_ABSOLUTE, (TYPE)2, (TYPE)5, (TYPE)2, &pass);                   \
        assert_int_equal(status, BBTC_VV_DYADIC_SUCCESS);                                 \
        assert_false(pass);                                                                \
        /* |-4 - (-2)| / |-2| = 1; boundary equality is inclusive. */                     \
        status = bbtc_vv_dyadic_compare_##NAME(                                            \
            BBTC_VV_DYADIC_REFERENCE_RELATIVE, -(TYPE)2, -(TYPE)4, (TYPE)1, &pass);       \
        assert_int_equal(status, BBTC_VV_DYADIC_SUCCESS);                                 \
        assert_true(pass);                                                                 \
        status = bbtc_vv_dyadic_compare_##NAME(                                            \
            BBTC_VV_DYADIC_REFERENCE_RELATIVE, -(TYPE)2, -(TYPE)4, (TYPE)0.5, &pass);     \
        assert_int_equal(status, BBTC_VV_DYADIC_SUCCESS);                                 \
        assert_false(pass);                                                                \
        /* Reference-relative is asymmetric: |2-1|/|1|=1; |1-2|/|2|=1/2. */              \
        status = bbtc_vv_dyadic_compare_##NAME(                                            \
            BBTC_VV_DYADIC_REFERENCE_RELATIVE, (TYPE)1, (TYPE)2, (TYPE)0.5, &pass);       \
        assert_int_equal(status, BBTC_VV_DYADIC_SUCCESS);                                 \
        assert_false(pass);                                                                \
        status = bbtc_vv_dyadic_compare_##NAME(                                            \
            BBTC_VV_DYADIC_REFERENCE_RELATIVE, (TYPE)2, (TYPE)1, (TYPE)0.5, &pass);       \
        assert_int_equal(status, BBTC_VV_DYADIC_SUCCESS);                                 \
        assert_true(pass);                                                                 \
        /* The rounded difference may equal the tolerance, but EXACT fails. */           \
        const TYPE two_to_p = scalbn##SUFFIX((TYPE)1, (P));                                \
        status = bbtc_vv_dyadic_compare_##NAME(                                            \
            BBTC_VV_DYADIC_ABSOLUTE, two_to_p, (TYPE)-1, two_to_p, &pass);                \
        assert_int_equal(status, BBTC_VV_DYADIC_SUCCESS);                                 \
        assert_false(pass);                                                                \
        status = bbtc_vv_dyadic_compare_##NAME(                                            \
            BBTC_VV_DYADIC_REFERENCE_RELATIVE, two_to_p, (TYPE)-1, (TYPE)1, &pass);       \
        assert_int_equal(status, BBTC_VV_DYADIC_SUCCESS);                                 \
        assert_false(pass);                                                                \
        /* An adjacent represented value differs from 1 by exactly one ULP. */            \
        const TYPE above = nextafter##SUFFIX((TYPE)1, (TYPE)2);                            \
        const TYPE ulp = scalbn##SUFFIX((TYPE)1, 1 - (P));                                  \
        status = bbtc_vv_dyadic_compare_##NAME(                                            \
            BBTC_VV_DYADIC_ABSOLUTE, (TYPE)1, above, ulp, &pass);                         \
        assert_int_equal(status, BBTC_VV_DYADIC_SUCCESS);                                 \
        assert_true(pass);                                                                 \
        status = bbtc_vv_dyadic_compare_##NAME(                                            \
            BBTC_VV_DYADIC_ABSOLUTE, (TYPE)1, above, (TYPE)0, &pass);                     \
        assert_int_equal(status, BBTC_VV_DYADIC_SUCCESS);                                 \
        assert_false(pass);                                                                \
    }

DEFINE_BASIC_TEST(float, float, f, FLT_MANT_DIG)
DEFINE_BASIC_TEST(double, double, , DBL_MANT_DIG)
DEFINE_BASIC_TEST(long_double, long double, l, LDBL_MANT_DIG)
#undef DEFINE_BASIC_TEST

/** @brief Invalid inputs must not report a successful or stale Boolean. */
static void test_failure_precedence(void** state)
{
    (void)state;
    bool pass = true;

    assert_int_equal(bbtc_vv_dyadic_compare_double(
        (bbtc_vv_dyadic_method_e)0, 1, 1, 0, &pass), BBTC_VV_DYADIC_INVALID_ARGUMENT);
    assert_false(pass);
    assert_int_equal(bbtc_vv_dyadic_compare_double(
        BBTC_VV_DYADIC_EXACT, 1, 1, 0, NULL), BBTC_VV_DYADIC_INVALID_ARGUMENT);
    assert_int_equal(bbtc_vv_dyadic_compare_double(
        BBTC_VV_DYADIC_ABSOLUTE, NAN, INFINITY, 0, &pass), BBTC_VV_DYADIC_NAN_INPUT);
    assert_false(pass);
    assert_int_equal(bbtc_vv_dyadic_compare_double(
        BBTC_VV_DYADIC_ABSOLUTE, 1, INFINITY, 0, &pass), BBTC_VV_DYADIC_NONFINITE_INPUT);
    assert_int_equal(bbtc_vv_dyadic_compare_double(
        BBTC_VV_DYADIC_ABSOLUTE, 1, 1, -1, &pass), BBTC_VV_DYADIC_INVALID_ARGUMENT);
    assert_int_equal(bbtc_vv_dyadic_compare_double(
        BBTC_VV_DYADIC_EXACT, 1, 1, 1, &pass), BBTC_VV_DYADIC_INVALID_ARGUMENT);
    assert_int_equal(bbtc_vv_dyadic_compare_double(
        BBTC_VV_DYADIC_REFERENCE_RELATIVE, 0, 0, 0, &pass), BBTC_VV_DYADIC_OUTSIDE_DOMAIN);
}

/**
 * @brief Exercise the full supported exponent span and exact relative product.
 *
 * Comparisons use exact identities: |M-(-M)| = 2M and 2M <= 2|M|.
 * For smallest subnormal epsilon, |0-epsilon| = epsilon exactly.
 */
#define DEFINE_EXTREME_TEST(NAME, TYPE, SUFFIX, MAX, MIN)                                \
    static void test_##NAME##_extremes(void** state)                                      \
    {                                                                                    \
        (void)state;                                                                     \
        bool pass = false;                                                               \
        bbtc_vv_dyadic_status_e status;                                                  \
        const TYPE max_value = (MAX);                                                    \
        const TYPE tiny = (MIN);                                                         \
        status = bbtc_vv_dyadic_compare_##NAME(                                          \
            BBTC_VV_DYADIC_ABSOLUTE, max_value, -max_value, max_value, &pass);           \
        assert_int_equal(status, BBTC_VV_DYADIC_SUCCESS);                               \
        assert_false(pass);                                                              \
        status = bbtc_vv_dyadic_compare_##NAME(                                          \
            BBTC_VV_DYADIC_REFERENCE_RELATIVE, max_value, -max_value, (TYPE)2, &pass);   \
        assert_int_equal(status, BBTC_VV_DYADIC_SUCCESS);                               \
        assert_true(pass);                                                               \
        status = bbtc_vv_dyadic_compare_##NAME(                                          \
            BBTC_VV_DYADIC_REFERENCE_RELATIVE, tiny, (TYPE)0, (TYPE)1, &pass);           \
        assert_int_equal(status, BBTC_VV_DYADIC_SUCCESS);                               \
        assert_true(pass);                                                               \
        status = bbtc_vv_dyadic_compare_##NAME(                                          \
            BBTC_VV_DYADIC_ABSOLUTE, (TYPE)0, tiny, tiny, &pass);                        \
        assert_int_equal(status, BBTC_VV_DYADIC_SUCCESS);                               \
        assert_true(pass);                                                               \
        status = bbtc_vv_dyadic_compare_##NAME(                                          \
            BBTC_VV_DYADIC_ABSOLUTE, (TYPE)0, tiny, (TYPE)0, &pass);                     \
        assert_int_equal(status, BBTC_VV_DYADIC_SUCCESS);                               \
        assert_false(pass);                                                              \
    }

DEFINE_EXTREME_TEST(float, float, f, FLT_MAX, FLT_TRUE_MIN)
DEFINE_EXTREME_TEST(double, double, , DBL_MAX, DBL_TRUE_MIN)
DEFINE_EXTREME_TEST(long_double, long double, l, LDBL_MAX, LDBL_TRUE_MIN)
#undef DEFINE_EXTREME_TEST

/** @brief Verify that the entire caller-visible fenv survives normal returns. */
static void test_preserve_fenv(void** state)
{
    (void)state;
    fenv_t original;
    assert_int_equal(fegetenv(&original), 0);

    const int modes[] = {FE_TONEAREST, FE_UPWARD, FE_DOWNWARD, FE_TOWARDZERO};
    for (size_t m = 0; m < sizeof(modes) / sizeof(modes[0]); ++m)
    {
        assert_int_equal(fesetround(modes[m]), 0);
        assert_int_equal(feclearexcept(FE_ALL_EXCEPT), 0);
        assert_int_equal(feraiseexcept(FE_INVALID | FE_INEXACT), 0);
        const int before = fetestexcept(FE_ALL_EXCEPT);
        bool pass = false;
        assert_int_equal(bbtc_vv_dyadic_compare_long_double(
            BBTC_VV_DYADIC_REFERENCE_RELATIVE,
            scalbnl(1.0L, 64), -1.0L, 1.0L, &pass), BBTC_VV_DYADIC_SUCCESS);
        assert_false(pass);
        assert_int_equal(fegetround(), modes[m]);
        assert_int_equal(fetestexcept(FE_ALL_EXCEPT), before);

        assert_int_equal(bbtc_vv_dyadic_compare_double(
            BBTC_VV_DYADIC_ABSOLUTE, NAN, INFINITY, 0.0, &pass), BBTC_VV_DYADIC_NAN_INPUT);
        assert_int_equal(fegetround(), modes[m]);
        assert_int_equal(fetestexcept(FE_ALL_EXCEPT), before);
    }
    assert_int_equal(fesetenv(&original), 0);
}

/**
 * @brief Exercise every standard exception flag alone, in combinations, and
 *        with each of the four supported rounding modes.
 *
 * @details
 * These cases cover all three typed entry points and important early-return
 * statuses. `feholdexcept` must isolate numeric work while `fesetenv` returns
 * the exact incoming state. The matrix deliberately samples all five common
 * IEC 60559 sticky flags *individually*; a preseeded INEXACT|INVALID mask
 * alone would conceal accidental loss of OVERFLOW, UNDERFLOW, or DIVBYZERO.
 *
 * The checks inspect the environment immediately after each kernel call,
 * before performing any further floating-point computations in the test.
 * This does not certify unmasked traps or nonstandard fenv implementations.
 */
#define DEFINE_FENV_MATRIX(NAME, TYPE, SUFFIX, P)                                          \
    static void test_##NAME##_fenv_matrix(void** state)                                  \
    {                                                                                     \
        (void)state;                                                                      \
        fenv_t original;                                                                  \
        assert_int_equal(fegetenv(&original), 0);                                         \
        const int modes[] = {FE_TONEAREST, FE_DOWNWARD, FE_UPWARD, FE_TOWARDZERO};         \
        const int flag_masks[] = {                                                       \
            0, FE_INVALID, FE_DIVBYZERO, FE_OVERFLOW, FE_UNDERFLOW, FE_INEXACT,            \
            FE_INVALID | FE_INEXACT, FE_OVERFLOW | FE_UNDERFLOW | FE_INEXACT,              \
            FE_INVALID | FE_DIVBYZERO | FE_OVERFLOW | FE_UNDERFLOW | FE_INEXACT            \
        };                                                                                \
        for (size_t m = 0; m < sizeof(modes) / sizeof(modes[0]); ++m)                    \
        {                                                                                 \
            for (size_t f = 0; f < sizeof(flag_masks) / sizeof(flag_masks[0]); ++f)       \
            {                                                                             \
                assert_int_equal(fesetround(modes[m]), 0);                               \
                assert_int_equal(feclearexcept(FE_ALL_EXCEPT), 0);                       \
                if (flag_masks[f] != 0)                                                   \
                    assert_int_equal(feraiseexcept(flag_masks[f]), 0);                   \
                const int before_flags = fetestexcept(FE_ALL_EXCEPT);                    \
                const int before_mode = fegetround();                                    \
                bool pass = true;                                                        \
                const TYPE two_to_p = scalbn##SUFFIX((TYPE)1, (P));                      \
                /* Reset flags after constructing this known exact input. */             \
                assert_int_equal(feclearexcept(FE_ALL_EXCEPT), 0);                       \
                if (flag_masks[f] != 0)                                                   \
                    assert_int_equal(feraiseexcept(flag_masks[f]), 0);                   \
                const int flags_before_call = fetestexcept(FE_ALL_EXCEPT);               \
                const bbtc_vv_dyadic_status_e success =                                 \
                    bbtc_vv_dyadic_compare_##NAME(                                       \
                        BBTC_VV_DYADIC_REFERENCE_RELATIVE,                              \
                        two_to_p, (TYPE)-1, (TYPE)1, &pass);                            \
                const int success_mode = fegetround();                                  \
                const int success_flags = fetestexcept(FE_ALL_EXCEPT);                   \
                assert_int_equal(success, BBTC_VV_DYADIC_SUCCESS);                      \
                assert_false(pass);                                                       \
                assert_int_equal(success_mode, before_mode);                            \
                assert_int_equal(success_flags, flags_before_call);                     \
                pass = true;                                                              \
                const bbtc_vv_dyadic_status_e bad_input =                               \
                    bbtc_vv_dyadic_compare_##NAME(                                       \
                        BBTC_VV_DYADIC_ABSOLUTE,                                         \
                        (TYPE)NAN, (TYPE)INFINITY, (TYPE)0, &pass);                       \
                const int bad_mode = fegetround();                                      \
                const int bad_flags = fetestexcept(FE_ALL_EXCEPT);                       \
                assert_int_equal(bad_input, BBTC_VV_DYADIC_NAN_INPUT);                   \
                assert_false(pass);                                                       \
                assert_int_equal(bad_mode, before_mode);                                \
                assert_int_equal(bad_flags, flags_before_call);                         \
                const bbtc_vv_dyadic_status_e bad_method =                              \
                    bbtc_vv_dyadic_compare_##NAME(                                       \
                        (bbtc_vv_dyadic_method_e)0,                                     \
                        (TYPE)1, (TYPE)1, (TYPE)0, &pass);                              \
                const int method_mode = fegetround();                                   \
                const int method_flags = fetestexcept(FE_ALL_EXCEPT);                    \
                assert_int_equal(bad_method, BBTC_VV_DYADIC_INVALID_ARGUMENT);          \
                assert_int_equal(method_mode, before_mode);                             \
                assert_int_equal(method_flags, flags_before_call);                      \
                assert_int_equal(fetestexcept(FE_ALL_EXCEPT), before_flags);             \
            }                                                                             \
        }                                                                                 \
        assert_int_equal(fesetenv(&original), 0);                                        \
    }

DEFINE_FENV_MATRIX(float, float, f, FLT_MANT_DIG)
DEFINE_FENV_MATRIX(double, double, , DBL_MANT_DIG)
DEFINE_FENV_MATRIX(long_double, long double, l, LDBL_MANT_DIG)
#undef DEFINE_FENV_MATRIX

/** @brief Register tests with CTest through cmocka. */
int main(void)
{
    const struct CMUnitTest tests[] =
    {
        cmocka_unit_test(test_float_mathematical),
        cmocka_unit_test(test_double_mathematical),
        cmocka_unit_test(test_long_double_mathematical),
        cmocka_unit_test(test_float_extremes),
        cmocka_unit_test(test_double_extremes),
        cmocka_unit_test(test_long_double_extremes),
        cmocka_unit_test(test_failure_precedence),
        cmocka_unit_test(test_preserve_fenv),
        cmocka_unit_test(test_float_fenv_matrix),
        cmocka_unit_test(test_double_fenv_matrix),
        cmocka_unit_test(test_long_double_fenv_matrix)
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
