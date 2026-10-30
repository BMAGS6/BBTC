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


/**
 * @brief Verify every numerical representation invariant after a successful call.
 *
 * @details
 * The metric's limbs are in little-endian *word order*, not necessarily host
 * byte order. A successful zero has used=0 and numerator_exponent=0, whereas a
 * nonzero numerator has a nonzero highest used limb. Every unused limb MUST be
 * zero, even after a previous invocation populated the same result object.
 * This checks field values only; C struct padding is deliberately ignored.
 *
 * @param metric Exact metric returned with BBTC_VV_STATUS_SUCCESS.
 */
static void assert_metric_canonical(const bbtc_vv_exact_metric_t* metric)
{
    assert_true(metric->used <= BBTC_VV_METRIC_LIMBS);
    assert_true(metric->denominator != 0);
    if (metric->used == 0)
        assert_int_equal(metric->numerator_exponent, 0);
    else
        assert_true(metric->word[metric->used - 1] != 0);

    for (size_t index = metric->used; index < BBTC_VV_METRIC_LIMBS; ++index)
        assert_int_equal(metric->word[index], UINT64_C(0));
}

/**
 * @brief Check the exact 520-limb pattern representing 2^end - 2^start.
 *
 * @details
 * For integer exponents end > start >= 0, binary subtraction implies that
 * EVERY bit position in [start,end) is one and all other bits are zero.
 * Walking the entire allocated buffer checks distant borrows and unused-tail
 * clearing without relying on this implementation's carry/borrow routines.
 * No native floating-point subtraction participates in the expected answer.
 *
 * @param metric Successful absolute metric with numerator exponent factored out.
 * @param start Lowest set bit of the exact integer numerator.
 * @param end One past its highest set bit.
 */
static void assert_ones_interval(
    const bbtc_vv_exact_metric_t* metric, size_t start, size_t end
)
{
    assert_true(start < end);
    assert_true(end <= BBTC_VV_METRIC_LIMBS * 64U);
    assert_int_equal(metric->used, (end + 63U) / 64U);

    for (size_t word = 0; word < BBTC_VV_METRIC_LIMBS; ++word)
    {
        uint64_t expected = UINT64_C(0);
        for (unsigned bit = 0; bit < 64U; ++bit)
        {
            const size_t position = word * 64U + (size_t)bit;
            if (position >= start && position < end)
                expected |= UINT64_C(1) << bit;
        }
        assert_int_equal(metric->word[word], expected);
    }
}

/**
 * @brief Produce three native-family proof-bound regression tests per type.
 *
 * @details
 * The expected numerator word patterns follow independently from two exact
 * identities: 1-epsilon = 2^0-2^q, and M-(-M)=2M. The relative tests verify
 * that a negative reference is used by MAGNITUDE, without narrowing the
 * denominator or silently dropping the smallest subnormal.
 *
 * P is native significand precision. MIN_EXP / MAX_EXP are C's native
 * floating exponent bounds, not values inferred from sizeof(TYPE).
 */
#define DEFINE_METRIC_AUDIT_TESTS(NAME, TYPE, P, MIN_EXP, MAX_EXP, TRUE_MIN, MAX_FINITE) \
    static void test_##NAME##_wide_borrow(void** state)                         \
    {                                                                           \
        (void)state;                                                            \
        bbtc_vv_exact_metric_t metric = {0};                                    \
        /* q=MIN_EXP-P is the smallest subnormal's binary exponent.        */  \
        /* Its normalized atom has mantissa 2^(P-1), exponent q-(P-1).      */  \
        const int atom_exp = (MIN_EXP) - 2 * (P) + 1;                           \
        const size_t lowest_bit = (size_t)((P) - 1);                            \
        const size_t past_highest = (size_t)(-atom_exp);                        \
        assert_int_equal(bbtc_vv_metric_exact_##NAME(                           \
            BBTC_VV_COMPARISON_ABSOLUTE, (TYPE)1, (TRUE_MIN), &metric),          \
            BBTC_VV_STATUS_SUCCESS);                                           \
        assert_int_equal(metric.numerator_exponent, atom_exp);                  \
        assert_int_equal(metric.denominator, 1);                               \
        assert_int_equal(metric.denominator_exponent, 0);                       \
        assert_ones_interval(&metric, lowest_bit, past_highest);                \
        assert_metric_canonical(&metric);                                      \
    }                                                                           \
    static void test_##NAME##_opposite_maxima(void** state)                     \
    {                                                                           \
        (void)state;                                                            \
        bbtc_vv_exact_metric_t metric = {0};                                    \
        /* M=(2^P-1)*2^(MAX_EXP-P), hence 2M has a p+1-bit coefficient. */   \
        /* For P=64 the carry crosses the boundary between limb 0 and 1. */   \
        const uint64_t max_sig = UINT64_MAX >> (64U - (unsigned)(P));           \
        const uint64_t expected_low = max_sig << 1U;                           \
        const uint64_t expected_high = max_sig >> 63U;                         \
        assert_int_equal(bbtc_vv_metric_exact_##NAME(                           \
            BBTC_VV_COMPARISON_ABSOLUTE, (MAX_FINITE), -(MAX_FINITE),           \
            &metric), BBTC_VV_STATUS_SUCCESS);                                 \
        assert_int_equal(metric.numerator_exponent, (MAX_EXP) - (P));           \
        assert_int_equal(metric.used, expected_high == 0 ? 1 : 2);             \
        assert_int_equal(metric.word[0], expected_low);                        \
        assert_int_equal(metric.word[1], expected_high);                       \
        assert_int_equal(metric.denominator, 1);                               \
        assert_int_equal(metric.denominator_exponent, 0);                       \
        assert_metric_canonical(&metric);                                      \
    }                                                                           \
    static void test_##NAME##_relative_atom(void** state)                       \
    {                                                                           \
        (void)state;                                                            \
        bbtc_vv_exact_metric_t metric = {0};                                    \
        /* |-epsilon-0|/|-epsilon|=1, with no rounded native division. */    \
        const int tiny_exp = (MIN_EXP) - 2 * (P) + 1;                           \
        const uint64_t tiny_sig = UINT64_C(1) << ((P) - 1);                    \
        const uint64_t max_sig = UINT64_MAX >> (64U - (unsigned)(P));           \
        assert_int_equal(bbtc_vv_metric_exact_##NAME(                           \
            BBTC_VV_COMPARISON_REFERENCE_RELATIVE, -(TRUE_MIN), (TYPE)0,        \
            &metric), BBTC_VV_STATUS_SUCCESS);                                 \
        assert_int_equal(metric.used, 1);                                      \
        assert_int_equal(metric.word[0], tiny_sig);                            \
        assert_int_equal(metric.numerator_exponent, tiny_exp);                  \
        assert_int_equal(metric.denominator, tiny_sig);                         \
        assert_int_equal(metric.denominator_exponent, tiny_exp);                \
        assert_metric_canonical(&metric);                                      \
        /* A zero rational keeps the nonzero reference denominator. */       \
        assert_int_equal(bbtc_vv_metric_exact_##NAME(                           \
            BBTC_VV_COMPARISON_REFERENCE_RELATIVE, -(MAX_FINITE),              \
            -(MAX_FINITE), &metric), BBTC_VV_STATUS_SUCCESS);                   \
        assert_int_equal(metric.used, 0);                                      \
        assert_int_equal(metric.numerator_exponent, 0);                         \
        assert_int_equal(metric.denominator, max_sig);                          \
        assert_int_equal(metric.denominator_exponent, (MAX_EXP) - (P));          \
        assert_metric_canonical(&metric);                                      \
    }

DEFINE_METRIC_AUDIT_TESTS(float, float, FLT_MANT_DIG, FLT_MIN_EXP, FLT_MAX_EXP,
    FLT_TRUE_MIN, FLT_MAX)
DEFINE_METRIC_AUDIT_TESTS(double, double, DBL_MANT_DIG, DBL_MIN_EXP, DBL_MAX_EXP,
    DBL_TRUE_MIN, DBL_MAX)
DEFINE_METRIC_AUDIT_TESTS(long_double, long double, LDBL_MANT_DIG,
    LDBL_MIN_EXP, LDBL_MAX_EXP, LDBL_TRUE_MIN, LDBL_MAX)

#undef DEFINE_METRIC_AUDIT_TESTS

/**
 * @brief Ensure errors erase an earlier successful multi-limb numerator.
 *
 * @details
 * Source contracts describe canonical FIELD clearing, not raw struct bytes.
 * This test first populates a long-double result spanning multiple limbs,
 * then makes a structurally invalid call using the same object and inspects
 * all 520 numerator words and every scalar field. In particular, stale
 * previously computed exact rational data must never escape on failure.
 */
static void test_metric_failure_clears_reused_output(void** state)
{
    (void)state;
    bbtc_vv_exact_metric_t metric = {0};
    assert_int_equal(bbtc_vv_metric_exact_long_double(
        BBTC_VV_COMPARISON_ABSOLUTE, (long double)1, LDBL_TRUE_MIN, &metric),
        BBTC_VV_STATUS_SUCCESS);
    assert_true(metric.used > 1);

    assert_int_equal(bbtc_vv_metric_exact_long_double(
        BBTC_VV_COMPARISON_EXACT, (long double)1, (long double)1, &metric),
        BBTC_VV_STATUS_INVALID_ARGUMENT);
    assert_int_equal(metric.used, 0);
    assert_int_equal(metric.numerator_exponent, 0);
    assert_int_equal(metric.denominator, 0);
    assert_int_equal(metric.denominator_exponent, 0);
    for (size_t index = 0; index < BBTC_VV_METRIC_LIMBS; ++index)
        assert_int_equal(metric.word[index], UINT64_C(0));
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
        cmocka_unit_test(test_float_wide_borrow),
        cmocka_unit_test(test_double_wide_borrow),
        cmocka_unit_test(test_long_double_wide_borrow),
        cmocka_unit_test(test_float_opposite_maxima),
        cmocka_unit_test(test_double_opposite_maxima),
        cmocka_unit_test(test_long_double_opposite_maxima),
        cmocka_unit_test(test_float_relative_atom),
        cmocka_unit_test(test_double_relative_atom),
        cmocka_unit_test(test_long_double_relative_atom),
        cmocka_unit_test(test_metric_failure_clears_reused_output),
        cmocka_unit_test(test_repr_capacity)
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
