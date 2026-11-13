/**
 * @file test_virial_exact_neighbors.c
 * @brief VV0.1-B2b.4a Stage 2: native one-ULP adversaries for B1 EXACT.
 *
 * @details
 * The accepted B1 EXACT policy compares the numeric values of two finite,
 * admissible native C scalars; it does not compare their object bytes. For each
 * of the 36 fixed mathematical references M-01..M-04, this suite checks:
 *
 *   1. Comparing a reference with itself returns operation SUCCESS and PASS.
 *   2. Comparing with nextafter(reference, -infinity) returns SUCCESS and FAIL.
 *   3. Comparing with nextafter(reference, +infinity) returns SUCCESS and FAIL.
 *
 * The two negative observations are the immediately adjacent representable
 * values in the SAME native scalar family. Reversing nextafter must return a
 * numerically equal value, giving a basic self-check of the adjacency probe.
 * The reference/quantity coordinates are fixed in the accepted B2a contract;
 * the literals here are kept independent of production evaluator output.
 *
 * All zeros require special attention: the immediate nonzero neighbors of
 * zero are SUBNORMAL. The test rejects unsupported hardware/fenv state and
 * verifies that the neighbor values remain subnormal, rather than treating
 * flushed-to-zero results as a valid EXACT PASS. Separately, +0 and -0 must
 * compare numerically equal in either argument order, but neither is an
 * interchangeable *bitwise* representation of the other.
 *
 * The native comparison result's axes remain distinct: successful execution
 * with mathematical FAIL is the expected negative-control outcome; an
 * unsupported environment, skipped comparison, or comparator error FAILS the
 * cmocka test. Every EXACT comparison must report NOT_COMPUTED for the
 * diagnostic metric, with a canonical positive-zero metric value.
 *
 * This is a DEVELOPER-ONLY comparator challenge, not another set of 36
 * production observations or a certification of compiler/fenv correctness.
 * It adds no dependency to bbtc::bbtc and provides no physical-model or
 * firearm safety assurance.
 *
 * @see docs/validation/VV0_1_B2B4A_EXACT_SEMANTICS_AND_ORACLE_PLAN.md
 * @see docs/validation/VVUQ_REFERENCE_ADAPTER_DESIGN_CONTRACT.md
 */

#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <fenv.h>
#include <math.h>

#include <cmocka.h>

#include "comparison.h"
#include "virial_fp_profile.h"
#include "virial_fp_control.h"

#if defined(__FAST_MATH__)
    #error "B2b.4a native EXACT tests require fast-math disabled"
#endif
#if defined(__FINITE_MATH_ONLY__) && __FINITE_MATH_ONLY__ != 0
    #error "B2b.4a native EXACT tests require finite-math-only disabled"
#endif
#if defined(__clang__)
    #pragma STDC FENV_ACCESS ON
#endif

/** @brief Contractually frozen mathematical case and output counts. */
enum
{
    BBTC_VV_NEIGHBOR_CASES = 4,
    BBTC_VV_NEIGHBOR_QUANTITIES = 3
};

/**
 * @brief Expected B, dB/dT, and d2B/dT2 for the four accepted float cases.
 *
 * @details Row order is M-01, M-02, M-03, M-04. Column order is B, first
 * derivative, second derivative. These exact dyadic hex literals are fixed
 * analytical references, NOT values produced by the Clenshaw evaluator.
 */
static const float s_expected_float[BBTC_VV_NEIGHBOR_CASES][BBTC_VV_NEIGHBOR_QUANTITIES] =
{
    { 0.0f,      0.0f,       0.0f },
    { 0x1p-10f,  0x1p-19f,   0.0f },
    { -0x1p-12f, 0.0f,       0x1p-24f },
    { -0x1p-13f, -0x1p-18f,  0x1p-24f }
};

/** @brief Independently declared binary64 counterparts to the float records. */
static const double s_expected_double[BBTC_VV_NEIGHBOR_CASES][BBTC_VV_NEIGHBOR_QUANTITIES] =
{
    { 0.0,      0.0,       0.0 },
    { 0x1p-10,  0x1p-19,   0.0 },
    { -0x1p-12, 0.0,       0x1p-24 },
    { -0x1p-13, -0x1p-18,  0x1p-24 }
};

/** @brief Native long-double references; no conversion through binary64. */
static const long double s_expected_long_double[BBTC_VV_NEIGHBOR_CASES][BBTC_VV_NEIGHBOR_QUANTITIES] =
{
    { 0.0L,      0.0L,       0.0L },
    { 0x1p-10L,  0x1p-19L,   0.0L },
    { -0x1p-12L, 0.0L,       0x1p-24L },
    { -0x1p-13L, -0x1p-18L,  0x1p-24L }
};

/**
 * @brief Require the existing, restrictive profile and hardware-control screens.
 *
 * @details Fail closed on unsupported formats, mismatched producer metadata,
 * non-nearest C rounding, wrong x87/MXCSR controls, unmasked exceptions,
 * DAZ, or FTZ. Calling this function does NOT grant scientific qualification;
 * SCREENED_UNQUALIFIED is intentionally the strongest screening outcome.
 * The screens run before creation of subnormals and again after fenv restore.
 */
static void assert_supported_native_environment(void)
{
    bbtc_vv_virial_profile_observation_t profile = {0};
    bbtc_vv_virial_profile_result_t profile_result = {0};
    bbtc_vv_virial_control_observation_t controls = {0};
    bbtc_vv_virial_control_result_t control_result = {0};

    assert_int_equal(
        bbtc_vv_virial_profile_screen(&profile, &profile_result),
        BBTC_VV_VIRIAL_PROFILE_SCREENED_UNQUALIFIED);
    assert_int_equal(profile_result.issue_mask, BBTC_VV_VIRIAL_ISSUE_NONE);

    assert_int_equal(
        bbtc_vv_virial_control_screen(&controls, &control_result),
        BBTC_VV_VIRIAL_CONTROL_SCREENED_UNQUALIFIED);
    assert_int_equal(control_result.issue_mask, BBTC_VV_VIRIAL_CONTROL_ISSUE_NONE);
}

/**
 * @brief Generate fully native comparator assertions and one test per family.
 *
 * @details Keeping the comparison helpers precision-specific ensures that
 * `nextafterf` and `bbtc_vv_compare_float` do not pass through double, while
 * `nextafterl` and `bbtc_vv_compare_long_double` retain long-double precision.
 * The caller-supplied fixture is an array of exact mathematical literals.
 *
 * The test snapshots the floating-point environment ONLY AFTER a successful
 * live-control preflight. `feholdexcept` then masks traps and clears sticky
 * flags within the test, allowing zero-neighbor underflow without contaminating
 * another test. `fesetenv` restores the full original state on successful
 * completion. An assertion failure aborts this dedicated CTest process.
 *
 * The reversible nextafter check does not independently certify libm's
 * implementation, but guards against gross probe mistakes. A zero neighbor
 * must have FP_SUBNORMAL classification, which additionally detects FTZ/DAZ
 * interference on supported x86 profiles.
 *
 * @param NAME Native suffix: float, double, or long_double.
 * @param TYPE Native C scalar type.
 * @param NEXTAFTER Native nextafterf, nextafter, or nextafterl function.
 * @param FIXTURES Fixed analytical array for this native scalar family.
 */
#define BBTC_VV_DEFINE_NEIGHBOR_TEST(NAME, TYPE, NEXTAFTER, FIXTURES)                 \
    static void assert_##NAME##_exact(TYPE expected, TYPE observed,                 \
                                       bbtc_vv_finding_e desired)                    \
    {                                                                                \
        const bbtc_vv_policy_##NAME##_t policy =                                    \
        {                                                                            \
            .method = BBTC_VV_COMPARISON_EXACT,                                      \
            .tolerance = (TYPE)0                                                     \
        };                                                                           \
        bbtc_vv_result_##NAME##_t result = {0};                                     \
        assert_int_equal(bbtc_vv_compare_##NAME(                                     \
            expected, observed, &policy, &result), BBTC_VV_STATUS_SUCCESS);          \
        assert_int_equal(result.finding, desired);                                  \
        assert_int_equal(result.metric_state, BBTC_VV_METRIC_NOT_COMPUTED);          \
        assert_true(result.metric == (TYPE)0);                                      \
        assert_false(signbit(result.metric));                                       \
    }                                                                                \
                                                                                     \
    /** @brief Challenge all twelve fixed analytical values in this family. */      \
    static void test_##NAME##_exact_neighbors(void** state)                         \
    {                                                                                \
        (void)state;                                                                 \
        assert_supported_native_environment();                                      \
        fenv_t original = {0};                                                       \
        assert_int_equal(feholdexcept(&original), 0);                                \
        size_t equality_checks = 0u;                                                \
        size_t adjacent_fail_checks = 0u;                                           \
                                                                                     \
        for (size_t i = 0u; i < BBTC_VV_NEIGHBOR_CASES; ++i)                         \
        {                                                                            \
            for (size_t q = 0u; q < BBTC_VV_NEIGHBOR_QUANTITIES; ++q)                \
            {                                                                        \
                /* Volatile forces a native runtime operand for nextafter. */        \
                volatile TYPE reference = (FIXTURES)[i][q];                         \
                const TYPE below = NEXTAFTER(reference, -(TYPE)INFINITY);            \
                const TYPE above = NEXTAFTER(reference, (TYPE)INFINITY);             \
                                                                                     \
                assert_true(isfinite(below) && isfinite(above));                     \
                assert_true(below < reference && above > reference);                 \
                assert_true(NEXTAFTER(below, (TYPE)INFINITY) == reference);           \
                assert_true(NEXTAFTER(above, -(TYPE)INFINITY) == reference);          \
                                                                                     \
                if (reference == (TYPE)0)                                            \
                {                                                                    \
                    assert_int_equal(fpclassify(below), FP_SUBNORMAL);               \
                    assert_int_equal(fpclassify(above), FP_SUBNORMAL);               \
                }                                                                    \
                                                                                     \
                assert_##NAME##_exact(reference, reference, BBTC_VV_FINDING_PASS);   \
                ++equality_checks;                                                  \
                assert_##NAME##_exact(reference, below, BBTC_VV_FINDING_FAIL);       \
                assert_##NAME##_exact(reference, above, BBTC_VV_FINDING_FAIL);       \
                adjacent_fail_checks += 2u;                                         \
            }                                                                        \
        }                                                                            \
                                                                                     \
        assert_int_equal(equality_checks,                                            \
                         BBTC_VV_NEIGHBOR_CASES * BBTC_VV_NEIGHBOR_QUANTITIES);      \
        assert_int_equal(adjacent_fail_checks, 2u * equality_checks);               \
                                                                                     \
        /* C numerical equality treats differently signed zeros as equal. */        \
        assert_##NAME##_exact((TYPE)0, -(TYPE)0, BBTC_VV_FINDING_PASS);              \
        assert_##NAME##_exact(-(TYPE)0, (TYPE)0, BBTC_VV_FINDING_PASS);              \
                                                                                     \
        /* Restore sticky flags, rounding and exception-control state. */           \
        assert_int_equal(fesetenv(&original), 0);                                    \
        assert_supported_native_environment();                                      \
    }

BBTC_VV_DEFINE_NEIGHBOR_TEST(float, float, nextafterf, s_expected_float)
BBTC_VV_DEFINE_NEIGHBOR_TEST(double, double, nextafter, s_expected_double)
BBTC_VV_DEFINE_NEIGHBOR_TEST(long_double, long double, nextafterl, s_expected_long_double)

#undef BBTC_VV_DEFINE_NEIGHBOR_TEST

/**
 * @brief Execute the three native-family adversarial tests as a single CTest.
 * @return Zero only when all 36 equalities, 72 neighbors and six signed-zero
 *         checks have completed with their separately expected findings.
 */
int main(void)
{
    const struct CMUnitTest tests[] =
    {
        cmocka_unit_test(test_float_exact_neighbors),
        cmocka_unit_test(test_double_exact_neighbors),
        cmocka_unit_test(test_long_double_exact_neighbors)
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}
