/**
 * @file test_virial_adapter_contract.c
 * @brief Native-family error-path and input-contract tests for VV0.1-B2b.2b.
 *
 * @details
 * Exercises the established public temperature-law status precedence through
 * the developer-only virial observation adapter. These are API contract tests,
 * NOT independent numerical references, physical validation, or the planned
 * 36-comparison mathematical runner. Every intentionally nonnull coefficient
 * pointer addresses readable caller-owned storage for the declared count.
 *
 * The test suite distinguishes:
 * - an absent required output (which precedes any law validation),
 * - law validation (which precedes evaluation-temperature validation),
 * - NaN precedence over infinity within law validation,
 * - a failed observation from a successfully evaluated numeric result, and
 * - the closed, inclusive temperature interval.
 *
 * All assertions inspect the documented output fields rather than padding
 * bytes. The optional validity flag is independent of the BBTC raw status.
 *
 * @warning Contract compliance does not establish scientific model accuracy,
 *          ammunition behavior, firearm safety, or physical validation.
 */
#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <stdbool.h>
#include <math.h>

#include <cmocka.h>

#include "virial_adapter.h"

/**
 * @brief Check that failure clearing reaches each documented float field.
 * @param terms Nonnull caller-owned terms storage to inspect.
 */
static void assert_cleared_float(
    const bbtc_ib_first_order_virial_temperature_terms_float_t* terms
)
{
    assert_true(terms->second_density_virial_coefficient_m3_per_kg == 0.0f);
    assert_true(terms->first_temperature_derivative_m3_per_kg_k == 0.0f);
    assert_true(terms->second_temperature_derivative_m3_per_kg_k2 == 0.0f);
}

/**
 * @brief Verify exact production status, invalidity, and full output clearing.
 *
 * @details
 * Initializes all three result fields with nonzero sentinels on each call.
 * Checks field values, not whole-structure bytes or padding. The caller must
 * supply valid storage for any nonnull borrowed coefficient array in law.
 *
 * @param law Possibly null law, for precedence tests.
 * @param temperature_k Evaluation temperature, possibly nonfinite.
 * @param expected_status Contractually expected raw BBTC status.
 */
static void expect_failure_float(
    const bbtc_ib_first_order_virial_temperature_law_float_t* law,
    float temperature_k,
    bbtc_status_e expected_status
)
{
    bbtc_ib_first_order_virial_temperature_terms_float_t terms =
    {
        .second_density_virial_coefficient_m3_per_kg = 7.0f,
        .first_temperature_derivative_m3_per_kg_k = 8.0f,
        .second_temperature_derivative_m3_per_kg_k2 = 9.0f
    };
    bool valid = true;

    const bbtc_status_e status = bbtc_vv_virial_observe_float(law,
                                                              temperature_k,
                                                              &terms,
                                                              &valid);
    assert_int_equal(status, expected_status);
    assert_false(valid);
    assert_cleared_float(&terms);
}

/**
 * @brief Check required-pointer precedence through the float adapter.
 * @param state Unused cmocka test state.
 */
static void test_float_required_pointers(void** state)
{
    (void)state;

    const float coefficients[] = {0x1p-10f, 0x1p-12f};
    const bbtc_ib_first_order_virial_temperature_law_float_t law =
    {
        .minimum_temperature_k = 256.0f,
        .maximum_temperature_k = 512.0f,
        .second_density_virial_chebyshev_coefficients_m3_per_kg = coefficients,
        .coefficient_count = 2
    };
    bool valid = true;

    /* The output-pointer check precedes both law and temperature validation. */
    assert_int_equal(bbtc_vv_virial_observe_float(NULL, NAN, NULL, &valid),
                     BBTC_STATUS_INVALID_ARGUMENT);
    assert_false(valid);

    /* With output storage available, the null law takes precedence over NaN. */
    expect_failure_float(NULL, NAN, BBTC_STATUS_INVALID_ARGUMENT);

    /* The optional validity pointer is never a required output. */
    assert_int_equal(bbtc_vv_virial_observe_float(&law, 384.0f, NULL, NULL),
                     BBTC_STATUS_INVALID_ARGUMENT);

    valid = true;
    assert_int_equal(bbtc_vv_virial_observe_float(&law, 384.0f, NULL, &valid),
                     BBTC_STATUS_INVALID_ARGUMENT);
    assert_false(valid);
}

/**
 * @brief Verify pointer, count, and interval rejection for float laws.
 * @param state Unused cmocka test state.
 */
static void test_float_law_domain(void** state)
{
    (void)state;

    const float coefficients[] = {0x1p-10f, 0x1p-12f};
    bbtc_ib_first_order_virial_temperature_law_float_t law =
    {
        .minimum_temperature_k = 256.0f,
        .maximum_temperature_k = 512.0f,
        .second_density_virial_chebyshev_coefficients_m3_per_kg = coefficients,
        .coefficient_count = 2
    };

    law.second_density_virial_chebyshev_coefficients_m3_per_kg = NULL;
    expect_failure_float(&law, NAN, BBTC_STATUS_INVALID_ARGUMENT);
    law.second_density_virial_chebyshev_coefficients_m3_per_kg = coefficients;

    law.coefficient_count = 0;
    expect_failure_float(&law, 384.0f, BBTC_STATUS_OUTSIDE_DOMAIN);
    law.coefficient_count = 2;

    law.minimum_temperature_k = 0.0f;
    expect_failure_float(&law, 384.0f, BBTC_STATUS_OUTSIDE_DOMAIN);
    law.minimum_temperature_k = 512.0f;
    expect_failure_float(&law, 384.0f, BBTC_STATUS_OUTSIDE_DOMAIN);
    law.minimum_temperature_k = 256.0f;

    law.maximum_temperature_k = 256.0f;
    expect_failure_float(&law, 384.0f, BBTC_STATUS_OUTSIDE_DOMAIN);
    law.maximum_temperature_k = 512.0f;

    assert_true(coefficients[0] == 0x1p-10f);
    assert_true(coefficients[1] == 0x1p-12f);
}

/**
 * @brief Check NaN/infinity precedence within native float law validation.
 *
 * @details
 * Law validation scans all declared readable coefficients, retains an
 * infinity indicator, and returns NaN if any coefficient is NaN; it thus
 * recognizes NaN even when infinity occurs earlier in the array. This is
 * not a blanket precedence guarantee across all BBTC public functions.
 *
 * @param state Unused cmocka test state.
 */
static void test_float_nonfinite_law_precedence(void** state)
{
    (void)state;

    const float coefficients[] = {0x1p-10f, 0x1p-12f};
    const float infinity_then_nan[] = {INFINITY, NAN};
    const float nan_then_infinity[] = {NAN, INFINITY};
    const float infinity_only[] = {0x1p-10f, INFINITY};
    bbtc_ib_first_order_virial_temperature_law_float_t law =
    {
        .minimum_temperature_k = 256.0f,
        .maximum_temperature_k = 512.0f,
        .second_density_virial_chebyshev_coefficients_m3_per_kg = coefficients,
        .coefficient_count = 2
    };

    law.second_density_virial_chebyshev_coefficients_m3_per_kg = infinity_then_nan;
    expect_failure_float(&law, 384.0f, BBTC_STATUS_NAN_INPUT);
    law.second_density_virial_chebyshev_coefficients_m3_per_kg = nan_then_infinity;
    expect_failure_float(&law, 384.0f, BBTC_STATUS_NAN_INPUT);
    law.second_density_virial_chebyshev_coefficients_m3_per_kg = infinity_only;
    expect_failure_float(&law, 384.0f, BBTC_STATUS_NONFINITE_INPUT);
    law.second_density_virial_chebyshev_coefficients_m3_per_kg = coefficients;

    law.minimum_temperature_k = NAN;
    expect_failure_float(&law, 384.0f, BBTC_STATUS_NAN_INPUT);
    law.minimum_temperature_k = 256.0f;
    law.maximum_temperature_k = NAN;
    expect_failure_float(&law, 384.0f, BBTC_STATUS_NAN_INPUT);
    law.maximum_temperature_k = 512.0f;

    law.minimum_temperature_k = INFINITY;
    law.second_density_virial_chebyshev_coefficients_m3_per_kg = nan_then_infinity;
    expect_failure_float(&law, 384.0f, BBTC_STATUS_NAN_INPUT);
    law.second_density_virial_chebyshev_coefficients_m3_per_kg = coefficients;
    expect_failure_float(&law, 384.0f, BBTC_STATUS_NONFINITE_INPUT);
    law.minimum_temperature_k = 256.0f;

    law.maximum_temperature_k = -INFINITY;
    expect_failure_float(&law, 384.0f, BBTC_STATUS_NONFINITE_INPUT);
}

/**
 * @brief Verify float evaluation-temperature classification and cleared reuse.
 * @param state Unused cmocka test state.
 */
static void test_float_temperature_and_output_reuse(void** state)
{
    (void)state;

    const float coefficients[] = {0x1p-10f, 0x1p-12f};
    bbtc_ib_first_order_virial_temperature_law_float_t law =
    {
        .minimum_temperature_k = 256.0f,
        .maximum_temperature_k = 512.0f,
        .second_density_virial_chebyshev_coefficients_m3_per_kg = coefficients,
        .coefficient_count = 2
    };
    bbtc_ib_first_order_virial_temperature_terms_float_t terms = {0};
    bool valid = false;

    expect_failure_float(&law, NAN, BBTC_STATUS_NAN_INPUT);
    expect_failure_float(&law, INFINITY, BBTC_STATUS_NONFINITE_INPUT);
    expect_failure_float(&law, -INFINITY, BBTC_STATUS_NONFINITE_INPUT);
    expect_failure_float(&law, 255.0f, BBTC_STATUS_OUTSIDE_DOMAIN);
    expect_failure_float(&law, 513.0f, BBTC_STATUS_OUTSIDE_DOMAIN);

    /* An invalid law must be reported before the NaN evaluation temperature. */
    law.coefficient_count = 0;
    expect_failure_float(&law, NAN, BBTC_STATUS_OUTSIDE_DOMAIN);
    law.coefficient_count = 2;

    /* Reusing previously successful storage must never preserve stale values. */
    assert_int_equal(bbtc_vv_virial_observe_float(&law, 384.0f, &terms, &valid),
                     BBTC_STATUS_SUCCESS);
    assert_true(valid);
    assert_true(terms.second_density_virial_coefficient_m3_per_kg != 0.0f);

    assert_int_equal(bbtc_vv_virial_observe_float(&law, NAN, &terms, &valid),
                     BBTC_STATUS_NAN_INPUT);
    assert_false(valid);
    assert_cleared_float(&terms);
}

/**
 * @brief Verify inclusive float temperature endpoints and borrowed ownership.
 *
 * @details
 * This test checks only execution success and finite output fields at the
 * documented endpoints. It does not compare them with independently derived
 * mathematical references; that remains a separate B2b runner obligation.
 *
 * @param state Unused cmocka test state.
 */
static void test_float_closed_interval_and_ownership(void** state)
{
    (void)state;

    const float coefficients[] = {0x1p-10f, 0x1p-12f};
    const bbtc_ib_first_order_virial_temperature_law_float_t law =
    {
        .minimum_temperature_k = 256.0f,
        .maximum_temperature_k = 512.0f,
        .second_density_virial_chebyshev_coefficients_m3_per_kg = coefficients,
        .coefficient_count = 2
    };
    bbtc_ib_first_order_virial_temperature_terms_float_t terms = {0};
    bool valid = false;

    assert_int_equal(bbtc_vv_virial_observe_float(&law, 256.0f, &terms, &valid),
                     BBTC_STATUS_SUCCESS);
    assert_true(valid);
    assert_true(isfinite(terms.second_density_virial_coefficient_m3_per_kg));
    assert_true(isfinite(terms.first_temperature_derivative_m3_per_kg_k));
    assert_true(isfinite(terms.second_temperature_derivative_m3_per_kg_k2));

    valid = false;
    assert_int_equal(bbtc_vv_virial_observe_float(&law, 512.0f, &terms, &valid),
                     BBTC_STATUS_SUCCESS);
    assert_true(valid);
    assert_true(isfinite(terms.second_density_virial_coefficient_m3_per_kg));
    assert_true(isfinite(terms.first_temperature_derivative_m3_per_kg_k));
    assert_true(isfinite(terms.second_temperature_derivative_m3_per_kg_k2));

    assert_true(coefficients[0] == 0x1p-10f);
    assert_true(coefficients[1] == 0x1p-12f);
}

/**
 * @brief Check that failure clearing reaches each documented double field.
 * @param terms Nonnull caller-owned terms storage to inspect.
 */
static void assert_cleared_double(
    const bbtc_ib_first_order_virial_temperature_terms_double_t* terms
)
{
    assert_true(terms->second_density_virial_coefficient_m3_per_kg == 0.0);
    assert_true(terms->first_temperature_derivative_m3_per_kg_k == 0.0);
    assert_true(terms->second_temperature_derivative_m3_per_kg_k2 == 0.0);
}

/**
 * @brief Verify exact production status, invalidity, and full output clearing.
 *
 * @details
 * Initializes all three result fields with nonzero sentinels on each call.
 * Checks field values, not whole-structure bytes or padding. The caller must
 * supply valid storage for any nonnull borrowed coefficient array in law.
 *
 * @param law Possibly null law, for precedence tests.
 * @param temperature_k Evaluation temperature, possibly nonfinite.
 * @param expected_status Contractually expected raw BBTC status.
 */
static void expect_failure_double(
    const bbtc_ib_first_order_virial_temperature_law_double_t* law,
    double temperature_k,
    bbtc_status_e expected_status
)
{
    bbtc_ib_first_order_virial_temperature_terms_double_t terms =
    {
        .second_density_virial_coefficient_m3_per_kg = 7.0,
        .first_temperature_derivative_m3_per_kg_k = 8.0,
        .second_temperature_derivative_m3_per_kg_k2 = 9.0
    };
    bool valid = true;

    const bbtc_status_e status = bbtc_vv_virial_observe_double(law,
                                                               temperature_k,
                                                               &terms,
                                                               &valid);
    assert_int_equal(status, expected_status);
    assert_false(valid);
    assert_cleared_double(&terms);
}

/**
 * @brief Check required-pointer precedence through the double adapter.
 * @param state Unused cmocka test state.
 */
static void test_double_required_pointers(void** state)
{
    (void)state;

    const double coefficients[] = {0x1p-10, 0x1p-12};
    const bbtc_ib_first_order_virial_temperature_law_double_t law =
    {
        .minimum_temperature_k = 256.0,
        .maximum_temperature_k = 512.0,
        .second_density_virial_chebyshev_coefficients_m3_per_kg = coefficients,
        .coefficient_count = 2
    };
    bool valid = true;

    /* The output-pointer check precedes both law and temperature validation. */
    assert_int_equal(bbtc_vv_virial_observe_double(NULL, NAN, NULL, &valid),
                     BBTC_STATUS_INVALID_ARGUMENT);
    assert_false(valid);

    /* With output storage available, the null law takes precedence over NaN. */
    expect_failure_double(NULL, NAN, BBTC_STATUS_INVALID_ARGUMENT);

    /* The optional validity pointer is never a required output. */
    assert_int_equal(bbtc_vv_virial_observe_double(&law, 384.0, NULL, NULL),
                     BBTC_STATUS_INVALID_ARGUMENT);

    valid = true;
    assert_int_equal(bbtc_vv_virial_observe_double(&law, 384.0, NULL, &valid),
                     BBTC_STATUS_INVALID_ARGUMENT);
    assert_false(valid);
}

/**
 * @brief Verify pointer, count, and interval rejection for double laws.
 * @param state Unused cmocka test state.
 */
static void test_double_law_domain(void** state)
{
    (void)state;

    const double coefficients[] = {0x1p-10, 0x1p-12};
    bbtc_ib_first_order_virial_temperature_law_double_t law =
    {
        .minimum_temperature_k = 256.0,
        .maximum_temperature_k = 512.0,
        .second_density_virial_chebyshev_coefficients_m3_per_kg = coefficients,
        .coefficient_count = 2
    };

    law.second_density_virial_chebyshev_coefficients_m3_per_kg = NULL;
    expect_failure_double(&law, NAN, BBTC_STATUS_INVALID_ARGUMENT);
    law.second_density_virial_chebyshev_coefficients_m3_per_kg = coefficients;

    law.coefficient_count = 0;
    expect_failure_double(&law, 384.0, BBTC_STATUS_OUTSIDE_DOMAIN);
    law.coefficient_count = 2;

    law.minimum_temperature_k = 0.0;
    expect_failure_double(&law, 384.0, BBTC_STATUS_OUTSIDE_DOMAIN);
    law.minimum_temperature_k = 512.0;
    expect_failure_double(&law, 384.0, BBTC_STATUS_OUTSIDE_DOMAIN);
    law.minimum_temperature_k = 256.0;

    law.maximum_temperature_k = 256.0;
    expect_failure_double(&law, 384.0, BBTC_STATUS_OUTSIDE_DOMAIN);
    law.maximum_temperature_k = 512.0;

    assert_true(coefficients[0] == 0x1p-10);
    assert_true(coefficients[1] == 0x1p-12);
}

/**
 * @brief Check NaN/infinity precedence within native double law validation.
 *
 * @details
 * Law validation scans all declared readable coefficients, retains an
 * infinity indicator, and returns NaN if any coefficient is NaN; it thus
 * recognizes NaN even when infinity occurs earlier in the array. This is
 * not a blanket precedence guarantee across all BBTC public functions.
 *
 * @param state Unused cmocka test state.
 */
static void test_double_nonfinite_law_precedence(void** state)
{
    (void)state;

    const double coefficients[] = {0x1p-10, 0x1p-12};
    const double infinity_then_nan[] = {INFINITY, NAN};
    const double nan_then_infinity[] = {NAN, INFINITY};
    const double infinity_only[] = {0x1p-10, INFINITY};
    bbtc_ib_first_order_virial_temperature_law_double_t law =
    {
        .minimum_temperature_k = 256.0,
        .maximum_temperature_k = 512.0,
        .second_density_virial_chebyshev_coefficients_m3_per_kg = coefficients,
        .coefficient_count = 2
    };

    law.second_density_virial_chebyshev_coefficients_m3_per_kg = infinity_then_nan;
    expect_failure_double(&law, 384.0, BBTC_STATUS_NAN_INPUT);
    law.second_density_virial_chebyshev_coefficients_m3_per_kg = nan_then_infinity;
    expect_failure_double(&law, 384.0, BBTC_STATUS_NAN_INPUT);
    law.second_density_virial_chebyshev_coefficients_m3_per_kg = infinity_only;
    expect_failure_double(&law, 384.0, BBTC_STATUS_NONFINITE_INPUT);
    law.second_density_virial_chebyshev_coefficients_m3_per_kg = coefficients;

    law.minimum_temperature_k = NAN;
    expect_failure_double(&law, 384.0, BBTC_STATUS_NAN_INPUT);
    law.minimum_temperature_k = 256.0;
    law.maximum_temperature_k = NAN;
    expect_failure_double(&law, 384.0, BBTC_STATUS_NAN_INPUT);
    law.maximum_temperature_k = 512.0;

    law.minimum_temperature_k = INFINITY;
    law.second_density_virial_chebyshev_coefficients_m3_per_kg = nan_then_infinity;
    expect_failure_double(&law, 384.0, BBTC_STATUS_NAN_INPUT);
    law.second_density_virial_chebyshev_coefficients_m3_per_kg = coefficients;
    expect_failure_double(&law, 384.0, BBTC_STATUS_NONFINITE_INPUT);
    law.minimum_temperature_k = 256.0;

    law.maximum_temperature_k = -INFINITY;
    expect_failure_double(&law, 384.0, BBTC_STATUS_NONFINITE_INPUT);
}

/**
 * @brief Verify double evaluation-temperature classification and cleared reuse.
 * @param state Unused cmocka test state.
 */
static void test_double_temperature_and_output_reuse(void** state)
{
    (void)state;

    const double coefficients[] = {0x1p-10, 0x1p-12};
    bbtc_ib_first_order_virial_temperature_law_double_t law =
    {
        .minimum_temperature_k = 256.0,
        .maximum_temperature_k = 512.0,
        .second_density_virial_chebyshev_coefficients_m3_per_kg = coefficients,
        .coefficient_count = 2
    };
    bbtc_ib_first_order_virial_temperature_terms_double_t terms = {0};
    bool valid = false;

    expect_failure_double(&law, NAN, BBTC_STATUS_NAN_INPUT);
    expect_failure_double(&law, INFINITY, BBTC_STATUS_NONFINITE_INPUT);
    expect_failure_double(&law, -INFINITY, BBTC_STATUS_NONFINITE_INPUT);
    expect_failure_double(&law, 255.0, BBTC_STATUS_OUTSIDE_DOMAIN);
    expect_failure_double(&law, 513.0, BBTC_STATUS_OUTSIDE_DOMAIN);

    /* An invalid law must be reported before the NaN evaluation temperature. */
    law.coefficient_count = 0;
    expect_failure_double(&law, NAN, BBTC_STATUS_OUTSIDE_DOMAIN);
    law.coefficient_count = 2;

    /* Reusing previously successful storage must never preserve stale values. */
    assert_int_equal(bbtc_vv_virial_observe_double(&law, 384.0, &terms, &valid),
                     BBTC_STATUS_SUCCESS);
    assert_true(valid);
    assert_true(terms.second_density_virial_coefficient_m3_per_kg != 0.0);

    assert_int_equal(bbtc_vv_virial_observe_double(&law, NAN, &terms, &valid),
                     BBTC_STATUS_NAN_INPUT);
    assert_false(valid);
    assert_cleared_double(&terms);
}

/**
 * @brief Verify inclusive double temperature endpoints and borrowed ownership.
 *
 * @details
 * This test checks only execution success and finite output fields at the
 * documented endpoints. It does not compare them with independently derived
 * mathematical references; that remains a separate B2b runner obligation.
 *
 * @param state Unused cmocka test state.
 */
static void test_double_closed_interval_and_ownership(void** state)
{
    (void)state;

    const double coefficients[] = {0x1p-10, 0x1p-12};
    const bbtc_ib_first_order_virial_temperature_law_double_t law =
    {
        .minimum_temperature_k = 256.0,
        .maximum_temperature_k = 512.0,
        .second_density_virial_chebyshev_coefficients_m3_per_kg = coefficients,
        .coefficient_count = 2
    };
    bbtc_ib_first_order_virial_temperature_terms_double_t terms = {0};
    bool valid = false;

    assert_int_equal(bbtc_vv_virial_observe_double(&law, 256.0, &terms, &valid),
                     BBTC_STATUS_SUCCESS);
    assert_true(valid);
    assert_true(isfinite(terms.second_density_virial_coefficient_m3_per_kg));
    assert_true(isfinite(terms.first_temperature_derivative_m3_per_kg_k));
    assert_true(isfinite(terms.second_temperature_derivative_m3_per_kg_k2));

    valid = false;
    assert_int_equal(bbtc_vv_virial_observe_double(&law, 512.0, &terms, &valid),
                     BBTC_STATUS_SUCCESS);
    assert_true(valid);
    assert_true(isfinite(terms.second_density_virial_coefficient_m3_per_kg));
    assert_true(isfinite(terms.first_temperature_derivative_m3_per_kg_k));
    assert_true(isfinite(terms.second_temperature_derivative_m3_per_kg_k2));

    assert_true(coefficients[0] == 0x1p-10);
    assert_true(coefficients[1] == 0x1p-12);
}

/**
 * @brief Check that failure clearing reaches each documented long double field.
 * @param terms Nonnull caller-owned terms storage to inspect.
 */
static void assert_cleared_long_double(
    const bbtc_ib_first_order_virial_temperature_terms_long_double_t* terms
)
{
    assert_true(terms->second_density_virial_coefficient_m3_per_kg == 0.0L);
    assert_true(terms->first_temperature_derivative_m3_per_kg_k == 0.0L);
    assert_true(terms->second_temperature_derivative_m3_per_kg_k2 == 0.0L);
}

/**
 * @brief Verify exact production status, invalidity, and full output clearing.
 *
 * @details
 * Initializes all three result fields with nonzero sentinels on each call.
 * Checks field values, not whole-structure bytes or padding. The caller must
 * supply valid storage for any nonnull borrowed coefficient array in law.
 *
 * @param law Possibly null law, for precedence tests.
 * @param temperature_k Evaluation temperature, possibly nonfinite.
 * @param expected_status Contractually expected raw BBTC status.
 */
static void expect_failure_long_double(
    const bbtc_ib_first_order_virial_temperature_law_long_double_t* law,
    long double temperature_k,
    bbtc_status_e expected_status
)
{
    bbtc_ib_first_order_virial_temperature_terms_long_double_t terms =
    {
        .second_density_virial_coefficient_m3_per_kg = 7.0L,
        .first_temperature_derivative_m3_per_kg_k = 8.0L,
        .second_temperature_derivative_m3_per_kg_k2 = 9.0L
    };
    bool valid = true;

    const bbtc_status_e status = bbtc_vv_virial_observe_long_double(law,
                                                                    temperature_k,
                                                                    &terms,
                                                                    &valid);
    assert_int_equal(status, expected_status);
    assert_false(valid);
    assert_cleared_long_double(&terms);
}

/**
 * @brief Check required-pointer precedence through the long double adapter.
 * @param state Unused cmocka test state.
 */
static void test_long_double_required_pointers(void** state)
{
    (void)state;

    const long double coefficients[] = {0x1p-10L, 0x1p-12L};
    const bbtc_ib_first_order_virial_temperature_law_long_double_t law =
    {
        .minimum_temperature_k = 256.0L,
        .maximum_temperature_k = 512.0L,
        .second_density_virial_chebyshev_coefficients_m3_per_kg = coefficients,
        .coefficient_count = 2
    };
    bool valid = true;

    /* The output-pointer check precedes both law and temperature validation. */
    assert_int_equal(bbtc_vv_virial_observe_long_double(NULL, NAN, NULL, &valid),
                     BBTC_STATUS_INVALID_ARGUMENT);
    assert_false(valid);

    /* With output storage available, the null law takes precedence over NaN. */
    expect_failure_long_double(NULL, NAN, BBTC_STATUS_INVALID_ARGUMENT);

    /* The optional validity pointer is never a required output. */
    assert_int_equal(bbtc_vv_virial_observe_long_double(&law, 384.0L, NULL, NULL),
                     BBTC_STATUS_INVALID_ARGUMENT);

    valid = true;
    assert_int_equal(bbtc_vv_virial_observe_long_double(&law, 384.0L, NULL, &valid),
                     BBTC_STATUS_INVALID_ARGUMENT);
    assert_false(valid);
}

/**
 * @brief Verify pointer, count, and interval rejection for long double laws.
 * @param state Unused cmocka test state.
 */
static void test_long_double_law_domain(void** state)
{
    (void)state;

    const long double coefficients[] = {0x1p-10L, 0x1p-12L};
    bbtc_ib_first_order_virial_temperature_law_long_double_t law =
    {
        .minimum_temperature_k = 256.0L,
        .maximum_temperature_k = 512.0L,
        .second_density_virial_chebyshev_coefficients_m3_per_kg = coefficients,
        .coefficient_count = 2
    };

    law.second_density_virial_chebyshev_coefficients_m3_per_kg = NULL;
    expect_failure_long_double(&law, NAN, BBTC_STATUS_INVALID_ARGUMENT);
    law.second_density_virial_chebyshev_coefficients_m3_per_kg = coefficients;

    law.coefficient_count = 0;
    expect_failure_long_double(&law, 384.0L, BBTC_STATUS_OUTSIDE_DOMAIN);
    law.coefficient_count = 2;

    law.minimum_temperature_k = 0.0L;
    expect_failure_long_double(&law, 384.0L, BBTC_STATUS_OUTSIDE_DOMAIN);
    law.minimum_temperature_k = 512.0L;
    expect_failure_long_double(&law, 384.0L, BBTC_STATUS_OUTSIDE_DOMAIN);
    law.minimum_temperature_k = 256.0L;

    law.maximum_temperature_k = 256.0L;
    expect_failure_long_double(&law, 384.0L, BBTC_STATUS_OUTSIDE_DOMAIN);
    law.maximum_temperature_k = 512.0L;

    assert_true(coefficients[0] == 0x1p-10L);
    assert_true(coefficients[1] == 0x1p-12L);
}

/**
 * @brief Check NaN/infinity precedence within native long double law validation.
 *
 * @details
 * Law validation scans all declared readable coefficients, retains an
 * infinity indicator, and returns NaN if any coefficient is NaN; it thus
 * recognizes NaN even when infinity occurs earlier in the array. This is
 * not a blanket precedence guarantee across all BBTC public functions.
 *
 * @param state Unused cmocka test state.
 */
static void test_long_double_nonfinite_law_precedence(void** state)
{
    (void)state;

    const long double coefficients[] = {0x1p-10L, 0x1p-12L};
    const long double infinity_then_nan[] = {INFINITY, NAN};
    const long double nan_then_infinity[] = {NAN, INFINITY};
    const long double infinity_only[] = {0x1p-10L, INFINITY};
    bbtc_ib_first_order_virial_temperature_law_long_double_t law =
    {
        .minimum_temperature_k = 256.0L,
        .maximum_temperature_k = 512.0L,
        .second_density_virial_chebyshev_coefficients_m3_per_kg = coefficients,
        .coefficient_count = 2
    };

    law.second_density_virial_chebyshev_coefficients_m3_per_kg = infinity_then_nan;
    expect_failure_long_double(&law, 384.0L, BBTC_STATUS_NAN_INPUT);
    law.second_density_virial_chebyshev_coefficients_m3_per_kg = nan_then_infinity;
    expect_failure_long_double(&law, 384.0L, BBTC_STATUS_NAN_INPUT);
    law.second_density_virial_chebyshev_coefficients_m3_per_kg = infinity_only;
    expect_failure_long_double(&law, 384.0L, BBTC_STATUS_NONFINITE_INPUT);
    law.second_density_virial_chebyshev_coefficients_m3_per_kg = coefficients;

    law.minimum_temperature_k = NAN;
    expect_failure_long_double(&law, 384.0L, BBTC_STATUS_NAN_INPUT);
    law.minimum_temperature_k = 256.0L;
    law.maximum_temperature_k = NAN;
    expect_failure_long_double(&law, 384.0L, BBTC_STATUS_NAN_INPUT);
    law.maximum_temperature_k = 512.0L;

    law.minimum_temperature_k = INFINITY;
    law.second_density_virial_chebyshev_coefficients_m3_per_kg = nan_then_infinity;
    expect_failure_long_double(&law, 384.0L, BBTC_STATUS_NAN_INPUT);
    law.second_density_virial_chebyshev_coefficients_m3_per_kg = coefficients;
    expect_failure_long_double(&law, 384.0L, BBTC_STATUS_NONFINITE_INPUT);
    law.minimum_temperature_k = 256.0L;

    law.maximum_temperature_k = -INFINITY;
    expect_failure_long_double(&law, 384.0L, BBTC_STATUS_NONFINITE_INPUT);
}

/**
 * @brief Verify long double evaluation-temperature classification and cleared reuse.
 * @param state Unused cmocka test state.
 */
static void test_long_double_temperature_and_output_reuse(void** state)
{
    (void)state;

    const long double coefficients[] = {0x1p-10L, 0x1p-12L};
    bbtc_ib_first_order_virial_temperature_law_long_double_t law =
    {
        .minimum_temperature_k = 256.0L,
        .maximum_temperature_k = 512.0L,
        .second_density_virial_chebyshev_coefficients_m3_per_kg = coefficients,
        .coefficient_count = 2
    };
    bbtc_ib_first_order_virial_temperature_terms_long_double_t terms = {0};
    bool valid = false;

    expect_failure_long_double(&law, NAN, BBTC_STATUS_NAN_INPUT);
    expect_failure_long_double(&law, INFINITY, BBTC_STATUS_NONFINITE_INPUT);
    expect_failure_long_double(&law, -INFINITY, BBTC_STATUS_NONFINITE_INPUT);
    expect_failure_long_double(&law, 255.0L, BBTC_STATUS_OUTSIDE_DOMAIN);
    expect_failure_long_double(&law, 513.0L, BBTC_STATUS_OUTSIDE_DOMAIN);

    /* An invalid law must be reported before the NaN evaluation temperature. */
    law.coefficient_count = 0;
    expect_failure_long_double(&law, NAN, BBTC_STATUS_OUTSIDE_DOMAIN);
    law.coefficient_count = 2;

    /* Reusing previously successful storage must never preserve stale values. */
    assert_int_equal(bbtc_vv_virial_observe_long_double(&law, 384.0L, &terms, &valid),
                     BBTC_STATUS_SUCCESS);
    assert_true(valid);
    assert_true(terms.second_density_virial_coefficient_m3_per_kg != 0.0L);

    assert_int_equal(bbtc_vv_virial_observe_long_double(&law, NAN, &terms, &valid),
                     BBTC_STATUS_NAN_INPUT);
    assert_false(valid);
    assert_cleared_long_double(&terms);
}

/**
 * @brief Verify inclusive long double temperature endpoints and borrowed ownership.
 *
 * @details
 * This test checks only execution success and finite output fields at the
 * documented endpoints. It does not compare them with independently derived
 * mathematical references; that remains a separate B2b runner obligation.
 *
 * @param state Unused cmocka test state.
 */
static void test_long_double_closed_interval_and_ownership(void** state)
{
    (void)state;

    const long double coefficients[] = {0x1p-10L, 0x1p-12L};
    const bbtc_ib_first_order_virial_temperature_law_long_double_t law =
    {
        .minimum_temperature_k = 256.0L,
        .maximum_temperature_k = 512.0L,
        .second_density_virial_chebyshev_coefficients_m3_per_kg = coefficients,
        .coefficient_count = 2
    };
    bbtc_ib_first_order_virial_temperature_terms_long_double_t terms = {0};
    bool valid = false;

    assert_int_equal(bbtc_vv_virial_observe_long_double(&law, 256.0L, &terms, &valid),
                     BBTC_STATUS_SUCCESS);
    assert_true(valid);
    assert_true(isfinite(terms.second_density_virial_coefficient_m3_per_kg));
    assert_true(isfinite(terms.first_temperature_derivative_m3_per_kg_k));
    assert_true(isfinite(terms.second_temperature_derivative_m3_per_kg_k2));

    valid = false;
    assert_int_equal(bbtc_vv_virial_observe_long_double(&law, 512.0L, &terms, &valid),
                     BBTC_STATUS_SUCCESS);
    assert_true(valid);
    assert_true(isfinite(terms.second_density_virial_coefficient_m3_per_kg));
    assert_true(isfinite(terms.first_temperature_derivative_m3_per_kg_k));
    assert_true(isfinite(terms.second_temperature_derivative_m3_per_kg_k2));

    assert_true(coefficients[0] == 0x1p-10L);
    assert_true(coefficients[1] == 0x1p-12L);
}

/**
 * @brief Run the isolated VVUQ B2b.2b contract-verification suite.
 * @return cmocka aggregate result; zero means all registered tests passed.
 */
int main(void)
{
    const struct CMUnitTest tests[] =
    {
        cmocka_unit_test(test_float_required_pointers),
        cmocka_unit_test(test_float_law_domain),
        cmocka_unit_test(test_float_nonfinite_law_precedence),
        cmocka_unit_test(test_float_temperature_and_output_reuse),
        cmocka_unit_test(test_float_closed_interval_and_ownership),
        cmocka_unit_test(test_double_required_pointers),
        cmocka_unit_test(test_double_law_domain),
        cmocka_unit_test(test_double_nonfinite_law_precedence),
        cmocka_unit_test(test_double_temperature_and_output_reuse),
        cmocka_unit_test(test_double_closed_interval_and_ownership),
        cmocka_unit_test(test_long_double_required_pointers),
        cmocka_unit_test(test_long_double_law_domain),
        cmocka_unit_test(test_long_double_nonfinite_law_precedence),
        cmocka_unit_test(test_long_double_temperature_and_output_reuse),
        cmocka_unit_test(test_long_double_closed_interval_and_ownership),
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}
