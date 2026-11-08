/**
 * @file test_virial_adapter.c
 * @brief Contract smoke tests of the B2b native-family BBTC observation adapter.
 *
 * @details
 * This test suite verifies only the developer adapter's ability to forward
 * actual public API calls, preserve raw status, preserve borrowed coefficient
 * inputs, convey full native result fields, and prevent failure-cleared terms
 * from being marked valid. It is NOT the future 36-comparison independent
 * reference runner and does not qualify EXACT arithmetic or physical models.
 *
 * Dyadic sample values are used here so incidental checks need no tolerance.
 * Later B2b work must separately qualify the independent reference fixture,
 * full EXACT comparison matrix, negative controls, and compiler/ABI/fenv.
 *
 * @warning A passing smoke test is not a numerical or scientific validation
 *          verdict and provides no ammunition or firearm safety assurance.
 */
#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <stdbool.h>
#include <float.h>
#include <math.h>

#include <cmocka.h>

#include "virial_adapter.h"

/**
 * @brief Exercise the FLOAT public API path with a borrowed, readable array.
 * @param state cmocka per-test state; unused.
 */
static void test_float_observation(void** state)
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
    bbtc_ib_first_order_virial_temperature_terms_float_t terms =
    {
        .second_density_virial_coefficient_m3_per_kg = -1.0f,
        .first_temperature_derivative_m3_per_kg_k = -1.0f,
        .second_temperature_derivative_m3_per_kg_k2 = -1.0f
    };
    bool valid = false;

    /* c0 + c1*T1(0) = 2^-10; dB/dT = c1/128 = 2^-19. */
    const bbtc_status_e status =
        bbtc_vv_virial_observe_float(&law, 384.0f, &terms, &valid);

    assert_int_equal(status, BBTC_STATUS_SUCCESS);
    assert_true(valid);
    assert_true(terms.second_density_virial_coefficient_m3_per_kg == 0x1p-10f);
    assert_true(terms.first_temperature_derivative_m3_per_kg_k == 0x1p-19f);
    assert_true(terms.second_temperature_derivative_m3_per_kg_k2 == 0.0f);
    assert_true(coefficients[0] == 0x1p-10f);
    assert_true(coefficients[1] == 0x1p-12f);
}


/**
 * @brief Exercise the DOUBLE path and test precedence and output clearing.
 * @param state cmocka per-test state; unused.
 */
static void test_double_failure_preserves_status_and_clears_fields(void** state)
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
    bbtc_ib_first_order_virial_temperature_terms_double_t terms =
    {
        .second_density_virial_coefficient_m3_per_kg = 99.0,
        .first_temperature_derivative_m3_per_kg_k = 99.0,
        .second_temperature_derivative_m3_per_kg_k2 = 99.0
    };
    bool valid = true;

    /* A null law is rejected AFTER BBTC clears nonnull output storage. */
    bbtc_status_e status = bbtc_vv_virial_observe_double(NULL, 384.0, &terms, &valid);
    assert_int_equal(status, BBTC_STATUS_INVALID_ARGUMENT);
    assert_false(valid);
    assert_true(terms.second_density_virial_coefficient_m3_per_kg == 0.0);
    assert_true(terms.first_temperature_derivative_m3_per_kg_k == 0.0);
    assert_true(terms.second_temperature_derivative_m3_per_kg_k2 == 0.0);

    /* A null output is sent to production as null, without dereferencing it. */
    valid = true;
    status = bbtc_vv_virial_observe_double(&law, 384.0, NULL, &valid);
    assert_int_equal(status, BBTC_STATUS_INVALID_ARGUMENT);
    assert_false(valid);

    /* The optional validity pointer may be omitted on a successful call. */
    status = bbtc_vv_virial_observe_double(&law, 384.0, &terms, NULL);
    assert_int_equal(status, BBTC_STATUS_SUCCESS);
    assert_true(terms.second_density_virial_coefficient_m3_per_kg == 0x1p-10);
    assert_true(terms.first_temperature_derivative_m3_per_kg_k == 0x1p-19);
    assert_true(terms.second_temperature_derivative_m3_per_kg_k2 == 0.0);
    assert_true(coefficients[0] == 0x1p-10);
    assert_true(coefficients[1] == 0x1p-12);
}


/**
 * @brief Show that LONG DOUBLE is dispatched and observed without narrowing.
 * @param state cmocka per-test state; unused.
 */
static void test_long_double_observation_and_nan_failure(void** state)
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
    bbtc_ib_first_order_virial_temperature_terms_long_double_t terms =
    {
        .second_density_virial_coefficient_m3_per_kg = -1.0L,
        .first_temperature_derivative_m3_per_kg_k = -1.0L,
        .second_temperature_derivative_m3_per_kg_k2 = -1.0L
    };
    bool valid = false;

    bbtc_status_e status =
        bbtc_vv_virial_observe_long_double(&law, 384.0L, &terms, &valid);
    assert_int_equal(status, BBTC_STATUS_SUCCESS);
    assert_true(valid);
    assert_true(terms.second_density_virial_coefficient_m3_per_kg == 0x1p-10L);
    assert_true(terms.first_temperature_derivative_m3_per_kg_k == 0x1p-19L);
    assert_true(terms.second_temperature_derivative_m3_per_kg_k2 == 0.0L);

    /* A later invalid temperature must not leave stale 'valid' observations. */
    status = bbtc_vv_virial_observe_long_double(&law, NAN, &terms, &valid);
    assert_int_equal(status, BBTC_STATUS_NAN_INPUT);
    assert_false(valid);
    assert_true(terms.second_density_virial_coefficient_m3_per_kg == 0.0L);
    assert_true(terms.first_temperature_derivative_m3_per_kg_k == 0.0L);
    assert_true(terms.second_temperature_derivative_m3_per_kg_k2 == 0.0L);
    assert_true(coefficients[0] == 0x1p-10L);
    assert_true(coefficients[1] == 0x1p-12L);
}


/**
 * @brief Detect accidental routing of a native double observation through float.
 *
 * @details
 * With a wider double significand, 1 + DBL_EPSILON is distinguishable from
 * 1 in double but rounds to 1 if narrowed to float. A single-coefficient
 * law makes the returned coefficient itself the observable, independently
 * of the more complicated polynomial test cases. Platforms without a wider
 * double significand still exercise successful native-double dispatch but
 * cannot use this particular value as a narrowing discriminator.
 *
 * @param state cmocka per-test state; unused.
 */
static void test_double_native_precision(void** state)
{
    (void)state;

    const double coefficients[] = {1.0 + DBL_EPSILON};
    const bbtc_ib_first_order_virial_temperature_law_double_t law =
    {
        .minimum_temperature_k = 256.0,
        .maximum_temperature_k = 512.0,
        .second_density_virial_chebyshev_coefficients_m3_per_kg = coefficients,
        .coefficient_count = 1
    };
    bbtc_ib_first_order_virial_temperature_terms_double_t terms = {0};
    bool valid = false;

    const bbtc_status_e status =
        bbtc_vv_virial_observe_double(&law, 384.0, &terms, &valid);

    assert_int_equal(status, BBTC_STATUS_SUCCESS);
    assert_true(valid);
    assert_true(terms.second_density_virial_coefficient_m3_per_kg == coefficients[0]);
    assert_true(terms.first_temperature_derivative_m3_per_kg_k == 0.0);
    assert_true(terms.second_temperature_derivative_m3_per_kg_k2 == 0.0);

#if DBL_MANT_DIG > FLT_MANT_DIG
    assert_true(coefficients[0] != (double)(float)coefficients[0]);
#endif
}


/**
 * @brief Detect accidental narrowing of long-double observations to double.
 *
 * @details
 * On extended-precision systems, 1 + LDBL_EPSILON is an exactly representable
 * long-double number that is not representable as a double. Preserving it
 * through the adapter therefore provides a discriminating native-family
 * smoke test. When long double has no extra significand precision, this test
 * still checks its public entry point but makes no extra-precision claim.
 *
 * @param state cmocka per-test state; unused.
 */
static void test_long_double_native_precision(void** state)
{
    (void)state;

    const long double coefficients[] = {1.0L + LDBL_EPSILON};
    const bbtc_ib_first_order_virial_temperature_law_long_double_t law =
    {
        .minimum_temperature_k = 256.0L,
        .maximum_temperature_k = 512.0L,
        .second_density_virial_chebyshev_coefficients_m3_per_kg = coefficients,
        .coefficient_count = 1
    };
    bbtc_ib_first_order_virial_temperature_terms_long_double_t terms = {0};
    bool valid = false;

    const bbtc_status_e status =
        bbtc_vv_virial_observe_long_double(&law, 384.0L, &terms, &valid);

    assert_int_equal(status, BBTC_STATUS_SUCCESS);
    assert_true(valid);
    assert_true(terms.second_density_virial_coefficient_m3_per_kg == coefficients[0]);
    assert_true(terms.first_temperature_derivative_m3_per_kg_k == 0.0L);
    assert_true(terms.second_temperature_derivative_m3_per_kg_k2 == 0.0L);

#if LDBL_MANT_DIG > DBL_MANT_DIG
    assert_true(coefficients[0] != (long double)(double)coefficients[0]);
#endif
}


/**
 * @brief Register the first developer-only B2b smoke cases with cmocka.
 * @return cmocka's aggregate status; zero indicates all cases passed.
 */
int main(void)
{
    const struct CMUnitTest tests[] =
    {
        cmocka_unit_test(test_float_observation),
        cmocka_unit_test(test_double_failure_preserves_status_and_clears_fields),
        cmocka_unit_test(test_long_double_observation_and_nan_failure),
        cmocka_unit_test(test_double_native_precision),
        cmocka_unit_test(test_long_double_native_precision)
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}
