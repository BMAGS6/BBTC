/**
 * @file
 * @brief Tests the temperature-dependent first-order virial model contract.
 */

#include <bbtc/internal_ballistics/first_order_virial_gas_model.h>

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>

/**
 * @brief Compares native-float values using a scale-aware relative tolerance.
 *
 * @details
 * The reference values in this file are nonzero. Scaling the tolerance by the
 * larger magnitude prevents a tiny derivative from passing merely because an
 * unrelated unit-sized absolute tolerance was used.
 */
static int
close_float(float actual, float expected, float relative_tolerance)
{
    const float actual_magnitude = actual < 0.0f ? -actual : actual;
    const float expected_magnitude = expected < 0.0f ? -expected : expected;
    const float difference = actual >= expected
        ? actual - expected
        : expected - actual;
    const float scale = actual_magnitude > expected_magnitude
        ? actual_magnitude
        : expected_magnitude;

    return difference <= relative_tolerance * scale
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Compares native-double values using a scale-aware relative tolerance.
 */
static int
close_double(double actual, double expected, double relative_tolerance)
{
    const double actual_magnitude = actual < 0.0 ? -actual : actual;
    const double expected_magnitude = expected < 0.0 ? -expected : expected;
    const double difference = actual >= expected
        ? actual - expected
        : expected - actual;
    const double scale = actual_magnitude > expected_magnitude
        ? actual_magnitude
        : expected_magnitude;

    return difference <= relative_tolerance * scale
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Compares native-long-double values using a relative tolerance.
 */
static int
close_long_double(long double actual,
                  long double expected,
                  long double relative_tolerance)
{
    const long double actual_magnitude = actual < 0.0L ? -actual : actual;
    const long double expected_magnitude = expected < 0.0L
        ? -expected
        : expected;
    const long double difference = actual >= expected
        ? actual - expected
        : expected - actual;
    const long double scale = actual_magnitude > expected_magnitude
        ? actual_magnitude
        : expected_magnitude;

    return difference <= relative_tolerance * scale
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Verifies null, ownership, finite-value, and interval validation.
 */
static int
test_temperature_law_validation(void)
{
    const double valid_coefficients[] =
    {
        1.0e-3,
        -2.0e-4
    };

    bbtc_ib_first_order_virial_temperature_law_double_t law =
    {
        .minimum_temperature_k = 300.0,
        .maximum_temperature_k = 500.0,
        .second_density_virial_chebyshev_coefficients_m3_per_kg =
            valid_coefficients,
        .coefficient_count = 2U
    };

    if (bbtc_ib_first_order_virial_temperature_law_validate_double(NULL)
        != BBTC_STATUS_INVALID_ARGUMENT)
    {
        return EXIT_FAILURE;
    }

    if (bbtc_ib_first_order_virial_temperature_law_validate_double(&law)
        != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    law.second_density_virial_chebyshev_coefficients_m3_per_kg = NULL;

    if (bbtc_ib_first_order_virial_temperature_law_validate_double(&law)
        != BBTC_STATUS_INVALID_ARGUMENT)
    {
        return EXIT_FAILURE;
    }

    law.minimum_temperature_k = NAN;

    if (bbtc_ib_first_order_virial_temperature_law_validate_double(&law)
        != BBTC_STATUS_INVALID_ARGUMENT)
    {
        return EXIT_FAILURE;
    }

    law.minimum_temperature_k = 300.0;
    law.second_density_virial_chebyshev_coefficients_m3_per_kg =
        valid_coefficients;
    law.coefficient_count = 0U;

    if (bbtc_ib_first_order_virial_temperature_law_validate_double(&law)
        != BBTC_STATUS_OUTSIDE_DOMAIN)
    {
        return EXIT_FAILURE;
    }

    law.coefficient_count = 2U;
    law.minimum_temperature_k = 0.0;

    if (bbtc_ib_first_order_virial_temperature_law_validate_double(&law)
        != BBTC_STATUS_OUTSIDE_DOMAIN)
    {
        return EXIT_FAILURE;
    }

    law.minimum_temperature_k = 500.0;

    if (bbtc_ib_first_order_virial_temperature_law_validate_double(&law)
        != BBTC_STATUS_OUTSIDE_DOMAIN)
    {
        return EXIT_FAILURE;
    }

    law.minimum_temperature_k = NAN;

    if (bbtc_ib_first_order_virial_temperature_law_validate_double(&law)
        != BBTC_STATUS_NAN_INPUT)
    {
        return EXIT_FAILURE;
    }

    {
        const double infinite_coefficients[] =
        {
            1.0e-3,
            INFINITY
        };

        law.minimum_temperature_k = 300.0;
        law.maximum_temperature_k = 500.0;
        law.second_density_virial_chebyshev_coefficients_m3_per_kg =
            infinite_coefficients;

        if (bbtc_ib_first_order_virial_temperature_law_validate_double(&law)
            != BBTC_STATUS_NONFINITE_INPUT)
        {
            return EXIT_FAILURE;
        }
    }

    {
        const double infinity_then_nan_coefficients[] =
        {
            INFINITY,
            NAN
        };

        law.minimum_temperature_k = 300.0;
        law.maximum_temperature_k = 500.0;
        law.second_density_virial_chebyshev_coefficients_m3_per_kg =
            infinity_then_nan_coefficients;

        if (bbtc_ib_first_order_virial_temperature_law_validate_double(&law)
            != BBTC_STATUS_NAN_INPUT)
        {
            return EXIT_FAILURE;
        }
    }

    {
        const double nan_coefficients[] =
        {
            1.0e-3,
            NAN
        };

        law.minimum_temperature_k = INFINITY;
        law.maximum_temperature_k = 500.0;
        law.second_density_virial_chebyshev_coefficients_m3_per_kg =
            nan_coefficients;

        if (bbtc_ib_first_order_virial_temperature_law_validate_double(&law)
            != BBTC_STATUS_NAN_INPUT)
        {
            return EXIT_FAILURE;
        }
    }

    return EXIT_SUCCESS;
}


/**
 * @brief Verifies the degree-zero constant and ideal-gas special cases.
 */
static int
test_constant_temperature_laws(void)
{
    const float float_coefficients[] = {0.0f};
    const double double_coefficients[] = {-2.5e-4};
    const long double long_double_coefficients[] = {3.0e-4L};

    const bbtc_ib_first_order_virial_temperature_law_float_t float_law =
    {
        .minimum_temperature_k = 250.0f,
        .maximum_temperature_k = 2500.0f,
        .second_density_virial_chebyshev_coefficients_m3_per_kg =
            float_coefficients,
        .coefficient_count = 1U
    };

    const bbtc_ib_first_order_virial_temperature_law_double_t double_law =
    {
        .minimum_temperature_k = 250.0,
        .maximum_temperature_k = 2500.0,
        .second_density_virial_chebyshev_coefficients_m3_per_kg =
            double_coefficients,
        .coefficient_count = 1U
    };

    const bbtc_ib_first_order_virial_temperature_law_long_double_t
        long_double_law =
    {
        .minimum_temperature_k = 250.0L,
        .maximum_temperature_k = 2500.0L,
        .second_density_virial_chebyshev_coefficients_m3_per_kg =
            long_double_coefficients,
        .coefficient_count = 1U
    };

    bbtc_ib_first_order_virial_temperature_terms_float_t float_terms =
    {
        .second_density_virial_coefficient_m3_per_kg = 1.0f,
        .first_temperature_derivative_m3_per_kg_k = 1.0f,
        .second_temperature_derivative_m3_per_kg_k2 = 1.0f
    };

    bbtc_ib_first_order_virial_temperature_terms_double_t double_terms = {0};
    bbtc_ib_first_order_virial_temperature_terms_long_double_t
        long_double_terms = {0};

    if (bbtc_ib_first_order_virial_temperature_law_evaluate_float(
            &float_law,
            1000.0f,
            &float_terms
        ) != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (float_terms.second_density_virial_coefficient_m3_per_kg != 0.0f ||
        float_terms.first_temperature_derivative_m3_per_kg_k != 0.0f ||
        float_terms.second_temperature_derivative_m3_per_kg_k2 != 0.0f)
    {
        return EXIT_FAILURE;
    }

    if (bbtc_ib_first_order_virial_temperature_law_evaluate_double(
            &double_law,
            250.0,
            &double_terms
        ) != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (double_terms.second_density_virial_coefficient_m3_per_kg
            != double_coefficients[0] ||
        double_terms.first_temperature_derivative_m3_per_kg_k != 0.0 ||
        double_terms.second_temperature_derivative_m3_per_kg_k2 != 0.0)
    {
        return EXIT_FAILURE;
    }

    if (bbtc_ib_first_order_virial_temperature_law_evaluate_long_double(
            &long_double_law,
            2500.0L,
            &long_double_terms
        ) != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (long_double_terms.second_density_virial_coefficient_m3_per_kg
            != long_double_coefficients[0] ||
        long_double_terms.first_temperature_derivative_m3_per_kg_k != 0.0L ||
        long_double_terms.second_temperature_derivative_m3_per_kg_k2 != 0.0L)
    {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}


/**
 * @brief Verifies analytic evaluation of a quadratic Chebyshev series.
 */
static int
test_quadratic_temperature_laws(void)
{
    /*
     * B(x) = c0*T0(x) + c1*T1(x) + c2*T2(x)
     *      = c0 + c1*x + c2*(2*x^2 - 1)
     *
     * The interval [300, 500] K gives x = 0 at 400 K and dx/dT = 0.01 K^-1.
     * At x = 0:
     *
     * B       = c0 - c2
     * dB/dT   = c1 * 0.01
     * d2B/dT2 = 4*c2 * 0.01^2
     */
    const float float_coefficients[] =
    {
        1.0e-3f,
        2.0e-4f,
        3.0e-5f
    };

    const double double_coefficients[] =
    {
        1.0e-3,
        2.0e-4,
        3.0e-5
    };

    const long double long_double_coefficients[] =
    {
        1.0e-3L,
        2.0e-4L,
        3.0e-5L
    };

    const bbtc_ib_first_order_virial_temperature_law_float_t float_law =
    {
        .minimum_temperature_k = 300.0f,
        .maximum_temperature_k = 500.0f,
        .second_density_virial_chebyshev_coefficients_m3_per_kg =
            float_coefficients,
        .coefficient_count = 3U
    };

    const bbtc_ib_first_order_virial_temperature_law_double_t double_law =
    {
        .minimum_temperature_k = 300.0,
        .maximum_temperature_k = 500.0,
        .second_density_virial_chebyshev_coefficients_m3_per_kg =
            double_coefficients,
        .coefficient_count = 3U
    };

    const bbtc_ib_first_order_virial_temperature_law_long_double_t
        long_double_law =
    {
        .minimum_temperature_k = 300.0L,
        .maximum_temperature_k = 500.0L,
        .second_density_virial_chebyshev_coefficients_m3_per_kg =
            long_double_coefficients,
        .coefficient_count = 3U
    };

    bbtc_ib_first_order_virial_temperature_terms_float_t float_terms = {0};
    bbtc_ib_first_order_virial_temperature_terms_double_t double_terms = {0};
    bbtc_ib_first_order_virial_temperature_terms_long_double_t
        long_double_terms = {0};

    if (bbtc_ib_first_order_virial_temperature_law_evaluate_float(
            &float_law,
            400.0f,
            &float_terms
        ) != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (close_float(
            float_terms.second_density_virial_coefficient_m3_per_kg,
            9.7e-4f,
            64.0f * FLT_EPSILON
        ) != EXIT_SUCCESS ||
        close_float(
            float_terms.first_temperature_derivative_m3_per_kg_k,
            2.0e-6f,
            64.0f * FLT_EPSILON
        ) != EXIT_SUCCESS ||
        close_float(
            float_terms.second_temperature_derivative_m3_per_kg_k2,
            1.2e-8f,
            64.0f * FLT_EPSILON
        ) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (bbtc_ib_first_order_virial_temperature_law_evaluate_double(
            &double_law,
            400.0,
            &double_terms
        ) != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (close_double(
            double_terms.second_density_virial_coefficient_m3_per_kg,
            9.7e-4,
            128.0 * DBL_EPSILON
        ) != EXIT_SUCCESS ||
        close_double(
            double_terms.first_temperature_derivative_m3_per_kg_k,
            2.0e-6,
            128.0 * DBL_EPSILON
        ) != EXIT_SUCCESS ||
        close_double(
            double_terms.second_temperature_derivative_m3_per_kg_k2,
            1.2e-8,
            128.0 * DBL_EPSILON
        ) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (bbtc_ib_first_order_virial_temperature_law_evaluate_long_double(
            &long_double_law,
            400.0L,
            &long_double_terms
        ) != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (close_long_double(
            long_double_terms.second_density_virial_coefficient_m3_per_kg,
            9.7e-4L,
            256.0L * LDBL_EPSILON
        ) != EXIT_SUCCESS ||
        close_long_double(
            long_double_terms.first_temperature_derivative_m3_per_kg_k,
            2.0e-6L,
            256.0L * LDBL_EPSILON
        ) != EXIT_SUCCESS ||
        close_long_double(
            long_double_terms.second_temperature_derivative_m3_per_kg_k2,
            1.2e-8L,
            256.0L * LDBL_EPSILON
        ) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    /*
     * Exercise both closed interval endpoints with a nonconstant law. These
     * checks verify the affine mapping itself rather than relying only on the
     * degree-zero endpoint cases above.
     */
    if (bbtc_ib_first_order_virial_temperature_law_evaluate_double(
            &double_law,
            300.0,
            &double_terms
        ) != BBTC_STATUS_SUCCESS ||
        close_double(
            double_terms.second_density_virial_coefficient_m3_per_kg,
            8.3e-4,
            128.0 * DBL_EPSILON
        ) != EXIT_SUCCESS ||
        close_double(
            double_terms.first_temperature_derivative_m3_per_kg_k,
            8.0e-7,
            128.0 * DBL_EPSILON
        ) != EXIT_SUCCESS ||
        close_double(
            double_terms.second_temperature_derivative_m3_per_kg_k2,
            1.2e-8,
            128.0 * DBL_EPSILON
        ) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (bbtc_ib_first_order_virial_temperature_law_evaluate_double(
            &double_law,
            500.0,
            &double_terms
        ) != BBTC_STATUS_SUCCESS ||
        close_double(
            double_terms.second_density_virial_coefficient_m3_per_kg,
            1.23e-3,
            128.0 * DBL_EPSILON
        ) != EXIT_SUCCESS ||
        close_double(
            double_terms.first_temperature_derivative_m3_per_kg_k,
            3.2e-6,
            128.0 * DBL_EPSILON
        ) != EXIT_SUCCESS ||
        close_double(
            double_terms.second_temperature_derivative_m3_per_kg_k2,
            1.2e-8,
            128.0 * DBL_EPSILON
        ) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}


/**
 * @brief Verifies evaluator failure statuses and output clearing.
 */
static int
test_evaluator_failures(void)
{
    const double coefficients[] = {1.0e-3};

    const bbtc_ib_first_order_virial_temperature_law_double_t law =
    {
        .minimum_temperature_k = 300.0,
        .maximum_temperature_k = 500.0,
        .second_density_virial_chebyshev_coefficients_m3_per_kg =
            coefficients,
        .coefficient_count = 1U
    };

    bbtc_ib_first_order_virial_temperature_terms_double_t terms =
    {
        .second_density_virial_coefficient_m3_per_kg = 1.0,
        .first_temperature_derivative_m3_per_kg_k = 1.0,
        .second_temperature_derivative_m3_per_kg_k2 = 1.0
    };

    if (bbtc_ib_first_order_virial_temperature_law_evaluate_double(
            NULL,
            400.0,
            &terms
        ) != BBTC_STATUS_INVALID_ARGUMENT)
    {
        return EXIT_FAILURE;
    }

    if (terms.second_density_virial_coefficient_m3_per_kg != 0.0 ||
        terms.first_temperature_derivative_m3_per_kg_k != 0.0 ||
        terms.second_temperature_derivative_m3_per_kg_k2 != 0.0)
    {
        return EXIT_FAILURE;
    }

    if (bbtc_ib_first_order_virial_temperature_law_evaluate_double(
            &law,
            400.0,
            NULL
        ) != BBTC_STATUS_INVALID_ARGUMENT)
    {
        return EXIT_FAILURE;
    }

    terms.second_density_virial_coefficient_m3_per_kg = 1.0;

    if (bbtc_ib_first_order_virial_temperature_law_evaluate_double(
            &law,
            NAN,
            &terms
        ) != BBTC_STATUS_NAN_INPUT)
    {
        return EXIT_FAILURE;
    }

    if (terms.second_density_virial_coefficient_m3_per_kg != 0.0)
    {
        return EXIT_FAILURE;
    }

    terms.second_density_virial_coefficient_m3_per_kg = 1.0;

    if (bbtc_ib_first_order_virial_temperature_law_evaluate_double(
            &law,
            INFINITY,
            &terms
        ) != BBTC_STATUS_NONFINITE_INPUT)
    {
        return EXIT_FAILURE;
    }

    if (terms.second_density_virial_coefficient_m3_per_kg != 0.0)
    {
        return EXIT_FAILURE;
    }

    terms.second_density_virial_coefficient_m3_per_kg = 1.0;

    if (bbtc_ib_first_order_virial_temperature_law_evaluate_double(
            &law,
            299.0,
            &terms
        ) != BBTC_STATUS_OUTSIDE_DOMAIN)
    {
        return EXIT_FAILURE;
    }

    return terms.second_density_virial_coefficient_m3_per_kg == 0.0
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Verifies nonfinite recurrence output is reported as numerical failure.
 *
 * @details
 * Every supplied parameter is finite. Evaluation at the upper endpoint gives
 * `x == 1`; the deliberately enormous finite coefficients then overflow the
 * recurrence. The evaluator must convert that arithmetic failure into
 * `BBTC_STATUS_NUMERICAL_FAILURE` and clear the caller-owned output record.
 */
static int
test_evaluator_numerical_failure(void)
{
    const double coefficients[] =
    {
        0.0,
        DBL_MAX,
        DBL_MAX
    };

    const bbtc_ib_first_order_virial_temperature_law_double_t law =
    {
        .minimum_temperature_k = 300.0,
        .maximum_temperature_k = 500.0,
        .second_density_virial_chebyshev_coefficients_m3_per_kg =
            coefficients,
        .coefficient_count = 3U
    };

    bbtc_ib_first_order_virial_temperature_terms_double_t terms =
    {
        .second_density_virial_coefficient_m3_per_kg = 1.0,
        .first_temperature_derivative_m3_per_kg_k = 1.0,
        .second_temperature_derivative_m3_per_kg_k2 = 1.0
    };

    if (bbtc_ib_first_order_virial_temperature_law_evaluate_double(
            &law,
            500.0,
            &terms
        ) != BBTC_STATUS_NUMERICAL_FAILURE)
    {
        return EXIT_FAILURE;
    }

    return terms.second_density_virial_coefficient_m3_per_kg == 0.0
            && terms.first_temperature_derivative_m3_per_kg_k == 0.0
            && terms.second_temperature_derivative_m3_per_kg_k2 == 0.0
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Verifies gas-model scalar, density, and nested-law validation.
 */
static int
test_gas_model_validation(void)
{
    const float float_coefficients[] = {-1.0e-4f};
    const double double_coefficients[] = {0.0};
    const long double long_double_coefficients[] = {1.0e-4L};

    bbtc_ib_first_order_virial_gas_model_float_t float_model =
    {
        .specific_gas_constant_j_per_kg_k = 300.0f,
        .ideal_gas_constant_volume_specific_heat_j_per_kg_k = 900.0f,
        .minimum_calibrated_density_kg_per_m3 = 0.0f,
        .maximum_calibrated_density_kg_per_m3 = 500.0f,
        .second_density_virial_coefficient_law =
        {
            .minimum_temperature_k = 300.0f,
            .maximum_temperature_k = 4000.0f,
            .second_density_virial_chebyshev_coefficients_m3_per_kg =
                float_coefficients,
            .coefficient_count = 1U
        }
    };

    bbtc_ib_first_order_virial_gas_model_double_t double_model =
    {
        .specific_gas_constant_j_per_kg_k = 300.0,
        .ideal_gas_constant_volume_specific_heat_j_per_kg_k = 900.0,
        .minimum_calibrated_density_kg_per_m3 = 0.0,
        .maximum_calibrated_density_kg_per_m3 = 500.0,
        .second_density_virial_coefficient_law =
        {
            .minimum_temperature_k = 300.0,
            .maximum_temperature_k = 4000.0,
            .second_density_virial_chebyshev_coefficients_m3_per_kg =
                double_coefficients,
            .coefficient_count = 1U
        }
    };

    const bbtc_ib_first_order_virial_gas_model_long_double_t
        long_double_model =
    {
        .specific_gas_constant_j_per_kg_k = 300.0L,
        .ideal_gas_constant_volume_specific_heat_j_per_kg_k = 900.0L,
        .minimum_calibrated_density_kg_per_m3 = 0.0L,
        .maximum_calibrated_density_kg_per_m3 = 500.0L,
        .second_density_virial_coefficient_law =
        {
            .minimum_temperature_k = 300.0L,
            .maximum_temperature_k = 4000.0L,
            .second_density_virial_chebyshev_coefficients_m3_per_kg =
                long_double_coefficients,
            .coefficient_count = 1U
        }
    };

    if (bbtc_ib_first_order_virial_gas_model_validate_float(&float_model)
            != BBTC_STATUS_SUCCESS ||
        bbtc_ib_first_order_virial_gas_model_validate_double(&double_model)
            != BBTC_STATUS_SUCCESS ||
        bbtc_ib_first_order_virial_gas_model_validate_long_double(
            &long_double_model
        ) != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (bbtc_ib_first_order_virial_gas_model_validate_double(NULL)
        != BBTC_STATUS_INVALID_ARGUMENT)
    {
        return EXIT_FAILURE;
    }

    double_model.specific_gas_constant_j_per_kg_k = 0.0;

    if (bbtc_ib_first_order_virial_gas_model_validate_double(&double_model)
        != BBTC_STATUS_OUTSIDE_DOMAIN)
    {
        return EXIT_FAILURE;
    }

    double_model.specific_gas_constant_j_per_kg_k = 300.0;
    double_model.ideal_gas_constant_volume_specific_heat_j_per_kg_k =
        INFINITY;

    if (bbtc_ib_first_order_virial_gas_model_validate_double(&double_model)
        != BBTC_STATUS_NONFINITE_INPUT)
    {
        return EXIT_FAILURE;
    }

    double_model.ideal_gas_constant_volume_specific_heat_j_per_kg_k = NAN;

    if (bbtc_ib_first_order_virial_gas_model_validate_double(&double_model)
        != BBTC_STATUS_NAN_INPUT)
    {
        return EXIT_FAILURE;
    }

    double_model.ideal_gas_constant_volume_specific_heat_j_per_kg_k = 900.0;
    double_model.minimum_calibrated_density_kg_per_m3 = -1.0;

    if (bbtc_ib_first_order_virial_gas_model_validate_double(&double_model)
        != BBTC_STATUS_OUTSIDE_DOMAIN)
    {
        return EXIT_FAILURE;
    }

    double_model.minimum_calibrated_density_kg_per_m3 = 500.0;

    if (bbtc_ib_first_order_virial_gas_model_validate_double(&double_model)
        != BBTC_STATUS_OUTSIDE_DOMAIN)
    {
        return EXIT_FAILURE;
    }

    double_model.minimum_calibrated_density_kg_per_m3 = 0.0;

    {
        const double nested_nan_coefficients[] = {NAN};

        double_model.second_density_virial_coefficient_law
            .second_density_virial_chebyshev_coefficients_m3_per_kg =
                nested_nan_coefficients;

        if (bbtc_ib_first_order_virial_gas_model_validate_double(&double_model)
            != BBTC_STATUS_NAN_INPUT)
        {
            return EXIT_FAILURE;
        }

        double_model.ideal_gas_constant_volume_specific_heat_j_per_kg_k =
            INFINITY;

        if (bbtc_ib_first_order_virial_gas_model_validate_double(&double_model)
            != BBTC_STATUS_NONFINITE_INPUT)
        {
            return EXIT_FAILURE;
        }

        double_model.ideal_gas_constant_volume_specific_heat_j_per_kg_k =
            900.0;
    }

    double_model.second_density_virial_coefficient_law
        .second_density_virial_chebyshev_coefficients_m3_per_kg =
            double_coefficients;

    double_model.second_density_virial_coefficient_law
        .second_density_virial_chebyshev_coefficients_m3_per_kg = NULL;

    return bbtc_ib_first_order_virial_gas_model_validate_double(&double_model)
            == BBTC_STATUS_INVALID_ARGUMENT
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Runs the first-order virial contract tests.
 */
int
main(void)
{
    if (test_temperature_law_validation() != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (test_constant_temperature_laws() != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (test_quadratic_temperature_laws() != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (test_evaluator_failures() != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (test_evaluator_numerical_failure() != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    return test_gas_model_validation();
}
