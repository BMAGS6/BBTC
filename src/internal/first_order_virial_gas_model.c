/**
 * @file
 * @brief Validation and temperature-law evaluation for the first-order
 *        density-virial gas-model backend.
 *
 * @details
 * The implementation intentionally stops before evaluating pressure or caloric
 * state. It provides the temperature-dependent coefficient terms needed by a
 * later thermodynamically consistent constitutive evaluator.
 */

#include "bbtc/internal_ballistics/first_order_virial_gas_model.h"

#include <stddef.h>
#include <math.h>

/*
 * Backward Clenshaw recurrence and analytic derivatives
 * ------------------------------------------------------
 *
 * The represented first-kind Chebyshev series is:
 *
 *     B(x) = sum(c_k * T_k(x), k = 0 .. N - 1)
 *
 * For N coefficients, backward Clenshaw evaluation defines:
 *
 *     b_N = b_(N+1) = 0
 *     b_k = 2*x*b_(k+1) - b_(k+2) + c_k, k = N-1 .. 1
 *
 * and reconstructs:
 *
 *     B(x) = x*b_1 - b_2 + c_0
 *
 * Differentiating the auxiliary recurrence gives:
 *
 *     b'_k =
 *         2*b_(k+1) + 2*x*b'_(k+1) - b'_(k+2)
 *
 *     b''_k =
 *         4*b'_(k+1) + 2*x*b''_(k+1) - b''_(k+2)
 *
 * Therefore:
 *
 *     dB/dx   = b_1 + x*b'_1 - b'_2
 *     d2B/dx2 = 2*b'_1 + x*b''_1 - b''_2
 *
 * The mapping x(T) is affine, so d2x/dT2 is zero:
 *
 *     dB/dT   = dB/dx * dx/dT
 *     d2B/dT2 = d2B/dx2 * (dx/dT)^2
 *
 * This form evaluates the series and both derivatives in one backward pass
 * with constant storage. No finite differencing, allocation, hidden global
 * state, or precision-family conversion is used.
 */

bbtc_status_e
bbtc_ib_first_order_virial_temperature_law_validate_float(
    const bbtc_ib_first_order_virial_temperature_law_float_t* const law
)
{
    if (law == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (law->second_density_virial_chebyshev_coefficients_m3_per_kg == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (!isfinite(law->minimum_temperature_k) ||
        !isfinite(law->maximum_temperature_k))
    {
        return BBTC_STATUS_NONFINITE_INPUT;
    }

    if (law->minimum_temperature_k <= 0.0f                       ||
        law->maximum_temperature_k <= law->minimum_temperature_k ||
        law->coefficient_count     == 0u)
    {
        return BBTC_STATUS_OUTSIDE_DOMAIN;
    }

    for (size_t coefficient_index = 0u;
         coefficient_index < law->coefficient_count;
         ++coefficient_index)
    {
        if (!isfinite(law->second_density_virial_chebyshev_coefficients_m3_per_kg[ coefficient_index ]))
            return BBTC_STATUS_NONFINITE_INPUT;
    }

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e
bbtc_ib_first_order_virial_temperature_law_evaluate_float(
    const bbtc_ib_first_order_virial_temperature_law_float_t* const law,
    float temperature_k,
    bbtc_ib_first_order_virial_temperature_terms_float_t* const terms
)
{
    bbtc_status_e status;
    float temperature_range_k;
    float normalized_temperature;
    float temperature_scale_per_k;

    /*
     * Clenshaw state for b_(k+1) and b_(k+2), plus the first and second
     * derivatives of those auxiliary values with respect to x.
     */
    float value_next       = 0.0f;
    float value_next_next  = 0.0f;
    float first_next       = 0.0f;
    float first_next_next  = 0.0f;
    float second_next      = 0.0f;
    float second_next_next = 0.0f;

    float polynomial_value;
    float first_derivative_wrt_x;
    float second_derivative_wrt_x;

    if (terms == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    *terms = (bbtc_ib_first_order_virial_temperature_terms_float_t){0};

    status = bbtc_ib_first_order_virial_temperature_law_validate_float(law);

    if (status != BBTC_STATUS_SUCCESS)
        return status;

    if (!isfinite(temperature_k))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (temperature_k < law->minimum_temperature_k ||
        temperature_k > law->maximum_temperature_k)
    {
        return BBTC_STATUS_OUTSIDE_DOMAIN;
    }

    temperature_range_k = law->maximum_temperature_k - law->minimum_temperature_k;

    normalized_temperature = 2.0f * (
                                 (temperature_k - law->minimum_temperature_k)
                              /  temperature_range_k
                            ) - 1.0f;

    /*
     * The explicit temperature-domain check above guarantees that the exact
     * mathematical normalized coordinate is in [-1, 1]. Floating-point
     * roundoff at an endpoint can still produce a value a few ulps outside,
     * so clamp only that already-validated coordinate before the recurrence.
     */
    if (normalized_temperature < -1.0f)
        normalized_temperature = -1.0f;

    else if (normalized_temperature > 1.0f)
        normalized_temperature = 1.0f;

    temperature_scale_per_k = 2.0f / temperature_range_k;

    /*
     * Backward Clenshaw evaluation starts with b_(N) = b_(N+1) = 0 and
     * evaluates coefficients c_(N-1) through c_1. The c_0 term is applied in
     * the final reconstruction, so the public coefficient convention remains
     * exactly sum(c_k * T_k(x)) with no hidden half weighting.
     */
    for (size_t coefficient_index = law->coefficient_count - 1u;
         coefficient_index > 0u;
         --coefficient_index)
    {
        const float current_value =
            2.0f * normalized_temperature * value_next
            - value_next_next
            + law->second_density_virial_chebyshev_coefficients_m3_per_kg[ coefficient_index ];

        const float current_first =
            2.0f * value_next
            + 2.0f * normalized_temperature * first_next
            - first_next_next;

        const float current_second =
            4.0f * first_next
            + 2.0f * normalized_temperature * second_next
            - second_next_next;

        value_next_next = value_next;
        value_next      = current_value;

        first_next_next = first_next;
        first_next      = current_first;

        second_next_next = second_next;
        second_next      = current_second;
    }

    polynomial_value =
        normalized_temperature * value_next
        - value_next_next
        + law->second_density_virial_chebyshev_coefficients_m3_per_kg[0];

    first_derivative_wrt_x =
        value_next
        + normalized_temperature * first_next
        - first_next_next;

    second_derivative_wrt_x =
        2.0f * first_next
        + normalized_temperature * second_next
        - second_next_next;

    terms->second_density_virial_coefficient_m3_per_kg = polynomial_value;

    if (first_derivative_wrt_x != 0.0f)
    {
        terms->first_temperature_derivative_m3_per_kg_k =
            first_derivative_wrt_x * temperature_scale_per_k;
    }

    if (second_derivative_wrt_x != 0.0f)
    {
        terms->second_temperature_derivative_m3_per_kg_k2 =
            second_derivative_wrt_x
            * temperature_scale_per_k
            * temperature_scale_per_k;
    }

    if (!isfinite(terms->second_density_virial_coefficient_m3_per_kg) ||
        !isfinite(terms->first_temperature_derivative_m3_per_kg_k)    ||
        !isfinite(terms->second_temperature_derivative_m3_per_kg_k2))
    {
        *terms = (bbtc_ib_first_order_virial_temperature_terms_float_t){0};
        return BBTC_STATUS_NUMERICAL_FAILURE;
    }

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e
bbtc_ib_first_order_virial_gas_model_validate_float(
    const bbtc_ib_first_order_virial_gas_model_float_t* const model
)
{
    bbtc_status_e status;

    if (model == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (!isfinite(model->specific_gas_constant_j_per_kg_k)                   ||
        !isfinite(model->ideal_gas_constant_volume_specific_heat_j_per_kg_k) ||
        !isfinite(model->minimum_calibrated_density_kg_per_m3)               ||
        !isfinite(model->maximum_calibrated_density_kg_per_m3))
    {
        return BBTC_STATUS_NONFINITE_INPUT;
    }

    if (model->specific_gas_constant_j_per_kg_k                   <= 0.0f ||
        model->ideal_gas_constant_volume_specific_heat_j_per_kg_k <= 0.0f ||
        model->minimum_calibrated_density_kg_per_m3               < 0.0f  ||
        model->maximum_calibrated_density_kg_per_m3
            <= model->minimum_calibrated_density_kg_per_m3)
    {
        return BBTC_STATUS_OUTSIDE_DOMAIN;
    }

    status = bbtc_ib_first_order_virial_temperature_law_validate_float(
                 &model->second_density_virial_coefficient_law);

    if (status != BBTC_STATUS_SUCCESS)
        return status;

    return BBTC_STATUS_SUCCESS;
}

bbtc_status_e
bbtc_ib_first_order_virial_temperature_law_validate_double(
    const bbtc_ib_first_order_virial_temperature_law_double_t* const law
)
{
    if (law == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (law->second_density_virial_chebyshev_coefficients_m3_per_kg == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (!isfinite(law->minimum_temperature_k) ||
        !isfinite(law->maximum_temperature_k))
    {
        return BBTC_STATUS_NONFINITE_INPUT;
    }

    if (law->minimum_temperature_k <= 0.0                        ||
        law->maximum_temperature_k <= law->minimum_temperature_k ||
        law->coefficient_count     == 0u)
    {
        return BBTC_STATUS_OUTSIDE_DOMAIN;
    }

    for (size_t coefficient_index = 0u;
         coefficient_index < law->coefficient_count;
         ++coefficient_index)
    {
        if (!isfinite(law->second_density_virial_chebyshev_coefficients_m3_per_kg[ coefficient_index ]))
        {
            return BBTC_STATUS_NONFINITE_INPUT;
        }
    }

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e
bbtc_ib_first_order_virial_temperature_law_evaluate_double(
    const bbtc_ib_first_order_virial_temperature_law_double_t* const law,
    double temperature_k,
    bbtc_ib_first_order_virial_temperature_terms_double_t* const terms
)
{
    bbtc_status_e status;
    double        temperature_range_k;
    double        normalized_temperature;
    double        temperature_scale_per_k;

    /*
     * Clenshaw state for b_(k+1) and b_(k+2), plus the first and second
     * derivatives of those auxiliary values with respect to x.
     */
    double value_next       = 0.0;
    double value_next_next  = 0.0;
    double first_next       = 0.0;
    double first_next_next  = 0.0;
    double second_next      = 0.0;
    double second_next_next = 0.0;

    double polynomial_value;
    double first_derivative_wrt_x;
    double second_derivative_wrt_x;

    if (terms == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    *terms = (bbtc_ib_first_order_virial_temperature_terms_double_t){0};

    status = bbtc_ib_first_order_virial_temperature_law_validate_double(law);

    if (status != BBTC_STATUS_SUCCESS)
        return status;

    if (!isfinite(temperature_k))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (temperature_k < law->minimum_temperature_k ||
        temperature_k > law->maximum_temperature_k)
    {
        return BBTC_STATUS_OUTSIDE_DOMAIN;
    }

    temperature_range_k =
        law->maximum_temperature_k - law->minimum_temperature_k;

    normalized_temperature = 2.0 * (
                                (temperature_k - law->minimum_temperature_k)
                                / temperature_range_k
                           ) - 1.0;

    /*
     * The explicit temperature-domain check above guarantees that the exact
     * mathematical normalized coordinate is in [-1, 1]. Floating-point
     * roundoff at an endpoint can still produce a value a few ulps outside,
     * so clamp only that already-validated coordinate before the recurrence.
     */
    if (normalized_temperature < -1.0)
        normalized_temperature = -1.0;

    else if (normalized_temperature > 1.0)
        normalized_temperature = 1.0;

    temperature_scale_per_k = 2.0 / temperature_range_k;

    /*
     * Backward Clenshaw evaluation starts with b_(N) = b_(N+1) = 0 and
     * evaluates coefficients c_(N-1) through c_1. The c_0 term is applied in
     * the final reconstruction, so the public coefficient convention remains
     * exactly sum(c_k * T_k(x)) with no hidden half weighting.
     */
    for (size_t coefficient_index = law->coefficient_count - 1u;
         coefficient_index > 0u;
         --coefficient_index)
    {
        const double current_value =
            2.0 * normalized_temperature * value_next
            - value_next_next
            + law->second_density_virial_chebyshev_coefficients_m3_per_kg[
                coefficient_index];

        const double current_first =
            2.0 * value_next
            + 2.0 * normalized_temperature * first_next
            - first_next_next;

        const double current_second =
            4.0 * first_next
            + 2.0 * normalized_temperature * second_next
            - second_next_next;

        value_next_next = value_next;
        value_next = current_value;

        first_next_next = first_next;
        first_next = current_first;

        second_next_next = second_next;
        second_next = current_second;
    }

    polynomial_value =
        normalized_temperature * value_next
        - value_next_next
        + law->second_density_virial_chebyshev_coefficients_m3_per_kg[0];

    first_derivative_wrt_x =
        value_next
        + normalized_temperature * first_next
        - first_next_next;

    second_derivative_wrt_x =
        2.0 * first_next
        + normalized_temperature * second_next
        - second_next_next;

    terms->second_density_virial_coefficient_m3_per_kg = polynomial_value;

    if (first_derivative_wrt_x != 0.0)
    {
        terms->first_temperature_derivative_m3_per_kg_k =
            first_derivative_wrt_x * temperature_scale_per_k;
    }

    if (second_derivative_wrt_x != 0.0)
    {
        terms->second_temperature_derivative_m3_per_kg_k2 =
            second_derivative_wrt_x
            * temperature_scale_per_k
            * temperature_scale_per_k;
    }

    if (!isfinite(terms->second_density_virial_coefficient_m3_per_kg) ||
        !isfinite(terms->first_temperature_derivative_m3_per_kg_k)    ||
        !isfinite(terms->second_temperature_derivative_m3_per_kg_k2))
    {
        *terms = (bbtc_ib_first_order_virial_temperature_terms_double_t){0};
        return BBTC_STATUS_NUMERICAL_FAILURE;
    }

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e
bbtc_ib_first_order_virial_gas_model_validate_double(
    const bbtc_ib_first_order_virial_gas_model_double_t* const model
)
{
    bbtc_status_e status;

    if (model == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (!isfinite(model->specific_gas_constant_j_per_kg_k)                   ||
        !isfinite(model->ideal_gas_constant_volume_specific_heat_j_per_kg_k) ||
        !isfinite(model->minimum_calibrated_density_kg_per_m3)               ||
        !isfinite(model->maximum_calibrated_density_kg_per_m3))
    {
        return BBTC_STATUS_NONFINITE_INPUT;
    }

    if (model->specific_gas_constant_j_per_kg_k                   <= 0.0 ||
        model->ideal_gas_constant_volume_specific_heat_j_per_kg_k <= 0.0 ||
        model->minimum_calibrated_density_kg_per_m3               <  0.0 ||
        model->maximum_calibrated_density_kg_per_m3
            <= model->minimum_calibrated_density_kg_per_m3)
    {
        return BBTC_STATUS_OUTSIDE_DOMAIN;
    }

    status = bbtc_ib_first_order_virial_temperature_law_validate_double(
                 &model->second_density_virial_coefficient_law);

    if (status != BBTC_STATUS_SUCCESS)
        return status;

    return BBTC_STATUS_SUCCESS;
}

bbtc_status_e
bbtc_ib_first_order_virial_temperature_law_validate_long_double(
    const bbtc_ib_first_order_virial_temperature_law_long_double_t* const law
)
{
    if (law == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (law->second_density_virial_chebyshev_coefficients_m3_per_kg == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (!isfinite(law->minimum_temperature_k) ||
        !isfinite(law->maximum_temperature_k))
    {
        return BBTC_STATUS_NONFINITE_INPUT;
    }

    if (law->minimum_temperature_k <= 0.0L                       ||
        law->maximum_temperature_k <= law->minimum_temperature_k ||
        law->coefficient_count     == 0u)
    {
        return BBTC_STATUS_OUTSIDE_DOMAIN;
    }

    for (size_t coefficient_index = 0u;
         coefficient_index < law->coefficient_count;
         ++coefficient_index)
    {
        if (!isfinite(law->second_density_virial_chebyshev_coefficients_m3_per_kg[ coefficient_index ]))
        {
            return BBTC_STATUS_NONFINITE_INPUT;
        }
    }

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e
bbtc_ib_first_order_virial_temperature_law_evaluate_long_double(
    const bbtc_ib_first_order_virial_temperature_law_long_double_t* const law,
    long double temperature_k,
    bbtc_ib_first_order_virial_temperature_terms_long_double_t* const terms
)
{
    bbtc_status_e status;
    long double   temperature_range_k;
    long double   normalized_temperature;
    long double   temperature_scale_per_k;

    /*
     * Clenshaw state for b_(k+1) and b_(k+2), plus the first and second
     * derivatives of those auxiliary values with respect to x.
     */
    long double value_next       = 0.0L;
    long double value_next_next  = 0.0L;
    long double first_next       = 0.0L;
    long double first_next_next  = 0.0L;
    long double second_next      = 0.0L;
    long double second_next_next = 0.0L;

    long double polynomial_value;
    long double first_derivative_wrt_x;
    long double second_derivative_wrt_x;

    if (terms == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    *terms = (bbtc_ib_first_order_virial_temperature_terms_long_double_t){0};

    status = bbtc_ib_first_order_virial_temperature_law_validate_long_double(law);

    if (status != BBTC_STATUS_SUCCESS)
        return status;

    if (!isfinite(temperature_k))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (temperature_k < law->minimum_temperature_k ||
        temperature_k > law->maximum_temperature_k)
    {
        return BBTC_STATUS_OUTSIDE_DOMAIN;
    }

    temperature_range_k =
        law->maximum_temperature_k - law->minimum_temperature_k;

    normalized_temperature = 2.0L * (
                                (temperature_k - law->minimum_temperature_k)
                                / temperature_range_k
                            ) - 1.0L;

    /*
     * The explicit temperature-domain check above guarantees that the exact
     * mathematical normalized coordinate is in [-1, 1]. Floating-point
     * roundoff at an endpoint can still produce a value a few ulps outside,
     * so clamp only that already-validated coordinate before the recurrence.
     */
    if (normalized_temperature < -1.0L)
        normalized_temperature = -1.0L;

    else if (normalized_temperature > 1.0L)
        normalized_temperature = 1.0L;

    temperature_scale_per_k = 2.0L / temperature_range_k;

    /*
     * Backward Clenshaw evaluation starts with b_(N) = b_(N+1) = 0 and
     * evaluates coefficients c_(N-1) through c_1. The c_0 term is applied in
     * the final reconstruction, so the public coefficient convention remains
     * exactly sum(c_k * T_k(x)) with no hidden half weighting.
     */
    for (size_t coefficient_index = law->coefficient_count - 1u;
         coefficient_index > 0u;
         --coefficient_index)
    {
        const long double current_value =
            2.0L * normalized_temperature * value_next
            - value_next_next
            + law->second_density_virial_chebyshev_coefficients_m3_per_kg[
                coefficient_index];

        const long double current_first =
            2.0L * value_next
            + 2.0L * normalized_temperature * first_next
            - first_next_next;

        const long double current_second =
            4.0L * first_next
            + 2.0L * normalized_temperature * second_next
            - second_next_next;

        value_next_next = value_next;
        value_next = current_value;

        first_next_next = first_next;
        first_next = current_first;

        second_next_next = second_next;
        second_next = current_second;
    }

    polynomial_value =
        normalized_temperature * value_next
        - value_next_next
        + law->second_density_virial_chebyshev_coefficients_m3_per_kg[0];

    first_derivative_wrt_x =
        value_next
        + normalized_temperature * first_next
        - first_next_next;

    second_derivative_wrt_x =
        2.0L * first_next
        + normalized_temperature * second_next
        - second_next_next;

    terms->second_density_virial_coefficient_m3_per_kg = polynomial_value;

    if (first_derivative_wrt_x != 0.0L)
    {
        terms->first_temperature_derivative_m3_per_kg_k =
            first_derivative_wrt_x * temperature_scale_per_k;
    }

    if (second_derivative_wrt_x != 0.0L)
    {
        terms->second_temperature_derivative_m3_per_kg_k2 =
            second_derivative_wrt_x
            * temperature_scale_per_k
            * temperature_scale_per_k;
    }

    if (!isfinite(terms->second_density_virial_coefficient_m3_per_kg) ||
        !isfinite(terms->first_temperature_derivative_m3_per_kg_k)    ||
        !isfinite(terms->second_temperature_derivative_m3_per_kg_k2))
    {
        *terms = (bbtc_ib_first_order_virial_temperature_terms_long_double_t){0};
        return BBTC_STATUS_NUMERICAL_FAILURE;
    }

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e
bbtc_ib_first_order_virial_gas_model_validate_long_double(
    const bbtc_ib_first_order_virial_gas_model_long_double_t* const model
)
{
    bbtc_status_e status;

    if (model == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (!isfinite(model->specific_gas_constant_j_per_kg_k)                   ||
        !isfinite(model->ideal_gas_constant_volume_specific_heat_j_per_kg_k) ||
        !isfinite(model->minimum_calibrated_density_kg_per_m3)               ||
        !isfinite(model->maximum_calibrated_density_kg_per_m3))
    {
        return BBTC_STATUS_NONFINITE_INPUT;
    }

    if (model->specific_gas_constant_j_per_kg_k                   <= 0.0L ||
        model->ideal_gas_constant_volume_specific_heat_j_per_kg_k <= 0.0L ||
        model->minimum_calibrated_density_kg_per_m3               <  0.0L ||
        model->maximum_calibrated_density_kg_per_m3
            <= model->minimum_calibrated_density_kg_per_m3)
    {
        return BBTC_STATUS_OUTSIDE_DOMAIN;
    }

    status = bbtc_ib_first_order_virial_temperature_law_validate_long_double(
                 &model->second_density_virial_coefficient_law);

    if (status != BBTC_STATUS_SUCCESS)
        return status;

    return BBTC_STATUS_SUCCESS;
}
