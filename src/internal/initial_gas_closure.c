/**
 * @file
 * @brief Initial free-gas mass/density closure for reduced gas backends.
 *
 * @details
 * This translation unit solves the initial mechanical gas state only. It does
 * not evaluate caloric state, represent combustion, integrate projectile
 * motion, or assess real ammunition/firearm safety.
 */
#include <stddef.h>
#include <math.h>

#include "bbtc/internal_ballistics/initial_gas_closure.h"


static bbtc_status_e
positive_ratio_over_product_float(float numerator,
                                  float factor_a,
                                  float factor_b,
                                  float* const result)
{
    int   numerator_exponent;
    int   factor_a_exponent;
    int   factor_b_exponent;
    long  result_exponent;
    float numerator_mantissa;
    float factor_a_mantissa;
    float factor_b_mantissa;
    float result_mantissa;
    float value;

    numerator_mantissa = frexpf(numerator, &numerator_exponent);
    factor_a_mantissa  = frexpf(factor_a, &factor_a_exponent);
    factor_b_mantissa  = frexpf(factor_b, &factor_b_exponent);

    result_mantissa =
        numerator_mantissa / (factor_a_mantissa * factor_b_mantissa);

    if (!isfinite(result_mantissa) || result_mantissa <= 0.0f)
        return BBTC_STATUS_NUMERICAL_FAILURE;

    result_exponent =
        (long)numerator_exponent
        - (long)factor_a_exponent
        - (long)factor_b_exponent;

    /* result_mantissa * (2 ^ result_exponent) */
    value = scalblnf(result_mantissa, result_exponent);

    if (!isfinite(value) || value <= 0.0f)
        return BBTC_STATUS_NUMERICAL_FAILURE;

    *result = value;
    return BBTC_STATUS_SUCCESS;
}

static bbtc_status_e
positive_ratio_over_product_double(double numerator,
                                   double factor_a,
                                   double factor_b,
                                   double* const result)
{
    int    numerator_exponent;
    int    factor_a_exponent;
    int    factor_b_exponent;
    long   result_exponent;
    double numerator_mantissa;
    double factor_a_mantissa;
    double factor_b_mantissa;
    double result_mantissa;
    double value;

    numerator_mantissa = frexp(numerator, &numerator_exponent);
    factor_a_mantissa  = frexp(factor_a, &factor_a_exponent);
    factor_b_mantissa  = frexp(factor_b, &factor_b_exponent);

    result_mantissa =
        numerator_mantissa / (factor_a_mantissa * factor_b_mantissa);

    if (!isfinite(result_mantissa) || result_mantissa <= 0.0)
        return BBTC_STATUS_NUMERICAL_FAILURE;

    result_exponent =
        (long)numerator_exponent
        - (long)factor_a_exponent
        - (long)factor_b_exponent;

    /* result_mantissa * (2 ^ result_exponent) */
    value = scalbln(result_mantissa, result_exponent);

    if (!isfinite(value) || value <= 0.0)
        return BBTC_STATUS_NUMERICAL_FAILURE;

    *result = value;
    return BBTC_STATUS_SUCCESS;
}

static bbtc_status_e
positive_ratio_over_product_long_double(long double numerator,
                                        long double factor_a,
                                        long double factor_b,
                                        long double* const result)
{
    int         numerator_exponent;
    int         factor_a_exponent;
    int         factor_b_exponent;
    long        result_exponent;
    long double numerator_mantissa;
    long double factor_a_mantissa;
    long double factor_b_mantissa;
    long double result_mantissa;
    long double value;

    numerator_mantissa = frexpl(numerator, &numerator_exponent);
    factor_a_mantissa  = frexpl(factor_a, &factor_a_exponent);
    factor_b_mantissa  = frexpl(factor_b, &factor_b_exponent);

    result_mantissa =
        numerator_mantissa / (factor_a_mantissa * factor_b_mantissa);

    if (!isfinite(result_mantissa) || result_mantissa <= 0.0L)
        return BBTC_STATUS_NUMERICAL_FAILURE;

    result_exponent =
        (long)numerator_exponent
        - (long)factor_a_exponent
        - (long)factor_b_exponent;

    /* result_mantissa * (2 ^ result_exponent) */
    value = scalblnl(result_mantissa, result_exponent);

    if (!isfinite(value) || value <= 0.0L)
        return BBTC_STATUS_NUMERICAL_FAILURE;

    *result = value;
    return BBTC_STATUS_SUCCESS;
}

static bbtc_status_e
positive_sqrt_product_float(float factor_a,
                            float factor_b,
                            float* const result)
{
    int   factor_a_exponent;
    int   factor_b_exponent;
    long  product_exponent;
    float factor_a_mantissa;
    float factor_b_mantissa;
    float product_mantissa;
    float value;

    factor_a_mantissa = frexpf(factor_a, &factor_a_exponent);
    factor_b_mantissa = frexpf(factor_b, &factor_b_exponent);
    product_mantissa  = factor_a_mantissa * factor_b_mantissa;
    product_exponent  = (long)factor_a_exponent + (long)factor_b_exponent;

    if ((product_exponent % 2L) != 0L)
    {
        product_mantissa *= 2.0f;
        product_exponent -= 1L;
    }

    value = scalblnf(sqrtf(product_mantissa), product_exponent / 2L);

    if (!isfinite(value) || value <= 0.0f)
        return BBTC_STATUS_NUMERICAL_FAILURE;

    *result = value;
    return BBTC_STATUS_SUCCESS;
}

static bbtc_status_e
positive_sqrt_product_double(double factor_a,
                             double factor_b,
                             double* const result)
{
    int    factor_a_exponent;
    int    factor_b_exponent;
    long   product_exponent;
    double factor_a_mantissa;
    double factor_b_mantissa;
    double product_mantissa;
    double value;

    factor_a_mantissa = frexp(factor_a, &factor_a_exponent);
    factor_b_mantissa = frexp(factor_b, &factor_b_exponent);
    product_mantissa  = factor_a_mantissa * factor_b_mantissa;
    product_exponent  = (long)factor_a_exponent + (long)factor_b_exponent;

    if ((product_exponent % 2L) != 0L)
    {
        product_mantissa *= 2.0;
        product_exponent -= 1L;
    }

    value = scalbln(sqrt(product_mantissa), product_exponent / 2L);

    if (!isfinite(value) || value <= 0.0)
        return BBTC_STATUS_NUMERICAL_FAILURE;

    *result = value;
    return BBTC_STATUS_SUCCESS;
}

static bbtc_status_e
positive_sqrt_product_long_double(long double factor_a,
                                  long double factor_b,
                                  long double* const result)
{
    int         factor_a_exponent;
    int         factor_b_exponent;
    long        product_exponent;
    long double factor_a_mantissa;
    long double factor_b_mantissa;
    long double product_mantissa;
    long double value;

    factor_a_mantissa = frexpl(factor_a, &factor_a_exponent);
    factor_b_mantissa = frexpl(factor_b, &factor_b_exponent);
    product_mantissa  = factor_a_mantissa * factor_b_mantissa;
    product_exponent  = (long)factor_a_exponent + (long)factor_b_exponent;

    if ((product_exponent % 2L) != 0L)
    {
        product_mantissa *= 2.0L;
        product_exponent -= 1L;
    }

    value = scalblnl(sqrtl(product_mantissa), product_exponent / 2L);

    if (!isfinite(value) || value <= 0.0L)
        return BBTC_STATUS_NUMERICAL_FAILURE;

    *result = value;
    return BBTC_STATUS_SUCCESS;
}

bbtc_status_e
bbtc_ib_noble_abel_initial_gas_solve_float(
    const bbtc_ib_noble_abel_gas_model_float_t* const model,
    const bbtc_ib_initial_gas_state_float_t* const initial_gas_state,
    float initial_free_gas_volume_m3,
    bbtc_ib_initial_gas_solution_float_t* const solution
)
{
    bbtc_status_e status;
    float         reduced_density;
    float         b_q;
    float         denominator;
    float         inverse_q;
    float         density_kg_per_m3;
    float         b_density;
    float         gas_mass_kg;

    if (solution == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    *solution = (bbtc_ib_initial_gas_solution_float_t){0};

    status = bbtc_ib_noble_abel_gas_model_validate_float(model);
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    status = bbtc_ib_initial_gas_state_validate_float(initial_gas_state);
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    if (isnan(initial_free_gas_volume_m3))
        return BBTC_STATUS_NAN_INPUT;

    if (isinf(initial_free_gas_volume_m3))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (initial_free_gas_volume_m3 <= 0.0f)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    status = positive_ratio_over_product_float(
        initial_gas_state->absolute_pressure_pa,
        model->specific_gas_constant_j_per_kg_k,
        initial_gas_state->temperature_k,
        &reduced_density
    );
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    if (model->covolume_m3_per_kg == 0.0f)
        /* Exact ideal-gas limit: rho = p / (R * T). */
        density_kg_per_m3 = reduced_density;

    else
    {
        b_q = model->covolume_m3_per_kg * reduced_density;

        if (isfinite(b_q))
        {
            denominator = fmaf(
                model->covolume_m3_per_kg,
                reduced_density,
                1.0f
            );

            if (!isfinite(denominator) || denominator <= 0.0f)
                return BBTC_STATUS_NUMERICAL_FAILURE;

            density_kg_per_m3 = reduced_density / denominator;
        }
        else
        {
            /*
             * Algebraically equivalent reciprocal form:
             * rho = 1 / (b + 1/q).
             */
            inverse_q = 1.0f / reduced_density;

            if (!isfinite(inverse_q))
                return BBTC_STATUS_NUMERICAL_FAILURE;

            denominator = model->covolume_m3_per_kg + inverse_q;

            if (!isfinite(denominator) || denominator <= 0.0f)
                return BBTC_STATUS_NUMERICAL_FAILURE;

            density_kg_per_m3 = 1.0f / denominator;
        }
    }

    if (!isfinite(density_kg_per_m3) || density_kg_per_m3 <= 0.0f)
        return BBTC_STATUS_NUMERICAL_FAILURE;

    if (model->covolume_m3_per_kg > 0.0f)
    {
        b_density = model->covolume_m3_per_kg * density_kg_per_m3;

        if (!isfinite(b_density) || b_density >= 1.0f)
            return BBTC_STATUS_NUMERICAL_FAILURE;
    }

    gas_mass_kg = density_kg_per_m3 * initial_free_gas_volume_m3;

    if (!isfinite(gas_mass_kg) || gas_mass_kg <= 0.0f)
        return BBTC_STATUS_NUMERICAL_FAILURE;

    solution->density_kg_per_m3 = density_kg_per_m3;
    solution->gas_mass_kg       = gas_mass_kg;

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e
bbtc_ib_noble_abel_initial_gas_solve_double(
    const bbtc_ib_noble_abel_gas_model_double_t* const model,
    const bbtc_ib_initial_gas_state_double_t* const initial_gas_state,
    double initial_free_gas_volume_m3,
    bbtc_ib_initial_gas_solution_double_t* const solution
)
{
    bbtc_status_e status;
    double        reduced_density;
    double        b_q;
    double        denominator;
    double        inverse_q;
    double        density_kg_per_m3;
    double        b_density;
    double        gas_mass_kg;

    if (solution == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    *solution = (bbtc_ib_initial_gas_solution_double_t){0};

    status = bbtc_ib_noble_abel_gas_model_validate_double(model);
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    status = bbtc_ib_initial_gas_state_validate_double(initial_gas_state);
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    if (isnan(initial_free_gas_volume_m3))
        return BBTC_STATUS_NAN_INPUT;

    if (isinf(initial_free_gas_volume_m3))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (initial_free_gas_volume_m3 <= 0.0)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    status = positive_ratio_over_product_double(
        initial_gas_state->absolute_pressure_pa,
        model->specific_gas_constant_j_per_kg_k,
        initial_gas_state->temperature_k,
        &reduced_density
    );
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    if (model->covolume_m3_per_kg == 0.0)
        /* Exact ideal-gas limit: rho = p / (R * T). */
        density_kg_per_m3 = reduced_density;

    else
    {
        b_q = model->covolume_m3_per_kg * reduced_density;

        if (isfinite(b_q))
        {
            denominator = fma(
                model->covolume_m3_per_kg,
                reduced_density,
                1.0
            );

            if (!isfinite(denominator) || denominator <= 0.0)
                return BBTC_STATUS_NUMERICAL_FAILURE;

            density_kg_per_m3 = reduced_density / denominator;
        }
        else
        {
            /*
             * Algebraically equivalent reciprocal form:
             * rho = 1 / (b + 1/q).
             */
            inverse_q = 1.0 / reduced_density;

            if (!isfinite(inverse_q))
                return BBTC_STATUS_NUMERICAL_FAILURE;

            denominator = model->covolume_m3_per_kg + inverse_q;

            if (!isfinite(denominator) || denominator <= 0.0)
                return BBTC_STATUS_NUMERICAL_FAILURE;

            density_kg_per_m3 = 1.0 / denominator;
        }
    }

    if (!isfinite(density_kg_per_m3) || density_kg_per_m3 <= 0.0)
        return BBTC_STATUS_NUMERICAL_FAILURE;

    if (model->covolume_m3_per_kg > 0.0)
    {
        b_density = model->covolume_m3_per_kg * density_kg_per_m3;

        if (!isfinite(b_density) || b_density >= 1.0)
            return BBTC_STATUS_NUMERICAL_FAILURE;
    }

    gas_mass_kg = density_kg_per_m3 * initial_free_gas_volume_m3;

    if (!isfinite(gas_mass_kg) || gas_mass_kg <= 0.0)
        return BBTC_STATUS_NUMERICAL_FAILURE;

    solution->density_kg_per_m3 = density_kg_per_m3;
    solution->gas_mass_kg       = gas_mass_kg;

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e
bbtc_ib_noble_abel_initial_gas_solve_long_double(
    const bbtc_ib_noble_abel_gas_model_long_double_t* const model,
    const bbtc_ib_initial_gas_state_long_double_t* const initial_gas_state,
    long double initial_free_gas_volume_m3,
    bbtc_ib_initial_gas_solution_long_double_t* const solution
)
{
    bbtc_status_e status;
    long double   reduced_density;
    long double   b_q;
    long double   denominator;
    long double   inverse_q;
    long double   density_kg_per_m3;
    long double   b_density;
    long double   gas_mass_kg;

    if (solution == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    *solution = (bbtc_ib_initial_gas_solution_long_double_t){0};

    status = bbtc_ib_noble_abel_gas_model_validate_long_double(model);
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    status = bbtc_ib_initial_gas_state_validate_long_double(initial_gas_state);
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    if (isnan(initial_free_gas_volume_m3))
        return BBTC_STATUS_NAN_INPUT;

    if (isinf(initial_free_gas_volume_m3))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (initial_free_gas_volume_m3 <= 0.0L)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    status = positive_ratio_over_product_long_double(
        initial_gas_state->absolute_pressure_pa,
        model->specific_gas_constant_j_per_kg_k,
        initial_gas_state->temperature_k,
        &reduced_density
    );
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    if (model->covolume_m3_per_kg == 0.0L)
        /* Exact ideal-gas limit: rho = p / (R * T). */
        density_kg_per_m3 = reduced_density;

    else
    {
        b_q = model->covolume_m3_per_kg * reduced_density;

        if (isfinite(b_q))
        {
            denominator = fmal(
                model->covolume_m3_per_kg,
                reduced_density,
                1.0L
            );
            if (!isfinite(denominator) || denominator <= 0.0L)
                return BBTC_STATUS_NUMERICAL_FAILURE;

            density_kg_per_m3 = reduced_density / denominator;
        }
        else
        {
            /*
             * Algebraically equivalent reciprocal form:
             * rho = 1 / (b + 1/q).
             */
            inverse_q = 1.0L / reduced_density;

            if (!isfinite(inverse_q))
                return BBTC_STATUS_NUMERICAL_FAILURE;

            denominator = model->covolume_m3_per_kg + inverse_q;

            if (!isfinite(denominator) || denominator <= 0.0L)
                return BBTC_STATUS_NUMERICAL_FAILURE;

            density_kg_per_m3 = 1.0L / denominator;
        }
    }

    if (!isfinite(density_kg_per_m3) || density_kg_per_m3 <= 0.0L)
        return BBTC_STATUS_NUMERICAL_FAILURE;

    if (model->covolume_m3_per_kg > 0.0L)
    {
        b_density = model->covolume_m3_per_kg * density_kg_per_m3;

        if (!isfinite(b_density) || b_density >= 1.0L)
            return BBTC_STATUS_NUMERICAL_FAILURE;
    }

    gas_mass_kg = density_kg_per_m3 * initial_free_gas_volume_m3;

    if (!isfinite(gas_mass_kg) || gas_mass_kg <= 0.0L)
        return BBTC_STATUS_NUMERICAL_FAILURE;

    solution->density_kg_per_m3 = density_kg_per_m3;
    solution->gas_mass_kg       = gas_mass_kg;

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e
bbtc_ib_first_order_virial_initial_gas_solve_float(
    const bbtc_ib_first_order_virial_gas_model_float_t* const model,
    const bbtc_ib_initial_gas_state_float_t* const initial_gas_state,
    float initial_free_gas_volume_m3,
    bbtc_ib_initial_gas_solution_float_t* const solution
)
{
    bbtc_applicability_flags_t applicability_flags = BBTC_APPLICABILITY_NONE_REPORTED;

    bbtc_status_e status;
    bbtc_ib_first_order_virial_temperature_terms_float_t virial_terms;
    float q;
    float b;
    float b_q;
    float discriminant;
    float root_term;
    float denominator;
    float sqrt_product;
    float density_kg_per_m3;
    float b_density;
    float gas_mass_kg;

    if (solution == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    *solution = (bbtc_ib_initial_gas_solution_float_t){0};

    status = bbtc_ib_first_order_virial_gas_model_validate_float(model);
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    status = bbtc_ib_initial_gas_state_validate_float(initial_gas_state);
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    if (isnan(initial_free_gas_volume_m3))
        return BBTC_STATUS_NAN_INPUT;

    if (isinf(initial_free_gas_volume_m3))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (initial_free_gas_volume_m3 <= 0.0f)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    status = bbtc_ib_first_order_virial_temperature_law_evaluate_float(
        &model->second_density_virial_coefficient_law,
        initial_gas_state->temperature_k,
        &virial_terms
    );
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    status = positive_ratio_over_product_float(
        initial_gas_state->absolute_pressure_pa,
        model->specific_gas_constant_j_per_kg_k,
        initial_gas_state->temperature_k,
        &q
    );
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    b = virial_terms.second_density_virial_coefficient_m3_per_kg;

    if (b == 0.0f)
        /* Exact ideal-gas limit. */
        density_kg_per_m3 = q;

    else if (b < 0.0f)
    {
        /*
         * The stable negative-B branch requires B*q > -1/4. If this
         * product overflows negatively, the exact state is already outside
         * that domain.
         */
        b_q = b * q;

        if (!isfinite(b_q) || b_q <= -0.25f)
            return BBTC_STATUS_OUTSIDE_DOMAIN;

        discriminant = fmaf(4.0f, b_q, 1.0f);

        if (!isfinite(discriminant) || discriminant <= 0.0f)
            return BBTC_STATUS_NUMERICAL_FAILURE;

        root_term   = sqrtf(discriminant);
        denominator = (1.0f + root_term) * 0.5f;

        density_kg_per_m3 = q / denominator;
    }
    else
    {
        /*
         * For positive B, evaluate sqrt(B*q) without forming B*q
         * directly, then use the stable positive-root form
         * rho = q / (1/2 + hypot(1/2, sqrt(B*q))).
         */
        status = positive_sqrt_product_float(b, q, &sqrt_product);
        if (status != BBTC_STATUS_SUCCESS)
            return status;

        root_term   = hypotf(0.5f, sqrt_product);
        denominator = 0.5f + root_term;

        density_kg_per_m3 = q / denominator;
    }

    if (!isfinite(density_kg_per_m3) || density_kg_per_m3 <= 0.0f)
        return BBTC_STATUS_NUMERICAL_FAILURE;

    if (b < 0.0f)
    {
        b_density = b * density_kg_per_m3;

        if (!isfinite(b_density) || b_density <= -0.5f)
            return BBTC_STATUS_NUMERICAL_FAILURE;
    }

    if (density_kg_per_m3 < model->minimum_calibrated_density_kg_per_m3 ||
        density_kg_per_m3 > model->maximum_calibrated_density_kg_per_m3)
    {
        applicability_flags |= BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN;
    }

    gas_mass_kg = density_kg_per_m3 * initial_free_gas_volume_m3;

    if (!isfinite(gas_mass_kg) || gas_mass_kg <= 0.0f)
        return BBTC_STATUS_NUMERICAL_FAILURE;

    solution->applicability_flags = applicability_flags;
    solution->density_kg_per_m3   = density_kg_per_m3;
    solution->gas_mass_kg         = gas_mass_kg;

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e
bbtc_ib_first_order_virial_initial_gas_solve_double(
    const bbtc_ib_first_order_virial_gas_model_double_t* const model,
    const bbtc_ib_initial_gas_state_double_t* const initial_gas_state,
    double initial_free_gas_volume_m3,
    bbtc_ib_initial_gas_solution_double_t* const solution
)
{
    bbtc_applicability_flags_t applicability_flags = BBTC_APPLICABILITY_NONE_REPORTED;
    bbtc_status_e status;
    bbtc_ib_first_order_virial_temperature_terms_double_t virial_terms;
    double q;
    double b;
    double b_q;
    double discriminant;
    double root_term;
    double denominator;
    double sqrt_product;
    double density_kg_per_m3;
    double b_density;
    double gas_mass_kg;

    if (solution == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    *solution = (bbtc_ib_initial_gas_solution_double_t){0};

    status = bbtc_ib_first_order_virial_gas_model_validate_double(model);
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    status = bbtc_ib_initial_gas_state_validate_double(initial_gas_state);
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    if (isnan(initial_free_gas_volume_m3))
        return BBTC_STATUS_NAN_INPUT;

    if (isinf(initial_free_gas_volume_m3))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (initial_free_gas_volume_m3 <= 0.0)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    status = bbtc_ib_first_order_virial_temperature_law_evaluate_double(
        &model->second_density_virial_coefficient_law,
        initial_gas_state->temperature_k,
        &virial_terms
    );
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    status = positive_ratio_over_product_double(
        initial_gas_state->absolute_pressure_pa,
        model->specific_gas_constant_j_per_kg_k,
        initial_gas_state->temperature_k,
        &q
    );
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    b = virial_terms.second_density_virial_coefficient_m3_per_kg;

    if (b == 0.0)
        /* Exact ideal-gas limit. */
        density_kg_per_m3 = q;

    else if (b < 0.0)
    {
        /*
         * The stable negative-B branch requires B*q > -1/4. If this
         * product overflows negatively, the exact state is already outside
         * that domain.
         */
        b_q = b * q;

        if (!isfinite(b_q) || b_q <= -0.25)
            return BBTC_STATUS_OUTSIDE_DOMAIN;

        discriminant = fma(4.0, b_q, 1.0);

        if (!isfinite(discriminant) || discriminant <= 0.0)
            return BBTC_STATUS_NUMERICAL_FAILURE;

        root_term   = sqrt(discriminant);
        denominator = (1.0 + root_term) * 0.5;

        density_kg_per_m3 = q / denominator;
    }
    else
    {
        /*
         * For positive B, evaluate sqrt(B*q) without forming B*q
         * directly, then use the stable positive-root form
         * rho = q / (1/2 + hypot(1/2, sqrt(B*q))).
         */
        status = positive_sqrt_product_double(b, q, &sqrt_product);
        if (status != BBTC_STATUS_SUCCESS)
            return status;

        root_term   = hypot(0.5, sqrt_product);
        denominator = 0.5 + root_term;

        density_kg_per_m3 = q / denominator;
    }

    if (!isfinite(density_kg_per_m3) || density_kg_per_m3 <= 0.0)
        return BBTC_STATUS_NUMERICAL_FAILURE;

    if (b < 0.0)
    {
        b_density = b * density_kg_per_m3;

        if (!isfinite(b_density) || b_density <= -0.5)
            return BBTC_STATUS_NUMERICAL_FAILURE;
    }

    if (density_kg_per_m3 < model->minimum_calibrated_density_kg_per_m3 ||
        density_kg_per_m3 > model->maximum_calibrated_density_kg_per_m3)
    {
        applicability_flags |= BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN;
    }

    gas_mass_kg = density_kg_per_m3 * initial_free_gas_volume_m3;

    if (!isfinite(gas_mass_kg) || gas_mass_kg <= 0.0)
        return BBTC_STATUS_NUMERICAL_FAILURE;

    solution->applicability_flags = applicability_flags;
    solution->density_kg_per_m3   = density_kg_per_m3;
    solution->gas_mass_kg         = gas_mass_kg;

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e
bbtc_ib_first_order_virial_initial_gas_solve_long_double(
    const bbtc_ib_first_order_virial_gas_model_long_double_t* const model,
    const bbtc_ib_initial_gas_state_long_double_t* const initial_gas_state,
    long double initial_free_gas_volume_m3,
    bbtc_ib_initial_gas_solution_long_double_t* const solution
)
{
    bbtc_applicability_flags_t applicability_flags = BBTC_APPLICABILITY_NONE_REPORTED;
    bbtc_status_e status;
    bbtc_ib_first_order_virial_temperature_terms_long_double_t virial_terms;
    long double q;
    long double b;
    long double b_q;
    long double discriminant;
    long double root_term;
    long double denominator;
    long double sqrt_product;
    long double density_kg_per_m3;
    long double b_density;
    long double gas_mass_kg;

    if (solution == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    *solution = (bbtc_ib_initial_gas_solution_long_double_t){0};

    status = bbtc_ib_first_order_virial_gas_model_validate_long_double(model);
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    status = bbtc_ib_initial_gas_state_validate_long_double(initial_gas_state);
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    if (isnan(initial_free_gas_volume_m3))
        return BBTC_STATUS_NAN_INPUT;

    if (isinf(initial_free_gas_volume_m3))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (initial_free_gas_volume_m3 <= 0.0L)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    status = bbtc_ib_first_order_virial_temperature_law_evaluate_long_double(
        &model->second_density_virial_coefficient_law,
        initial_gas_state->temperature_k,
        &virial_terms
    );
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    status = positive_ratio_over_product_long_double(
        initial_gas_state->absolute_pressure_pa,
        model->specific_gas_constant_j_per_kg_k,
        initial_gas_state->temperature_k,
        &q
    );
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    b = virial_terms.second_density_virial_coefficient_m3_per_kg;

    if (b == 0.0L)
        /* Exact ideal-gas limit. */
        density_kg_per_m3 = q;

    else if (b < 0.0L)
    {
        /*
         * The stable negative-B branch requires B*q > -1/4. If this
         * product overflows negatively, the exact state is already outside
         * that domain.
         */
        b_q = b * q;

        if (!isfinite(b_q) || b_q <= -0.25L)
            return BBTC_STATUS_OUTSIDE_DOMAIN;

        discriminant = fmal(4.0L, b_q, 1.0L);

        if (!isfinite(discriminant) || discriminant <= 0.0L)
            return BBTC_STATUS_NUMERICAL_FAILURE;

        root_term   = sqrtl(discriminant);
        denominator = (1.0L + root_term) * 0.5L;

        density_kg_per_m3 = q / denominator;
    }
    else
    {
        /*
         * For positive B, evaluate sqrt(B*q) without forming B*q
         * directly, then use the stable positive-root form
         * rho = q / (1/2 + hypot(1/2, sqrt(B*q))).
         */
        status = positive_sqrt_product_long_double(b, q, &sqrt_product);
        if (status != BBTC_STATUS_SUCCESS)
            return status;

        root_term   = hypotl(0.5L, sqrt_product);
        denominator = 0.5L + root_term;

        density_kg_per_m3 = q / denominator;
    }

    if (!isfinite(density_kg_per_m3) || density_kg_per_m3 <= 0.0L)
        return BBTC_STATUS_NUMERICAL_FAILURE;

    if (b < 0.0L)
    {
        b_density = b * density_kg_per_m3;

        if (!isfinite(b_density) || b_density <= -0.5L)
            return BBTC_STATUS_NUMERICAL_FAILURE;
    }

    if (density_kg_per_m3 < model->minimum_calibrated_density_kg_per_m3 ||
        density_kg_per_m3 > model->maximum_calibrated_density_kg_per_m3)
    {
        applicability_flags |= BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN;
    }

    gas_mass_kg = density_kg_per_m3 * initial_free_gas_volume_m3;

    if (!isfinite(gas_mass_kg) || gas_mass_kg <= 0.0L)
        return BBTC_STATUS_NUMERICAL_FAILURE;

    solution->applicability_flags = applicability_flags;
    solution->density_kg_per_m3   = density_kg_per_m3;
    solution->gas_mass_kg         = gas_mass_kg;

    return BBTC_STATUS_SUCCESS;
}
