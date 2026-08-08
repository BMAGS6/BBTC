/**
 * @file
 * @brief Thermodynamic evaluation for BBTC's reduced gas-model backends.
 *
 * @details
 * The functions in this translation unit evaluate constitutive state only.
 * They do not infer gas mass, represent combustion, integrate projectile
 * motion, or assess real ammunition/firearm safety.
 */

#include "bbtc/internal_ballistics/reduced_gas_thermodynamics.h"

#include <stddef.h>
#include <math.h>

bbtc_status_e
bbtc_ib_noble_abel_thermodynamics_evaluate_float(
    const bbtc_ib_noble_abel_gas_model_float_t* const model,
    float density_kg_per_m3,
    float temperature_k,
    const bbtc_ib_caloric_reference_float_t* const caloric_reference,
    bbtc_ib_reduced_gas_thermodynamic_result_float_t* const result
)
{
    bbtc_status_e status;
    float covolume_density;
    float available_volume_factor;
    float temperature_offset_k;
    float pressure_pa;
    float specific_internal_energy_j_per_kg;
    float pressure_density_derivative;
    float pressure_temperature_derivative;

    if (result == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    *result = (bbtc_ib_reduced_gas_thermodynamic_result_float_t){0};

    status = bbtc_ib_noble_abel_gas_model_validate_float(model);
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    status = bbtc_ib_caloric_reference_validate_float(caloric_reference);
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    if (!isfinite(density_kg_per_m3) || !isfinite(temperature_k))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (density_kg_per_m3 < 0.0f || temperature_k <= 0.0f)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    covolume_density = model->covolume_m3_per_kg * density_kg_per_m3;
    if (!isfinite(covolume_density))
        return BBTC_STATUS_NUMERICAL_FAILURE;

    available_volume_factor = 1.0f - covolume_density;
    if (!isfinite(available_volume_factor))
        return BBTC_STATUS_NUMERICAL_FAILURE;

    if (available_volume_factor <= 0.0f)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    temperature_offset_k =
        temperature_k - caloric_reference->reference_temperature_k;

    pressure_pa =
        density_kg_per_m3
        * model->specific_gas_constant_j_per_kg_k
        * temperature_k
        / available_volume_factor;

    specific_internal_energy_j_per_kg =
        caloric_reference->reference_specific_internal_energy_j_per_kg
        + model->constant_volume_specific_heat_j_per_kg_k
        * temperature_offset_k;

    pressure_density_derivative =
        model->specific_gas_constant_j_per_kg_k
        * temperature_k
        / (available_volume_factor * available_volume_factor);

    pressure_temperature_derivative =
        density_kg_per_m3
        * model->specific_gas_constant_j_per_kg_k
        / available_volume_factor;

    if (!isfinite(temperature_offset_k)                ||
        !isfinite(pressure_pa)                         ||
        !isfinite(specific_internal_energy_j_per_kg)   ||
        !isfinite(pressure_density_derivative)         ||
        !isfinite(pressure_temperature_derivative))
    {
        return BBTC_STATUS_NUMERICAL_FAILURE;
    }

    result->pressure_pa = pressure_pa;

    result->specific_internal_energy_j_per_kg =
        specific_internal_energy_j_per_kg;

    result->constant_volume_specific_heat_j_per_kg_k =
        model->constant_volume_specific_heat_j_per_kg_k;

    result->pressure_density_derivative_at_constant_temperature_pa_m3_per_kg =
        pressure_density_derivative;

    result->pressure_temperature_derivative_at_constant_density_pa_per_k =
        pressure_temperature_derivative;

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e
bbtc_ib_noble_abel_thermodynamics_evaluate_double(
    const bbtc_ib_noble_abel_gas_model_double_t* const model,
    double density_kg_per_m3,
    double temperature_k,
    const bbtc_ib_caloric_reference_double_t* const caloric_reference,
    bbtc_ib_reduced_gas_thermodynamic_result_double_t* const result
)
{
    bbtc_status_e status;
    double covolume_density;
    double available_volume_factor;
    double temperature_offset_k;
    double pressure_pa;
    double specific_internal_energy_j_per_kg;
    double pressure_density_derivative;
    double pressure_temperature_derivative;

    if (result == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    *result = (bbtc_ib_reduced_gas_thermodynamic_result_double_t){0};

    status = bbtc_ib_noble_abel_gas_model_validate_double(model);
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    status = bbtc_ib_caloric_reference_validate_double(caloric_reference);
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    if (!isfinite(density_kg_per_m3) || !isfinite(temperature_k))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (density_kg_per_m3 < 0.0 || temperature_k <= 0.0)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    covolume_density = model->covolume_m3_per_kg * density_kg_per_m3;
    if (!isfinite(covolume_density))
        return BBTC_STATUS_NUMERICAL_FAILURE;

    available_volume_factor = 1.0 - covolume_density;
    if (!isfinite(available_volume_factor))
        return BBTC_STATUS_NUMERICAL_FAILURE;

    if (available_volume_factor <= 0.0)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    temperature_offset_k =
        temperature_k - caloric_reference->reference_temperature_k;

    pressure_pa =
        density_kg_per_m3
        * model->specific_gas_constant_j_per_kg_k
        * temperature_k
        / available_volume_factor;

    specific_internal_energy_j_per_kg =
        caloric_reference->reference_specific_internal_energy_j_per_kg
        + model->constant_volume_specific_heat_j_per_kg_k
        * temperature_offset_k;

    pressure_density_derivative =
        model->specific_gas_constant_j_per_kg_k
        * temperature_k
        / (available_volume_factor * available_volume_factor);

    pressure_temperature_derivative =
        density_kg_per_m3
        * model->specific_gas_constant_j_per_kg_k
        / available_volume_factor;

    if (!isfinite(temperature_offset_k)                ||
        !isfinite(pressure_pa)                         ||
        !isfinite(specific_internal_energy_j_per_kg)   ||
        !isfinite(pressure_density_derivative)         ||
        !isfinite(pressure_temperature_derivative))
    {
        return BBTC_STATUS_NUMERICAL_FAILURE;
    }

    result->pressure_pa = pressure_pa;

    result->specific_internal_energy_j_per_kg =
        specific_internal_energy_j_per_kg;

    result->constant_volume_specific_heat_j_per_kg_k =
        model->constant_volume_specific_heat_j_per_kg_k;

    result->pressure_density_derivative_at_constant_temperature_pa_m3_per_kg =
        pressure_density_derivative;

    result->pressure_temperature_derivative_at_constant_density_pa_per_k =
        pressure_temperature_derivative;

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e
bbtc_ib_noble_abel_thermodynamics_evaluate_long_double(
    const bbtc_ib_noble_abel_gas_model_long_double_t* const model,
    long double density_kg_per_m3,
    long double temperature_k,
    const bbtc_ib_caloric_reference_long_double_t* const caloric_reference,
    bbtc_ib_reduced_gas_thermodynamic_result_long_double_t* const result
)
{
    bbtc_status_e status;
    long double covolume_density;
    long double available_volume_factor;
    long double temperature_offset_k;
    long double pressure_pa;
    long double specific_internal_energy_j_per_kg;
    long double pressure_density_derivative;
    long double pressure_temperature_derivative;

    if (result == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    *result = (bbtc_ib_reduced_gas_thermodynamic_result_long_double_t){0};

    status = bbtc_ib_noble_abel_gas_model_validate_long_double(model);
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    status = bbtc_ib_caloric_reference_validate_long_double(caloric_reference);
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    if (!isfinite(density_kg_per_m3) || !isfinite(temperature_k))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (density_kg_per_m3 < 0.0L || temperature_k <= 0.0L)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    covolume_density = model->covolume_m3_per_kg * density_kg_per_m3;
    if (!isfinite(covolume_density))
        return BBTC_STATUS_NUMERICAL_FAILURE;

    available_volume_factor = 1.0L - covolume_density;
    if (!isfinite(available_volume_factor))
        return BBTC_STATUS_NUMERICAL_FAILURE;

    if (available_volume_factor <= 0.0L)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    temperature_offset_k =
        temperature_k - caloric_reference->reference_temperature_k;

    pressure_pa =
        density_kg_per_m3
        * model->specific_gas_constant_j_per_kg_k
        * temperature_k
        / available_volume_factor;

    specific_internal_energy_j_per_kg =
        caloric_reference->reference_specific_internal_energy_j_per_kg
        + model->constant_volume_specific_heat_j_per_kg_k
        * temperature_offset_k;

    pressure_density_derivative =
        model->specific_gas_constant_j_per_kg_k
        * temperature_k
        / (available_volume_factor * available_volume_factor);

    pressure_temperature_derivative =
        density_kg_per_m3
        * model->specific_gas_constant_j_per_kg_k
        / available_volume_factor;

    if (!isfinite(temperature_offset_k)                ||
        !isfinite(pressure_pa)                         ||
        !isfinite(specific_internal_energy_j_per_kg)   ||
        !isfinite(pressure_density_derivative)         ||
        !isfinite(pressure_temperature_derivative))
    {
        return BBTC_STATUS_NUMERICAL_FAILURE;
    }

    result->pressure_pa = pressure_pa;

    result->specific_internal_energy_j_per_kg =
        specific_internal_energy_j_per_kg;

    result->constant_volume_specific_heat_j_per_kg_k =
        model->constant_volume_specific_heat_j_per_kg_k;

    result->pressure_density_derivative_at_constant_temperature_pa_m3_per_kg =
        pressure_density_derivative;

    result->pressure_temperature_derivative_at_constant_density_pa_per_k =
        pressure_temperature_derivative;

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e
bbtc_ib_first_order_virial_thermodynamics_evaluate_float(
    const bbtc_ib_first_order_virial_gas_model_float_t* const model,
    float density_kg_per_m3,
    float temperature_k,
    const bbtc_ib_caloric_reference_float_t* const caloric_reference,
    bbtc_ib_reduced_gas_thermodynamic_result_float_t* const result
)
{
    bbtc_status_e status;
    bbtc_ib_first_order_virial_temperature_terms_float_t virial_terms;
    bbtc_applicability_flags_t applicability_flags = BBTC_APPLICABILITY_NONE_REPORTED;
    float b_density;
    float mechanical_factor;
    float stability_factor;
    float temperature_squared_k2;
    float heat_capacity_correction;
    float density_gas_constant;
    float state_cv;
    float temperature_offset_k;
    float ideal_caloric_increment;
    float residual_internal_energy;
    float pressure_pa;
    float specific_internal_energy_j_per_kg;
    float pressure_density_derivative;
    float rho_temperature_b1;
    float pressure_temperature_factor;
    float pressure_temperature_derivative;

    if (result == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    *result = (bbtc_ib_reduced_gas_thermodynamic_result_float_t){0};

    status = bbtc_ib_first_order_virial_gas_model_validate_float(model);
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    status = bbtc_ib_caloric_reference_validate_float(caloric_reference);
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    if (!isfinite(density_kg_per_m3) || !isfinite(temperature_k))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (density_kg_per_m3 < 0.0f || temperature_k <= 0.0f)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    status = bbtc_ib_first_order_virial_temperature_law_evaluate_float(
                 &model->second_density_virial_coefficient_law,
                 temperature_k,
                 &virial_terms
    );

    if (status != BBTC_STATUS_SUCCESS)
        return status;

    if (density_kg_per_m3 < model->minimum_calibrated_density_kg_per_m3  ||
        density_kg_per_m3 > model->maximum_calibrated_density_kg_per_m3)
    {
        applicability_flags |= BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN;
    }

    b_density = virial_terms.second_density_virial_coefficient_m3_per_kg
              * density_kg_per_m3;

    if (!isfinite(b_density))
        return BBTC_STATUS_NUMERICAL_FAILURE;

    mechanical_factor = 1.0f + b_density;
    stability_factor  = 1.0f + 2.0f * b_density;

    if (!isfinite(mechanical_factor) || !isfinite(stability_factor))
        return BBTC_STATUS_NUMERICAL_FAILURE;

    if (mechanical_factor <= 0.0f || stability_factor <= 0.0f)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    temperature_squared_k2 = temperature_k * temperature_k;
    if (!isfinite(temperature_squared_k2))
        return BBTC_STATUS_NUMERICAL_FAILURE;

    heat_capacity_correction = 2.0f
                             * temperature_k
                             * virial_terms.first_temperature_derivative_m3_per_kg_k
                             +
                             temperature_squared_k2
                             * virial_terms.second_temperature_derivative_m3_per_kg_k2;

    if (!isfinite(heat_capacity_correction))
        return BBTC_STATUS_NUMERICAL_FAILURE;

    density_gas_constant =
        density_kg_per_m3 * model->specific_gas_constant_j_per_kg_k;

    if (!isfinite(density_gas_constant))
        return BBTC_STATUS_NUMERICAL_FAILURE;

    state_cv = model->ideal_gas_constant_volume_specific_heat_j_per_kg_k
             - density_gas_constant * heat_capacity_correction;

    if (!isfinite(state_cv))
        return BBTC_STATUS_NUMERICAL_FAILURE;

    if (state_cv <= 0.0f)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    temperature_offset_k =
        temperature_k - caloric_reference->reference_temperature_k;

    ideal_caloric_increment =
        model->ideal_gas_constant_volume_specific_heat_j_per_kg_k
        * temperature_offset_k;

    residual_internal_energy =
        density_gas_constant
        * temperature_squared_k2
        * virial_terms.first_temperature_derivative_m3_per_kg_k;

    pressure_pa =
        density_gas_constant * temperature_k * mechanical_factor;

    specific_internal_energy_j_per_kg =
        caloric_reference->reference_specific_internal_energy_j_per_kg
        + ideal_caloric_increment
        - residual_internal_energy;

    pressure_density_derivative =
        model->specific_gas_constant_j_per_kg_k
        * temperature_k
        * stability_factor;

    rho_temperature_b1 =
        density_kg_per_m3
        * temperature_k
        * virial_terms.first_temperature_derivative_m3_per_kg_k;

    pressure_temperature_factor =
        1.0f + b_density + rho_temperature_b1;

    pressure_temperature_derivative =
        density_gas_constant * pressure_temperature_factor;

    if (!isfinite(temperature_offset_k)                   ||
        !isfinite(ideal_caloric_increment)                ||
        !isfinite(residual_internal_energy)               ||
        !isfinite(pressure_pa)                            ||
        !isfinite(specific_internal_energy_j_per_kg)      ||
        !isfinite(pressure_density_derivative)            ||
        !isfinite(rho_temperature_b1)                     ||
        !isfinite(pressure_temperature_factor)            ||
        !isfinite(pressure_temperature_derivative))
    {
        return BBTC_STATUS_NUMERICAL_FAILURE;
    }

    result->applicability_flags = applicability_flags;

    result->pressure_pa = pressure_pa;

    result->specific_internal_energy_j_per_kg =
        specific_internal_energy_j_per_kg;

    result->constant_volume_specific_heat_j_per_kg_k = state_cv;

    result->pressure_density_derivative_at_constant_temperature_pa_m3_per_kg =
        pressure_density_derivative;

    result->pressure_temperature_derivative_at_constant_density_pa_per_k =
        pressure_temperature_derivative;

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e
bbtc_ib_first_order_virial_thermodynamics_evaluate_double(
    const bbtc_ib_first_order_virial_gas_model_double_t* const model,
    double density_kg_per_m3,
    double temperature_k,
    const bbtc_ib_caloric_reference_double_t* const caloric_reference,
    bbtc_ib_reduced_gas_thermodynamic_result_double_t* const result
)
{
    bbtc_status_e status;
    bbtc_ib_first_order_virial_temperature_terms_double_t virial_terms;
    bbtc_applicability_flags_t applicability_flags = BBTC_APPLICABILITY_NONE_REPORTED;
    double b_density;
    double mechanical_factor;
    double stability_factor;
    double temperature_squared_k2;
    double heat_capacity_correction;
    double density_gas_constant;
    double state_cv;
    double temperature_offset_k;
    double ideal_caloric_increment;
    double residual_internal_energy;
    double pressure_pa;
    double specific_internal_energy_j_per_kg;
    double pressure_density_derivative;
    double rho_temperature_b1;
    double pressure_temperature_factor;
    double pressure_temperature_derivative;

    if (result == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    *result = (bbtc_ib_reduced_gas_thermodynamic_result_double_t){0};

    status = bbtc_ib_first_order_virial_gas_model_validate_double(model);

    if (status != BBTC_STATUS_SUCCESS)
        return status;

    status = bbtc_ib_caloric_reference_validate_double(caloric_reference);

    if (status != BBTC_STATUS_SUCCESS)
        return status;

    if (!isfinite(density_kg_per_m3) || !isfinite(temperature_k))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (density_kg_per_m3 < 0.0 || temperature_k <= 0.0)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    status = bbtc_ib_first_order_virial_temperature_law_evaluate_double(
        &model->second_density_virial_coefficient_law,
        temperature_k,
        &virial_terms
    );

    if (status != BBTC_STATUS_SUCCESS)
        return status;

    if (density_kg_per_m3 < model->minimum_calibrated_density_kg_per_m3 ||
        density_kg_per_m3 > model->maximum_calibrated_density_kg_per_m3)
    {
        applicability_flags |= BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN;
    }

    b_density = virial_terms.second_density_virial_coefficient_m3_per_kg
              * density_kg_per_m3;

    if (!isfinite(b_density))
        return BBTC_STATUS_NUMERICAL_FAILURE;

    mechanical_factor = 1.0 + b_density;
    stability_factor  = 1.0 + 2.0 * b_density;

    if (!isfinite(mechanical_factor) || !isfinite(stability_factor))
        return BBTC_STATUS_NUMERICAL_FAILURE;

    if (mechanical_factor <= 0.0 || stability_factor <= 0.0)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    temperature_squared_k2 = temperature_k * temperature_k;
    if (!isfinite(temperature_squared_k2))
        return BBTC_STATUS_NUMERICAL_FAILURE;

    heat_capacity_correction = 2.0
                             * temperature_k
                             * virial_terms.first_temperature_derivative_m3_per_kg_k
                             +
                             temperature_squared_k2
                             * virial_terms.second_temperature_derivative_m3_per_kg_k2;

    if (!isfinite(heat_capacity_correction))
        return BBTC_STATUS_NUMERICAL_FAILURE;

    density_gas_constant =
        density_kg_per_m3 * model->specific_gas_constant_j_per_kg_k;

    if (!isfinite(density_gas_constant))
        return BBTC_STATUS_NUMERICAL_FAILURE;

    state_cv =
        model->ideal_gas_constant_volume_specific_heat_j_per_kg_k
        - density_gas_constant * heat_capacity_correction;

    if (!isfinite(state_cv))
        return BBTC_STATUS_NUMERICAL_FAILURE;

    if (state_cv <= 0.0)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    temperature_offset_k =
        temperature_k - caloric_reference->reference_temperature_k;

    ideal_caloric_increment =
        model->ideal_gas_constant_volume_specific_heat_j_per_kg_k
        * temperature_offset_k;

    residual_internal_energy =
        density_gas_constant
        * temperature_squared_k2
        * virial_terms.first_temperature_derivative_m3_per_kg_k;

    pressure_pa =
        density_gas_constant * temperature_k * mechanical_factor;

    specific_internal_energy_j_per_kg =
        caloric_reference->reference_specific_internal_energy_j_per_kg
        + ideal_caloric_increment
        - residual_internal_energy;

    pressure_density_derivative =
        model->specific_gas_constant_j_per_kg_k
        * temperature_k
        * stability_factor;

    rho_temperature_b1 =
        density_kg_per_m3
        * temperature_k
        * virial_terms.first_temperature_derivative_m3_per_kg_k;

    pressure_temperature_factor =
        1.0 + b_density + rho_temperature_b1;

    pressure_temperature_derivative =
        density_gas_constant * pressure_temperature_factor;

    if (!isfinite(temperature_offset_k)                   ||
        !isfinite(ideal_caloric_increment)                ||
        !isfinite(residual_internal_energy)               ||
        !isfinite(pressure_pa)                            ||
        !isfinite(specific_internal_energy_j_per_kg)      ||
        !isfinite(pressure_density_derivative)            ||
        !isfinite(rho_temperature_b1)                     ||
        !isfinite(pressure_temperature_factor)            ||
        !isfinite(pressure_temperature_derivative))
    {
        return BBTC_STATUS_NUMERICAL_FAILURE;
    }

    result->applicability_flags = applicability_flags;

    result->pressure_pa = pressure_pa;

    result->specific_internal_energy_j_per_kg =
        specific_internal_energy_j_per_kg;

    result->constant_volume_specific_heat_j_per_kg_k = state_cv;

    result->pressure_density_derivative_at_constant_temperature_pa_m3_per_kg =
        pressure_density_derivative;

    result->pressure_temperature_derivative_at_constant_density_pa_per_k =
        pressure_temperature_derivative;

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e
bbtc_ib_first_order_virial_thermodynamics_evaluate_long_double(
    const bbtc_ib_first_order_virial_gas_model_long_double_t* const model,
    long double density_kg_per_m3,
    long double temperature_k,
    const bbtc_ib_caloric_reference_long_double_t* const caloric_reference,
    bbtc_ib_reduced_gas_thermodynamic_result_long_double_t* const result
)
{
    bbtc_status_e status;
    bbtc_ib_first_order_virial_temperature_terms_long_double_t virial_terms;
    bbtc_applicability_flags_t applicability_flags = BBTC_APPLICABILITY_NONE_REPORTED;
    long double b_density;
    long double mechanical_factor;
    long double stability_factor;
    long double temperature_squared_k2;
    long double heat_capacity_correction;
    long double density_gas_constant;
    long double state_cv;
    long double temperature_offset_k;
    long double ideal_caloric_increment;
    long double residual_internal_energy;
    long double pressure_pa;
    long double specific_internal_energy_j_per_kg;
    long double pressure_density_derivative;
    long double rho_temperature_b1;
    long double pressure_temperature_factor;
    long double pressure_temperature_derivative;

    if (result == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    *result = (bbtc_ib_reduced_gas_thermodynamic_result_long_double_t){0};

    status = bbtc_ib_first_order_virial_gas_model_validate_long_double(model);
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    status = bbtc_ib_caloric_reference_validate_long_double(caloric_reference);
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    if (!isfinite(density_kg_per_m3) || !isfinite(temperature_k))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (density_kg_per_m3 < 0.0L || temperature_k <= 0.0L)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    status = bbtc_ib_first_order_virial_temperature_law_evaluate_long_double(
        &model->second_density_virial_coefficient_law,
        temperature_k,
        &virial_terms
    );

    if (status != BBTC_STATUS_SUCCESS)
        return status;

    if (density_kg_per_m3 < model->minimum_calibrated_density_kg_per_m3 ||
        density_kg_per_m3 > model->maximum_calibrated_density_kg_per_m3)
    {
        applicability_flags |= BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN;
    }

    b_density = virial_terms.second_density_virial_coefficient_m3_per_kg
              * density_kg_per_m3;

    if (!isfinite(b_density))
        return BBTC_STATUS_NUMERICAL_FAILURE;

    mechanical_factor = 1.0L + b_density;
    stability_factor  = 1.0L + 2.0L * b_density;

    if (!isfinite(mechanical_factor) || !isfinite(stability_factor))
        return BBTC_STATUS_NUMERICAL_FAILURE;

    if (mechanical_factor <= 0.0L || stability_factor <= 0.0L)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    temperature_squared_k2 = temperature_k * temperature_k;
    if (!isfinite(temperature_squared_k2))
        return BBTC_STATUS_NUMERICAL_FAILURE;

    heat_capacity_correction = 2.0L
                             * temperature_k
                             * virial_terms.first_temperature_derivative_m3_per_kg_k
                             +
                             temperature_squared_k2
                             * virial_terms.second_temperature_derivative_m3_per_kg_k2;

    if (!isfinite(heat_capacity_correction))
        return BBTC_STATUS_NUMERICAL_FAILURE;

    density_gas_constant =
        density_kg_per_m3 * model->specific_gas_constant_j_per_kg_k;

    if (!isfinite(density_gas_constant))
        return BBTC_STATUS_NUMERICAL_FAILURE;

    state_cv =
        model->ideal_gas_constant_volume_specific_heat_j_per_kg_k
        - density_gas_constant * heat_capacity_correction;

    if (!isfinite(state_cv))
        return BBTC_STATUS_NUMERICAL_FAILURE;

    if (state_cv <= 0.0L)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    temperature_offset_k =
        temperature_k - caloric_reference->reference_temperature_k;

    ideal_caloric_increment =
        model->ideal_gas_constant_volume_specific_heat_j_per_kg_k
        * temperature_offset_k;

    residual_internal_energy =
        density_gas_constant
        * temperature_squared_k2
        * virial_terms.first_temperature_derivative_m3_per_kg_k;

    pressure_pa =
        density_gas_constant * temperature_k * mechanical_factor;

    specific_internal_energy_j_per_kg =
        caloric_reference->reference_specific_internal_energy_j_per_kg
        + ideal_caloric_increment
        - residual_internal_energy;

    pressure_density_derivative =
        model->specific_gas_constant_j_per_kg_k
        * temperature_k
        * stability_factor;

    rho_temperature_b1 =
        density_kg_per_m3
        * temperature_k
        * virial_terms.first_temperature_derivative_m3_per_kg_k;

    pressure_temperature_factor =
        1.0L + b_density + rho_temperature_b1;

    pressure_temperature_derivative =
        density_gas_constant * pressure_temperature_factor;

    if (!isfinite(temperature_offset_k)                   ||
        !isfinite(ideal_caloric_increment)                ||
        !isfinite(residual_internal_energy)               ||
        !isfinite(pressure_pa)                            ||
        !isfinite(specific_internal_energy_j_per_kg)      ||
        !isfinite(pressure_density_derivative)            ||
        !isfinite(rho_temperature_b1)                     ||
        !isfinite(pressure_temperature_factor)            ||
        !isfinite(pressure_temperature_derivative))
    {
        return BBTC_STATUS_NUMERICAL_FAILURE;
    }

    result->applicability_flags = applicability_flags;

    result->pressure_pa = pressure_pa;

    result->specific_internal_energy_j_per_kg =
        specific_internal_energy_j_per_kg;

    result->constant_volume_specific_heat_j_per_kg_k = state_cv;

    result->pressure_density_derivative_at_constant_temperature_pa_m3_per_kg =
        pressure_density_derivative;

    result->pressure_temperature_derivative_at_constant_density_pa_per_k =
        pressure_temperature_derivative;

    return BBTC_STATUS_SUCCESS;
}
