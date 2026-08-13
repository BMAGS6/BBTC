/**
 * @file
 * @brief Verifies BBTC use from an independent CMake consumer project.
 */

#include <bbtc/bbtc.h>

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/**
 * @brief Compares one BBTC diagnostic string against its stable text.
 *
 * @param actual String returned by the BBTC public API.
 * @param expected Stable text required by the public contract.
 *
 * @return `EXIT_SUCCESS` when the strings match; otherwise `EXIT_FAILURE`.
 */
static int
check_string(const char* const actual,
             const char* const expected)
{
    if (actual == NULL)
        return EXIT_FAILURE;

    return strcmp(actual, expected);
}

/**
 * @brief Verifies public precision metadata through the consumer target.
 *
 * @return `EXIT_SUCCESS` when precision metadata is coherent; otherwise
 *         `EXIT_FAILURE`.
 */
static int
check_precision_contract(void)
{
    bbtc_precision_info_t info = {0};

    if (bbtc_precision_info(BBTC_PRECISION_DOUBLE, &info) != BBTC_STATUS_SUCCESS)
        return EXIT_FAILURE;

    if (info.precision     != BBTC_PRECISION_DOUBLE     ||
        info.radix         <  UINT32_C(2)               ||
        info.storage_bytes != (uint32_t)sizeof(double))
    {
        return EXIT_FAILURE;
    }

    return check_string(bbtc_precision_string(BBTC_PRECISION_DOUBLE),
                        "double");
}

/**
 * @brief Verifies precision-qualified geometry through the consumer target.
 *
 * @return `EXIT_SUCCESS` when the geometry contract works; otherwise
 *         `EXIT_FAILURE`.
 */
static uint8_t
check_geometry_contract(void)
{
    const bbtc_ib_geometry_double_t geometry =
    {
        .initial_behind_projectile_volume_m3 = 4.0e-6,
        .bore_cross_sectional_area_m2        = 5.0e-5,
        .projectile_effective_base_area_m2   = 4.8e-5,
        .projectile_travel_to_muzzle_m       = 0.6
    };

    return bbtc_ib_geometry_validate_double(&geometry) == BBTC_STATUS_SUCCESS
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Verifies precision-qualified projectile data through the consumer.
 *
 * @return `EXIT_SUCCESS` when the projectile contract works; otherwise
 *         `EXIT_FAILURE`.
 */
static int
check_projectile_contract(void)
{
    const bbtc_ib_projectile_double_t projectile =
    {
        .mass_kg = 0.01134
    };

    return bbtc_ib_projectile_validate_double(&projectile) == BBTC_STATUS_SUCCESS
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Verifies precision-qualified propellant-charge data through the consumer.
 *
 * @return `EXIT_SUCCESS` when the propellant-charge contract works; otherwise
 *         `EXIT_FAILURE`.
 */
static int
check_propellant_charge_contract(void)
{
    const bbtc_ib_propellant_charge_double_t charge =
    {
        .charge_mass_kg                    = 0.0030,
        .condensed_phase_density_kg_per_m3 = 1600.0
    };

    return bbtc_ib_propellant_charge_validate_double(&charge) == BBTC_STATUS_SUCCESS
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}

/**
 * @brief Verifies composed loading-state evaluation through the consumer.
 *
 * @return `EXIT_SUCCESS` when composition and derived volumes work; otherwise
 *         `EXIT_FAILURE`.
 */
static int
check_loading_state_contract(void)
{
    const bbtc_ib_loading_state_double_t loading_state =
    {
        .geometry =
        {
            .initial_behind_projectile_volume_m3 = 8.0,
            .bore_cross_sectional_area_m2        = 1.0,
            .projectile_effective_base_area_m2   = 1.0,
            .projectile_travel_to_muzzle_m       = 1.0
        },

        .projectile =
        {
            .mass_kg = 1.0
        },

        .propellant_charge =
        {
            .charge_mass_kg                    = 6.0,
            .condensed_phase_density_kg_per_m3 = 2.0
        }
    };

    bbtc_ib_loading_state_volumes_double_t volumes = {0};

    if (bbtc_ib_loading_state_evaluate_double(&loading_state, &volumes)
        != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    return volumes.condensed_propellant_volume_m3 == 3.0 && volumes.initial_free_gas_volume_m3 == 5.0
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Verifies initial gas-state validation through the consumer.
 *
 * @return `EXIT_SUCCESS` when the initial gas-state contract works; otherwise
 *         `EXIT_FAILURE`.
 */
static int
check_initial_gas_state_contract(void)
{
    const bbtc_ib_initial_gas_state_double_t state =
    {
        .absolute_pressure_pa = 101325.0,
        .temperature_k        = 293.15
    };

    return bbtc_ib_initial_gas_state_validate_double(&state) == BBTC_STATUS_SUCCESS
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Verifies Noble-Abel gas-model validation through the consumer.
 *
 * @return `EXIT_SUCCESS` when the gas-model contract works; otherwise
 *         `EXIT_FAILURE`.
 */
static int
check_noble_abel_gas_model_contract(void)
{
    const bbtc_ib_noble_abel_gas_model_double_t model =
    {
        .specific_gas_constant_j_per_kg_k         = 287.0,
        .constant_volume_specific_heat_j_per_kg_k = 718.0,
        .covolume_m3_per_kg                        = 0.001
    };

    return bbtc_ib_noble_abel_gas_model_validate_double(&model)
            == BBTC_STATUS_SUCCESS
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Verifies first-order virial validation and evaluation through the
 *        independent consumer target.
 *
 * @return `EXIT_SUCCESS` when the public virial contract works; otherwise
 *         `EXIT_FAILURE`.
 */
static int
check_first_order_virial_gas_model_contract(void)
{
    const double coefficients[] =
    {
        0.0
    };

    const bbtc_ib_first_order_virial_gas_model_double_t model =
    {
        .specific_gas_constant_j_per_kg_k = 287.0,
        .ideal_gas_constant_volume_specific_heat_j_per_kg_k = 718.0,
        .minimum_calibrated_density_kg_per_m3 = 0.0,
        .maximum_calibrated_density_kg_per_m3 = 500.0,
        .second_density_virial_coefficient_law =
        {
            .minimum_temperature_k = 250.0,
            .maximum_temperature_k = 4000.0,
            .second_density_virial_chebyshev_coefficients_m3_per_kg =
                coefficients,
            .coefficient_count = 1U
        }
    };

    bbtc_ib_first_order_virial_temperature_terms_double_t terms = {0};

    if (bbtc_ib_first_order_virial_gas_model_validate_double(&model)
        != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (bbtc_ib_first_order_virial_temperature_law_evaluate_double(
            &model.second_density_virial_coefficient_law,
            1000.0,
            &terms
        ) != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    return terms.second_density_virial_coefficient_m3_per_kg == 0.0 &&
           terms.first_temperature_derivative_m3_per_kg_k    == 0.0 &&
           terms.second_temperature_derivative_m3_per_kg_k2  == 0.0
            ? EXIT_SUCCESS
            : EXIT_FAILURE;
}




/**
 * @brief Verifies initial free-gas closure through the independent consumer.
 *
 * @details
 * Both concrete reduced-gas backends are reduced to the same exact ideal-gas
 * state. This deliberately tests public-header visibility, static-library
 * linkage (including the transitive math-library dependency), solution-record
 * semantics, applicability metadata, and the new IB0.3j entry points. Detailed
 * nonideal and numerical-edge behavior remains in the dedicated closure test.
 *
 * @return `EXIT_SUCCESS` when both public closure APIs recover the exact
 *         expected density and gas mass; otherwise `EXIT_FAILURE`.
 */
static int
check_initial_gas_closure_contract(void)
{
    const double virial_coefficients[] =
    {
        0.0
    };

    const bbtc_ib_initial_gas_state_double_t initial_gas_state =
    {
        .absolute_pressure_pa = 600.0,
        .temperature_k = 3.0
    };

    const bbtc_ib_noble_abel_gas_model_double_t noble_abel_model =
    {
        .specific_gas_constant_j_per_kg_k = 100.0,
        .constant_volume_specific_heat_j_per_kg_k = 500.0,
        .covolume_m3_per_kg = 0.0
    };

    const bbtc_ib_first_order_virial_gas_model_double_t virial_model =
    {
        .specific_gas_constant_j_per_kg_k = 100.0,
        .ideal_gas_constant_volume_specific_heat_j_per_kg_k = 500.0,
        .minimum_calibrated_density_kg_per_m3 = 0.0,
        .maximum_calibrated_density_kg_per_m3 = 5.0,
        .second_density_virial_coefficient_law =
        {
            .minimum_temperature_k = 1.0,
            .maximum_temperature_k = 5.0,
            .second_density_virial_chebyshev_coefficients_m3_per_kg =
                virial_coefficients,
            .coefficient_count = 1U
        }
    };

    bbtc_ib_initial_gas_solution_double_t noble_abel_solution = {0};
    bbtc_ib_initial_gas_solution_double_t virial_solution = {0};

    if (bbtc_ib_noble_abel_initial_gas_solve_double(
            &noble_abel_model,
            &initial_gas_state,
            0.25,
            &noble_abel_solution
        ) != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (bbtc_ib_first_order_virial_initial_gas_solve_double(
            &virial_model,
            &initial_gas_state,
            0.25,
            &virial_solution
        ) != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (noble_abel_solution.applicability_flags
            != BBTC_APPLICABILITY_NONE_REPORTED
        || virial_solution.applicability_flags
            != BBTC_APPLICABILITY_NONE_REPORTED)
    {
        return EXIT_FAILURE;
    }

    if (noble_abel_solution.density_kg_per_m3 != 2.0
        || noble_abel_solution.gas_mass_kg != 0.5
        || virial_solution.density_kg_per_m3 != 2.0
        || virial_solution.gas_mass_kg != 0.5)
    {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}


/**
 * @brief Verifies the reduced-gas thermodynamic evaluators through the
 *        independent CMake consumer.
 *
 * @details
 * This deliberately exercises both concrete reduced-gas backends through the
 * public umbrella header. The parameter choices reduce both models to the same
 * exact ideal-gas state so that the consumer test verifies linkage, record
 * visibility, caloric-reference semantics, common result layout, and the
 * concrete evaluator entry points without introducing calibration data.
 *
 * @return `EXIT_SUCCESS` when both public thermodynamic evaluators produce the
 *         expected state; otherwise `EXIT_FAILURE`.
 */
static int
check_reduced_gas_thermodynamics_contract(void)
{
    const double virial_coefficients[] =
    {
        0.0
    };

    const bbtc_ib_caloric_reference_double_t caloric_reference =
    {
        .reference_temperature_k = 300.0,
        .reference_specific_internal_energy_j_per_kg = 1000.0
    };

    const bbtc_ib_noble_abel_gas_model_double_t noble_abel_model =
    {
        .specific_gas_constant_j_per_kg_k = 100.0,
        .constant_volume_specific_heat_j_per_kg_k = 500.0,
        .covolume_m3_per_kg = 0.0
    };

    const bbtc_ib_first_order_virial_gas_model_double_t virial_model =
    {
        .specific_gas_constant_j_per_kg_k = 100.0,
        .ideal_gas_constant_volume_specific_heat_j_per_kg_k = 500.0,
        .minimum_calibrated_density_kg_per_m3 = 0.0,
        .maximum_calibrated_density_kg_per_m3 = 5.0,
        .second_density_virial_coefficient_law =
        {
            .minimum_temperature_k = 200.0,
            .maximum_temperature_k = 400.0,
            .second_density_virial_chebyshev_coefficients_m3_per_kg =
                virial_coefficients,
            .coefficient_count = 1U
        }
    };

    bbtc_ib_reduced_gas_thermodynamic_result_double_t noble_abel_result = {0};
    bbtc_ib_reduced_gas_thermodynamic_result_double_t virial_result = {0};

    if (bbtc_ib_caloric_reference_validate_double(&caloric_reference)
        != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (bbtc_ib_noble_abel_thermodynamics_evaluate_double(
            &noble_abel_model,
            2.0,
            300.0,
            &caloric_reference,
            &noble_abel_result
        ) != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (bbtc_ib_first_order_virial_thermodynamics_evaluate_double(
            &virial_model,
            2.0,
            300.0,
            &caloric_reference,
            &virial_result
        ) != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (noble_abel_result.applicability_flags
            != BBTC_APPLICABILITY_NONE_REPORTED
        || virial_result.applicability_flags
            != BBTC_APPLICABILITY_NONE_REPORTED)
    {
        return EXIT_FAILURE;
    }

    if (noble_abel_result.pressure_pa != 60000.0
        || noble_abel_result.specific_internal_energy_j_per_kg != 1000.0
        || noble_abel_result.constant_volume_specific_heat_j_per_kg_k != 500.0
        || noble_abel_result.pressure_density_derivative_at_constant_temperature_pa_m3_per_kg
            != 30000.0
        || noble_abel_result.pressure_temperature_derivative_at_constant_density_pa_per_k
            != 200.0)
    {
        return EXIT_FAILURE;
    }

    if (virial_result.pressure_pa != noble_abel_result.pressure_pa
        || virial_result.specific_internal_energy_j_per_kg
            != noble_abel_result.specific_internal_energy_j_per_kg
        || virial_result.constant_volume_specific_heat_j_per_kg_k
            != noble_abel_result.constant_volume_specific_heat_j_per_kg_k
        || virial_result.pressure_density_derivative_at_constant_temperature_pa_m3_per_kg
            != noble_abel_result.pressure_density_derivative_at_constant_temperature_pa_m3_per_kg
        || virial_result.pressure_temperature_derivative_at_constant_density_pa_per_k
            != noble_abel_result.pressure_temperature_derivative_at_constant_density_pa_per_k)
    {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

/**
 * @brief Verifies reduced propellant thermochemistry through the independent consumer.
 *
 * @details
 * This public-boundary fixture deliberately uses binary-exact values so the
 * test checks API visibility, linkage, native-double records, source semantics,
 * and field meaning without introducing tolerance noise. Detailed edge and
 * representability behavior remains in the dedicated IB0.4a unit test.
 *
 * @return `EXIT_SUCCESS` when the public thermochemical-source API produces
 *         the expected split masses and reaction-energy release; otherwise
 *         `EXIT_FAILURE`.
 */
static int
check_propellant_thermochemistry_contract(void)
{
    const bbtc_ib_propellant_thermochemistry_double_t thermochemistry =
    {
        .gas_product_mass_fraction = 0.75,
        .specific_reaction_internal_energy_release_j_per_kg = 4.0
    };

    bbtc_ib_propellant_thermochemical_source_double_t source = {0};

    if (bbtc_ib_propellant_thermochemistry_validate_double(&thermochemistry)
        != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (bbtc_ib_propellant_thermochemical_source_evaluate_double(
            &thermochemistry,
            2.0,
            &source
        ) != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (source.gas_product_mass_kg != 1.5
        || source.condensed_product_mass_kg != 0.5
        || source.reaction_internal_energy_release_j != 8.0)
    {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}



/**
 * @brief Exercises public BBTC headers and linked diagnostic symbols.
 *
 * @return `EXIT_SUCCESS` when the external consumer contract works; otherwise
 *         `EXIT_FAILURE`.
 */
int main(void)
{

    if (check_propellant_thermochemistry_contract() != EXIT_SUCCESS)
        return EXIT_FAILURE;


    if (check_string(bbtc_status_string(BBTC_STATUS_SUCCESS),
                     "success")
        != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (check_precision_contract() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (check_geometry_contract() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (check_projectile_contract() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (check_propellant_charge_contract() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (check_loading_state_contract() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (check_initial_gas_state_contract() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (check_noble_abel_gas_model_contract() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (check_first_order_virial_gas_model_contract() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (check_initial_gas_closure_contract() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (check_reduced_gas_thermodynamics_contract() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (check_string(bbtc_ib_termination_string(BBTC_IB_TERMINATION_MUZZLE_EXIT),
                     "projectile reached muzzle exit")
        != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (check_string(bbtc_warning_flag_string(BBTC_WARNING_DATA_EXTRAPOLATED),
                     "data record extrapolated")
        != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    return check_string(bbtc_applicability_flag_string(BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN),
                        "outside documented calibration domain");
}
