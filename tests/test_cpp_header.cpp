#include <bbtc/bbtc.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

static_assert(
    std::is_same<
        std::underlying_type<bbtc_status_e>::type,
        std::uint8_t
    >::value,
    "bbtc_status_e must have uint8_t representation"
);

static_assert(BBTC_STATUS_SUCCESS == 0,
              "BBTC status value zero must mean success");

static_assert(std::is_same<std::underlying_type<bbtc_ib_termination_e>::type,
                           std::uint8_t
              >::value,
              "bbtc_ib_termination_e must have uint8_t representation"
);

static_assert(std::is_same<std::underlying_type<bbtc_precision_e>::type,
                           std::uint8_t
              >::value,
              "bbtc_precision_e must have uint8_t representation"
);

static_assert(
    std::is_same<
        std::underlying_type<bbtc_warning_flag_e>::type,
        std::uint64_t
    >::value,
    "bbtc_warning_flag_e must have uint64_t representation"
);

static_assert(
    std::is_same<
        std::underlying_type<bbtc_applicability_flag_e>::type,
        std::uint64_t
    >::value,
    "bbtc_applicability_flag_e must have uint64_t representation"
);

static_assert(
    std::is_same<bbtc_warning_flags_t, std::uint64_t>::value,
    "bbtc_warning_flags_t must be uint64_t"
);

static_assert(
    std::is_same<bbtc_applicability_flags_t, std::uint64_t>::value,
    "bbtc_applicability_flags_t must be uint64_t"
);

static_assert(
    (
        BBTC_WARNING_HISTORY_TRUNCATED
        | BBTC_WARNING_DATA_EXTRAPOLATED
    ) == UINT64_C(0x21),
    "warning bits must combine into a uint64_t mask"
);

static_assert(
    (
        BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN
        | BBTC_APPLICABILITY_DATA_PROVENANCE_UNKNOWN
    ) == UINT64_C(0x09),
    "applicability bits must combine into a uint64_t mask"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_ib_geometry_float_t{}
                .initial_behind_projectile_volume_m3
        ),
        float
    >::value,
    "float geometry must use native float fields"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_ib_geometry_double_t{}
                .initial_behind_projectile_volume_m3
        ),
        double
    >::value,
    "double geometry must use native double fields"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_ib_geometry_long_double_t{}
                .initial_behind_projectile_volume_m3
        ),
        long double
    >::value,
    "long-double geometry must use native long-double fields"
);


static_assert(std::is_same<decltype(bbtc_ib_projectile_float_t{}.mass_kg),
                           float
              >::value,
              "float projectile must use native float mass");

static_assert(std::is_same<decltype(bbtc_ib_projectile_double_t{}.mass_kg),
                           double
              >::value,
              "double projectile must use native double mass");

static_assert(std::is_same<decltype(bbtc_ib_projectile_long_double_t{}.mass_kg),
                           long double
              >::value,
              "long-double projectile must use native long-double mass");


static_assert(
    std::is_same<
        decltype(bbtc_ib_propellant_charge_float_t{}.charge_mass_kg),
        float
    >::value,
    "float propellant charge must use native float mass"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_ib_propellant_charge_float_t{}
                .condensed_phase_density_kg_per_m3
        ),
        float
    >::value,
    "float propellant charge must use native float density"
);

static_assert(
    std::is_same<
        decltype(bbtc_ib_propellant_charge_double_t{}.charge_mass_kg),
        double
    >::value,
    "double propellant charge must use native double mass"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_ib_propellant_charge_double_t{}
                .condensed_phase_density_kg_per_m3
        ),
        double
    >::value,
    "double propellant charge must use native double density"
);

static_assert(
    std::is_same<
        decltype(bbtc_ib_propellant_charge_long_double_t{}.charge_mass_kg),
        long double
    >::value,
    "long-double propellant charge must use native long-double mass"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_ib_propellant_charge_long_double_t{}
                .condensed_phase_density_kg_per_m3
        ),
        long double
    >::value,
    "long-double propellant charge must use native long-double density"
);

static_assert(
    std::is_same<
        decltype(bbtc_ib_loading_state_float_t{}.geometry),
        bbtc_ib_geometry_float_t
    >::value,
    "float loading state must own native float geometry"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_ib_loading_state_volumes_double_t{}
                .initial_free_gas_volume_m3
        ),
        double
    >::value,
    "double loading-state volumes must use native double"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_ib_loading_state_volumes_long_double_t{}
                .condensed_propellant_volume_m3
        ),
        long double
    >::value,
    "long-double loading-state volumes must use native long double"
);


static_assert(
    std::is_same<
        decltype(
            bbtc_ib_initial_gas_state_float_t{}
                .absolute_pressure_pa
        ),
        float
    >::value,
    "float initial gas state must use native float pressure"
);

static_assert(
    std::is_same<
        decltype(bbtc_ib_initial_gas_state_double_t{}.temperature_k),
        double
    >::value,
    "double initial gas state must use native double temperature"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_ib_initial_gas_state_long_double_t{}
                .absolute_pressure_pa
        ),
        long double
    >::value,
    "long-double initial gas state must use native long-double pressure"
);


static_assert(
    std::is_same<
        decltype(bbtc_ib_initial_propellant_condition_float_t{}.temperature_k),
        float
    >::value,
    "float initial propellant condition must use native float temperature"
);
static_assert(
    std::is_same<
        decltype(bbtc_ib_initial_propellant_condition_double_t{}.temperature_k),
        double
    >::value,
    "double initial propellant condition must use native double temperature"
);
static_assert(
    std::is_same<
        decltype(
            bbtc_ib_initial_propellant_condition_long_double_t{}.temperature_k
        ),
        long double
    >::value,
    "long-double initial propellant condition must use native long-double temperature"
);


static_assert(
    std::is_same<
        decltype(
            bbtc_ib_noble_abel_gas_model_float_t{}
                .specific_gas_constant_j_per_kg_k
        ),
        float
    >::value,
    "float Noble-Abel gas model must use native float"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_ib_noble_abel_gas_model_double_t{}
                .constant_volume_specific_heat_j_per_kg_k
        ),
        double
    >::value,
    "double Noble-Abel gas model must use native double"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_ib_noble_abel_gas_model_long_double_t{}
                .covolume_m3_per_kg
        ),
        long double
    >::value,
    "long-double Noble-Abel gas model must use native long double"
);


static_assert(
    std::is_same<
        decltype(
            bbtc_ib_first_order_virial_temperature_law_float_t{}
                .minimum_temperature_k
        ),
        float
    >::value,
    "float virial temperature law must use native float"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_ib_first_order_virial_temperature_law_double_t{}
                .coefficient_count
        ),
        std::size_t
    >::value,
    "virial coefficient count must use size_t"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_ib_first_order_virial_temperature_terms_long_double_t{}
                .second_temperature_derivative_m3_per_kg_k2
        ),
        long double
    >::value,
    "long-double virial derivative terms must use native long double"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_ib_first_order_virial_gas_model_double_t{}
                .specific_gas_constant_j_per_kg_k
        ),
        double
    >::value,
    "double virial gas model must use native double"
);



static_assert(
    std::is_same<
        decltype(
            bbtc_ib_caloric_reference_float_t{}
                .reference_temperature_k
        ),
        float
    >::value,
    "float caloric reference must use native float"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_ib_caloric_reference_double_t{}
                .reference_specific_internal_energy_j_per_kg
        ),
        double
    >::value,
    "double caloric reference must use native double"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_ib_caloric_reference_long_double_t{}
                .reference_temperature_k
        ),
        long double
    >::value,
    "long-double caloric reference must use native long double"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_ib_reduced_gas_thermodynamic_result_float_t{}
                .pressure_pa
        ),
        float
    >::value,
    "float reduced-gas result must use native float"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_ib_reduced_gas_thermodynamic_result_double_t{}
                .specific_internal_energy_j_per_kg
        ),
        double
    >::value,
    "double reduced-gas result must use native double"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_ib_reduced_gas_thermodynamic_result_long_double_t{}
                .constant_volume_specific_heat_j_per_kg_k
        ),
        long double
    >::value,
    "long-double reduced-gas result must use native long double"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_ib_reduced_gas_thermodynamic_result_double_t{}
                .applicability_flags
        ),
        bbtc_applicability_flags_t
    >::value,
    "reduced-gas applicability metadata must use the public flag-mask type"
);



static_assert(
    std::is_same<
        decltype(
            bbtc_ib_initial_gas_solution_float_t{}
                .density_kg_per_m3
        ),
        float
    >::value,
    "float initial-gas solution must use native float density"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_ib_initial_gas_solution_double_t{}
                .gas_mass_kg
        ),
        double
    >::value,
    "double initial-gas solution must use native double mass"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_ib_initial_gas_solution_long_double_t{}
                .density_kg_per_m3
        ),
        long double
    >::value,
    "long-double initial-gas solution must use native long-double density"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_ib_initial_gas_solution_double_t{}
                .applicability_flags
        ),
        bbtc_applicability_flags_t
    >::value,
    "initial-gas solution applicability metadata must use the public flag-mask type"
);



static_assert(
    std::is_same<
        decltype(
            bbtc_ib_propellant_thermochemistry_float_t{}
                .gas_product_mass_fraction
        ),
        float
    >::value,
    "float propellant thermochemistry must use native float"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_ib_propellant_thermochemistry_double_t{}
                .specific_reaction_internal_energy_release_j_per_kg
        ),
        double
    >::value,
    "double propellant thermochemistry must use native double"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_ib_propellant_thermochemistry_long_double_t{}
                .gas_product_mass_fraction
        ),
        long double
    >::value,
    "long-double propellant thermochemistry must use native long double"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_ib_propellant_thermochemical_source_float_t{}
                .gas_product_mass_kg
        ),
        float
    >::value,
    "float propellant thermochemical source must use native float"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_ib_propellant_thermochemical_source_double_t{}
                .condensed_product_mass_kg
        ),
        double
    >::value,
    "double propellant thermochemical source must use native double"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_ib_propellant_thermochemical_source_long_double_t{}
                .reaction_internal_energy_release_j
        ),
        long double
    >::value,
    "long-double propellant thermochemical source must use native long double"
);


static_assert(
    std::is_same<
        decltype(
            bbtc_ib_propellant_grain_state_float_t{}
                .remaining_volume_m3
        ),
        float
    >::value,
    "float propellant-grain state must use native float"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_ib_propellant_grain_state_double_t{}
                .burning_surface_area_m2
        ),
        double
    >::value,
    "double propellant-grain state must use native double"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_ib_propellant_grain_state_long_double_t{}
                .remaining_regression_to_burnout_m
        ),
        long double
    >::value,
    "long-double propellant-grain state must use native long double"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_ib_spherical_grain_geometry_float_t{}
                .initial_radius_m
        ),
        float
    >::value,
    "float spherical grain must use native float"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_ib_solid_cylindrical_grain_geometry_double_t{}
                .initial_length_m
        ),
        double
    >::value,
    "double solid cylindrical grain must use native double"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_ib_rectangular_prismatic_grain_geometry_long_double_t{}
                .initial_thickness_m
        ),
        long double
    >::value,
    "long-double rectangular grain must use native long double"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_ib_single_perforated_cylindrical_grain_geometry_double_t{}
                .initial_inner_radius_m
        ),
        double
    >::value,
    "double single-perforated grain must use native double"
);


int main()
{


    bbtc_ib_rectangular_prismatic_grain_geometry_double_t grain_geometry = {};
    grain_geometry.initial_length_m = 4.0;
    grain_geometry.initial_width_m = 4.0;
    grain_geometry.initial_thickness_m = 4.0;

    bbtc_ib_propellant_grain_state_double_t grain_state = {};

    if (bbtc_ib_rectangular_prismatic_grain_geometry_validate_double(&grain_geometry)
            != BBTC_STATUS_SUCCESS
        || bbtc_ib_rectangular_prismatic_grain_evaluate_double(
            &grain_geometry,
            1.0,
            &grain_state
        ) != BBTC_STATUS_SUCCESS
        || grain_state.remaining_volume_m3 != 8.0
        || grain_state.burning_surface_area_m2 != 24.0
        || grain_state.remaining_regression_to_burnout_m != 1.0
        || grain_state.consumed_volume_fraction != 0.875)
    {
        return 1;
    }

    bbtc_ib_propellant_thermochemistry_double_t thermochemistry = {};
    thermochemistry.gas_product_mass_fraction = 0.75;
    thermochemistry.specific_reaction_internal_energy_release_j_per_kg = 4.0;

    bbtc_ib_propellant_thermochemical_source_double_t thermochemical_source = {};

    if (bbtc_ib_propellant_thermochemistry_validate_double(&thermochemistry)
            != BBTC_STATUS_SUCCESS
        || bbtc_ib_propellant_thermochemical_source_evaluate_double(
            &thermochemistry,
            2.0,
            &thermochemical_source
        ) != BBTC_STATUS_SUCCESS
        || thermochemical_source.gas_product_mass_kg != 1.5
        || thermochemical_source.condensed_product_mass_kg != 0.5
        || thermochemical_source.reaction_internal_energy_release_j != 8.0)
    {
        return 1;
    }


    if (
        std::strcmp(
            bbtc_status_string(BBTC_STATUS_SUCCESS),
            "success"
        ) != 0
    )
    {
        return 1;
    }

    bbtc_precision_info_t precision_info = {};

    if (
        bbtc_precision_info(BBTC_PRECISION_DOUBLE, &precision_info)
            != BBTC_STATUS_SUCCESS
        || precision_info.precision != BBTC_PRECISION_DOUBLE
        || precision_info.storage_bytes
            != static_cast<std::uint32_t>(sizeof(double))
        || std::strcmp(
            bbtc_precision_string(BBTC_PRECISION_DOUBLE),
            "double"
        ) != 0
    )
    {
        return 1;
    }


    bbtc_ib_geometry_double_t geometry = {};
    geometry.initial_behind_projectile_volume_m3 = 4.0e-6;
    geometry.bore_cross_sectional_area_m2 = 5.0e-5;
    geometry.projectile_effective_base_area_m2 = 4.8e-5;
    geometry.projectile_travel_to_muzzle_m = 0.6;

    if (bbtc_ib_geometry_validate_double(&geometry) != BBTC_STATUS_SUCCESS)
    {
        return 1;
    }


    bbtc_ib_projectile_double_t projectile = {};
    projectile.mass_kg = 0.01134;

    if (bbtc_ib_projectile_validate_double(&projectile) != BBTC_STATUS_SUCCESS)
    {
        return 1;
    }


    bbtc_ib_propellant_charge_double_t charge = {};
    charge.charge_mass_kg = 0.0030;
    charge.condensed_phase_density_kg_per_m3 = 1600.0;

    if (bbtc_ib_propellant_charge_validate_double(&charge) != BBTC_STATUS_SUCCESS)
    {
        return 1;
    }

    bbtc_ib_loading_state_double_t loading_state = {};
    loading_state.geometry.initial_behind_projectile_volume_m3 = 8.0;
    loading_state.geometry.bore_cross_sectional_area_m2 = 1.0;
    loading_state.geometry.projectile_effective_base_area_m2 = 1.0;
    loading_state.geometry.projectile_travel_to_muzzle_m = 1.0;
    loading_state.projectile.mass_kg = 1.0;
    loading_state.propellant_charge.charge_mass_kg = 6.0;
    loading_state.propellant_charge.condensed_phase_density_kg_per_m3 = 2.0;
    bbtc_ib_loading_state_volumes_double_t loading_volumes = {};

    if (bbtc_ib_loading_state_evaluate_double(&loading_state, &loading_volumes)
            != BBTC_STATUS_SUCCESS
        || loading_volumes.condensed_propellant_volume_m3 != 3.0
        || loading_volumes.initial_free_gas_volume_m3 != 5.0)
    {
        return 1;
    }


    bbtc_ib_initial_gas_state_double_t initial_gas_state = {};
    initial_gas_state.absolute_pressure_pa = 101325.0;
    initial_gas_state.temperature_k = 293.15;

    if (bbtc_ib_initial_gas_state_validate_double(&initial_gas_state)
        != BBTC_STATUS_SUCCESS)
    {
        return 1;
    }


    bbtc_ib_initial_propellant_condition_double_t propellant_condition = {};
    propellant_condition.temperature_k = 293.15;

    if (bbtc_ib_initial_propellant_condition_validate_double(
            &propellant_condition
        ) != BBTC_STATUS_SUCCESS)
    {
        return 1;
    }


    bbtc_ib_noble_abel_gas_model_double_t gas_model = {};
    gas_model.specific_gas_constant_j_per_kg_k = 287.0;
    gas_model.constant_volume_specific_heat_j_per_kg_k = 718.0;
    gas_model.covolume_m3_per_kg = 0.001;

    if (bbtc_ib_noble_abel_gas_model_validate_double(&gas_model)
        != BBTC_STATUS_SUCCESS)
    {
        return 1;
    }


    const double virial_coefficients[] =
    {
        0.0
    };

    bbtc_ib_first_order_virial_gas_model_double_t virial_model = {};
    virial_model.specific_gas_constant_j_per_kg_k = 287.0;
    virial_model.ideal_gas_constant_volume_specific_heat_j_per_kg_k = 718.0;
    virial_model.minimum_calibrated_density_kg_per_m3 = 0.0;
    virial_model.maximum_calibrated_density_kg_per_m3 = 500.0;
    virial_model.second_density_virial_coefficient_law.minimum_temperature_k =
        250.0;
    virial_model.second_density_virial_coefficient_law.maximum_temperature_k =
        4000.0;
    virial_model.second_density_virial_coefficient_law
        .second_density_virial_chebyshev_coefficients_m3_per_kg =
            virial_coefficients;
    virial_model.second_density_virial_coefficient_law.coefficient_count = 1U;

    bbtc_ib_first_order_virial_temperature_terms_double_t virial_terms = {};

    if (bbtc_ib_first_order_virial_gas_model_validate_double(&virial_model)
            != BBTC_STATUS_SUCCESS
        || bbtc_ib_first_order_virial_temperature_law_evaluate_double(
            &virial_model.second_density_virial_coefficient_law,
            1000.0,
            &virial_terms
        ) != BBTC_STATUS_SUCCESS
        || virial_terms.second_density_virial_coefficient_m3_per_kg != 0.0
        || virial_terms.first_temperature_derivative_m3_per_kg_k != 0.0
        || virial_terms.second_temperature_derivative_m3_per_kg_k2 != 0.0)
    {
        return 1;
    }




    /*
     * Exercise the IB0.3j closure API from C++ using value initialization plus
     * named member assignment. This intentionally avoids positional aggregate
     * initialization so C++ interoperability does not accidentally freeze
     * public record field order as an ABI assumption.
     */
    bbtc_ib_initial_gas_state_double_t closure_initial_gas_state = {};
    closure_initial_gas_state.absolute_pressure_pa = 600.0;
    closure_initial_gas_state.temperature_k = 3.0;

    bbtc_ib_noble_abel_gas_model_double_t closure_noble_abel_model = {};
    closure_noble_abel_model.specific_gas_constant_j_per_kg_k = 100.0;
    closure_noble_abel_model.constant_volume_specific_heat_j_per_kg_k = 500.0;
    closure_noble_abel_model.covolume_m3_per_kg = 0.0;

    const double closure_virial_coefficients[] =
    {
        0.0
    };

    bbtc_ib_first_order_virial_gas_model_double_t closure_virial_model = {};
    closure_virial_model.specific_gas_constant_j_per_kg_k = 100.0;
    closure_virial_model.ideal_gas_constant_volume_specific_heat_j_per_kg_k =
        500.0;
    closure_virial_model.minimum_calibrated_density_kg_per_m3 = 0.0;
    closure_virial_model.maximum_calibrated_density_kg_per_m3 = 5.0;
    closure_virial_model.second_density_virial_coefficient_law
        .minimum_temperature_k = 1.0;
    closure_virial_model.second_density_virial_coefficient_law
        .maximum_temperature_k = 5.0;
    closure_virial_model.second_density_virial_coefficient_law
        .second_density_virial_chebyshev_coefficients_m3_per_kg =
            closure_virial_coefficients;
    closure_virial_model.second_density_virial_coefficient_law
        .coefficient_count = 1U;

    bbtc_ib_initial_gas_solution_double_t closure_noble_abel_solution = {};
    bbtc_ib_initial_gas_solution_double_t closure_virial_solution = {};

    if (bbtc_ib_noble_abel_initial_gas_solve_double(
            &closure_noble_abel_model,
            &closure_initial_gas_state,
            0.25,
            &closure_noble_abel_solution
        ) != BBTC_STATUS_SUCCESS
        || bbtc_ib_first_order_virial_initial_gas_solve_double(
            &closure_virial_model,
            &closure_initial_gas_state,
            0.25,
            &closure_virial_solution
        ) != BBTC_STATUS_SUCCESS)
    {
        return 1;
    }

    if (closure_noble_abel_solution.applicability_flags
            != BBTC_APPLICABILITY_NONE_REPORTED
        || closure_virial_solution.applicability_flags
            != BBTC_APPLICABILITY_NONE_REPORTED
        || closure_noble_abel_solution.density_kg_per_m3 != 2.0
        || closure_noble_abel_solution.gas_mass_kg != 0.5
        || closure_virial_solution.density_kg_per_m3 != 2.0
        || closure_virial_solution.gas_mass_kg != 0.5)
    {
        return 1;
    }


    bbtc_ib_caloric_reference_double_t caloric_reference = {};
    caloric_reference.reference_temperature_k = 300.0;
    caloric_reference.reference_specific_internal_energy_j_per_kg = 1000.0;

    bbtc_ib_noble_abel_gas_model_double_t thermodynamic_noble_abel_model = {};
    thermodynamic_noble_abel_model.specific_gas_constant_j_per_kg_k = 100.0;
    thermodynamic_noble_abel_model.constant_volume_specific_heat_j_per_kg_k =
        500.0;
    thermodynamic_noble_abel_model.covolume_m3_per_kg = 0.0;

    const double thermodynamic_virial_coefficients[] =
    {
        0.0
    };

    bbtc_ib_first_order_virial_gas_model_double_t thermodynamic_virial_model =
        {};
    thermodynamic_virial_model.specific_gas_constant_j_per_kg_k = 100.0;
    thermodynamic_virial_model
        .ideal_gas_constant_volume_specific_heat_j_per_kg_k = 500.0;
    thermodynamic_virial_model.minimum_calibrated_density_kg_per_m3 = 0.0;
    thermodynamic_virial_model.maximum_calibrated_density_kg_per_m3 = 5.0;
    thermodynamic_virial_model.second_density_virial_coefficient_law
        .minimum_temperature_k = 200.0;
    thermodynamic_virial_model.second_density_virial_coefficient_law
        .maximum_temperature_k = 400.0;
    thermodynamic_virial_model.second_density_virial_coefficient_law
        .second_density_virial_chebyshev_coefficients_m3_per_kg =
            thermodynamic_virial_coefficients;
    thermodynamic_virial_model.second_density_virial_coefficient_law
        .coefficient_count = 1U;

    bbtc_ib_reduced_gas_thermodynamic_result_double_t noble_abel_result = {};
    bbtc_ib_reduced_gas_thermodynamic_result_double_t virial_result = {};

    if (bbtc_ib_caloric_reference_validate_double(&caloric_reference)
            != BBTC_STATUS_SUCCESS
        || bbtc_ib_noble_abel_thermodynamics_evaluate_double(
            &thermodynamic_noble_abel_model,
            2.0,
            300.0,
            &caloric_reference,
            &noble_abel_result
        ) != BBTC_STATUS_SUCCESS
        || bbtc_ib_first_order_virial_thermodynamics_evaluate_double(
            &thermodynamic_virial_model,
            2.0,
            300.0,
            &caloric_reference,
            &virial_result
        ) != BBTC_STATUS_SUCCESS)
    {
        return 1;
    }

    if (noble_abel_result.applicability_flags
            != BBTC_APPLICABILITY_NONE_REPORTED
        || virial_result.applicability_flags
            != BBTC_APPLICABILITY_NONE_REPORTED
        || noble_abel_result.pressure_pa != 60000.0
        || noble_abel_result.specific_internal_energy_j_per_kg != 1000.0
        || noble_abel_result.constant_volume_specific_heat_j_per_kg_k != 500.0
        || noble_abel_result
            .pressure_density_derivative_at_constant_temperature_pa_m3_per_kg
            != 30000.0
        || noble_abel_result
            .pressure_temperature_derivative_at_constant_density_pa_per_k
            != 200.0
        || virial_result.pressure_pa != noble_abel_result.pressure_pa
        || virial_result.specific_internal_energy_j_per_kg
            != noble_abel_result.specific_internal_energy_j_per_kg
        || virial_result.constant_volume_specific_heat_j_per_kg_k
            != noble_abel_result.constant_volume_specific_heat_j_per_kg_k
        || virial_result
            .pressure_density_derivative_at_constant_temperature_pa_m3_per_kg
            != noble_abel_result
                .pressure_density_derivative_at_constant_temperature_pa_m3_per_kg
        || virial_result
            .pressure_temperature_derivative_at_constant_density_pa_per_k
            != noble_abel_result
                .pressure_temperature_derivative_at_constant_density_pa_per_k)
    {
        return 1;
    }


    if (
        std::strcmp(
            bbtc_ib_termination_string(BBTC_IB_TERMINATION_NOT_RUN),
            "simulation not run"
        ) != 0
    )
    {
        return 1;
    }

    if (
        std::strcmp(
            bbtc_warning_flag_string(BBTC_WARNING_NONE),
            "no warning reported"
        ) != 0
    )
    {
        return 1;
    }

    return std::strcmp(
        bbtc_applicability_flag_string(
            BBTC_APPLICABILITY_NONE_REPORTED
        ),
        "no applicability limitation reported"
    );
}
