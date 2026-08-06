#include <bbtc/bbtc.h>

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


int main()
{
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


    const bbtc_ib_projectile_double_t projectile =
    {
        0.01134
    };

    if (bbtc_ib_projectile_validate_double(&projectile) != BBTC_STATUS_SUCCESS)
    {
        return 1;
    }


    const bbtc_ib_propellant_charge_double_t charge =
    {
        0.0030,
        1600.0
    };

    if (bbtc_ib_propellant_charge_validate_double(&charge) != BBTC_STATUS_SUCCESS)
    {
        return 1;
    }

    const bbtc_ib_loading_state_double_t loading_state =
    {
        {
            8.0,
            1.0,
            1.0,
            1.0
        },
        {
            1.0
        },
        {
            6.0,
            2.0
        }
    };
    bbtc_ib_loading_state_volumes_double_t loading_volumes = {};

    if (bbtc_ib_loading_state_evaluate_double(&loading_state, &loading_volumes)
            != BBTC_STATUS_SUCCESS
        || loading_volumes.condensed_propellant_volume_m3 != 3.0
        || loading_volumes.initial_free_gas_volume_m3 != 5.0)
    {
        return 1;
    }


    const bbtc_ib_initial_gas_state_double_t initial_gas_state =
    {
        101325.0,
        293.15
    };

    if (bbtc_ib_initial_gas_state_validate_double(&initial_gas_state)
        != BBTC_STATUS_SUCCESS)
    {
        return 1;
    }


    const bbtc_ib_noble_abel_gas_model_double_t gas_model =
    {
        287.0,
        718.0,
        0.001
    };

    if (bbtc_ib_noble_abel_gas_model_validate_double(&gas_model)
        != BBTC_STATUS_SUCCESS)
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
