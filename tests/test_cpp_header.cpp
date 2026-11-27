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
    std::is_same<decltype(bbtc_eb_vec3_float_t{}.x), float>::value,
    "float EB vector must use native float components"
);

static_assert(
    std::is_same<decltype(bbtc_eb_vec3_double_t{}.z), double>::value,
    "double EB vector must use native double components"
);

static_assert(
    std::is_same<
        decltype(bbtc_eb_vec3_long_double_t{}.y),
        long double
    >::value,
    "long-double EB vector must use native long-double components"
);

static_assert(
    std::is_same<decltype(bbtc_eb_quat_double_t{}.w), double>::value,
    "double EB quaternion scalar component must use native double"
);

static_assert(
    std::is_same<
        decltype(bbtc_eb_rigid_body_state_float_t{}
                     .orientation_world_from_body),
        bbtc_eb_quat_float_t
    >::value,
    "float EB state orientation must use the native float quaternion"
);

static_assert(
    std::is_same<
        decltype(bbtc_eb_rigid_body_state_double_t{}.position_m),
        bbtc_eb_vec3_double_t
    >::value,
    "double EB state position must use the native double vector"
);

static_assert(
    std::is_same<
        decltype(bbtc_eb_rigid_body_state_double_t{}.velocity_m_per_s),
        bbtc_eb_vec3_double_t
    >::value,
    "double EB state velocity must use the native double vector"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_eb_rigid_body_state_long_double_t{}
                .angular_velocity_body_rad_per_s
        ),
        bbtc_eb_vec3_long_double_t
    >::value,
    "long-double EB state angular velocity must use its native vector"
);

static_assert(
    std::is_same<
        decltype(bbtc_eb_rigid_body_state_long_double_t{}.time_s),
        long double
    >::value,
    "long-double EB state time must use native long double"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_eb_principal_inertia_float_t{}.moment_about_x_kg_m2
        ),
        float
    >::value,
    "float EB principal inertia must use native float"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_eb_principal_inertia_double_t{}.moment_about_y_kg_m2
        ),
        double
    >::value,
    "double EB principal inertia must use native double"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_eb_principal_inertia_long_double_t{}.moment_about_z_kg_m2
        ),
        long double
    >::value,
    "long-double EB principal inertia must use native long double"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_eb_projectile_properties_double_t{}.principal_inertia
        ),
        bbtc_eb_principal_inertia_double_t
    >::value,
    "double EB projectile properties must own double principal inertia"
);

static_assert(
    std::is_same<
        decltype(bbtc_eb_projectile_properties_float_t{}.mass_kg),
        float
    >::value,
    "float EB projectile properties must use native float"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_eb_projectile_properties_long_double_t{}.reference_area_m2
        ),
        long double
    >::value,
    "long-double EB projectile properties must use native long double"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_eb_environment_sample_float_t{}.air_velocity_world_m_per_s
        ),
        bbtc_eb_vec3_float_t
    >::value,
    "float EB environment air velocity must use native float vector"
);

static_assert(
    std::is_same<
        decltype(bbtc_eb_environment_sample_double_t{}.temperature_k),
        double
    >::value,
    "double EB environment temperature must use native double"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_eb_environment_sample_long_double_t{}
                .speed_of_sound_m_per_s
        ),
        long double
    >::value,
    "long-double EB environment sound speed must use native long double"
);

static_assert(
    std::is_same<
        decltype(bbtc_eb_environment_provider_float_t{}.sample),
        bbtc_eb_environment_provider_float_fn
    >::value,
    "float EB environment provider must use native float callback type"
);

static_assert(
    std::is_same<
        decltype(bbtc_eb_environment_provider_double_t{}.context),
        const void*
    >::value,
    "EB environment provider context must be borrowed const void pointer"
);

static_assert(
    std::is_same<
        decltype(bbtc_eb_environment_provider_long_double_t{}.sample),
        bbtc_eb_environment_provider_long_double_fn
    >::value,
    "long-double EB environment provider must use native callback type"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_eb_reference_frame_sample_float_t{}
                .gravitational_acceleration_reference_m_per_s2
        ),
        bbtc_eb_vec3_float_t
    >::value,
    "float EB reference-frame gravity must use native float vector"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_eb_reference_frame_sample_double_t{}
                .reference_frame_origin_acceleration_reference_m_per_s2
        ),
        bbtc_eb_vec3_double_t
    >::value,
    "double EB reference-frame origin acceleration must use native vector"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_eb_reference_frame_sample_long_double_t{}
                .reference_frame_angular_velocity_reference_rad_per_s
        ),
        bbtc_eb_vec3_long_double_t
    >::value,
    "long-double EB frame angular velocity must use native vector"
);

static_assert(
    std::is_same<
        decltype(bbtc_eb_reference_frame_provider_float_t{}.sample),
        bbtc_eb_reference_frame_provider_float_fn
    >::value,
    "float EB reference-frame provider must use native float callback type"
);

static_assert(
    std::is_same<
        decltype(bbtc_eb_reference_frame_provider_double_t{}.context),
        const void*
    >::value,
    "EB reference-frame provider context must be borrowed const void pointer"
);

static_assert(
    std::is_same<
        decltype(bbtc_eb_reference_frame_provider_long_double_t{}.sample),
        bbtc_eb_reference_frame_provider_long_double_fn
    >::value,
    "long-double EB reference-frame provider must use native callback type"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_eb_force_moment_float_t{}.applied_force_reference_n
        ),
        bbtc_eb_vec3_float_t
    >::value,
    "float EB applied force must use native float vector"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_eb_force_moment_double_t{}.applied_moment_body_n_m
        ),
        bbtc_eb_vec3_double_t
    >::value,
    "double EB applied moment must use native double vector"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_eb_force_moment_long_double_t{}.applied_force_reference_n
        ),
        bbtc_eb_vec3_long_double_t
    >::value,
    "long-double EB applied force must use native long-double vector"
);


template<typename...>
using bbtc_test_void_t = void;

template<typename T, typename = void>
struct bbtc_test_has_time_derivative : std::false_type
{
};

template<typename T>
struct bbtc_test_has_time_derivative<
    T,
    bbtc_test_void_t<decltype(((T*)0)->time_derivative)>
> : std::true_type
{
};

static_assert(
    std::is_same<
        decltype(
            bbtc_eb_rigid_body_state_derivative_float_t{}
                .orientation_world_from_body_derivative_per_s
        ),
        bbtc_eb_quat_float_t
    >::value,
    "float EB orientation derivative must use native float quaternion"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_eb_rigid_body_state_derivative_double_t{}
                .velocity_derivative_reference_m_per_s2
        ),
        bbtc_eb_vec3_double_t
    >::value,
    "double EB velocity derivative must use native double vector"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_eb_rigid_body_state_derivative_long_double_t{}
                .angular_velocity_derivative_body_rad_per_s2
        ),
        bbtc_eb_vec3_long_double_t
    >::value,
    "long-double EB angular-velocity derivative must use native vector"
);

static_assert(
    !bbtc_test_has_time_derivative<
        bbtc_eb_rigid_body_state_derivative_float_t
    >::value,
    "B9 derivative must not contain a time_derivative member"
);
static_assert(
    !bbtc_test_has_time_derivative<
        bbtc_eb_rigid_body_state_derivative_double_t
    >::value,
    "B9 double derivative must not contain a time_derivative member"
);
static_assert(
    !bbtc_test_has_time_derivative<
        bbtc_eb_rigid_body_state_derivative_long_double_t
    >::value,
    "B9 long-double derivative must not contain a time_derivative member"
);


static_assert(
    std::is_same<
        decltype(bbtc_eb_state_derivative_provider_float_t{}.evaluate),
        bbtc_eb_state_derivative_provider_float_fn
    >::value,
    "float EB state-derivative provider must use native callback type"
);

static_assert(
    std::is_same<
        decltype(bbtc_eb_state_derivative_provider_double_t{}.context),
        const void*
    >::value,
    "EB state-derivative provider context must be borrowed const void pointer"
);

static_assert(
    std::is_same<
        decltype(bbtc_eb_state_derivative_provider_long_double_t{}.evaluate),
        bbtc_eb_state_derivative_provider_long_double_fn
    >::value,
    "long-double EB state-derivative provider must use native callback type"
);



using bbtc_test_inertial_vacuum_float_fn = bbtc_status_e (*)(
    const bbtc_eb_rigid_body_state_float_t*,
    const bbtc_eb_projectile_properties_float_t*,
    const bbtc_eb_reference_frame_sample_float_t*,
    const bbtc_eb_force_moment_float_t*,
    bbtc_eb_rigid_body_state_derivative_float_t*
);

using bbtc_test_inertial_vacuum_double_fn = bbtc_status_e (*)(
    const bbtc_eb_rigid_body_state_double_t*,
    const bbtc_eb_projectile_properties_double_t*,
    const bbtc_eb_reference_frame_sample_double_t*,
    const bbtc_eb_force_moment_double_t*,
    bbtc_eb_rigid_body_state_derivative_double_t*
);

using bbtc_test_inertial_vacuum_long_double_fn = bbtc_status_e (*)(
    const bbtc_eb_rigid_body_state_long_double_t*,
    const bbtc_eb_projectile_properties_long_double_t*,
    const bbtc_eb_reference_frame_sample_long_double_t*,
    const bbtc_eb_force_moment_long_double_t*,
    bbtc_eb_rigid_body_state_derivative_long_double_t*
);

static_assert(
    std::is_same<
        decltype(&bbtc_eb_inertial_vacuum_derivative_evaluate_float),
        bbtc_test_inertial_vacuum_float_fn
    >::value,
    "float EB1.1 inertial-vacuum evaluator must use the frozen signature"
);

static_assert(
    std::is_same<
        decltype(&bbtc_eb_inertial_vacuum_derivative_evaluate_double),
        bbtc_test_inertial_vacuum_double_fn
    >::value,
    "double EB1.1 inertial-vacuum evaluator must use the frozen signature"
);

static_assert(
    std::is_same<
        decltype(&bbtc_eb_inertial_vacuum_derivative_evaluate_long_double),
        bbtc_test_inertial_vacuum_long_double_fn
    >::value,
    "long-double EB1.1 inertial-vacuum evaluator must use the frozen signature"
);



using bbtc_test_vacuum_provider_validate_float_fn = bbtc_status_e (*)(
    const bbtc_eb_inertial_vacuum_provider_config_float_t*
);
using bbtc_test_vacuum_provider_validate_double_fn = bbtc_status_e (*)(
    const bbtc_eb_inertial_vacuum_provider_config_double_t*
);
using bbtc_test_vacuum_provider_validate_long_double_fn = bbtc_status_e (*)(
    const bbtc_eb_inertial_vacuum_provider_config_long_double_t*
);

using bbtc_test_vacuum_provider_bind_float_fn = bbtc_status_e (*)(
    const bbtc_eb_inertial_vacuum_provider_config_float_t*,
    bbtc_eb_state_derivative_provider_float_t*
);
using bbtc_test_vacuum_provider_bind_double_fn = bbtc_status_e (*)(
    const bbtc_eb_inertial_vacuum_provider_config_double_t*,
    bbtc_eb_state_derivative_provider_double_t*
);
using bbtc_test_vacuum_provider_bind_long_double_fn = bbtc_status_e (*)(
    const bbtc_eb_inertial_vacuum_provider_config_long_double_t*,
    bbtc_eb_state_derivative_provider_long_double_t*
);

static_assert(
    std::is_same<
        decltype(&bbtc_eb_inertial_vacuum_provider_config_validate_float),
        bbtc_test_vacuum_provider_validate_float_fn
    >::value,
    "float EB1.2 config validator must use the frozen signature"
);

static_assert(
    std::is_same<
        decltype(&bbtc_eb_inertial_vacuum_provider_config_validate_double),
        bbtc_test_vacuum_provider_validate_double_fn
    >::value,
    "double EB1.2 config validator must use the frozen signature"
);

static_assert(
    std::is_same<
        decltype(&bbtc_eb_inertial_vacuum_provider_config_validate_long_double),
        bbtc_test_vacuum_provider_validate_long_double_fn
    >::value,
    "long-double EB1.2 config validator must use the frozen signature"
);

static_assert(
    std::is_same<
        decltype(&bbtc_eb_inertial_vacuum_provider_bind_float),
        bbtc_test_vacuum_provider_bind_float_fn
    >::value,
    "float EB1.2 binder must use the frozen signature"
);

static_assert(
    std::is_same<
        decltype(&bbtc_eb_inertial_vacuum_provider_bind_double),
        bbtc_test_vacuum_provider_bind_double_fn
    >::value,
    "double EB1.2 binder must use the frozen signature"
);

static_assert(
    std::is_same<
        decltype(&bbtc_eb_inertial_vacuum_provider_bind_long_double),
        bbtc_test_vacuum_provider_bind_long_double_fn
    >::value,
    "long-double EB1.2 binder must use the frozen signature"
);

static_assert(
    std::is_same<
        decltype(bbtc_eb_inertial_vacuum_provider_config_double_t{}.projectile),
        bbtc_eb_projectile_properties_double_t
    >::value,
    "EB1.2 projectile configuration must be stored by value"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_eb_inertial_vacuum_provider_config_double_t{}
                .constant_force_moment
        ),
        bbtc_eb_force_moment_double_t
    >::value,
    "EB1.2 constant force/moment configuration must be stored by value"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_eb_inertial_vacuum_provider_config_double_t{}
                .reference_frame_provider
        ),
        bbtc_eb_reference_frame_provider_double_t
    >::value,
    "EB1.2 B7 provider descriptor must be stored by value"
);


using bbtc_test_rk4_float_fn = bbtc_status_e (*)(
    const bbtc_eb_state_derivative_provider_float_t*,
    float,
    bbtc_eb_rigid_body_state_float_t*
);

using bbtc_test_rk4_double_fn = bbtc_status_e (*)(
    const bbtc_eb_state_derivative_provider_double_t*,
    double,
    bbtc_eb_rigid_body_state_double_t*
);

using bbtc_test_rk4_long_double_fn = bbtc_status_e (*)(
    const bbtc_eb_state_derivative_provider_long_double_t*,
    long double,
    bbtc_eb_rigid_body_state_long_double_t*
);

static_assert(
    std::is_same<
        decltype(&bbtc_eb_rk4_step_float),
        bbtc_test_rk4_float_fn
    >::value,
    "float EB RK4 step must use the frozen native B11 signature"
);

static_assert(
    std::is_same<
        decltype(&bbtc_eb_rk4_step_double),
        bbtc_test_rk4_double_fn
    >::value,
    "double EB RK4 step must use the frozen native B11 signature"
);

static_assert(
    std::is_same<
        decltype(&bbtc_eb_rk4_step_long_double),
        bbtc_test_rk4_long_double_fn
    >::value,
    "long-double EB RK4 step must use the frozen native B11 signature"
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


static_assert(
    std::is_same<
        decltype(
            bbtc_ib_pressure_power_burn_kinetics_float_t{}
                .reference_burn_rate_m_per_s
        ),
        float
    >::value,
    "float pressure-power kinetics must use native float fields"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_ib_pressure_power_burn_kinetics_double_t{}
                .pressure_exponent
        ),
        double
    >::value,
    "double pressure-power kinetics must use native double fields"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_ib_pressure_power_burn_kinetics_long_double_t{}
                .maximum_calibrated_pressure_pa
        ),
        long double
    >::value,
    "long-double pressure-power kinetics must use native long-double fields"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_ib_propellant_burn_kinetics_result_double_t{}
                .applicability_flags
        ),
        bbtc_applicability_flags_t
    >::value,
    "burn-kinetics result must use the public applicability-mask type"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_ib_propellant_burn_kinetics_result_long_double_t{}
                .burn_rate_m_per_s
        ),
        long double
    >::value,
    "long-double burn-kinetics result must use native long double"
);


static_assert(
    std::is_same<
        decltype(
            bbtc_ib_pressure_burn_point_float_t{}
                .pressure_pa
        ),
        float
    >::value,
    "float pressure-burn point must use native float pressure"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_ib_pressure_burn_point_long_double_t{}
                .burn_rate_m_per_s
        ),
        long double
    >::value,
    "long-double pressure-burn point must use native long-double burn rate"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_ib_pressure_burn_table_double_t{}
                .points
        ),
        const bbtc_ib_pressure_burn_point_double_t*
    >::value,
    "pressure-burn table must borrow immutable matching point storage"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_ib_pressure_burn_table_long_double_t{}
                .point_count
        ),
        std::size_t
    >::value,
    "pressure-burn table point count must use size_t"
);


static_assert(
    std::is_same<
        decltype(
            bbtc_ib_propellant_mass_result_float_t{}
                .remaining_mass_kg
        ),
        float
    >::value,
    "float propellant-mass result must use native float"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_ib_propellant_mass_result_double_t{}
                .reacted_mass_rate_kg_per_s
        ),
        double
    >::value,
    "double propellant-mass rate must use native double"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_ib_propellant_mass_result_long_double_t{}
                .equivalent_population_scale
        ),
        long double
    >::value,
    "long-double propellant population scale must use native long double"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_ib_propellant_mass_result_double_t{}
                .applicability_flags
        ),
        bbtc_applicability_flags_t
    >::value,
    "propellant-mass result must use public applicability-mask type"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_ib_propellant_source_rate_float_t{}
                .gas_product_mass_rate_kg_per_s
        ),
        float
    >::value,
    "float propellant source rate must use native float"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_ib_propellant_source_rate_double_t{}
                .reaction_energy_release_rate_w
        ),
        double
    >::value,
    "double reaction-energy release rate must use native double"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_ib_propellant_source_rate_long_double_t{}
                .condensed_product_mass_rate_kg_per_s
        ),
        long double
    >::value,
    "long-double propellant source rate must use native long double"
);

static_assert(
    std::is_same<
        decltype(
            bbtc_ib_propellant_source_rate_double_t{}
                .applicability_flags
        ),
        bbtc_applicability_flags_t
    >::value,
    "propellant source-rate result must use public applicability-mask type"
);


extern "C" bbtc_status_e
cpp_environment_provider(
    const void* context,
    const bbtc_eb_vec3_double_t* position_world_m,
    double time_s,
    bbtc_eb_environment_sample_double_t* out_sample
)
{
    (void)context;

    out_sample->air_velocity_world_m_per_s.x = position_world_m->x + time_s;
    out_sample->air_velocity_world_m_per_s.y = position_world_m->y;
    out_sample->air_velocity_world_m_per_s.z = position_world_m->z;
    out_sample->temperature_k = 288.15;
    out_sample->absolute_pressure_pa = 101325.0;
    out_sample->air_density_kg_per_m3 = 1.225;
    out_sample->speed_of_sound_m_per_s = 340.29;

    return BBTC_STATUS_SUCCESS;
}

/**
 * @brief Native-double B7 callback used by the C++ umbrella-header test.
 */
extern "C" bbtc_status_e
cpp_reference_frame_provider(
    const void* context,
    const bbtc_eb_vec3_double_t* position_reference_m,
    double time_s,
    bbtc_eb_reference_frame_sample_double_t* out_sample
)
{
    (void)context;

    *out_sample = bbtc_eb_reference_frame_sample_double_t{};
    out_sample->gravitational_acceleration_reference_m_per_s2.x =
        position_reference_m->x + time_s;
    out_sample->gravitational_acceleration_reference_m_per_s2.z = 9.81;
    out_sample->reference_frame_angular_velocity_reference_rad_per_s.y =
        time_s;

    return BBTC_STATUS_SUCCESS;
}



int main()
{
    /*
     * Exercise the EB0.1-B2 public validator through the C++ umbrella-header
     * boundary. The state is intentionally populated by named assignment so
     * this test does not introduce positional aggregate-layout assumptions.
     */
    bbtc_eb_rigid_body_state_double_t eb_state = {};
    eb_state.orientation_world_from_body.w = 1.0;
    eb_state.position_m.x = -1.0;
    eb_state.position_m.y = 2.0;
    eb_state.position_m.z = -3.0;
    eb_state.velocity_m_per_s.x = 800.0;
    eb_state.angular_velocity_body_rad_per_s.x = 1000.0;
    eb_state.time_s = 0.0;

    if (bbtc_eb_rigid_body_state_validate_double(&eb_state)
        != BBTC_STATUS_SUCCESS)
    {
        return 1;
    }

    bbtc_eb_projectile_properties_double_t eb_projectile = {};
    eb_projectile.principal_inertia.moment_about_x_kg_m2 = 1.0;
    eb_projectile.principal_inertia.moment_about_y_kg_m2 = 2.0;
    eb_projectile.principal_inertia.moment_about_z_kg_m2 = 2.0;
    eb_projectile.mass_kg = 1.0;
    eb_projectile.reference_diameter_m = 2.0;
    eb_projectile.reference_area_m2 = 1.0;

    if (bbtc_eb_projectile_properties_validate_double(&eb_projectile)
        != BBTC_STATUS_SUCCESS)
    {
        return 1;
    }

    /*
     * Exercise EB0.1-B4 through the C++ umbrella-header boundary using named
     * assignment so this compatibility test does not freeze aggregate
     * initialization or incidental padding as part of the public ABI.
     */
    bbtc_eb_environment_sample_double_t eb_environment = {};
    eb_environment.air_velocity_world_m_per_s.x = -5.0;
    eb_environment.air_velocity_world_m_per_s.y = 1.0;
    eb_environment.temperature_k = 288.15;
    eb_environment.absolute_pressure_pa = 101325.0;
    eb_environment.air_density_kg_per_m3 = 1.225;
    eb_environment.speed_of_sound_m_per_s = 340.29;

    if (bbtc_eb_environment_sample_validate_double(&eb_environment)
        != BBTC_STATUS_SUCCESS)
    {
        return 1;
    }

    bbtc_eb_environment_provider_double_t eb_provider = {};
    eb_provider.sample = cpp_environment_provider;
    eb_provider.context = nullptr;

    bbtc_eb_vec3_double_t eb_provider_position = {};
    eb_provider_position.x = 10.0;
    eb_provider_position.y = 20.0;
    eb_provider_position.z = 30.0;

    bbtc_eb_environment_sample_double_t eb_provider_sample = {};

    if (bbtc_eb_environment_provider_sample_double(
            &eb_provider,
            &eb_provider_position,
            2.0,
            &eb_provider_sample
        ) != BBTC_STATUS_SUCCESS
        || eb_provider_sample.air_velocity_world_m_per_s.x != 12.0)
    {
        return 1;
    }

    /*
     * Exercise EB0.1-B6 through the C++ umbrella-header boundary.
     */
    bbtc_eb_reference_frame_sample_double_t eb_reference_frame = {};
    eb_reference_frame.gravitational_acceleration_reference_m_per_s2.z =
        9.81;
    eb_reference_frame
        .reference_frame_origin_acceleration_reference_m_per_s2.x = -0.02;
    eb_reference_frame
        .reference_frame_angular_velocity_reference_rad_per_s.y =
            7.292115e-5;

    if (bbtc_eb_reference_frame_sample_validate_double(&eb_reference_frame)
        != BBTC_STATUS_SUCCESS)
    {
        return 1;
    }


    /*
     * Exercise EB0.1-B7 through the C++ umbrella-header boundary. This proves
     * that callback, descriptor, position, dispatch, and B6 output types are
     * usable without private headers.
     */
    bbtc_eb_reference_frame_provider_double_t eb_reference_provider = {};
    eb_reference_provider.sample = cpp_reference_frame_provider;
    eb_reference_provider.context = nullptr;

    bbtc_eb_vec3_double_t eb_reference_provider_position = {};
    eb_reference_provider_position.x = 10.0;
    eb_reference_provider_position.y = 20.0;
    eb_reference_provider_position.z = 30.0;

    bbtc_eb_reference_frame_sample_double_t eb_reference_provider_sample = {};

    if (bbtc_eb_reference_frame_provider_sample_double(
            &eb_reference_provider,
            &eb_reference_provider_position,
            2.0,
            &eb_reference_provider_sample
        ) != BBTC_STATUS_SUCCESS
        || eb_reference_provider_sample
               .gravitational_acceleration_reference_m_per_s2.x != 12.0
        || eb_reference_provider_sample
               .reference_frame_angular_velocity_reference_rad_per_s.y != 2.0)
    {
        return 1;
    }


    /*
     * Exercise EB0.1-B8 through the C++ umbrella-header boundary. Force and
     * moment are deliberately populated independently so this test verifies
     * representation/validation without implying equations-of-motion coupling.
     */
    bbtc_eb_force_moment_double_t eb_force_moment = {};
    eb_force_moment.applied_force_reference_n.x = -125.0;
    eb_force_moment.applied_force_reference_n.y = 4.5;
    eb_force_moment.applied_moment_body_n_m.y = -7.0;
    eb_force_moment.applied_moment_body_n_m.z = 42.0;

    if (bbtc_eb_force_moment_validate_double(&eb_force_moment)
        != BBTC_STATUS_SUCCESS)
    {
        return 1;
    }


    /*
     * Exercise EB0.1-B9 through the C++ umbrella-header boundary. The
     * quaternion derivative is deliberately non-unit to prove that B9 does not
     * apply orientation-state normalization rules to derivative storage.
     */
    bbtc_eb_rigid_body_state_derivative_double_t eb_derivative = {};
    eb_derivative.orientation_world_from_body_derivative_per_s.x = 2.0;
    eb_derivative.orientation_world_from_body_derivative_per_s.w = -5.0;
    eb_derivative.position_derivative_reference_m_per_s.x = 700.0;
    eb_derivative.velocity_derivative_reference_m_per_s2.z = -9.81;
    eb_derivative.angular_velocity_derivative_body_rad_per_s2.y = 4.0;

    if (bbtc_eb_rigid_body_state_derivative_validate_double(&eb_derivative)
        != BBTC_STATUS_SUCCESS)
    {
        return 1;
    }


    /*
     * Exercise EB0.1-B10 through the C++ umbrella-header boundary.
     */
    const bbtc_eb_state_derivative_provider_double_t eb_derivative_provider =
    {
        [](const void*,
           const bbtc_eb_rigid_body_state_double_t* state,
           bbtc_eb_rigid_body_state_derivative_double_t* out_derivative)
            -> bbtc_status_e
        {
            out_derivative->position_derivative_reference_m_per_s =
                state->velocity_m_per_s;
            return BBTC_STATUS_SUCCESS;
        },
        nullptr
    };

    const bbtc_eb_rigid_body_state_double_t eb_provider_state =
    {
        {0.0, 0.0, 0.0, 1.0},
        {0.0, 0.0, 0.0},
        {100.0, 2.0, -3.0},
        {0.0, 0.0, 0.0},
        1.0
    };

    bbtc_eb_rigid_body_state_derivative_double_t eb_provider_derivative = {};

    if (bbtc_eb_state_derivative_provider_evaluate_double(
            &eb_derivative_provider,
            &eb_provider_state,
            &eb_provider_derivative
        ) != BBTC_STATUS_SUCCESS
        || eb_provider_derivative.position_derivative_reference_m_per_s.x
            != 100.0)
    {
        return 1;
    }


    /*
     * Exercise EB0.1-B11 through the C++ umbrella-header boundary. The B10
     * callback supplies constant translational velocity derivative data, so
     * RK4 advances position exactly for this synthetic case.
     */
    const bbtc_eb_state_derivative_provider_double_t eb_rk4_provider =
    {
        [](const void*,
           const bbtc_eb_rigid_body_state_double_t* state,
           bbtc_eb_rigid_body_state_derivative_double_t* out_derivative)
            -> bbtc_status_e
        {
            out_derivative->position_derivative_reference_m_per_s =
                state->velocity_m_per_s;
            return BBTC_STATUS_SUCCESS;
        },
        nullptr
    };

    bbtc_eb_rigid_body_state_double_t eb_rk4_state =
    {
        {0.0, 0.0, 0.0, 1.0},
        {0.0, 0.0, 0.0},
        {100.0, 2.0, -3.0},
        {0.0, 0.0, 0.0},
        0.0
    };

    if (bbtc_eb_rk4_step_double(&eb_rk4_provider, 0.5, &eb_rk4_state)
            != BBTC_STATUS_SUCCESS
        || eb_rk4_state.position_m.x != 50.0
        || eb_rk4_state.position_m.y != 1.0
        || eb_rk4_state.position_m.z != -1.5
        || eb_rk4_state.time_s != 0.5)
    {
        return 1;
    }


    /*
     * Exercise EB1.1 inertial-vacuum derivative evaluation through the C++
     * umbrella-header boundary using only public types and functions.
     */
    bbtc_eb_rigid_body_state_double_t eb_vacuum_state = {};
    eb_vacuum_state.orientation_world_from_body.w = 1.0;
    eb_vacuum_state.velocity_m_per_s.x = 10.0;
    eb_vacuum_state.angular_velocity_body_rad_per_s.x = 2.0;

    bbtc_eb_projectile_properties_double_t eb_vacuum_projectile = {};
    eb_vacuum_projectile.principal_inertia.moment_about_x_kg_m2 = 3.0;
    eb_vacuum_projectile.principal_inertia.moment_about_y_kg_m2 = 3.0;
    eb_vacuum_projectile.principal_inertia.moment_about_z_kg_m2 = 3.0;
    eb_vacuum_projectile.mass_kg = 2.0;
    eb_vacuum_projectile.reference_diameter_m = 1.0;
    eb_vacuum_projectile.reference_area_m2 = 1.0;

    bbtc_eb_reference_frame_sample_double_t eb_vacuum_reference_frame = {};
    eb_vacuum_reference_frame
        .gravitational_acceleration_reference_m_per_s2.z = -9.0;

    bbtc_eb_force_moment_double_t eb_vacuum_force_moment = {};
    eb_vacuum_force_moment.applied_force_reference_n.x = 4.0;
    eb_vacuum_force_moment.applied_moment_body_n_m.x = 6.0;

    bbtc_eb_rigid_body_state_derivative_double_t eb_vacuum_derivative = {};

    if (bbtc_eb_inertial_vacuum_derivative_evaluate_double(
            &eb_vacuum_state,
            &eb_vacuum_projectile,
            &eb_vacuum_reference_frame,
            &eb_vacuum_force_moment,
            &eb_vacuum_derivative
        ) != BBTC_STATUS_SUCCESS
        || eb_vacuum_derivative.position_derivative_reference_m_per_s.x
            != 10.0
        || eb_vacuum_derivative.velocity_derivative_reference_m_per_s2.x
            != 2.0
        || eb_vacuum_derivative.velocity_derivative_reference_m_per_s2.z
            != -9.0
        || eb_vacuum_derivative.orientation_world_from_body_derivative_per_s.x
            != 1.0
        || eb_vacuum_derivative.angular_velocity_derivative_body_rad_per_s2.x
            != 2.0)
    {
        return 1;
    }


    /*
     * Exercise EB1.2 provider binding and derivative dispatch through the C++
     * umbrella-header boundary.
     */
    bbtc_eb_inertial_vacuum_provider_config_double_t eb_vacuum_config = {};
    eb_vacuum_config.projectile.principal_inertia.moment_about_x_kg_m2 = 3.0;
    eb_vacuum_config.projectile.principal_inertia.moment_about_y_kg_m2 = 3.0;
    eb_vacuum_config.projectile.principal_inertia.moment_about_z_kg_m2 = 3.0;
    eb_vacuum_config.projectile.mass_kg = 2.0;
    eb_vacuum_config.projectile.reference_diameter_m = 1.0;
    eb_vacuum_config.projectile.reference_area_m2 = 1.0;
    eb_vacuum_config.constant_force_moment.applied_force_reference_n.x = 4.0;
    eb_vacuum_config.reference_frame_provider.sample =
        [](const void*,
           const bbtc_eb_vec3_double_t*,
           double,
           bbtc_eb_reference_frame_sample_double_t* out_sample)
            -> bbtc_status_e
        {
            *out_sample = {};
            out_sample->gravitational_acceleration_reference_m_per_s2.z = -8.0;
            return BBTC_STATUS_SUCCESS;
        };

    bbtc_eb_state_derivative_provider_double_t eb_vacuum_provider = {};
    bbtc_eb_rigid_body_state_derivative_double_t eb_bound_derivative = {};

    if (bbtc_eb_inertial_vacuum_provider_config_validate_double(
            &eb_vacuum_config
        ) != BBTC_STATUS_SUCCESS
        || bbtc_eb_inertial_vacuum_provider_bind_double(
            &eb_vacuum_config,
            &eb_vacuum_provider
        ) != BBTC_STATUS_SUCCESS
        || eb_vacuum_provider.context != &eb_vacuum_config
        || bbtc_eb_state_derivative_provider_evaluate_double(
            &eb_vacuum_provider,
            &eb_vacuum_state,
            &eb_bound_derivative
        ) != BBTC_STATUS_SUCCESS
        || eb_bound_derivative.velocity_derivative_reference_m_per_s2.x != 2.0
        || eb_bound_derivative.velocity_derivative_reference_m_per_s2.z != -8.0)
    {
        return 1;
    }

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

    bbtc_ib_propellant_mass_result_double_t source_mass = {};
    source_mass.applicability_flags =
        BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN;
    source_mass.reacted_mass_rate_kg_per_s = 2.0;

    bbtc_ib_propellant_source_rate_double_t source_rate = {};

    if (bbtc_ib_propellant_source_rate_evaluate_double(
            &thermochemistry,
            &source_mass,
            &source_rate
        ) != BBTC_STATUS_SUCCESS
        || source_rate.applicability_flags
            != BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN
        || source_rate.gas_product_mass_rate_kg_per_s != 1.5
        || source_rate.condensed_product_mass_rate_kg_per_s != 0.5
        || source_rate.reaction_energy_release_rate_w != 8.0)
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
