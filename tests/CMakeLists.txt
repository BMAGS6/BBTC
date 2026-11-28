add_executable(
    bbtc_test_status
    test_status.c
)

target_link_libraries(
    bbtc_test_status
    PRIVATE
        bbtc::bbtc
)

set_target_properties(
    bbtc_test_status
    PROPERTIES
        C_EXTENSIONS OFF
        C_STANDARD 23
        C_STANDARD_REQUIRED ON
)

add_test(
    NAME bbtc.status
    COMMAND bbtc_test_status
)

add_executable(
    bbtc_test_precision
    test_precision.c
)

target_link_libraries(
    bbtc_test_precision
    PRIVATE
        bbtc::bbtc
)

set_target_properties(
    bbtc_test_precision
    PROPERTIES
        C_EXTENSIONS OFF
        C_STANDARD 23
        C_STANDARD_REQUIRED ON
)

add_test(
    NAME bbtc.precision
    COMMAND bbtc_test_precision
)

add_executable(
    bbtc_test_diagnostics
    test_diagnostics.c
)

target_link_libraries(
    bbtc_test_diagnostics
    PRIVATE
        bbtc::bbtc
)

set_target_properties(
    bbtc_test_diagnostics
    PROPERTIES
        C_EXTENSIONS OFF
        C_STANDARD 23
        C_STANDARD_REQUIRED ON
)

add_test(
    NAME bbtc.diagnostics
    COMMAND bbtc_test_diagnostics
)

add_executable(
    bbtc_test_ib_geometry
    test_ib_geometry.c
)

target_link_libraries(
    bbtc_test_ib_geometry
    PRIVATE
        bbtc::bbtc
)

set_target_properties(
    bbtc_test_ib_geometry
    PROPERTIES
        C_EXTENSIONS OFF
        C_STANDARD 23
        C_STANDARD_REQUIRED ON
)

add_test(
    NAME bbtc.ib_geometry
    COMMAND bbtc_test_ib_geometry
)

add_executable(
    bbtc_test_ib_projectile
    test_ib_projectile.c
)

target_link_libraries(
    bbtc_test_ib_projectile
    PRIVATE
        bbtc::bbtc
)

set_target_properties(
    bbtc_test_ib_projectile
    PROPERTIES
        C_EXTENSIONS OFF
        C_STANDARD 23
        C_STANDARD_REQUIRED ON
)

add_test(
    NAME bbtc.ib_projectile
    COMMAND bbtc_test_ib_projectile
)

add_executable(
    bbtc_test_ib_propellant_charge
    test_ib_propellant_charge.c
)

target_link_libraries(
    bbtc_test_ib_propellant_charge
    PRIVATE
        bbtc::bbtc
)

set_target_properties(
    bbtc_test_ib_propellant_charge
    PROPERTIES
        C_EXTENSIONS OFF
        C_STANDARD 23
        C_STANDARD_REQUIRED ON
)

add_test(
    NAME bbtc.ib_propellant_charge
    COMMAND bbtc_test_ib_propellant_charge
)

add_executable(
    bbtc_test_ib_propellant_grain_geometry
    test_ib_propellant_grain_geometry.c
)

target_link_libraries(
    bbtc_test_ib_propellant_grain_geometry
    PRIVATE
        bbtc::bbtc
)

set_target_properties(
    bbtc_test_ib_propellant_grain_geometry
    PROPERTIES
        C_EXTENSIONS OFF
        C_STANDARD 23
        C_STANDARD_REQUIRED ON
)

add_test(
    NAME bbtc.ib_propellant_grain_geometry
    COMMAND bbtc_test_ib_propellant_grain_geometry
)

add_executable(
    bbtc_test_ib_propellant_thermochemistry
    test_ib_propellant_thermochemistry.c
)

target_link_libraries(
    bbtc_test_ib_propellant_thermochemistry
    PRIVATE
        bbtc::bbtc
)

set_target_properties(
    bbtc_test_ib_propellant_thermochemistry
    PROPERTIES
        C_EXTENSIONS OFF
        C_STANDARD 23
        C_STANDARD_REQUIRED ON
)

add_test(
    NAME bbtc.ib_propellant_thermochemistry
    COMMAND bbtc_test_ib_propellant_thermochemistry
)

add_executable(
    bbtc_test_ib_propellant_burn_kinetics
    test_ib_propellant_burn_kinetics.c
)

target_link_libraries(
    bbtc_test_ib_propellant_burn_kinetics
    PRIVATE
        bbtc::bbtc
)

set_target_properties(
    bbtc_test_ib_propellant_burn_kinetics
    PROPERTIES
        C_EXTENSIONS OFF
        C_STANDARD 23
        C_STANDARD_REQUIRED ON
)

add_test(
    NAME bbtc.ib_propellant_burn_kinetics
    COMMAND bbtc_test_ib_propellant_burn_kinetics
)

add_executable(
    bbtc_test_ib_pressure_burn_table
    test_ib_pressure_burn_table.c
)

target_link_libraries(
    bbtc_test_ib_pressure_burn_table
    PRIVATE
        bbtc::bbtc
)

set_target_properties(
    bbtc_test_ib_pressure_burn_table
    PROPERTIES
        C_EXTENSIONS OFF
        C_STANDARD 23
        C_STANDARD_REQUIRED ON
)

add_test(
    NAME bbtc.ib_pressure_burn_table
    COMMAND bbtc_test_ib_pressure_burn_table
)

add_executable(
    bbtc_test_ib_propellant_mass
    test_ib_propellant_mass.c
)

target_link_libraries(
    bbtc_test_ib_propellant_mass
    PRIVATE
        bbtc::bbtc
)

set_target_properties(
    bbtc_test_ib_propellant_mass
    PROPERTIES
        C_EXTENSIONS OFF
        C_STANDARD 23
        C_STANDARD_REQUIRED ON
)

add_test(
    NAME bbtc.ib_propellant_mass
    COMMAND bbtc_test_ib_propellant_mass
)

add_executable(
    bbtc_test_ib_propellant_source_rate
    test_ib_propellant_source_rate.c
)

target_link_libraries(
    bbtc_test_ib_propellant_source_rate
    PRIVATE
        bbtc::bbtc
)

set_target_properties(
    bbtc_test_ib_propellant_source_rate
    PROPERTIES
        C_EXTENSIONS OFF
        C_STANDARD 23
        C_STANDARD_REQUIRED ON
)

add_test(
    NAME bbtc.ib_propellant_source_rate
    COMMAND bbtc_test_ib_propellant_source_rate
)


add_executable(
    bbtc_test_ib_loading_state
    test_ib_loading_state.c
)

target_link_libraries(
    bbtc_test_ib_loading_state
    PRIVATE
        bbtc::bbtc
)

set_target_properties(
    bbtc_test_ib_loading_state
    PROPERTIES
        C_EXTENSIONS OFF
        C_STANDARD 23
        C_STANDARD_REQUIRED ON
)

add_test(
    NAME bbtc.ib_loading_state
    COMMAND bbtc_test_ib_loading_state
)

add_executable(
    bbtc_test_ib_initial_gas_state
    test_ib_initial_gas_state.c
)

target_link_libraries(
    bbtc_test_ib_initial_gas_state
    PRIVATE
        bbtc::bbtc
)

set_target_properties(
    bbtc_test_ib_initial_gas_state
    PROPERTIES
        C_EXTENSIONS OFF
        C_STANDARD 23
        C_STANDARD_REQUIRED ON
)

add_test(
    NAME bbtc.ib_initial_gas_state
    COMMAND bbtc_test_ib_initial_gas_state
)

add_executable(
    bbtc_test_ib_initial_propellant_condition
    test_ib_initial_propellant_condition.c
)

target_link_libraries(
    bbtc_test_ib_initial_propellant_condition
    PRIVATE
        bbtc::bbtc
)

set_target_properties(
    bbtc_test_ib_initial_propellant_condition
    PROPERTIES
        C_EXTENSIONS OFF
        C_STANDARD 23
        C_STANDARD_REQUIRED ON
)

add_test(
    NAME bbtc.ib_initial_propellant_condition
    COMMAND bbtc_test_ib_initial_propellant_condition
)

add_executable(
    bbtc_test_ib_noble_abel_gas_model
    test_ib_noble_abel_gas_model.c
)

target_link_libraries(
    bbtc_test_ib_noble_abel_gas_model
    PRIVATE
        bbtc::bbtc
)

set_target_properties(
    bbtc_test_ib_noble_abel_gas_model
    PROPERTIES
        C_EXTENSIONS OFF
        C_STANDARD 23
        C_STANDARD_REQUIRED ON
)

add_test(
    NAME bbtc.ib_noble_abel_gas_model
    COMMAND bbtc_test_ib_noble_abel_gas_model
)

add_executable(
    bbtc_test_ib_first_order_virial_gas_model
    test_ib_first_order_virial_gas_model.c
)

target_link_libraries(
    bbtc_test_ib_first_order_virial_gas_model
    PRIVATE
        bbtc::bbtc
)

set_target_properties(
    bbtc_test_ib_first_order_virial_gas_model
    PROPERTIES
        C_EXTENSIONS OFF
        C_STANDARD 23
        C_STANDARD_REQUIRED ON
)

add_test(
    NAME bbtc.ib_first_order_virial_gas_model
    COMMAND bbtc_test_ib_first_order_virial_gas_model
)

add_executable(
    bbtc_test_ib_reduced_gas_thermodynamics
    test_ib_reduced_gas_thermodynamics.c
)

target_link_libraries(
    bbtc_test_ib_reduced_gas_thermodynamics
    PRIVATE
        bbtc::bbtc
)

set_target_properties(
    bbtc_test_ib_reduced_gas_thermodynamics
    PROPERTIES
        C_EXTENSIONS OFF
        C_STANDARD 23
        C_STANDARD_REQUIRED ON
)

add_test(
    NAME bbtc.ib_reduced_gas_thermodynamics
    COMMAND bbtc_test_ib_reduced_gas_thermodynamics
)

add_executable(
    bbtc_test_ib_initial_gas_closure
    test_ib_initial_gas_closure.c
)

target_link_libraries(
    bbtc_test_ib_initial_gas_closure
    PRIVATE
        bbtc::bbtc
)

set_target_properties(
    bbtc_test_ib_initial_gas_closure
    PROPERTIES
        C_EXTENSIONS OFF
        C_STANDARD 23
        C_STANDARD_REQUIRED ON
)

add_test(
    NAME bbtc.ib_initial_gas_closure
    COMMAND bbtc_test_ib_initial_gas_closure
)

add_executable(
    bbtc_test_eb_state
    test_eb_state.c
)

target_link_libraries(
    bbtc_test_eb_state
    PRIVATE
        bbtc::bbtc
)

set_target_properties(
    bbtc_test_eb_state
    PROPERTIES
        C_EXTENSIONS OFF
        C_STANDARD 23
        C_STANDARD_REQUIRED ON
)

add_test(
    NAME bbtc.eb_state
    COMMAND bbtc_test_eb_state
)

add_executable(
    bbtc_test_eb_projectile
    test_eb_projectile.c
)

target_link_libraries(
    bbtc_test_eb_projectile
    PRIVATE
        bbtc::bbtc
)

set_target_properties(
    bbtc_test_eb_projectile
    PROPERTIES
        C_EXTENSIONS OFF
        C_STANDARD 23
        C_STANDARD_REQUIRED ON
)

add_test(
    NAME bbtc.eb_projectile
    COMMAND bbtc_test_eb_projectile
)

add_executable(
    bbtc_test_eb_environment
    test_eb_environment.c
)

target_link_libraries(
    bbtc_test_eb_environment
    PRIVATE
        bbtc::bbtc
)

set_target_properties(
    bbtc_test_eb_environment
    PROPERTIES
        C_EXTENSIONS OFF
        C_STANDARD 23
        C_STANDARD_REQUIRED ON
)

add_test(
    NAME bbtc.eb_environment
    COMMAND bbtc_test_eb_environment
)

add_executable(
    bbtc_test_eb_environment_provider
    test_eb_environment_provider.c
)

target_link_libraries(
    bbtc_test_eb_environment_provider
    PRIVATE
        bbtc::bbtc
)

set_target_properties(
    bbtc_test_eb_environment_provider
    PROPERTIES
        C_EXTENSIONS OFF
        C_STANDARD 23
        C_STANDARD_REQUIRED ON
)

add_test(
    NAME bbtc.eb_environment_provider
    COMMAND bbtc_test_eb_environment_provider
)

add_executable(
    bbtc_test_eb_reference_frame
    test_eb_reference_frame.c
)

target_link_libraries(
    bbtc_test_eb_reference_frame
    PRIVATE
        bbtc::bbtc
)

set_target_properties(
    bbtc_test_eb_reference_frame
    PROPERTIES
        C_EXTENSIONS OFF
        C_STANDARD 23
        C_STANDARD_REQUIRED ON
)

add_test(
    NAME bbtc.eb_reference_frame
    COMMAND bbtc_test_eb_reference_frame
)

add_executable(
    bbtc_test_eb_reference_frame_provider
    test_eb_reference_frame_provider.c
)

target_link_libraries(
    bbtc_test_eb_reference_frame_provider
    PRIVATE
        bbtc::bbtc
)

set_target_properties(
    bbtc_test_eb_reference_frame_provider
    PROPERTIES
        C_EXTENSIONS OFF
        C_STANDARD 23
        C_STANDARD_REQUIRED ON
)

add_test(
    NAME bbtc.eb_reference_frame_provider
    COMMAND bbtc_test_eb_reference_frame_provider
)

add_executable(
    bbtc_test_eb_force_moment
    test_eb_force_moment.c
)

target_link_libraries(
    bbtc_test_eb_force_moment
    PRIVATE
        bbtc::bbtc
)

set_target_properties(
    bbtc_test_eb_force_moment
    PROPERTIES
        C_EXTENSIONS OFF
        C_STANDARD 23
        C_STANDARD_REQUIRED ON
)

add_test(
    NAME bbtc.eb_force_moment
    COMMAND bbtc_test_eb_force_moment
)

add_executable(
    bbtc_test_eb_state_derivative
    test_eb_state_derivative.c
)

target_link_libraries(
    bbtc_test_eb_state_derivative
    PRIVATE
        bbtc::bbtc
)

set_target_properties(
    bbtc_test_eb_state_derivative
    PROPERTIES
        C_EXTENSIONS OFF
        C_STANDARD 23
        C_STANDARD_REQUIRED ON
)

add_test(
    NAME bbtc.eb_state_derivative
    COMMAND bbtc_test_eb_state_derivative
)

add_executable(
    bbtc_test_eb_state_derivative_provider
    test_eb_state_derivative_provider.c
)

target_link_libraries(
    bbtc_test_eb_state_derivative_provider
    PRIVATE
        bbtc::bbtc
)

set_target_properties(
    bbtc_test_eb_state_derivative_provider
    PROPERTIES
        C_EXTENSIONS OFF
        C_STANDARD 23
        C_STANDARD_REQUIRED ON
)

add_test(
    NAME bbtc.eb_state_derivative_provider
    COMMAND bbtc_test_eb_state_derivative_provider
)

add_executable(
    bbtc_test_eb_integrator
    test_eb_integrator.c
)

target_link_libraries(
    bbtc_test_eb_integrator
    PRIVATE
        bbtc::bbtc
)

set_target_properties(
    bbtc_test_eb_integrator
    PROPERTIES
        C_EXTENSIONS OFF
        C_STANDARD 23
        C_STANDARD_REQUIRED ON
)

add_test(
    NAME bbtc.eb_integrator
    COMMAND bbtc_test_eb_integrator
)

add_executable(
    bbtc_test_eb_inertial_vacuum
    test_eb_inertial_vacuum.c
)

target_link_libraries(
    bbtc_test_eb_inertial_vacuum
    PRIVATE
        bbtc::bbtc
)

set_target_properties(
    bbtc_test_eb_inertial_vacuum
    PROPERTIES
        C_EXTENSIONS OFF
        C_STANDARD 23
        C_STANDARD_REQUIRED ON
)

add_test(
    NAME bbtc.eb_inertial_vacuum
    COMMAND bbtc_test_eb_inertial_vacuum
)

add_executable(
    bbtc_test_eb_inertial_vacuum_provider
    test_eb_inertial_vacuum_provider.c
)

target_link_libraries(
    bbtc_test_eb_inertial_vacuum_provider
    PRIVATE
        bbtc::bbtc
)

set_target_properties(
    bbtc_test_eb_inertial_vacuum_provider
    PROPERTIES
        C_EXTENSIONS OFF
        C_STANDARD 23
        C_STANDARD_REQUIRED ON
)

add_test(
    NAME bbtc.eb_inertial_vacuum_provider
    COMMAND bbtc_test_eb_inertial_vacuum_provider
)

add_executable(
    bbtc_test_eb_inertial_vacuum_propagation
    test_eb_inertial_vacuum_propagation.c
)

target_link_libraries(
    bbtc_test_eb_inertial_vacuum_propagation
    PRIVATE
        bbtc::bbtc
)

set_target_properties(
    bbtc_test_eb_inertial_vacuum_propagation
    PROPERTIES
        C_EXTENSIONS OFF
        C_STANDARD 23
        C_STANDARD_REQUIRED ON
)

add_test(
    NAME bbtc.eb_inertial_vacuum_propagation
    COMMAND bbtc_test_eb_inertial_vacuum_propagation
)

add_executable(
    bbtc_test_eb_relative_air
    test_eb_relative_air.c
)

target_link_libraries(
    bbtc_test_eb_relative_air
    PRIVATE
        bbtc::bbtc
)

set_target_properties(
    bbtc_test_eb_relative_air
    PROPERTIES
        C_EXTENSIONS OFF
        C_STANDARD 23
        C_STANDARD_REQUIRED ON
)

add_test(
    NAME bbtc.eb_relative_air
    COMMAND bbtc_test_eb_relative_air
)

add_executable(
    bbtc_test_eb_math_internal
    test_eb_math_internal.c
)

target_include_directories(
    bbtc_test_eb_math_internal
    PRIVATE
        "${PROJECT_SOURCE_DIR}/src"
)

target_link_libraries(
    bbtc_test_eb_math_internal
    PRIVATE
        bbtc::bbtc
)

set_target_properties(
    bbtc_test_eb_math_internal
    PROPERTIES
        C_EXTENSIONS OFF
        C_STANDARD 23
        C_STANDARD_REQUIRED ON
)

add_test(
    NAME bbtc.eb_math_internal
    COMMAND bbtc_test_eb_math_internal
)

set(
    BBTC_CONSUMER_FIXTURE_SOURCE_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/consumer"
)

set(
    BBTC_CONSUMER_FIXTURE_BINARY_DIR
    "${CMAKE_CURRENT_BINARY_DIR}/consumer"
)

configure_file(
    consumer/run_consumer_fixture.cmake.in
    run_consumer_fixture.cmake
    @ONLY
)

add_test(
    NAME bbtc.consumer
    COMMAND
        "${CMAKE_COMMAND}"
        -P
        "${CMAKE_CURRENT_BINARY_DIR}/run_consumer_fixture.cmake"
)

# C++ is optional and is used only to verify public-header interoperability.
if(NOT CMAKE_CXX_COMPILER_LOADED)
    include(CheckLanguage)
    check_language(CXX)

    if(CMAKE_CXX_COMPILER)
        enable_language(CXX)
    endif()
endif()

if(CMAKE_CXX_COMPILER_LOADED)
    add_executable(
        bbtc_test_cpp_header
        test_cpp_header.cpp
    )

    target_link_libraries(
        bbtc_test_cpp_header
        PRIVATE
            bbtc::bbtc
    )

    target_compile_features(
        bbtc_test_cpp_header
        PRIVATE
            cxx_std_11
    )

    set_target_properties(
        bbtc_test_cpp_header
        PROPERTIES
            CXX_EXTENSIONS OFF
    )

    add_test(
        NAME bbtc.cpp_header
        COMMAND bbtc_test_cpp_header
    )
endif()
