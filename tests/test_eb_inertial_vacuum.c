/**
 * @file
 * @brief Direct equation-level tests for EB1.1 inertial-vacuum 6DOF dynamics.
 *
 * These tests deliberately exercise the physical derivative evaluator without
 * B7 provider sampling, B10 provider composition, or B11 integration. Exact
 * arithmetic cases independently freeze translation, quaternion multiplication
 * order, principal-axis Euler dynamics, fail-closed output, and native-family
 * behavior.
 */

#include <bbtc/bbtc.h>

#include <float.h>
#include <math.h>
#include <stdlib.h>

typedef bbtc_status_e (*bbtc_eb_inertial_vacuum_float_function_t)(
    const bbtc_eb_rigid_body_state_float_t*,
    const bbtc_eb_projectile_properties_float_t*,
    const bbtc_eb_reference_frame_sample_float_t*,
    const bbtc_eb_force_moment_float_t*,
    bbtc_eb_rigid_body_state_derivative_float_t*
);

typedef bbtc_status_e (*bbtc_eb_inertial_vacuum_double_function_t)(
    const bbtc_eb_rigid_body_state_double_t*,
    const bbtc_eb_projectile_properties_double_t*,
    const bbtc_eb_reference_frame_sample_double_t*,
    const bbtc_eb_force_moment_double_t*,
    bbtc_eb_rigid_body_state_derivative_double_t*
);

typedef bbtc_status_e (*bbtc_eb_inertial_vacuum_long_double_function_t)(
    const bbtc_eb_rigid_body_state_long_double_t*,
    const bbtc_eb_projectile_properties_long_double_t*,
    const bbtc_eb_reference_frame_sample_long_double_t*,
    const bbtc_eb_force_moment_long_double_t*,
    bbtc_eb_rigid_body_state_derivative_long_double_t*
);

static_assert(_Generic(
    &bbtc_eb_inertial_vacuum_derivative_evaluate_float,
    bbtc_eb_inertial_vacuum_float_function_t: 1,
    default: 0
));

static_assert(_Generic(
    &bbtc_eb_inertial_vacuum_derivative_evaluate_double,
    bbtc_eb_inertial_vacuum_double_function_t: 1,
    default: 0
));

static_assert(_Generic(
    &bbtc_eb_inertial_vacuum_derivative_evaluate_long_double,
    bbtc_eb_inertial_vacuum_long_double_function_t: 1,
    default: 0
));

static bbtc_eb_rigid_body_state_double_t
valid_double_state(void)
{
    return (bbtc_eb_rigid_body_state_double_t)
    {
        .orientation_world_from_body =
        {
            .x = 0.0,
            .y = 0.0,
            .z = 0.0,
            .w = 1.0
        },
        .position_m =
        {
            .x = 1.0,
            .y = 2.0,
            .z = 3.0
        },
        .velocity_m_per_s =
        {
            .x = 10.0,
            .y = -20.0,
            .z = 30.0
        },
        .angular_velocity_body_rad_per_s = {0},
        .time_s = 0.0
    };
}

static bbtc_eb_projectile_properties_double_t
valid_double_projectile(void)
{
    return (bbtc_eb_projectile_properties_double_t)
    {
        .principal_inertia =
        {
            .moment_about_x_kg_m2 = 2.0,
            .moment_about_y_kg_m2 = 3.0,
            .moment_about_z_kg_m2 = 4.0
        },
        .mass_kg = 2.0,
        .reference_diameter_m = 1.0,
        .reference_area_m2 = 1.0
    };
}

static bbtc_eb_reference_frame_sample_double_t
valid_double_reference_frame(void)
{
    return (bbtc_eb_reference_frame_sample_double_t){0};
}

static bbtc_eb_force_moment_double_t
zero_double_force_moment(void)
{
    return (bbtc_eb_force_moment_double_t){0};
}

static bbtc_eb_rigid_body_state_derivative_double_t
poison_double_derivative(void)
{
    return (bbtc_eb_rigid_body_state_derivative_double_t)
    {
        .orientation_world_from_body_derivative_per_s =
        {
            .x = 1.0,
            .y = 2.0,
            .z = 3.0,
            .w = 4.0
        },
        .position_derivative_reference_m_per_s =
        {
            .x = 5.0,
            .y = 6.0,
            .z = 7.0
        },
        .velocity_derivative_reference_m_per_s2 =
        {
            .x = 8.0,
            .y = 9.0,
            .z = 10.0
        },
        .angular_velocity_derivative_body_rad_per_s2 =
        {
            .x = 11.0,
            .y = 12.0,
            .z = 13.0
        }
    };
}

static int
double_derivative_is_zero(
    const bbtc_eb_rigid_body_state_derivative_double_t* const derivative
)
{
    return derivative->orientation_world_from_body_derivative_per_s.x == 0.0
        && derivative->orientation_world_from_body_derivative_per_s.y == 0.0
        && derivative->orientation_world_from_body_derivative_per_s.z == 0.0
        && derivative->orientation_world_from_body_derivative_per_s.w == 0.0
        && derivative->position_derivative_reference_m_per_s.x == 0.0
        && derivative->position_derivative_reference_m_per_s.y == 0.0
        && derivative->position_derivative_reference_m_per_s.z == 0.0
        && derivative->velocity_derivative_reference_m_per_s2.x == 0.0
        && derivative->velocity_derivative_reference_m_per_s2.y == 0.0
        && derivative->velocity_derivative_reference_m_per_s2.z == 0.0
        && derivative->angular_velocity_derivative_body_rad_per_s2.x == 0.0
        && derivative->angular_velocity_derivative_body_rad_per_s2.y == 0.0
        && derivative->angular_velocity_derivative_body_rad_per_s2.z == 0.0;
}

static int
expect_double_failure(
    const bbtc_eb_rigid_body_state_double_t* const state,
    const bbtc_eb_projectile_properties_double_t* const projectile,
    const bbtc_eb_reference_frame_sample_double_t* const reference_frame,
    const bbtc_eb_force_moment_double_t* const force_moment,
    const bbtc_status_e expected_status
)
{
    bbtc_eb_rigid_body_state_derivative_double_t derivative =
        poison_double_derivative();

    const bbtc_status_e status =
        bbtc_eb_inertial_vacuum_derivative_evaluate_double(
            state,
            projectile,
            reference_frame,
            force_moment,
            &derivative
        );

    return status == expected_status && double_derivative_is_zero(&derivative)
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}

static int
test_null_and_validation_failures(void)
{
    bbtc_eb_rigid_body_state_double_t state = valid_double_state();
    bbtc_eb_projectile_properties_double_t projectile =
        valid_double_projectile();
    bbtc_eb_reference_frame_sample_double_t reference_frame =
        valid_double_reference_frame();
    bbtc_eb_force_moment_double_t force_moment = zero_double_force_moment();

    if (bbtc_eb_inertial_vacuum_derivative_evaluate_double(
            &state,
            &projectile,
            &reference_frame,
            &force_moment,
            NULL
        ) != BBTC_STATUS_INVALID_ARGUMENT)
    {
        return EXIT_FAILURE;
    }

    if (expect_double_failure(
            NULL,
            &projectile,
            &reference_frame,
            &force_moment,
            BBTC_STATUS_INVALID_ARGUMENT
        ) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (expect_double_failure(
            &state,
            NULL,
            &reference_frame,
            &force_moment,
            BBTC_STATUS_INVALID_ARGUMENT
        ) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    state.time_s = NAN;
    if (expect_double_failure(
            &state,
            NULL,
            &reference_frame,
            &force_moment,
            BBTC_STATUS_INVALID_ARGUMENT
        ) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (expect_double_failure(
            &state,
            &projectile,
            &reference_frame,
            &force_moment,
            BBTC_STATUS_NAN_INPUT
        ) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    state = valid_double_state();
    projectile.mass_kg = 0.0;
    if (expect_double_failure(
            &state,
            &projectile,
            &reference_frame,
            &force_moment,
            BBTC_STATUS_OUTSIDE_DOMAIN
        ) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    projectile = valid_double_projectile();
    reference_frame.gravitational_acceleration_reference_m_per_s2.x =
        INFINITY;
    if (expect_double_failure(
            &state,
            &projectile,
            &reference_frame,
            &force_moment,
            BBTC_STATUS_NONFINITE_INPUT
        ) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    reference_frame = valid_double_reference_frame();
    force_moment.applied_moment_body_n_m.z = NAN;
    return expect_double_failure(
        &state,
        &projectile,
        &reference_frame,
        &force_moment,
        BBTC_STATUS_NAN_INPUT
    );
}

static int
test_inertial_reference_restriction(void)
{
    const bbtc_eb_rigid_body_state_double_t state = valid_double_state();
    const bbtc_eb_projectile_properties_double_t projectile =
        valid_double_projectile();
    bbtc_eb_reference_frame_sample_double_t reference_frame =
        valid_double_reference_frame();
    const bbtc_eb_force_moment_double_t force_moment =
        zero_double_force_moment();
    bbtc_eb_rigid_body_state_derivative_double_t derivative = {0};

    reference_frame.reference_frame_origin_acceleration_reference_m_per_s2 =
        (bbtc_eb_vec3_double_t)
        {
            .x = -0.0,
            .y = 0.0,
            .z = -0.0
        };
    reference_frame.reference_frame_angular_velocity_reference_rad_per_s =
        (bbtc_eb_vec3_double_t)
        {
            .x = 0.0,
            .y = -0.0,
            .z = 0.0
        };
    reference_frame.reference_frame_angular_acceleration_reference_rad_per_s2 =
        (bbtc_eb_vec3_double_t)
        {
            .x = -0.0,
            .y = -0.0,
            .z = 0.0
        };
    reference_frame.gravitational_acceleration_reference_m_per_s2.y = -9.0;

    if (bbtc_eb_inertial_vacuum_derivative_evaluate_double(
            &state,
            &projectile,
            &reference_frame,
            &force_moment,
            &derivative
        ) != BBTC_STATUS_SUCCESS
        || derivative.velocity_derivative_reference_m_per_s2.y != -9.0)
    {
        return EXIT_FAILURE;
    }

    reference_frame = valid_double_reference_frame();
    reference_frame.reference_frame_origin_acceleration_reference_m_per_s2.x =
        1.0;
    if (expect_double_failure(
            &state,
            &projectile,
            &reference_frame,
            &force_moment,
            BBTC_STATUS_UNSUPPORTED_MODEL_OR_OPTION
        ) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    reference_frame = valid_double_reference_frame();
    reference_frame.reference_frame_angular_velocity_reference_rad_per_s.y =
        1.0;
    if (expect_double_failure(
            &state,
            &projectile,
            &reference_frame,
            &force_moment,
            BBTC_STATUS_UNSUPPORTED_MODEL_OR_OPTION
        ) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    reference_frame = valid_double_reference_frame();
    reference_frame.reference_frame_angular_acceleration_reference_rad_per_s2.z =
        1.0;
    return expect_double_failure(
        &state,
        &projectile,
        &reference_frame,
        &force_moment,
        BBTC_STATUS_UNSUPPORTED_MODEL_OR_OPTION
    );
}

static int
test_translational_equations(void)
{
    const bbtc_eb_rigid_body_state_double_t state = valid_double_state();
    const bbtc_eb_projectile_properties_double_t projectile =
        valid_double_projectile();
    bbtc_eb_reference_frame_sample_double_t reference_frame =
        valid_double_reference_frame();
    bbtc_eb_force_moment_double_t force_moment = zero_double_force_moment();
    bbtc_eb_rigid_body_state_derivative_double_t derivative = {0};

    /*
     * Freeze the exact no-load/no-gravity baseline independently: zero body
     * angular velocity must produce an exact zero quaternion derivative, and
     * zero applied force plus zero gravity must produce zero translational
     * acceleration.
     */
    if (bbtc_eb_inertial_vacuum_derivative_evaluate_double(
            &state,
            &projectile,
            &reference_frame,
            &force_moment,
            &derivative
        ) != BBTC_STATUS_SUCCESS
        || derivative.velocity_derivative_reference_m_per_s2.x != 0.0
        || derivative.velocity_derivative_reference_m_per_s2.y != 0.0
        || derivative.velocity_derivative_reference_m_per_s2.z != 0.0
        || derivative.orientation_world_from_body_derivative_per_s.x != 0.0
        || derivative.orientation_world_from_body_derivative_per_s.y != 0.0
        || derivative.orientation_world_from_body_derivative_per_s.z != 0.0
        || derivative.orientation_world_from_body_derivative_per_s.w != 0.0)
    {
        return EXIT_FAILURE;
    }

    reference_frame.gravitational_acceleration_reference_m_per_s2 =
        (bbtc_eb_vec3_double_t)
        {
            .x = 1.0,
            .y = -2.0,
            .z = 3.0
        };
    force_moment.applied_force_reference_n =
        (bbtc_eb_vec3_double_t)
        {
            .x = 4.0,
            .y = -6.0,
            .z = 8.0
        };

    if (bbtc_eb_inertial_vacuum_derivative_evaluate_double(
            &state,
            &projectile,
            &reference_frame,
            &force_moment,
            &derivative
        ) != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    return derivative.position_derivative_reference_m_per_s.x == 10.0
            && derivative.position_derivative_reference_m_per_s.y == -20.0
            && derivative.position_derivative_reference_m_per_s.z == 30.0
            && derivative.velocity_derivative_reference_m_per_s2.x == 3.0
            && derivative.velocity_derivative_reference_m_per_s2.y == -5.0
            && derivative.velocity_derivative_reference_m_per_s2.z == 7.0
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}

static int
test_quaternion_multiplication_order(void)
{
    bbtc_eb_rigid_body_state_double_t state = valid_double_state();
    bbtc_eb_projectile_properties_double_t projectile =
        valid_double_projectile();
    const bbtc_eb_reference_frame_sample_double_t reference_frame =
        valid_double_reference_frame();
    const bbtc_eb_force_moment_double_t force_moment =
        zero_double_force_moment();
    bbtc_eb_rigid_body_state_derivative_double_t derivative = {0};

    state.orientation_world_from_body =
        (bbtc_eb_quat_double_t)
        {
            .x = 0.5,
            .y = 0.5,
            .z = 0.5,
            .w = 0.5
        };
    state.angular_velocity_body_rad_per_s =
        (bbtc_eb_vec3_double_t)
        {
            .x = 2.0,
            .y = 3.0,
            .z = 5.0
        };

    /*
     * Spherical inertia removes Euler gyroscopic coupling so this case isolates
     * the quaternion convention from rotational-dynamics expectations.
     */
    projectile.principal_inertia =
        (bbtc_eb_principal_inertia_double_t)
        {
            .moment_about_x_kg_m2 = 2.0,
            .moment_about_y_kg_m2 = 2.0,
            .moment_about_z_kg_m2 = 2.0
        };

    if (bbtc_eb_inertial_vacuum_derivative_evaluate_double(
            &state,
            &projectile,
            &reference_frame,
            &force_moment,
            &derivative
        ) != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    /*
     * 0.5 * q * omega = (1, 0, 1.5, -2.5).
     * The historical reversed order 0.5 * omega * q would instead be
     * (0, 1.5, 1, -2.5), so this oracle cannot pass with the old defect.
     */
    return derivative.orientation_world_from_body_derivative_per_s.x == 1.0
            && derivative.orientation_world_from_body_derivative_per_s.y == 0.0
            && derivative.orientation_world_from_body_derivative_per_s.z == 1.5
            && derivative.orientation_world_from_body_derivative_per_s.w == -2.5
            && derivative.angular_velocity_derivative_body_rad_per_s2.x == 0.0
            && derivative.angular_velocity_derivative_body_rad_per_s2.y == 0.0
            && derivative.angular_velocity_derivative_body_rad_per_s2.z == 0.0
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}

static int
test_euler_rotational_dynamics(void)
{
    bbtc_eb_rigid_body_state_double_t state = valid_double_state();
    bbtc_eb_projectile_properties_double_t projectile =
        valid_double_projectile();
    const bbtc_eb_reference_frame_sample_double_t reference_frame =
        valid_double_reference_frame();
    bbtc_eb_force_moment_double_t force_moment = zero_double_force_moment();
    bbtc_eb_rigid_body_state_derivative_double_t derivative = {0};

    projectile.principal_inertia =
        (bbtc_eb_principal_inertia_double_t)
        {
            .moment_about_x_kg_m2 = 2.0,
            .moment_about_y_kg_m2 = 2.0,
            .moment_about_z_kg_m2 = 2.0
        };
    state.angular_velocity_body_rad_per_s =
        (bbtc_eb_vec3_double_t)
        {
            .x = 3.0,
            .y = 4.0,
            .z = 5.0
        };

    if (bbtc_eb_inertial_vacuum_derivative_evaluate_double(
            &state,
            &projectile,
            &reference_frame,
            &force_moment,
            &derivative
        ) != BBTC_STATUS_SUCCESS
        || derivative.angular_velocity_derivative_body_rad_per_s2.x != 0.0
        || derivative.angular_velocity_derivative_body_rad_per_s2.y != 0.0
        || derivative.angular_velocity_derivative_body_rad_per_s2.z != 0.0)
    {
        return EXIT_FAILURE;
    }

    projectile = valid_double_projectile();
    state.angular_velocity_body_rad_per_s =
        (bbtc_eb_vec3_double_t){.x = 6.0, .y = 0.0, .z = 0.0};

    if (bbtc_eb_inertial_vacuum_derivative_evaluate_double(
            &state,
            &projectile,
            &reference_frame,
            &force_moment,
            &derivative
        ) != BBTC_STATUS_SUCCESS
        || derivative.angular_velocity_derivative_body_rad_per_s2.x != 0.0
        || derivative.angular_velocity_derivative_body_rad_per_s2.y != 0.0
        || derivative.angular_velocity_derivative_body_rad_per_s2.z != 0.0)
    {
        return EXIT_FAILURE;
    }

    force_moment.applied_moment_body_n_m.x = 4.0;
    if (bbtc_eb_inertial_vacuum_derivative_evaluate_double(
            &state,
            &projectile,
            &reference_frame,
            &force_moment,
            &derivative
        ) != BBTC_STATUS_SUCCESS
        || derivative.angular_velocity_derivative_body_rad_per_s2.x != 2.0
        || derivative.angular_velocity_derivative_body_rad_per_s2.y != 0.0
        || derivative.angular_velocity_derivative_body_rad_per_s2.z != 0.0)
    {
        return EXIT_FAILURE;
    }

    state.angular_velocity_body_rad_per_s =
        (bbtc_eb_vec3_double_t){.x = 5.0, .y = 7.0, .z = 11.0};
    force_moment.applied_moment_body_n_m =
        (bbtc_eb_vec3_double_t){.x = 13.0, .y = 16.0, .z = 19.0};

    if (bbtc_eb_inertial_vacuum_derivative_evaluate_double(
            &state,
            &projectile,
            &reference_frame,
            &force_moment,
            &derivative
        ) != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    return derivative.angular_velocity_derivative_body_rad_per_s2.x == -32.0
            && derivative.angular_velocity_derivative_body_rad_per_s2.y == 42.0
            && derivative.angular_velocity_derivative_body_rad_per_s2.z == -4.0
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}

static int
test_generated_numerical_failures(void)
{
    bbtc_eb_rigid_body_state_double_t state = valid_double_state();
    bbtc_eb_projectile_properties_double_t projectile =
        valid_double_projectile();
    const bbtc_eb_reference_frame_sample_double_t reference_frame =
        valid_double_reference_frame();
    bbtc_eb_force_moment_double_t force_moment = zero_double_force_moment();

    projectile.mass_kg = 0.5;
    force_moment.applied_force_reference_n.x = DBL_MAX;
    if (expect_double_failure(
            &state,
            &projectile,
            &reference_frame,
            &force_moment,
            BBTC_STATUS_NUMERICAL_FAILURE
        ) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    projectile = valid_double_projectile();
    force_moment = zero_double_force_moment();
    projectile.principal_inertia =
        (bbtc_eb_principal_inertia_double_t)
        {
            .moment_about_x_kg_m2 = 1.0,
            .moment_about_y_kg_m2 = 1.0,
            .moment_about_z_kg_m2 = 1.0
        };
    state.orientation_world_from_body =
        (bbtc_eb_quat_double_t)
        {
            .x = 0.5,
            .y = 0.5,
            .z = 0.5,
            .w = 0.5
        };
    state.angular_velocity_body_rad_per_s =
        (bbtc_eb_vec3_double_t)
        {
            .x = DBL_MAX,
            .y = DBL_MAX,
            .z = DBL_MAX
        };

    if (expect_double_failure(
            &state,
            &projectile,
            &reference_frame,
            &force_moment,
            BBTC_STATUS_NUMERICAL_FAILURE
        ) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    state = valid_double_state();
    projectile = valid_double_projectile();
    projectile.principal_inertia =
        (bbtc_eb_principal_inertia_double_t)
        {
            .moment_about_x_kg_m2 = 1.0,
            .moment_about_y_kg_m2 = 2.0,
            .moment_about_z_kg_m2 = 2.0
        };
    state.angular_velocity_body_rad_per_s =
        (bbtc_eb_vec3_double_t)
        {
            .x = DBL_MAX,
            .y = DBL_MAX,
            .z = 0.0
        };

    return expect_double_failure(
        &state,
        &projectile,
        &reference_frame,
        &force_moment,
        BBTC_STATUS_NUMERICAL_FAILURE
    );
}


static int
test_inputs_are_read_only(void)
{
    bbtc_eb_rigid_body_state_double_t state = valid_double_state();
    bbtc_eb_projectile_properties_double_t projectile =
        valid_double_projectile();
    bbtc_eb_reference_frame_sample_double_t reference_frame =
        valid_double_reference_frame();
    bbtc_eb_force_moment_double_t force_moment = zero_double_force_moment();

    state.orientation_world_from_body =
        (bbtc_eb_quat_double_t){.x = 0.5, .y = 0.5, .z = 0.5, .w = 0.5};
    state.angular_velocity_body_rad_per_s =
        (bbtc_eb_vec3_double_t){.x = 2.0, .y = 3.0, .z = 5.0};
    state.time_s = 7.0;

    projectile.principal_inertia =
        (bbtc_eb_principal_inertia_double_t)
        {
            .moment_about_x_kg_m2 = 2.0,
            .moment_about_y_kg_m2 = 3.0,
            .moment_about_z_kg_m2 = 4.0
        };
    projectile.mass_kg = 2.0;
    projectile.reference_diameter_m = 0.01;
    projectile.reference_area_m2 = 0.0001;

    reference_frame.gravitational_acceleration_reference_m_per_s2 =
        (bbtc_eb_vec3_double_t){.x = 1.0, .y = -2.0, .z = 3.0};

    force_moment.applied_force_reference_n =
        (bbtc_eb_vec3_double_t){.x = 4.0, .y = -6.0, .z = 8.0};
    force_moment.applied_moment_body_n_m =
        (bbtc_eb_vec3_double_t){.x = 13.0, .y = 16.0, .z = 19.0};

    const bbtc_eb_rigid_body_state_double_t state_before = state;
    const bbtc_eb_projectile_properties_double_t projectile_before =
        projectile;
    const bbtc_eb_reference_frame_sample_double_t reference_frame_before =
        reference_frame;
    const bbtc_eb_force_moment_double_t force_moment_before = force_moment;

    bbtc_eb_rigid_body_state_derivative_double_t derivative = {0};

    if (bbtc_eb_inertial_vacuum_derivative_evaluate_double(
            &state,
            &projectile,
            &reference_frame,
            &force_moment,
            &derivative
        ) != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    return state.orientation_world_from_body.x
                == state_before.orientation_world_from_body.x
            && state.orientation_world_from_body.y
                == state_before.orientation_world_from_body.y
            && state.orientation_world_from_body.z
                == state_before.orientation_world_from_body.z
            && state.orientation_world_from_body.w
                == state_before.orientation_world_from_body.w
            && state.position_m.x == state_before.position_m.x
            && state.position_m.y == state_before.position_m.y
            && state.position_m.z == state_before.position_m.z
            && state.velocity_m_per_s.x == state_before.velocity_m_per_s.x
            && state.velocity_m_per_s.y == state_before.velocity_m_per_s.y
            && state.velocity_m_per_s.z == state_before.velocity_m_per_s.z
            && state.angular_velocity_body_rad_per_s.x
                == state_before.angular_velocity_body_rad_per_s.x
            && state.angular_velocity_body_rad_per_s.y
                == state_before.angular_velocity_body_rad_per_s.y
            && state.angular_velocity_body_rad_per_s.z
                == state_before.angular_velocity_body_rad_per_s.z
            && state.time_s == state_before.time_s
            && projectile.principal_inertia.moment_about_x_kg_m2
                == projectile_before.principal_inertia.moment_about_x_kg_m2
            && projectile.principal_inertia.moment_about_y_kg_m2
                == projectile_before.principal_inertia.moment_about_y_kg_m2
            && projectile.principal_inertia.moment_about_z_kg_m2
                == projectile_before.principal_inertia.moment_about_z_kg_m2
            && projectile.mass_kg == projectile_before.mass_kg
            && projectile.reference_diameter_m
                == projectile_before.reference_diameter_m
            && projectile.reference_area_m2 == projectile_before.reference_area_m2
            && reference_frame.gravitational_acceleration_reference_m_per_s2.x
                == reference_frame_before
                       .gravitational_acceleration_reference_m_per_s2.x
            && reference_frame.gravitational_acceleration_reference_m_per_s2.y
                == reference_frame_before
                       .gravitational_acceleration_reference_m_per_s2.y
            && reference_frame.gravitational_acceleration_reference_m_per_s2.z
                == reference_frame_before
                       .gravitational_acceleration_reference_m_per_s2.z
            && reference_frame
                   .reference_frame_origin_acceleration_reference_m_per_s2.x
                == reference_frame_before
                       .reference_frame_origin_acceleration_reference_m_per_s2.x
            && reference_frame
                   .reference_frame_origin_acceleration_reference_m_per_s2.y
                == reference_frame_before
                       .reference_frame_origin_acceleration_reference_m_per_s2.y
            && reference_frame
                   .reference_frame_origin_acceleration_reference_m_per_s2.z
                == reference_frame_before
                       .reference_frame_origin_acceleration_reference_m_per_s2.z
            && reference_frame
                   .reference_frame_angular_velocity_reference_rad_per_s.x
                == reference_frame_before
                       .reference_frame_angular_velocity_reference_rad_per_s.x
            && reference_frame
                   .reference_frame_angular_velocity_reference_rad_per_s.y
                == reference_frame_before
                       .reference_frame_angular_velocity_reference_rad_per_s.y
            && reference_frame
                   .reference_frame_angular_velocity_reference_rad_per_s.z
                == reference_frame_before
                       .reference_frame_angular_velocity_reference_rad_per_s.z
            && reference_frame
                   .reference_frame_angular_acceleration_reference_rad_per_s2.x
                == reference_frame_before
                       .reference_frame_angular_acceleration_reference_rad_per_s2.x
            && reference_frame
                   .reference_frame_angular_acceleration_reference_rad_per_s2.y
                == reference_frame_before
                       .reference_frame_angular_acceleration_reference_rad_per_s2.y
            && reference_frame
                   .reference_frame_angular_acceleration_reference_rad_per_s2.z
                == reference_frame_before
                       .reference_frame_angular_acceleration_reference_rad_per_s2.z
            && force_moment.applied_force_reference_n.x
                == force_moment_before.applied_force_reference_n.x
            && force_moment.applied_force_reference_n.y
                == force_moment_before.applied_force_reference_n.y
            && force_moment.applied_force_reference_n.z
                == force_moment_before.applied_force_reference_n.z
            && force_moment.applied_moment_body_n_m.x
                == force_moment_before.applied_moment_body_n_m.x
            && force_moment.applied_moment_body_n_m.y
                == force_moment_before.applied_moment_body_n_m.y
            && force_moment.applied_moment_body_n_m.z
                == force_moment_before.applied_moment_body_n_m.z
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}

static int
test_other_precision_families(void)
{
    const bbtc_eb_rigid_body_state_float_t float_state =
    {
        .orientation_world_from_body = {.x = 0.0f, .y = 0.0f, .z = 0.0f, .w = 1.0f},
        .position_m = {0},
        .velocity_m_per_s = {.x = 1.0f, .y = 2.0f, .z = 3.0f},
        .angular_velocity_body_rad_per_s = {.x = 2.0f, .y = 0.0f, .z = 0.0f},
        .time_s = 0.0f
    };
    const bbtc_eb_projectile_properties_float_t float_projectile =
    {
        .principal_inertia =
        {
            .moment_about_x_kg_m2 = 2.0f,
            .moment_about_y_kg_m2 = 2.0f,
            .moment_about_z_kg_m2 = 2.0f
        },
        .mass_kg = 2.0f,
        .reference_diameter_m = 1.0f,
        .reference_area_m2 = 1.0f
    };
    const bbtc_eb_reference_frame_sample_float_t float_reference_frame = {0};
    const bbtc_eb_force_moment_float_t float_force_moment =
    {
        .applied_force_reference_n = {.x = 4.0f, .y = 0.0f, .z = 0.0f},
        .applied_moment_body_n_m = {.x = 4.0f, .y = 0.0f, .z = 0.0f}
    };
    bbtc_eb_rigid_body_state_derivative_float_t float_derivative = {0};

    if (bbtc_eb_inertial_vacuum_derivative_evaluate_float(
            &float_state,
            &float_projectile,
            &float_reference_frame,
            &float_force_moment,
            &float_derivative
        ) != BBTC_STATUS_SUCCESS
        || float_derivative.position_derivative_reference_m_per_s.x != 1.0f
        || float_derivative.velocity_derivative_reference_m_per_s2.x != 2.0f
        || float_derivative.orientation_world_from_body_derivative_per_s.x != 1.0f
        || float_derivative.angular_velocity_derivative_body_rad_per_s2.x != 2.0f)
    {
        return EXIT_FAILURE;
    }

    const bbtc_eb_rigid_body_state_long_double_t long_double_state =
    {
        .orientation_world_from_body = {.x = 0.0L, .y = 0.0L, .z = 0.0L, .w = 1.0L},
        .position_m = {0},
        .velocity_m_per_s = {.x = 1.0L, .y = 2.0L, .z = 3.0L},
        .angular_velocity_body_rad_per_s = {.x = 2.0L, .y = 0.0L, .z = 0.0L},
        .time_s = 0.0L
    };
    const bbtc_eb_projectile_properties_long_double_t long_double_projectile =
    {
        .principal_inertia =
        {
            .moment_about_x_kg_m2 = 2.0L,
            .moment_about_y_kg_m2 = 2.0L,
            .moment_about_z_kg_m2 = 2.0L
        },
        .mass_kg = 2.0L,
        .reference_diameter_m = 1.0L,
        .reference_area_m2 = 1.0L
    };
    const bbtc_eb_reference_frame_sample_long_double_t
        long_double_reference_frame = {0};
    const bbtc_eb_force_moment_long_double_t long_double_force_moment =
    {
        .applied_force_reference_n = {.x = 4.0L, .y = 0.0L, .z = 0.0L},
        .applied_moment_body_n_m = {.x = 4.0L, .y = 0.0L, .z = 0.0L}
    };
    bbtc_eb_rigid_body_state_derivative_long_double_t
        long_double_derivative = {0};

    if (bbtc_eb_inertial_vacuum_derivative_evaluate_long_double(
            &long_double_state,
            &long_double_projectile,
            &long_double_reference_frame,
            &long_double_force_moment,
            &long_double_derivative
        ) != BBTC_STATUS_SUCCESS
        || long_double_derivative.position_derivative_reference_m_per_s.x != 1.0L
        || long_double_derivative.velocity_derivative_reference_m_per_s2.x != 2.0L
        || long_double_derivative
               .orientation_world_from_body_derivative_per_s.x != 1.0L
        || long_double_derivative
               .angular_velocity_derivative_body_rad_per_s2.x != 2.0L)
    {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

int
main(void)
{
    if (test_null_and_validation_failures() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_inertial_reference_restriction() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_translational_equations() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_quaternion_multiplication_order() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_euler_rotational_dynamics() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_generated_numerical_failures() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_inputs_are_read_only() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    return test_other_precision_families();
}
