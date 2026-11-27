/**
 * @file
 * @brief Tests the EB0.1-B9 rigid-body state-derivative value contract.
 *
 * The suite freezes exact member identities/order, validates zero and arbitrary
 * finite derivative values, proves that quaternion derivatives are not treated
 * as orientations, checks complete-record NaN-before-infinity precedence, and
 * exercises all three native precision families.
 *
 * No test computes equations of motion. Relationships such as `dr/dt = v`,
 * force/mass acceleration, quaternion kinematics, and Euler rotational
 * dynamics belong to later EB1 work rather than B9 value validation.
 */

#include <bbtc/bbtc.h>

#include <math.h>
#include <stddef.h>
#include <stdlib.h>

static_assert(_Generic(
    ((bbtc_eb_rigid_body_state_derivative_float_t*)0)
        ->orientation_world_from_body_derivative_per_s,
    bbtc_eb_quat_float_t: 1,
    default: 0
));
static_assert(_Generic(
    ((bbtc_eb_rigid_body_state_derivative_float_t*)0)
        ->position_derivative_reference_m_per_s,
    bbtc_eb_vec3_float_t: 1,
    default: 0
));
static_assert(_Generic(
    ((bbtc_eb_rigid_body_state_derivative_double_t*)0)
        ->velocity_derivative_reference_m_per_s2,
    bbtc_eb_vec3_double_t: 1,
    default: 0
));
static_assert(_Generic(
    ((bbtc_eb_rigid_body_state_derivative_long_double_t*)0)
        ->angular_velocity_derivative_body_rad_per_s2,
    bbtc_eb_vec3_long_double_t: 1,
    default: 0
));

/*
 * Freeze every member's native precision-qualified type, rather than checking
 * only a representative subset. These assertions deliberately say nothing
 * about structure byte size or padding.
 */
static_assert(_Generic(
    ((bbtc_eb_rigid_body_state_derivative_float_t*)0)
        ->velocity_derivative_reference_m_per_s2,
    bbtc_eb_vec3_float_t: 1,
    default: 0
));
static_assert(_Generic(
    ((bbtc_eb_rigid_body_state_derivative_float_t*)0)
        ->angular_velocity_derivative_body_rad_per_s2,
    bbtc_eb_vec3_float_t: 1,
    default: 0
));

static_assert(_Generic(
    ((bbtc_eb_rigid_body_state_derivative_double_t*)0)
        ->orientation_world_from_body_derivative_per_s,
    bbtc_eb_quat_double_t: 1,
    default: 0
));
static_assert(_Generic(
    ((bbtc_eb_rigid_body_state_derivative_double_t*)0)
        ->position_derivative_reference_m_per_s,
    bbtc_eb_vec3_double_t: 1,
    default: 0
));
static_assert(_Generic(
    ((bbtc_eb_rigid_body_state_derivative_double_t*)0)
        ->angular_velocity_derivative_body_rad_per_s2,
    bbtc_eb_vec3_double_t: 1,
    default: 0
));

static_assert(_Generic(
    ((bbtc_eb_rigid_body_state_derivative_long_double_t*)0)
        ->orientation_world_from_body_derivative_per_s,
    bbtc_eb_quat_long_double_t: 1,
    default: 0
));
static_assert(_Generic(
    ((bbtc_eb_rigid_body_state_derivative_long_double_t*)0)
        ->position_derivative_reference_m_per_s,
    bbtc_eb_vec3_long_double_t: 1,
    default: 0
));
static_assert(_Generic(
    ((bbtc_eb_rigid_body_state_derivative_long_double_t*)0)
        ->velocity_derivative_reference_m_per_s2,
    bbtc_eb_vec3_long_double_t: 1,
    default: 0
));

#define BBTC_EB_ASSERT_STATE_DERIVATIVE_ORDER(TYPE)                        \
    static_assert(                                                        \
        offsetof(TYPE, orientation_world_from_body_derivative_per_s)      \
            < offsetof(TYPE, position_derivative_reference_m_per_s)       \
    );                                                                    \
    static_assert(                                                        \
        offsetof(TYPE, position_derivative_reference_m_per_s)             \
            < offsetof(TYPE, velocity_derivative_reference_m_per_s2)      \
    );                                                                    \
    static_assert(                                                        \
        offsetof(TYPE, velocity_derivative_reference_m_per_s2)            \
            < offsetof(TYPE, angular_velocity_derivative_body_rad_per_s2) \
    )

BBTC_EB_ASSERT_STATE_DERIVATIVE_ORDER(
    bbtc_eb_rigid_body_state_derivative_float_t
);
BBTC_EB_ASSERT_STATE_DERIVATIVE_ORDER(
    bbtc_eb_rigid_body_state_derivative_double_t
);
BBTC_EB_ASSERT_STATE_DERIVATIVE_ORDER(
    bbtc_eb_rigid_body_state_derivative_long_double_t
);

#undef BBTC_EB_ASSERT_STATE_DERIVATIVE_ORDER

static bbtc_eb_rigid_body_state_derivative_double_t
representative_double_value(void)
{
    return (bbtc_eb_rigid_body_state_derivative_double_t)
    {
        .orientation_world_from_body_derivative_per_s =
        {
            .x = 2.0,
            .y = -3.0,
            .z = 4.0,
            .w = -5.0
        },
        .position_derivative_reference_m_per_s =
        {
            .x = 700.0,
            .y = -2.0,
            .z = 1.0
        },
        .velocity_derivative_reference_m_per_s2 =
        {
            .x = -9.0,
            .y = 8.0,
            .z = -7.0
        },
        .angular_velocity_derivative_body_rad_per_s2 =
        {
            .x = 6.0,
            .y = -5.0,
            .z = 4.0
        }
    };
}

static int
test_zero_is_valid(void)
{
    const bbtc_eb_rigid_body_state_derivative_double_t derivative = {0};

    return bbtc_eb_rigid_body_state_derivative_validate_double(&derivative)
            == BBTC_STATUS_SUCCESS
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}

static int
test_arbitrary_finite_values(void)
{
    bbtc_eb_rigid_body_state_derivative_double_t derivative =
        representative_double_value();

    if (bbtc_eb_rigid_body_state_derivative_validate_double(&derivative)
        != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    derivative.position_derivative_reference_m_per_s.x = 1.0e300;
    derivative.velocity_derivative_reference_m_per_s2.y = -1.0e-300;
    derivative.angular_velocity_derivative_body_rad_per_s2.z = 2.0e250;
    derivative.orientation_world_from_body_derivative_per_s.w = -3.0e-250;

    return bbtc_eb_rigid_body_state_derivative_validate_double(&derivative)
            == BBTC_STATUS_SUCCESS
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}

static int
test_diagnostic_precedence(void)
{
    if (bbtc_eb_rigid_body_state_derivative_validate_double(NULL)
        != BBTC_STATUS_INVALID_ARGUMENT)
    {
        return EXIT_FAILURE;
    }

    bbtc_eb_rigid_body_state_derivative_double_t derivative =
        representative_double_value();

    derivative.orientation_world_from_body_derivative_per_s.x = INFINITY;
    derivative.angular_velocity_derivative_body_rad_per_s2.z = NAN;

    if (bbtc_eb_rigid_body_state_derivative_validate_double(&derivative)
        != BBTC_STATUS_NAN_INPUT)
    {
        return EXIT_FAILURE;
    }

    derivative = representative_double_value();
    derivative.velocity_derivative_reference_m_per_s2.y = NAN;

    if (bbtc_eb_rigid_body_state_derivative_validate_double(&derivative)
        != BBTC_STATUS_NAN_INPUT)
    {
        return EXIT_FAILURE;
    }

    derivative = representative_double_value();
    derivative.position_derivative_reference_m_per_s.z = -INFINITY;

    if (bbtc_eb_rigid_body_state_derivative_validate_double(&derivative)
        != BBTC_STATUS_NONFINITE_INPUT)
    {
        return EXIT_FAILURE;
    }

    derivative = representative_double_value();
    derivative.orientation_world_from_body_derivative_per_s.w = INFINITY;

    return bbtc_eb_rigid_body_state_derivative_validate_double(&derivative)
            == BBTC_STATUS_NONFINITE_INPUT
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}

static int
test_validation_is_read_only(void)
{
    bbtc_eb_rigid_body_state_derivative_double_t derivative =
        representative_double_value();
    const bbtc_eb_rigid_body_state_derivative_double_t before = derivative;

    if (bbtc_eb_rigid_body_state_derivative_validate_double(&derivative)
        != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    return derivative.orientation_world_from_body_derivative_per_s.x
                == before.orientation_world_from_body_derivative_per_s.x
            && derivative.orientation_world_from_body_derivative_per_s.y
                == before.orientation_world_from_body_derivative_per_s.y
            && derivative.orientation_world_from_body_derivative_per_s.z
                == before.orientation_world_from_body_derivative_per_s.z
            && derivative.orientation_world_from_body_derivative_per_s.w
                == before.orientation_world_from_body_derivative_per_s.w
            && derivative.position_derivative_reference_m_per_s.x
                == before.position_derivative_reference_m_per_s.x
            && derivative.position_derivative_reference_m_per_s.y
                == before.position_derivative_reference_m_per_s.y
            && derivative.position_derivative_reference_m_per_s.z
                == before.position_derivative_reference_m_per_s.z
            && derivative.velocity_derivative_reference_m_per_s2.x
                == before.velocity_derivative_reference_m_per_s2.x
            && derivative.velocity_derivative_reference_m_per_s2.y
                == before.velocity_derivative_reference_m_per_s2.y
            && derivative.velocity_derivative_reference_m_per_s2.z
                == before.velocity_derivative_reference_m_per_s2.z
            && derivative.angular_velocity_derivative_body_rad_per_s2.x
                == before.angular_velocity_derivative_body_rad_per_s2.x
            && derivative.angular_velocity_derivative_body_rad_per_s2.y
                == before.angular_velocity_derivative_body_rad_per_s2.y
            && derivative.angular_velocity_derivative_body_rad_per_s2.z
                == before.angular_velocity_derivative_body_rad_per_s2.z
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}

static int
test_other_precision_families(void)
{
    const bbtc_eb_rigid_body_state_derivative_float_t float_derivative =
    {
        .orientation_world_from_body_derivative_per_s =
        {
            .x = 2.0f,
            .y = -3.0f,
            .z = 4.0f,
            .w = -5.0f
        },
        .position_derivative_reference_m_per_s =
        {
            .x = 1.0f,
            .y = -2.0f,
            .z = 3.0f
        },
        .velocity_derivative_reference_m_per_s2 =
        {
            .x = -4.0f,
            .y = 5.0f,
            .z = -6.0f
        },
        .angular_velocity_derivative_body_rad_per_s2 =
        {
            .x = 7.0f,
            .y = -8.0f,
            .z = 9.0f
        }
    };

    const bbtc_eb_rigid_body_state_derivative_long_double_t
        long_double_derivative =
    {
        .orientation_world_from_body_derivative_per_s =
        {
            .x = 2.0L,
            .y = -3.0L,
            .z = 4.0L,
            .w = -5.0L
        },
        .position_derivative_reference_m_per_s =
        {
            .x = 1.0L,
            .y = -2.0L,
            .z = 3.0L
        },
        .velocity_derivative_reference_m_per_s2 =
        {
            .x = -4.0L,
            .y = 5.0L,
            .z = -6.0L
        },
        .angular_velocity_derivative_body_rad_per_s2 =
        {
            .x = 7.0L,
            .y = -8.0L,
            .z = 9.0L
        }
    };

    return bbtc_eb_rigid_body_state_derivative_validate_float(
               &float_derivative
           ) == BBTC_STATUS_SUCCESS
            && bbtc_eb_rigid_body_state_derivative_validate_long_double(
                   &long_double_derivative
               ) == BBTC_STATUS_SUCCESS
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}

int
main(void)
{
    if (test_zero_is_valid() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_arbitrary_finite_values() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_diagnostic_precedence() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_validation_is_read_only() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    return test_other_precision_families();
}
