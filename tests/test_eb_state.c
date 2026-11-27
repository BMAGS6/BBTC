/**
 * @file
 * @brief Tests EB0.1-B1 representation and EB0.1-B2 state validation.
 *
 * Representation tests preserve the B1 ABI/type contracts. Validation tests
 * add the B2 fail-closed semantics, quaternion acceptance tolerance, and
 * deterministic error precedence.
 */

#include <bbtc/bbtc.h>

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>

/* Every vector family must retain its native scalar representation. */
static_assert(_Generic(((bbtc_eb_vec3_float_t*)0)->x,
                       float: 1,
                       default: 0));
static_assert(_Generic(((bbtc_eb_vec3_double_t*)0)->y,
                       double: 1,
                       default: 0));
static_assert(_Generic(((bbtc_eb_vec3_long_double_t*)0)->z,
                       long double: 1,
                       default: 0));

/* Quaternion components must likewise remain native to each precision family. */
static_assert(_Generic(((bbtc_eb_quat_float_t*)0)->w,
                       float: 1,
                       default: 0));
static_assert(_Generic(((bbtc_eb_quat_double_t*)0)->x,
                       double: 1,
                       default: 0));
static_assert(_Generic(((bbtc_eb_quat_long_double_t*)0)->w,
                       long double: 1,
                       default: 0));

/* Nested state members must use the matching precision-qualified record types. */
static_assert(_Generic(((bbtc_eb_rigid_body_state_float_t*)0)
                           ->orientation_world_from_body,
                       bbtc_eb_quat_float_t: 1,
                       default: 0));
static_assert(_Generic(((bbtc_eb_rigid_body_state_double_t*)0)->position_m,
                       bbtc_eb_vec3_double_t: 1,
                       default: 0));
static_assert(_Generic(((bbtc_eb_rigid_body_state_long_double_t*)0)
                           ->angular_velocity_body_rad_per_s,
                       bbtc_eb_vec3_long_double_t: 1,
                       default: 0));
static_assert(_Generic(((bbtc_eb_rigid_body_state_float_t*)0)->time_s,
                       float: 1,
                       default: 0));
static_assert(_Generic(((bbtc_eb_rigid_body_state_double_t*)0)->time_s,
                       double: 1,
                       default: 0));
static_assert(_Generic(((bbtc_eb_rigid_body_state_long_double_t*)0)->time_s,
                       long double: 1,
                       default: 0));

/*
 * The contract freezes declaration order without freezing byte offsets or
 * padding. C guarantees that later non-bit-field structure members have higher
 * addresses, so increasing offsetof() values verify the order portably.
 */
static_assert(
    offsetof(bbtc_eb_rigid_body_state_double_t, orientation_world_from_body)
        < offsetof(bbtc_eb_rigid_body_state_double_t, position_m)
);
static_assert(
    offsetof(bbtc_eb_rigid_body_state_double_t, position_m)
        < offsetof(bbtc_eb_rigid_body_state_double_t, velocity_m_per_s)
);
static_assert(
    offsetof(bbtc_eb_rigid_body_state_double_t, velocity_m_per_s)
        < offsetof(
            bbtc_eb_rigid_body_state_double_t,
            angular_velocity_body_rad_per_s
        )
);
static_assert(
    offsetof(
        bbtc_eb_rigid_body_state_double_t,
        angular_velocity_body_rad_per_s
    ) < offsetof(bbtc_eb_rigid_body_state_double_t, time_s)
);


/**
 * @brief Verifies designated initialization of the native-float state.
 *
 * Binary-exact values are used so this representation-only test does not
 * accidentally introduce a numerical-tolerance policy before arithmetic
 * exists.
 *
 * @return `EXIT_SUCCESS` when every stored value is preserved.
 */
static int
test_float_state(void)
{
    const bbtc_eb_rigid_body_state_float_t state =
    {
        .orientation_world_from_body =
        {
            .x = 0.0f,
            .y = 0.0f,
            .z = 0.0f,
            .w = 1.0f
        },
        .position_m = { .x = 1.0f, .y = 2.0f, .z = 3.0f },
        .velocity_m_per_s = { .x = 4.0f, .y = 5.0f, .z = 6.0f },
        .angular_velocity_body_rad_per_s =
        {
            .x = 7.0f,
            .y = 8.0f,
            .z = 9.0f
        },
        .time_s = 10.0f
    };

    return state.orientation_world_from_body.w == 1.0f
        && state.position_m.x == 1.0f
        && state.velocity_m_per_s.y == 5.0f
        && state.angular_velocity_body_rad_per_s.z == 9.0f
        && state.time_s == 10.0f
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Verifies designated initialization of the native-double state.
 *
 * @return `EXIT_SUCCESS` when nested vector/quaternion values and time retain
 *         their exact assigned values.
 */
static int
test_double_state(void)
{
    const bbtc_eb_rigid_body_state_double_t state =
    {
        .orientation_world_from_body =
        {
            .x = 0.5,
            .y = 0.5,
            .z = -0.5,
            .w = 0.5
        },
        .position_m = { .x = 16.0, .y = 32.0, .z = 64.0 },
        .velocity_m_per_s = { .x = 128.0, .y = 256.0, .z = 512.0 },
        .angular_velocity_body_rad_per_s =
        {
            .x = 1.0,
            .y = 2.0,
            .z = 4.0
        },
        .time_s = 8.0
    };

    return state.orientation_world_from_body.x == 0.5
        && state.orientation_world_from_body.z == -0.5
        && state.position_m.z == 64.0
        && state.velocity_m_per_s.x == 128.0
        && state.angular_velocity_body_rad_per_s.y == 2.0
        && state.time_s == 8.0
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Verifies long-double storage and zero-initialized quaternion semantics.
 *
 * The zero record is deliberately inspected rather than "fixed." EB0.1-B1
 * requires deterministic zero-initialized storage while explicitly refusing to
 * pretend that an all-zero quaternion is the identity orientation.
 *
 * @return `EXIT_SUCCESS` when native long-double values are preserved and a
 *         zero-initialized quaternion remains exactly zero.
 */
static int
test_long_double_state_and_zero_storage(void)
{
    const bbtc_eb_rigid_body_state_long_double_t state =
    {
        .orientation_world_from_body =
        {
            .x = 0.0L,
            .y = 0.0L,
            .z = 0.0L,
            .w = 1.0L
        },
        .position_m = { .x = 2.0L, .y = 4.0L, .z = 8.0L },
        .velocity_m_per_s = { .x = 16.0L, .y = 32.0L, .z = 64.0L },
        .angular_velocity_body_rad_per_s =
        {
            .x = 128.0L,
            .y = 256.0L,
            .z = 512.0L
        },
        .time_s = 1024.0L
    };

    const bbtc_eb_rigid_body_state_long_double_t zero_state = {0};

    if (state.position_m.y != 4.0L
        || state.velocity_m_per_s.z != 64.0L
        || state.angular_velocity_body_rad_per_s.x != 128.0L
        || state.time_s != 1024.0L)
    {
        return EXIT_FAILURE;
    }

    return zero_state.orientation_world_from_body.x == 0.0L
        && zero_state.orientation_world_from_body.y == 0.0L
        && zero_state.orientation_world_from_body.z == 0.0L
        && zero_state.orientation_world_from_body.w == 0.0L
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}

/**
 * @brief Returns a valid native-double state suitable for mutation tests.
 */
static bbtc_eb_rigid_body_state_double_t
valid_double_state(void)
{
    return (bbtc_eb_rigid_body_state_double_t)
    {
        .orientation_world_from_body = {0.0, 0.0, 0.0, 1.0},
        .position_m = {-100.0, 50.0, -25.0},
        .velocity_m_per_s = {800.0, -20.0, 10.0},
        .angular_velocity_body_rad_per_s = {-5.0, 15.0, 1200.0},
        .time_s = 0.25
    };
}

/**
 * @brief Verifies domain rules and the float-compatible unit-norm tolerance.
 */
static int
test_double_validation_domains(void)
{
    bbtc_eb_rigid_body_state_double_t state = valid_double_state();

    if (bbtc_eb_rigid_body_state_validate_double(NULL)
        != BBTC_STATUS_INVALID_ARGUMENT)
    {
        return EXIT_FAILURE;
    }

    if (bbtc_eb_rigid_body_state_validate_double(&state)
        != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    state.time_s = -1.0;
    if (bbtc_eb_rigid_body_state_validate_double(&state)
        != BBTC_STATUS_OUTSIDE_DOMAIN)
    {
        return EXIT_FAILURE;
    }

    state = valid_double_state();
    state.orientation_world_from_body = (bbtc_eb_quat_double_t){0};
    if (bbtc_eb_rigid_body_state_validate_double(&state)
        != BBTC_STATUS_OUTSIDE_DOMAIN)
    {
        return EXIT_FAILURE;
    }

    state = valid_double_state();
    state.orientation_world_from_body.w = 2.0;
    if (bbtc_eb_rigid_body_state_validate_double(&state)
        != BBTC_STATUS_OUTSIDE_DOMAIN)
    {
        return EXIT_FAILURE;
    }

    state = valid_double_state();
    state.orientation_world_from_body.w =
        sqrt(1.0 + 32.0 * (double)FLT_EPSILON);
    if (bbtc_eb_rigid_body_state_validate_double(&state)
        != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    state = valid_double_state();
    state.orientation_world_from_body.w =
        sqrt(1.0 + 128.0 * (double)FLT_EPSILON);
    return bbtc_eb_rigid_body_state_validate_double(&state)
            == BBTC_STATUS_OUTSIDE_DOMAIN
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}

/**
 * @brief Verifies NaN/infinity/domain precedence across the complete state.
 */
static int
test_validation_precedence(void)
{
    bbtc_eb_rigid_body_state_double_t state = valid_double_state();
    state.position_m.x = NAN;
    state.velocity_m_per_s.y = INFINITY;
    state.time_s = -1.0;
    state.orientation_world_from_body = (bbtc_eb_quat_double_t){0};

    if (bbtc_eb_rigid_body_state_validate_double(&state)
        != BBTC_STATUS_NAN_INPUT)
    {
        return EXIT_FAILURE;
    }

    state.position_m.x = 0.0;
    if (bbtc_eb_rigid_body_state_validate_double(&state)
        != BBTC_STATUS_NONFINITE_INPUT)
    {
        return EXIT_FAILURE;
    }

    state.velocity_m_per_s.y = 0.0;
    return bbtc_eb_rigid_body_state_validate_double(&state)
            == BBTC_STATUS_OUTSIDE_DOMAIN
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}

/**
 * @brief Exercises public validation through native float and long double.
 */
static int
test_other_validation_families(void)
{
    const bbtc_eb_rigid_body_state_float_t float_state =
    {
        .orientation_world_from_body = {0.0f, 0.0f, 0.0f, 1.0f},
        .position_m = {-1.0f, 2.0f, -3.0f},
        .velocity_m_per_s = {4.0f, -5.0f, 6.0f},
        .angular_velocity_body_rad_per_s = {-7.0f, 8.0f, -9.0f},
        .time_s = -0.0f
    };

    const bbtc_eb_rigid_body_state_long_double_t long_double_state =
    {
        .orientation_world_from_body = {0.0L, 0.0L, 0.0L, 1.0L},
        .position_m = {-1.0L, 2.0L, -3.0L},
        .velocity_m_per_s = {4.0L, -5.0L, 6.0L},
        .angular_velocity_body_rad_per_s = {-7.0L, 8.0L, -9.0L},
        .time_s = 1.0L
    };

    return bbtc_eb_rigid_body_state_validate_float(&float_state)
            == BBTC_STATUS_SUCCESS
        && bbtc_eb_rigid_body_state_validate_long_double(&long_double_state)
            == BBTC_STATUS_SUCCESS
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


int
main(void)
{
    if (test_float_state() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_double_state() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_long_double_state_and_zero_storage() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_double_validation_domains() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_validation_precedence() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    return test_other_validation_families();
}
