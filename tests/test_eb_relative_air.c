/**
 * @file
 * @brief Tests the EB2.1 projectile-relative-air derived-flow contract.
 *
 * These tests freeze:
 *
 * - public record field families and declaration order;
 * - evaluator signatures for float, double, and long double;
 * - projectile-minus-air relative-velocity sign convention;
 * - Euclidean relative speed;
 * - positive-density Mach and zero-density Mach sentinel semantics;
 * - dynamic-pressure units and equations;
 * - zero-speed and vacuum behavior;
 * - global null/NaN/infinity/B4-domain diagnostic precedence;
 * - exact-zero output on every failure;
 * - read-only input behavior; and
 * - native-family numerical robustness against avoidable overflow/underflow.
 */

#include <bbtc/bbtc.h>

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>


static_assert(
    _Generic(
        ((bbtc_eb_relative_air_float_t*)0)
            ->projectile_relative_to_air_velocity_world_m_per_s,
        bbtc_eb_vec3_float_t: 1,
        default: 0
    )
);
static_assert(
    _Generic(
        ((bbtc_eb_relative_air_double_t*)0)
            ->projectile_relative_to_air_velocity_world_m_per_s,
        bbtc_eb_vec3_double_t: 1,
        default: 0
    )
);
static_assert(
    _Generic(
        ((bbtc_eb_relative_air_long_double_t*)0)
            ->projectile_relative_to_air_velocity_world_m_per_s,
        bbtc_eb_vec3_long_double_t: 1,
        default: 0
    )
);

#define BBTC_EB_ASSERT_RELATIVE_AIR_SCALAR_TYPES(TYPE, SCALAR)             \
    static_assert(                                                        \
        _Generic(                                                         \
            ((TYPE*)0)->projectile_relative_to_air_speed_m_per_s,         \
            SCALAR: 1,                                                    \
            default: 0                                                    \
        )                                                                 \
    );                                                                    \
    static_assert(                                                        \
        _Generic(((TYPE*)0)->mach_number, SCALAR: 1, default: 0)          \
    );                                                                    \
    static_assert(                                                        \
        _Generic(((TYPE*)0)->dynamic_pressure_pa, SCALAR: 1, default: 0)  \
    )

BBTC_EB_ASSERT_RELATIVE_AIR_SCALAR_TYPES(
    bbtc_eb_relative_air_float_t,
    float
);
BBTC_EB_ASSERT_RELATIVE_AIR_SCALAR_TYPES(
    bbtc_eb_relative_air_double_t,
    double
);
BBTC_EB_ASSERT_RELATIVE_AIR_SCALAR_TYPES(
    bbtc_eb_relative_air_long_double_t,
    long double
);

#undef BBTC_EB_ASSERT_RELATIVE_AIR_SCALAR_TYPES

#define BBTC_EB_ASSERT_RELATIVE_AIR_ORDER(TYPE)                            \
    static_assert(                                                        \
        offsetof(TYPE, projectile_relative_to_air_velocity_world_m_per_s) \
            < offsetof(TYPE, projectile_relative_to_air_speed_m_per_s)    \
    );                                                                    \
    static_assert(                                                        \
        offsetof(TYPE, projectile_relative_to_air_speed_m_per_s)          \
            < offsetof(TYPE, mach_number)                                 \
    );                                                                    \
    static_assert(                                                        \
        offsetof(TYPE, mach_number)                                       \
            < offsetof(TYPE, dynamic_pressure_pa)                         \
    )

BBTC_EB_ASSERT_RELATIVE_AIR_ORDER(bbtc_eb_relative_air_float_t);
BBTC_EB_ASSERT_RELATIVE_AIR_ORDER(bbtc_eb_relative_air_double_t);
BBTC_EB_ASSERT_RELATIVE_AIR_ORDER(bbtc_eb_relative_air_long_double_t);

#undef BBTC_EB_ASSERT_RELATIVE_AIR_ORDER

typedef bbtc_status_e
(*relative_air_float_fn)(
    const bbtc_eb_vec3_float_t*,
    const bbtc_eb_environment_sample_float_t*,
    bbtc_eb_relative_air_float_t*
);

typedef bbtc_status_e
(*relative_air_double_fn)(
    const bbtc_eb_vec3_double_t*,
    const bbtc_eb_environment_sample_double_t*,
    bbtc_eb_relative_air_double_t*
);

typedef bbtc_status_e
(*relative_air_long_double_fn)(
    const bbtc_eb_vec3_long_double_t*,
    const bbtc_eb_environment_sample_long_double_t*,
    bbtc_eb_relative_air_long_double_t*
);

static_assert(
    _Generic(
        &bbtc_eb_relative_air_evaluate_float,
        relative_air_float_fn: 1,
        default: 0
    )
);
static_assert(
    _Generic(
        &bbtc_eb_relative_air_evaluate_double,
        relative_air_double_fn: 1,
        default: 0
    )
);
static_assert(
    _Generic(
        &bbtc_eb_relative_air_evaluate_long_double,
        relative_air_long_double_fn: 1,
        default: 0
    )
);


/**
 * @brief Return one valid native-float B4 sample with zero air velocity.
 */
static bbtc_eb_environment_sample_float_t
valid_environment_float(float density, float sound_speed)
{
    return (bbtc_eb_environment_sample_float_t)
    {
        .air_velocity_world_m_per_s = {0},
        .temperature_k = 1.0f,
        .absolute_pressure_pa = 0.0f,
        .air_density_kg_per_m3 = density,
        .speed_of_sound_m_per_s = sound_speed
    };
}


/**
 * @brief Return one valid native-double B4 sample with zero air velocity.
 */
static bbtc_eb_environment_sample_double_t
valid_environment_double(double density, double sound_speed)
{
    return (bbtc_eb_environment_sample_double_t)
    {
        .air_velocity_world_m_per_s = {0},
        .temperature_k = 1.0,
        .absolute_pressure_pa = 0.0,
        .air_density_kg_per_m3 = density,
        .speed_of_sound_m_per_s = sound_speed
    };
}


/**
 * @brief Return one valid native-long-double B4 sample with zero air velocity.
 */
static bbtc_eb_environment_sample_long_double_t
valid_environment_long_double(long double density, long double sound_speed)
{
    return (bbtc_eb_environment_sample_long_double_t)
    {
        .air_velocity_world_m_per_s = {0},
        .temperature_k = 1.0L,
        .absolute_pressure_pa = 0.0L,
        .air_density_kg_per_m3 = density,
        .speed_of_sound_m_per_s = sound_speed
    };
}


/** @brief Report whether every native-float EB2.1 result field is zero. */
static int
relative_air_is_zero_float(const bbtc_eb_relative_air_float_t* result)
{
    return result->projectile_relative_to_air_velocity_world_m_per_s.x == 0.0f
        && result->projectile_relative_to_air_velocity_world_m_per_s.y == 0.0f
        && result->projectile_relative_to_air_velocity_world_m_per_s.z == 0.0f
        && result->projectile_relative_to_air_speed_m_per_s == 0.0f
        && result->mach_number == 0.0f
        && result->dynamic_pressure_pa == 0.0f;
}


/** @brief Report whether every native-double EB2.1 result field is zero. */
static int
relative_air_is_zero_double(const bbtc_eb_relative_air_double_t* result)
{
    return result->projectile_relative_to_air_velocity_world_m_per_s.x == 0.0
        && result->projectile_relative_to_air_velocity_world_m_per_s.y == 0.0
        && result->projectile_relative_to_air_velocity_world_m_per_s.z == 0.0
        && result->projectile_relative_to_air_speed_m_per_s == 0.0
        && result->mach_number == 0.0
        && result->dynamic_pressure_pa == 0.0;
}


/** @brief Report whether every native-long-double EB2.1 result field is zero. */
static int
relative_air_is_zero_long_double(
    const bbtc_eb_relative_air_long_double_t* result
)
{
    return result->projectile_relative_to_air_velocity_world_m_per_s.x == 0.0L
        && result->projectile_relative_to_air_velocity_world_m_per_s.y == 0.0L
        && result->projectile_relative_to_air_velocity_world_m_per_s.z == 0.0L
        && result->projectile_relative_to_air_speed_m_per_s == 0.0L
        && result->mach_number == 0.0L
        && result->dynamic_pressure_pa == 0.0L;
}


/**
 * @brief Compare ordinary-scale native-double values with a small ulp cushion.
 */
static int
close_double(double actual, double expected)
{
    const double scale = fmax(1.0, fabs(expected));
    return fabs(actual - expected) <= 32.0 * DBL_EPSILON * scale;
}


/**
 * @brief Freeze the contract's stationary-air 3-4-5 example.
 *
 * The contract deliberately uses density 1.2 kg/m^3 and sound speed 2.5 m/s:
 * speed is 5 m/s, Mach is 2, and dynamic pressure is 15 Pa.
 */
static int
test_stationary_air_example(void)
{
    const bbtc_eb_vec3_double_t projectile_velocity =
    {
        .x = 3.0,
        .y = 4.0,
        .z = 0.0
    };

    const bbtc_eb_environment_sample_double_t environment =
    {
        .air_velocity_world_m_per_s = {0},
        .temperature_k = 288.15,
        .absolute_pressure_pa = 101325.0,
        .air_density_kg_per_m3 = 1.2,
        .speed_of_sound_m_per_s = 2.5
    };

    bbtc_eb_relative_air_double_t result = {0};

    if (bbtc_eb_relative_air_evaluate_double(
            &projectile_velocity,
            &environment,
            &result
        ) != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    return result.projectile_relative_to_air_velocity_world_m_per_s.x == 3.0
            && result.projectile_relative_to_air_velocity_world_m_per_s.y == 4.0
            && result.projectile_relative_to_air_velocity_world_m_per_s.z == 0.0
            && result.projectile_relative_to_air_speed_m_per_s == 5.0
            && result.mach_number == 2.0
            && close_double(result.dynamic_pressure_pa, 15.0)
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Freeze projectile-minus-air sign convention with non-collinear wind.
 *
 * The chosen difference (7,-6,6) has exact norm 11. Density 2 and sound speed
 * 11 then produce exact Mach 1 and exact dynamic pressure 121 Pa.
 */
static int
test_moving_air_sign_convention(void)
{
    const bbtc_eb_vec3_double_t projectile_velocity =
    {
        .x = 10.0,
        .y = -2.0,
        .z = 5.0
    };

    bbtc_eb_environment_sample_double_t environment =
        valid_environment_double(2.0, 11.0);

    environment.air_velocity_world_m_per_s =
        (bbtc_eb_vec3_double_t){.x = 3.0, .y = 4.0, .z = -1.0};

    bbtc_eb_relative_air_double_t result = {0};

    if (bbtc_eb_relative_air_evaluate_double(
            &projectile_velocity,
            &environment,
            &result
        ) != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    return result.projectile_relative_to_air_velocity_world_m_per_s.x == 7.0
            && result.projectile_relative_to_air_velocity_world_m_per_s.y == -6.0
            && result.projectile_relative_to_air_velocity_world_m_per_s.z == 6.0
            && result.projectile_relative_to_air_speed_m_per_s == 11.0
            && result.mach_number == 1.0
            && result.dynamic_pressure_pa == 121.0
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Verify zero relative speed has no invented direction or pressure.
 */
static int
test_zero_relative_speed(void)
{
    const bbtc_eb_vec3_double_t projectile_velocity =
    {
        .x = 12.0,
        .y = -3.0,
        .z = 8.0
    };

    bbtc_eb_environment_sample_double_t environment =
        valid_environment_double(1.25, 5.0);

    environment.air_velocity_world_m_per_s = projectile_velocity;

    bbtc_eb_relative_air_double_t result =
    {
        .projectile_relative_to_air_velocity_world_m_per_s =
            {.x = 1.0, .y = 1.0, .z = 1.0},
        .projectile_relative_to_air_speed_m_per_s = 1.0,
        .mach_number = 1.0,
        .dynamic_pressure_pa = 1.0
    };

    return bbtc_eb_relative_air_evaluate_double(
               &projectile_velocity,
               &environment,
               &result
           ) == BBTC_STATUS_SUCCESS
            && relative_air_is_zero_double(&result)
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Verify exact zero-density vacuum avoids sound-speed division.
 *
 * Relative velocity and speed remain meaningful kinematic quantities in the
 * zero-density sample, while Mach and dynamic pressure are exactly zero.
 */
static int
test_zero_density_vacuum(void)
{
    const bbtc_eb_vec3_double_t projectile_velocity =
        {.x = 3.0, .y = 4.0, .z = 0.0};

    const bbtc_eb_environment_sample_double_t environment =
        valid_environment_double(0.0, 0.0);

    bbtc_eb_relative_air_double_t result = {0};

    return bbtc_eb_relative_air_evaluate_double(
               &projectile_velocity,
               &environment,
               &result
           ) == BBTC_STATUS_SUCCESS
            && result.projectile_relative_to_air_speed_m_per_s == 5.0
            && result.mach_number == 0.0
            && result.dynamic_pressure_pa == 0.0
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Verify null handling, global nonfinite precedence, and B4 domain status.
 */
static int
test_validation_and_failure_state(void)
{
    bbtc_eb_vec3_double_t projectile_velocity = {.x = 1.0};
    bbtc_eb_environment_sample_double_t environment =
        valid_environment_double(1.0, 1.0);
    bbtc_eb_relative_air_double_t result =
    {
        .projectile_relative_to_air_speed_m_per_s = 9.0,
        .mach_number = 9.0,
        .dynamic_pressure_pa = 9.0
    };

    if (bbtc_eb_relative_air_evaluate_double(
            &projectile_velocity,
            &environment,
            NULL
        ) != BBTC_STATUS_INVALID_ARGUMENT)
    {
        return EXIT_FAILURE;
    }

    if (bbtc_eb_relative_air_evaluate_double(
            NULL,
            &environment,
            &result
        ) != BBTC_STATUS_INVALID_ARGUMENT
        || !relative_air_is_zero_double(&result))
    {
        return EXIT_FAILURE;
    }

    result = (bbtc_eb_relative_air_double_t)
    {
        .projectile_relative_to_air_speed_m_per_s = 9.0,
        .mach_number = 9.0,
        .dynamic_pressure_pa = 9.0
    };

    if (bbtc_eb_relative_air_evaluate_double(
            &projectile_velocity,
            NULL,
            &result
        ) != BBTC_STATUS_INVALID_ARGUMENT
        || !relative_air_is_zero_double(&result))
    {
        return EXIT_FAILURE;
    }

    projectile_velocity.x = INFINITY;
    environment.temperature_k = NAN;

    if (bbtc_eb_relative_air_evaluate_double(
            &projectile_velocity,
            &environment,
            &result
        ) != BBTC_STATUS_NAN_INPUT
        || !relative_air_is_zero_double(&result))
    {
        return EXIT_FAILURE;
    }

    projectile_velocity.x = NAN;
    environment.temperature_k = INFINITY;

    if (bbtc_eb_relative_air_evaluate_double(
            &projectile_velocity,
            &environment,
            &result
        ) != BBTC_STATUS_NAN_INPUT
        || !relative_air_is_zero_double(&result))
    {
        return EXIT_FAILURE;
    }

    projectile_velocity.x = INFINITY;
    environment.temperature_k = 0.0;

    if (bbtc_eb_relative_air_evaluate_double(
            &projectile_velocity,
            &environment,
            &result
        ) != BBTC_STATUS_NONFINITE_INPUT
        || !relative_air_is_zero_double(&result))
    {
        return EXIT_FAILURE;
    }

    projectile_velocity.x = 1.0;
    environment = valid_environment_double(1.0, 1.0);
    environment.temperature_k = 0.0;

    return bbtc_eb_relative_air_evaluate_double(
               &projectile_velocity,
               &environment,
               &result
           ) == BBTC_STATUS_OUTSIDE_DOMAIN
            && relative_air_is_zero_double(&result)
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Verify successful evaluation never mutates either caller input.
 *
 * Comparisons are fieldwise rather than bytewise because structure padding is
 * an implementation detail and is deliberately not part of the BBTC ABI
 * contract.
 */
static int
test_inputs_are_read_only(void)
{
    bbtc_eb_vec3_double_t projectile_velocity = {.x = 10.0, .y = 2.0, .z = -4.0};
    bbtc_eb_environment_sample_double_t environment =
        valid_environment_double(2.0, 5.0);
    environment.air_velocity_world_m_per_s =
        (bbtc_eb_vec3_double_t){.x = 5.0, .y = -2.0, .z = 8.0};

    const bbtc_eb_vec3_double_t projectile_before = projectile_velocity;
    const bbtc_eb_environment_sample_double_t environment_before = environment;
    bbtc_eb_relative_air_double_t result = {0};

    if (bbtc_eb_relative_air_evaluate_double(
            &projectile_velocity,
            &environment,
            &result
        ) != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    return projectile_velocity.x == projectile_before.x
            && projectile_velocity.y == projectile_before.y
            && projectile_velocity.z == projectile_before.z
            && environment.air_velocity_world_m_per_s.x
                == environment_before.air_velocity_world_m_per_s.x
            && environment.air_velocity_world_m_per_s.y
                == environment_before.air_velocity_world_m_per_s.y
            && environment.air_velocity_world_m_per_s.z
                == environment_before.air_velocity_world_m_per_s.z
            && environment.temperature_k == environment_before.temperature_k
            && environment.absolute_pressure_pa
                == environment_before.absolute_pressure_pa
            && environment.air_density_kg_per_m3
                == environment_before.air_density_kg_per_m3
            && environment.speed_of_sound_m_per_s
                == environment_before.speed_of_sound_m_per_s
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Exercise ordinary native-float and native-long-double evaluator paths.
 */
static int
test_other_precision_families(void)
{
    const bbtc_eb_vec3_float_t float_velocity = {.x = 3.0f, .y = 4.0f};
    const bbtc_eb_environment_sample_float_t float_environment =
        valid_environment_float(2.0f, 5.0f);
    bbtc_eb_relative_air_float_t float_result = {0};

    if (bbtc_eb_relative_air_evaluate_float(
            &float_velocity,
            &float_environment,
            &float_result
        ) != BBTC_STATUS_SUCCESS
        || float_result.projectile_relative_to_air_speed_m_per_s != 5.0f
        || float_result.mach_number != 1.0f
        || float_result.dynamic_pressure_pa != 25.0f)
    {
        return EXIT_FAILURE;
    }

    const bbtc_eb_vec3_long_double_t long_double_velocity =
        {.x = 3.0L, .y = 4.0L};
    const bbtc_eb_environment_sample_long_double_t long_double_environment =
        valid_environment_long_double(2.0L, 5.0L);
    bbtc_eb_relative_air_long_double_t long_double_result = {0};

    return bbtc_eb_relative_air_evaluate_long_double(
               &long_double_velocity,
               &long_double_environment,
               &long_double_result
           ) == BBTC_STATUS_SUCCESS
            && long_double_result.projectile_relative_to_air_speed_m_per_s == 5.0L
            && long_double_result.mach_number == 1.0L
            && long_double_result.dynamic_pressure_pa == 25.0L
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Exercise EB2.1 representability boundaries in native float.
 *
 * This matrix covers generated subtraction overflow, norm overflow, Mach
 * overflow, avoidable dynamic-pressure overflow, avoidable intermediate
 * underflow, unavoidable final-result underflow, and true final overflow.
 */
static int
test_numerical_robustness_float(void)
{
    const float min_positive = nextafterf((float)0, (float)1);
    const int edge_exponent = (FLT_MAX_EXP + 1) / 2;

    bbtc_eb_environment_sample_float_t environment =
        valid_environment_float((float)0, (float)0);
    bbtc_eb_vec3_float_t projectile_velocity = {0};
    bbtc_eb_relative_air_float_t result = {0};

    /*
     * Finite subtraction operands can generate infinity. Generated infinity is
     * a numerical failure, distinct from nonfinite caller input.
     */
    projectile_velocity.x = FLT_MAX;
    environment.air_velocity_world_m_per_s.x = -FLT_MAX;

    if (bbtc_eb_relative_air_evaluate_float(
            &projectile_velocity,
            &environment,
            &result
        ) != BBTC_STATUS_NUMERICAL_FAILURE
        || !relative_air_is_zero_float(&result))
    {
        return EXIT_FAILURE;
    }

    /*
     * Each component is finite, but the true Euclidean norm exceeds the
     * selected family's finite range.
     */
    environment = valid_environment_float((float)0, (float)0);
    projectile_velocity = (bbtc_eb_vec3_float_t){0};
    projectile_velocity.x = (float)(0.75f * FLT_MAX);
    projectile_velocity.y = (float)(0.75f * FLT_MAX);

    if (bbtc_eb_relative_air_evaluate_float(
            &projectile_velocity,
            &environment,
            &result
        ) != BBTC_STATUS_NUMERICAL_FAILURE
        || !relative_air_is_zero_float(&result))
    {
        return EXIT_FAILURE;
    }

    /* A finite speed divided by a tiny positive finite sound speed overflows. */
    environment = valid_environment_float((float)1, min_positive);
    projectile_velocity = (bbtc_eb_vec3_float_t){.x = (float)1};

    if (bbtc_eb_relative_air_evaluate_float(
            &projectile_velocity,
            &environment,
            &result
        ) != BBTC_STATUS_NUMERICAL_FAILURE
        || !relative_air_is_zero_float(&result))
    {
        return EXIT_FAILURE;
    }

    /*
     * speed^2 would overflow, but rho is correspondingly small and the final
     * dynamic pressure is exactly 2^(edge_exponent - 1), which is finite.
     */
    const float large_speed =
        scalbnf((float)1, edge_exponent);
    const float small_density =
        scalbnf((float)1, -edge_exponent);
    const float expected_large_pressure =
        scalbnf((float)1, edge_exponent - 1);

    environment = valid_environment_float(small_density, large_speed);
    projectile_velocity =
        (bbtc_eb_vec3_float_t){.x = large_speed};

    if (bbtc_eb_relative_air_evaluate_float(
            &projectile_velocity,
            &environment,
            &result
        ) != BBTC_STATUS_SUCCESS
        || result.dynamic_pressure_pa != expected_large_pressure
        || result.mach_number != (float)1)
    {
        return EXIT_FAILURE;
    }

    /*
     * Half of the smallest positive scalar may disappear as an intermediate,
     * yet multiplying the complete expression by 1024^2 gives the representable
     * nonzero result min_positive * 2^19.
     */
    const float expected_recovered_pressure =
        scalbnf(min_positive, 19);

    environment =
        valid_environment_float(min_positive, (float)1024);
    projectile_velocity =
        (bbtc_eb_vec3_float_t){.x = (float)1024};

    if (bbtc_eb_relative_air_evaluate_float(
            &projectile_velocity,
            &environment,
            &result
        ) != BBTC_STATUS_SUCCESS
        || result.dynamic_pressure_pa != expected_recovered_pressure
        || !(result.dynamic_pressure_pa > (float)0))
    {
        return EXIT_FAILURE;
    }

    /*
     * Here the mathematical final result itself is below the positive
     * representable range. EB2.1 accepts ordinary final rounding to +0.
     */
    environment =
        valid_environment_float(min_positive, (float)0.5);
    projectile_velocity =
        (bbtc_eb_vec3_float_t){.x = (float)0.5};

    if (bbtc_eb_relative_air_evaluate_float(
            &projectile_velocity,
            &environment,
            &result
        ) != BBTC_STATUS_SUCCESS
        || result.dynamic_pressure_pa != (float)0
        || signbit(result.dynamic_pressure_pa))
    {
        return EXIT_FAILURE;
    }

    /* A genuinely too-large final pressure must fail closed. */
    const float overflowing_speed =
        (float)1.5 * scalbnf((float)1, edge_exponent);

    environment =
        valid_environment_float((float)1, overflowing_speed);
    projectile_velocity =
        (bbtc_eb_vec3_float_t){.x = overflowing_speed};

    return bbtc_eb_relative_air_evaluate_float(
               &projectile_velocity,
               &environment,
               &result
           ) == BBTC_STATUS_NUMERICAL_FAILURE
            && relative_air_is_zero_float(&result)
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Exercise EB2.1 representability boundaries in native double.
 *
 * This matrix covers generated subtraction overflow, norm overflow, Mach
 * overflow, avoidable dynamic-pressure overflow, avoidable intermediate
 * underflow, unavoidable final-result underflow, and true final overflow.
 */
static int
test_numerical_robustness_double(void)
{
    const double min_positive = nextafter((double)0, (double)1);
    const int edge_exponent = (DBL_MAX_EXP + 1) / 2;

    bbtc_eb_environment_sample_double_t environment =
        valid_environment_double((double)0, (double)0);
    bbtc_eb_vec3_double_t projectile_velocity = {0};
    bbtc_eb_relative_air_double_t result = {0};

    /*
     * Finite subtraction operands can generate infinity. Generated infinity is
     * a numerical failure, distinct from nonfinite caller input.
     */
    projectile_velocity.x = DBL_MAX;
    environment.air_velocity_world_m_per_s.x = -DBL_MAX;

    if (bbtc_eb_relative_air_evaluate_double(
            &projectile_velocity,
            &environment,
            &result
        ) != BBTC_STATUS_NUMERICAL_FAILURE
        || !relative_air_is_zero_double(&result))
    {
        return EXIT_FAILURE;
    }

    /*
     * Each component is finite, but the true Euclidean norm exceeds the
     * selected family's finite range.
     */
    environment = valid_environment_double((double)0, (double)0);
    projectile_velocity = (bbtc_eb_vec3_double_t){0};
    projectile_velocity.x = (double)(0.75 * DBL_MAX);
    projectile_velocity.y = (double)(0.75 * DBL_MAX);

    if (bbtc_eb_relative_air_evaluate_double(
            &projectile_velocity,
            &environment,
            &result
        ) != BBTC_STATUS_NUMERICAL_FAILURE
        || !relative_air_is_zero_double(&result))
    {
        return EXIT_FAILURE;
    }

    /* A finite speed divided by a tiny positive finite sound speed overflows. */
    environment = valid_environment_double((double)1, min_positive);
    projectile_velocity = (bbtc_eb_vec3_double_t){.x = (double)1};

    if (bbtc_eb_relative_air_evaluate_double(
            &projectile_velocity,
            &environment,
            &result
        ) != BBTC_STATUS_NUMERICAL_FAILURE
        || !relative_air_is_zero_double(&result))
    {
        return EXIT_FAILURE;
    }

    /*
     * speed^2 would overflow, but rho is correspondingly small and the final
     * dynamic pressure is exactly 2^(edge_exponent - 1), which is finite.
     */
    const double large_speed =
        scalbn((double)1, edge_exponent);
    const double small_density =
        scalbn((double)1, -edge_exponent);
    const double expected_large_pressure =
        scalbn((double)1, edge_exponent - 1);

    environment = valid_environment_double(small_density, large_speed);
    projectile_velocity =
        (bbtc_eb_vec3_double_t){.x = large_speed};

    if (bbtc_eb_relative_air_evaluate_double(
            &projectile_velocity,
            &environment,
            &result
        ) != BBTC_STATUS_SUCCESS
        || result.dynamic_pressure_pa != expected_large_pressure
        || result.mach_number != (double)1)
    {
        return EXIT_FAILURE;
    }

    /*
     * Half of the smallest positive scalar may disappear as an intermediate,
     * yet multiplying the complete expression by 1024^2 gives the representable
     * nonzero result min_positive * 2^19.
     */
    const double expected_recovered_pressure =
        scalbn(min_positive, 19);

    environment =
        valid_environment_double(min_positive, (double)1024);
    projectile_velocity =
        (bbtc_eb_vec3_double_t){.x = (double)1024};

    if (bbtc_eb_relative_air_evaluate_double(
            &projectile_velocity,
            &environment,
            &result
        ) != BBTC_STATUS_SUCCESS
        || result.dynamic_pressure_pa != expected_recovered_pressure
        || !(result.dynamic_pressure_pa > (double)0))
    {
        return EXIT_FAILURE;
    }

    /*
     * Here the mathematical final result itself is below the positive
     * representable range. EB2.1 accepts ordinary final rounding to +0.
     */
    environment =
        valid_environment_double(min_positive, (double)0.5);
    projectile_velocity =
        (bbtc_eb_vec3_double_t){.x = (double)0.5};

    if (bbtc_eb_relative_air_evaluate_double(
            &projectile_velocity,
            &environment,
            &result
        ) != BBTC_STATUS_SUCCESS
        || result.dynamic_pressure_pa != (double)0
        || signbit(result.dynamic_pressure_pa))
    {
        return EXIT_FAILURE;
    }

    /* A genuinely too-large final pressure must fail closed. */
    const double overflowing_speed =
        (double)1.5 * scalbn((double)1, edge_exponent);

    environment =
        valid_environment_double((double)1, overflowing_speed);
    projectile_velocity =
        (bbtc_eb_vec3_double_t){.x = overflowing_speed};

    return bbtc_eb_relative_air_evaluate_double(
               &projectile_velocity,
               &environment,
               &result
           ) == BBTC_STATUS_NUMERICAL_FAILURE
            && relative_air_is_zero_double(&result)
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Exercise EB2.1 representability boundaries in native long double.
 *
 * This matrix covers generated subtraction overflow, norm overflow, Mach
 * overflow, avoidable dynamic-pressure overflow, avoidable intermediate
 * underflow, unavoidable final-result underflow, and true final overflow.
 */
static int
test_numerical_robustness_long_double(void)
{
    const long double min_positive = nextafterl((long double)0, (long double)1);
    const int edge_exponent = (LDBL_MAX_EXP + 1) / 2;

    bbtc_eb_environment_sample_long_double_t environment =
        valid_environment_long_double((long double)0, (long double)0);
    bbtc_eb_vec3_long_double_t projectile_velocity = {0};
    bbtc_eb_relative_air_long_double_t result = {0};

    /*
     * Finite subtraction operands can generate infinity. Generated infinity is
     * a numerical failure, distinct from nonfinite caller input.
     */
    projectile_velocity.x = LDBL_MAX;
    environment.air_velocity_world_m_per_s.x = -LDBL_MAX;

    if (bbtc_eb_relative_air_evaluate_long_double(
            &projectile_velocity,
            &environment,
            &result
        ) != BBTC_STATUS_NUMERICAL_FAILURE
        || !relative_air_is_zero_long_double(&result))
    {
        return EXIT_FAILURE;
    }

    /*
     * Each component is finite, but the true Euclidean norm exceeds the
     * selected family's finite range.
     */
    environment = valid_environment_long_double((long double)0, (long double)0);
    projectile_velocity = (bbtc_eb_vec3_long_double_t){0};
    projectile_velocity.x = (long double)(0.75L * LDBL_MAX);
    projectile_velocity.y = (long double)(0.75L * LDBL_MAX);

    if (bbtc_eb_relative_air_evaluate_long_double(
            &projectile_velocity,
            &environment,
            &result
        ) != BBTC_STATUS_NUMERICAL_FAILURE
        || !relative_air_is_zero_long_double(&result))
    {
        return EXIT_FAILURE;
    }

    /* A finite speed divided by a tiny positive finite sound speed overflows. */
    environment = valid_environment_long_double((long double)1, min_positive);
    projectile_velocity = (bbtc_eb_vec3_long_double_t){.x = (long double)1};

    if (bbtc_eb_relative_air_evaluate_long_double(
            &projectile_velocity,
            &environment,
            &result
        ) != BBTC_STATUS_NUMERICAL_FAILURE
        || !relative_air_is_zero_long_double(&result))
    {
        return EXIT_FAILURE;
    }

    /*
     * speed^2 would overflow, but rho is correspondingly small and the final
     * dynamic pressure is exactly 2^(edge_exponent - 1), which is finite.
     */
    const long double large_speed =
        scalbnl((long double)1, edge_exponent);
    const long double small_density =
        scalbnl((long double)1, -edge_exponent);
    const long double expected_large_pressure =
        scalbnl((long double)1, edge_exponent - 1);

    environment = valid_environment_long_double(small_density, large_speed);
    projectile_velocity =
        (bbtc_eb_vec3_long_double_t){.x = large_speed};

    if (bbtc_eb_relative_air_evaluate_long_double(
            &projectile_velocity,
            &environment,
            &result
        ) != BBTC_STATUS_SUCCESS
        || result.dynamic_pressure_pa != expected_large_pressure
        || result.mach_number != (long double)1)
    {
        return EXIT_FAILURE;
    }

    /*
     * Half of the smallest positive scalar may disappear as an intermediate,
     * yet multiplying the complete expression by 1024^2 gives the representable
     * nonzero result min_positive * 2^19.
     */
    const long double expected_recovered_pressure =
        scalbnl(min_positive, 19);

    environment =
        valid_environment_long_double(min_positive, (long double)1024);
    projectile_velocity =
        (bbtc_eb_vec3_long_double_t){.x = (long double)1024};

    if (bbtc_eb_relative_air_evaluate_long_double(
            &projectile_velocity,
            &environment,
            &result
        ) != BBTC_STATUS_SUCCESS
        || result.dynamic_pressure_pa != expected_recovered_pressure
        || !(result.dynamic_pressure_pa > (long double)0))
    {
        return EXIT_FAILURE;
    }

    /*
     * Here the mathematical final result itself is below the positive
     * representable range. EB2.1 accepts ordinary final rounding to +0.
     */
    environment =
        valid_environment_long_double(min_positive, (long double)0.5);
    projectile_velocity =
        (bbtc_eb_vec3_long_double_t){.x = (long double)0.5};

    if (bbtc_eb_relative_air_evaluate_long_double(
            &projectile_velocity,
            &environment,
            &result
        ) != BBTC_STATUS_SUCCESS
        || result.dynamic_pressure_pa != (long double)0
        || signbit(result.dynamic_pressure_pa))
    {
        return EXIT_FAILURE;
    }

    /* A genuinely too-large final pressure must fail closed. */
    const long double overflowing_speed =
        (long double)1.5 * scalbnl((long double)1, edge_exponent);

    environment =
        valid_environment_long_double((long double)1, overflowing_speed);
    projectile_velocity =
        (bbtc_eb_vec3_long_double_t){.x = overflowing_speed};

    return bbtc_eb_relative_air_evaluate_long_double(
               &projectile_velocity,
               &environment,
               &result
           ) == BBTC_STATUS_NUMERICAL_FAILURE
            && relative_air_is_zero_long_double(&result)
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}

/**
 * @brief Run the complete EB2.1 public-contract test matrix.
 */
int
main(void)
{
    if (test_stationary_air_example() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_moving_air_sign_convention() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_zero_relative_speed() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_zero_density_vacuum() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_validation_and_failure_state() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_inputs_are_read_only() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_other_precision_families() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_numerical_robustness_float() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_numerical_robustness_double() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_numerical_robustness_long_double() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    return EXIT_SUCCESS;
}
