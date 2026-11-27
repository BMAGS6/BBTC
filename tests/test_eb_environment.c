/**
 * @file
 * @brief Tests EB0.1-B4 local environment representation and validation.
 *
 * These tests freeze native precision-family representation, declaration order,
 * world/reference-frame air-velocity semantics, exact-vacuum admissibility,
 * intentionally weak thermodynamic cross-field coupling, deterministic
 * diagnostic precedence, and read-only fail-closed validation.
 */

#include <bbtc/bbtc.h>

#include <math.h>
#include <stddef.h>
#include <stdlib.h>

/* The nested air-velocity record remains native in every precision family. */
static_assert(_Generic(((bbtc_eb_environment_sample_float_t*)0)
                           ->air_velocity_world_m_per_s,
                       bbtc_eb_vec3_float_t: 1,
                       default: 0));
static_assert(_Generic(((bbtc_eb_environment_sample_double_t*)0)
                           ->air_velocity_world_m_per_s,
                       bbtc_eb_vec3_double_t: 1,
                       default: 0));
static_assert(_Generic(((bbtc_eb_environment_sample_long_double_t*)0)
                           ->air_velocity_world_m_per_s,
                       bbtc_eb_vec3_long_double_t: 1,
                       default: 0));

/* Representative scalar members must retain their native precision families. */
static_assert(_Generic(((bbtc_eb_environment_sample_float_t*)0)->temperature_k,
                       float: 1,
                       default: 0));
static_assert(_Generic(((bbtc_eb_environment_sample_double_t*)0)
                           ->absolute_pressure_pa,
                       double: 1,
                       default: 0));
static_assert(_Generic(((bbtc_eb_environment_sample_long_double_t*)0)
                           ->air_density_kg_per_m3,
                       long double: 1,
                       default: 0));
static_assert(_Generic(((bbtc_eb_environment_sample_long_double_t*)0)
                           ->speed_of_sound_m_per_s,
                       long double: 1,
                       default: 0));

/*
 * C guarantees increasing addresses for later non-bit-field members. These
 * assertions freeze declaration order without freezing implementation-specific
 * padding, alignment, or one universal sizeof value.
 */
#define BBTC_EB_ASSERT_ENVIRONMENT_ORDER(TYPE)                            \
    static_assert(                                                        \
        offsetof(TYPE, air_velocity_world_m_per_s)                        \
            < offsetof(TYPE, temperature_k)                               \
    );                                                                    \
    static_assert(                                                        \
        offsetof(TYPE, temperature_k)                                     \
            < offsetof(TYPE, absolute_pressure_pa)                        \
    );                                                                    \
    static_assert(                                                        \
        offsetof(TYPE, absolute_pressure_pa)                              \
            < offsetof(TYPE, air_density_kg_per_m3)                       \
    );                                                                    \
    static_assert(                                                        \
        offsetof(TYPE, air_density_kg_per_m3)                             \
            < offsetof(TYPE, speed_of_sound_m_per_s)                      \
    )

BBTC_EB_ASSERT_ENVIRONMENT_ORDER(bbtc_eb_environment_sample_float_t);
BBTC_EB_ASSERT_ENVIRONMENT_ORDER(bbtc_eb_environment_sample_double_t);
BBTC_EB_ASSERT_ENVIRONMENT_ORDER(bbtc_eb_environment_sample_long_double_t);

#undef BBTC_EB_ASSERT_ENVIRONMENT_ORDER


/**
 * @brief Returns one representative positive-density native-double sample.
 */
static bbtc_eb_environment_sample_double_t
valid_double_sample(void)
{
    return (bbtc_eb_environment_sample_double_t)
    {
        .air_velocity_world_m_per_s =
        {
            .x = -12.0,
            .y = 3.0,
            .z = 0.0
        },
        .temperature_k = 288.15,
        .absolute_pressure_pa = 101325.0,
        .air_density_kg_per_m3 = 1.225,
        .speed_of_sound_m_per_s = 340.29
    };
}


/**
 * @brief Verifies ordinary atmosphere and signed world-frame air velocity.
 */
static int
test_valid_positive_density_sample(void)
{
    const bbtc_eb_environment_sample_double_t sample = valid_double_sample();

    if (bbtc_eb_environment_sample_validate_double(NULL)
        != BBTC_STATUS_INVALID_ARGUMENT)
    {
        return EXIT_FAILURE;
    }

    if (bbtc_eb_environment_sample_validate_double(&sample)
        != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    return sample.air_velocity_world_m_per_s.x < 0.0
            && sample.air_velocity_world_m_per_s.y > 0.0
            && sample.air_velocity_world_m_per_s.z == 0.0
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Verifies the exact-vacuum representation and zero-density policy.
 */
static int
test_vacuum_policy(void)
{
    bbtc_eb_environment_sample_double_t sample = valid_double_sample();

    /*
     * Exact vacuum keeps a positive finite temperature while pressure, density,
     * and sound speed are zero. The finite air-velocity value is deliberately
     * nonzero: validation must not rewrite it merely because density is zero.
     */
    sample.air_velocity_world_m_per_s.x = 100.0;
    sample.air_velocity_world_m_per_s.y = -200.0;
    sample.air_velocity_world_m_per_s.z = 300.0;
    sample.temperature_k = 1.0;
    sample.absolute_pressure_pa = 0.0;
    sample.air_density_kg_per_m3 = 0.0;
    sample.speed_of_sound_m_per_s = 0.0;

    if (bbtc_eb_environment_sample_validate_double(&sample)
        != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    /*
     * B4 does not impose an equation of state. Zero density plus nonzero
     * pressure is not the canonical exact-vacuum representation, but it remains
     * valid at this deliberately weak value-validation boundary.
     */
    sample.absolute_pressure_pa = 50000.0;

    return bbtc_eb_environment_sample_validate_double(&sample)
            == BBTC_STATUS_SUCCESS
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Verifies individual scalar domain requirements.
 */
static int
test_domain_requirements(void)
{
    const bbtc_eb_environment_sample_double_t zero = {0};

    if (bbtc_eb_environment_sample_validate_double(&zero)
        != BBTC_STATUS_OUTSIDE_DOMAIN)
    {
        return EXIT_FAILURE;
    }

    bbtc_eb_environment_sample_double_t sample = valid_double_sample();

#define BBTC_EXPECT_DOMAIN_FAILURE(MEMBER, VALUE)                         \
    do                                                                    \
    {                                                                     \
        sample = valid_double_sample();                                   \
        sample.MEMBER = (VALUE);                                          \
        if (bbtc_eb_environment_sample_validate_double(&sample)           \
            != BBTC_STATUS_OUTSIDE_DOMAIN)                                \
        {                                                                 \
            return EXIT_FAILURE;                                          \
        }                                                                 \
    } while (0)

    BBTC_EXPECT_DOMAIN_FAILURE(temperature_k, 0.0);
    BBTC_EXPECT_DOMAIN_FAILURE(temperature_k, -1.0);
    BBTC_EXPECT_DOMAIN_FAILURE(absolute_pressure_pa, -1.0);
    BBTC_EXPECT_DOMAIN_FAILURE(air_density_kg_per_m3, -1.0);
    BBTC_EXPECT_DOMAIN_FAILURE(speed_of_sound_m_per_s, -1.0);

#undef BBTC_EXPECT_DOMAIN_FAILURE

    sample = valid_double_sample();
    sample.speed_of_sound_m_per_s = 0.0;

    return bbtc_eb_environment_sample_validate_double(&sample)
            == BBTC_STATUS_OUTSIDE_DOMAIN
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Verifies that B4 validation does not impose a hidden equation of state.
 */
static int
test_cross_field_independence(void)
{
    bbtc_eb_environment_sample_double_t sample = valid_double_sample();

    /*
     * These values are deliberately not intended to describe one standard or
     * ideal-gas atmosphere. Each field satisfies its own B4 domain, which is
     * sufficient at this sampled-value boundary.
     */
    sample.temperature_k = 2.0;
    sample.absolute_pressure_pa = 900000.0;
    sample.air_density_kg_per_m3 = 0.25;
    sample.speed_of_sound_m_per_s = 7.0;

    return bbtc_eb_environment_sample_validate_double(&sample)
            == BBTC_STATUS_SUCCESS
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Verifies complete-record NaN, infinity, then domain precedence.
 */
static int
test_validation_precedence(void)
{
    bbtc_eb_environment_sample_double_t sample = valid_double_sample();

    sample.air_velocity_world_m_per_s.x = NAN;
    sample.absolute_pressure_pa = INFINITY;
    sample.temperature_k = -1.0;

    if (bbtc_eb_environment_sample_validate_double(&sample)
        != BBTC_STATUS_NAN_INPUT)
    {
        return EXIT_FAILURE;
    }

    sample.air_velocity_world_m_per_s.x = 0.0;

    if (bbtc_eb_environment_sample_validate_double(&sample)
        != BBTC_STATUS_NONFINITE_INPUT)
    {
        return EXIT_FAILURE;
    }

    sample.absolute_pressure_pa = 101325.0;

    return bbtc_eb_environment_sample_validate_double(&sample)
            == BBTC_STATUS_OUTSIDE_DOMAIN
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Verifies validation never mutates caller-owned environment storage.
 */
static int
test_validation_is_read_only(void)
{
    bbtc_eb_environment_sample_double_t sample = valid_double_sample();
    const bbtc_eb_environment_sample_double_t before = sample;

    if (bbtc_eb_environment_sample_validate_double(&sample)
        != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    return sample.air_velocity_world_m_per_s.x
                == before.air_velocity_world_m_per_s.x
        && sample.air_velocity_world_m_per_s.y
                == before.air_velocity_world_m_per_s.y
        && sample.air_velocity_world_m_per_s.z
                == before.air_velocity_world_m_per_s.z
        && sample.temperature_k == before.temperature_k
        && sample.absolute_pressure_pa == before.absolute_pressure_pa
        && sample.air_density_kg_per_m3 == before.air_density_kg_per_m3
        && sample.speed_of_sound_m_per_s == before.speed_of_sound_m_per_s
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Exercises native float and long-double validation entry points.
 */
static int
test_other_precision_families(void)
{
    const bbtc_eb_environment_sample_float_t float_sample =
    {
        .air_velocity_world_m_per_s =
        {
            .x = -1.0f,
            .y = 0.0f,
            .z = 1.0f
        },
        .temperature_k = 300.0f,
        .absolute_pressure_pa = 100000.0f,
        .air_density_kg_per_m3 = 1.0f,
        .speed_of_sound_m_per_s = 340.0f
    };

    const bbtc_eb_environment_sample_long_double_t long_double_vacuum =
    {
        .air_velocity_world_m_per_s =
        {
            .x = 1.0L,
            .y = -2.0L,
            .z = 3.0L
        },
        .temperature_k = 1.0L,
        .absolute_pressure_pa = 0.0L,
        .air_density_kg_per_m3 = 0.0L,
        .speed_of_sound_m_per_s = 0.0L
    };

    return bbtc_eb_environment_sample_validate_float(&float_sample)
                == BBTC_STATUS_SUCCESS
        && bbtc_eb_environment_sample_validate_long_double(
            &long_double_vacuum
        ) == BBTC_STATUS_SUCCESS
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


int
main(void)
{
    if (test_valid_positive_density_sample() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_vacuum_policy() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_domain_requirements() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_cross_field_independence() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_validation_precedence() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_validation_is_read_only() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    return test_other_precision_families();
}
