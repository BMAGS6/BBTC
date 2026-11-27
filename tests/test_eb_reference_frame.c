/**
 * @file
 * @brief Tests EB0.1-B6 reference-frame representation and validation.
 *
 * These tests freeze native precision-family representation, declaration order,
 * zero-sample validity, arbitrary finite-vector admissibility, independence of
 * gravity and frame kinematics, deterministic NaN/infinity precedence, and
 * read-only validation.
 */

#include <bbtc/bbtc.h>

#include <math.h>
#include <stddef.h>
#include <stdlib.h>

#define BBTC_EB_ASSERT_REFERENCE_FRAME_MEMBER_TYPES(SAMPLE_T, VEC_T)       \
    static_assert(_Generic(                                                \
        ((SAMPLE_T*)0)->gravitational_acceleration_reference_m_per_s2,     \
        VEC_T: 1,                                                          \
        default: 0                                                         \
    ));                                                                    \
    static_assert(_Generic(                                                \
        ((SAMPLE_T*)0)                                                     \
            ->reference_frame_origin_acceleration_reference_m_per_s2,      \
        VEC_T: 1,                                                          \
        default: 0                                                         \
    ));                                                                    \
    static_assert(_Generic(                                                \
        ((SAMPLE_T*)0)                                                     \
            ->reference_frame_angular_velocity_reference_rad_per_s,        \
        VEC_T: 1,                                                          \
        default: 0                                                         \
    ));                                                                    \
    static_assert(_Generic(                                                \
        ((SAMPLE_T*)0)                                                     \
            ->reference_frame_angular_acceleration_reference_rad_per_s2,   \
        VEC_T: 1,                                                          \
        default: 0                                                         \
    ))

BBTC_EB_ASSERT_REFERENCE_FRAME_MEMBER_TYPES(
    bbtc_eb_reference_frame_sample_float_t,
    bbtc_eb_vec3_float_t
);
BBTC_EB_ASSERT_REFERENCE_FRAME_MEMBER_TYPES(
    bbtc_eb_reference_frame_sample_double_t,
    bbtc_eb_vec3_double_t
);
BBTC_EB_ASSERT_REFERENCE_FRAME_MEMBER_TYPES(
    bbtc_eb_reference_frame_sample_long_double_t,
    bbtc_eb_vec3_long_double_t
);

#undef BBTC_EB_ASSERT_REFERENCE_FRAME_MEMBER_TYPES

#define BBTC_EB_ASSERT_REFERENCE_FRAME_ORDER(TYPE)                         \
    static_assert(                                                         \
        offsetof(TYPE, gravitational_acceleration_reference_m_per_s2)      \
            < offsetof(                                                    \
                TYPE,                                                      \
                reference_frame_origin_acceleration_reference_m_per_s2     \
            )                                                              \
    );                                                                     \
    static_assert(                                                         \
        offsetof(                                                          \
            TYPE,                                                          \
            reference_frame_origin_acceleration_reference_m_per_s2         \
        )                                                                  \
            < offsetof(                                                    \
                TYPE,                                                      \
                reference_frame_angular_velocity_reference_rad_per_s       \
            )                                                              \
    );                                                                     \
    static_assert(                                                         \
        offsetof(                                                          \
            TYPE,                                                          \
            reference_frame_angular_velocity_reference_rad_per_s           \
        )                                                                  \
            < offsetof(                                                    \
                TYPE,                                                      \
                reference_frame_angular_acceleration_reference_rad_per_s2  \
            )                                                              \
    )

BBTC_EB_ASSERT_REFERENCE_FRAME_ORDER(bbtc_eb_reference_frame_sample_float_t);
BBTC_EB_ASSERT_REFERENCE_FRAME_ORDER(bbtc_eb_reference_frame_sample_double_t);
BBTC_EB_ASSERT_REFERENCE_FRAME_ORDER(
    bbtc_eb_reference_frame_sample_long_double_t
);

#undef BBTC_EB_ASSERT_REFERENCE_FRAME_ORDER


/**
 * @brief Returns one representative Earth-like finite native-double sample.
 */
static bbtc_eb_reference_frame_sample_double_t
earth_like_double_sample(void)
{
    return (bbtc_eb_reference_frame_sample_double_t)
    {
        .gravitational_acceleration_reference_m_per_s2 =
        {
            .x = 0.012,
            .y = -0.021,
            .z = 9.81
        },
        .reference_frame_origin_acceleration_reference_m_per_s2 =
        {
            .x = -0.019,
            .y = 0.004,
            .z = 0.011
        },
        .reference_frame_angular_velocity_reference_rad_per_s =
        {
            .x = 0.0,
            .y = 5.16e-5,
            .z = -5.16e-5
        },
        .reference_frame_angular_acceleration_reference_rad_per_s2 =
        {
            .x = 0.0,
            .y = 0.0,
            .z = 0.0
        }
    };
}


/**
 * @brief Verifies null rejection and exact all-zero-sample validity.
 */
static int
test_zero_sample_and_null(void)
{
    const bbtc_eb_reference_frame_sample_double_t zero = {0};

    return bbtc_eb_reference_frame_sample_validate_double(NULL)
                == BBTC_STATUS_INVALID_ARGUMENT
        && bbtc_eb_reference_frame_sample_validate_double(&zero)
                == BBTC_STATUS_SUCCESS
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Verifies arbitrary finite signed vectors and Earth-like values.
 */
static int
test_finite_values_are_accepted(void)
{
    bbtc_eb_reference_frame_sample_double_t sample = earth_like_double_sample();

    if (bbtc_eb_reference_frame_sample_validate_double(&sample)
        != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    sample.gravitational_acceleration_reference_m_per_s2 =
        (bbtc_eb_vec3_double_t){.x = -1000.0, .y = 0.0, .z = 1000.0};
    sample.reference_frame_origin_acceleration_reference_m_per_s2 =
        (bbtc_eb_vec3_double_t){.x = 500.0, .y = -250.0, .z = 125.0};
    sample.reference_frame_angular_velocity_reference_rad_per_s =
        (bbtc_eb_vec3_double_t){.x = -3.0, .y = 4.0, .z = -5.0};
    sample.reference_frame_angular_acceleration_reference_rad_per_s2 =
        (bbtc_eb_vec3_double_t){.x = 7.0, .y = -8.0, .z = 9.0};

    return bbtc_eb_reference_frame_sample_validate_double(&sample)
            == BBTC_STATUS_SUCCESS
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Verifies angular velocity and acceleration have no hidden relation.
 */
static int
test_rotational_independence(void)
{
    bbtc_eb_reference_frame_sample_double_t sample = {0};

    sample.reference_frame_angular_acceleration_reference_rad_per_s2.x = 3.0;

    if (bbtc_eb_reference_frame_sample_validate_double(&sample)
        != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    sample = (bbtc_eb_reference_frame_sample_double_t){0};
    sample.reference_frame_angular_velocity_reference_rad_per_s.y = -2.0;

    return bbtc_eb_reference_frame_sample_validate_double(&sample)
            == BBTC_STATUS_SUCCESS
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Verifies gravity and origin acceleration are independently accepted.
 */
static int
test_gravity_origin_independence(void)
{
    bbtc_eb_reference_frame_sample_double_t sample = {0};

    sample.gravitational_acceleration_reference_m_per_s2.z = 9.8;
    sample.reference_frame_origin_acceleration_reference_m_per_s2.z = -123.0;

    if (bbtc_eb_reference_frame_sample_validate_double(&sample)
        != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    sample.gravitational_acceleration_reference_m_per_s2.z = -0.25;
    sample.reference_frame_origin_acceleration_reference_m_per_s2.z = 0.0;

    return bbtc_eb_reference_frame_sample_validate_double(&sample)
            == BBTC_STATUS_SUCCESS
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Verifies complete-record NaN-before-infinity diagnostic precedence.
 */
static int
test_validation_precedence(void)
{
    bbtc_eb_reference_frame_sample_double_t sample = earth_like_double_sample();

    sample.reference_frame_angular_acceleration_reference_rad_per_s2.z = NAN;
    sample.gravitational_acceleration_reference_m_per_s2.x = INFINITY;

    if (bbtc_eb_reference_frame_sample_validate_double(&sample)
        != BBTC_STATUS_NAN_INPUT)
    {
        return EXIT_FAILURE;
    }

    sample.reference_frame_angular_acceleration_reference_rad_per_s2.z = 0.0;

    if (bbtc_eb_reference_frame_sample_validate_double(&sample)
        != BBTC_STATUS_NONFINITE_INPUT)
    {
        return EXIT_FAILURE;
    }

    sample.gravitational_acceleration_reference_m_per_s2.x = -INFINITY;

    return bbtc_eb_reference_frame_sample_validate_double(&sample)
            == BBTC_STATUS_NONFINITE_INPUT
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Verifies validation never mutates caller-owned B6 storage.
 */
static int
test_validation_is_read_only(void)
{
    bbtc_eb_reference_frame_sample_double_t sample = earth_like_double_sample();
    const bbtc_eb_reference_frame_sample_double_t before = sample;

    if (bbtc_eb_reference_frame_sample_validate_double(&sample)
        != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

#define BBTC_EB_VEC3_EQUAL(A, B)                                           \
    ((A).x == (B).x && (A).y == (B).y && (A).z == (B).z)

    const int unchanged =
        BBTC_EB_VEC3_EQUAL(
            sample.gravitational_acceleration_reference_m_per_s2,
            before.gravitational_acceleration_reference_m_per_s2
        )
        && BBTC_EB_VEC3_EQUAL(
            sample.reference_frame_origin_acceleration_reference_m_per_s2,
            before.reference_frame_origin_acceleration_reference_m_per_s2
        )
        && BBTC_EB_VEC3_EQUAL(
            sample.reference_frame_angular_velocity_reference_rad_per_s,
            before.reference_frame_angular_velocity_reference_rad_per_s
        )
        && BBTC_EB_VEC3_EQUAL(
            sample.reference_frame_angular_acceleration_reference_rad_per_s2,
            before.reference_frame_angular_acceleration_reference_rad_per_s2
        );

#undef BBTC_EB_VEC3_EQUAL

    return unchanged ? EXIT_SUCCESS : EXIT_FAILURE;
}


/**
 * @brief Exercises native float and long-double B6 validation entry points.
 */
static int
test_other_precision_families(void)
{
    bbtc_eb_reference_frame_sample_float_t float_sample = {0};
    float_sample.gravitational_acceleration_reference_m_per_s2.z = -9.0f;
    float_sample.reference_frame_angular_velocity_reference_rad_per_s.x =
        1.0f;

    bbtc_eb_reference_frame_sample_long_double_t long_double_sample = {0};
    long_double_sample
        .reference_frame_origin_acceleration_reference_m_per_s2.y = -2.0L;
    long_double_sample
        .reference_frame_angular_acceleration_reference_rad_per_s2.z = 3.0L;

    return bbtc_eb_reference_frame_sample_validate_float(&float_sample)
                == BBTC_STATUS_SUCCESS
        && bbtc_eb_reference_frame_sample_validate_long_double(
            &long_double_sample
        ) == BBTC_STATUS_SUCCESS
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Runs the EB0.1-B6 reference-frame sample contract tests.
 */
int
main(void)
{
    if (test_zero_sample_and_null() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_finite_values_are_accepted() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_rotational_independence() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_gravity_origin_independence() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_validation_precedence() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_validation_is_read_only() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    return test_other_precision_families();
}
