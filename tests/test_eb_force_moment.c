/**
 * @file
 * @brief Tests the EB0.1-B8 applied force/moment value contract.
 *
 * The suite freezes precision-qualified member types and declaration order,
 * verifies zero and arbitrary finite values, proves complete-record
 * NaN-before-infinity precedence, checks read-only validation, and exercises
 * all three native precision families.
 *
 * No test derives acceleration, angular acceleration, or a relationship
 * between force and moment. Those behaviors belong to later equations-of-
 * motion checkpoints.
 */

#include <bbtc/bbtc.h>

#include <math.h>
#include <stddef.h>
#include <stdlib.h>

static_assert(_Generic(
    ((bbtc_eb_force_moment_float_t*)0)->applied_force_reference_n,
    bbtc_eb_vec3_float_t: 1,
    default: 0
));
static_assert(_Generic(
    ((bbtc_eb_force_moment_float_t*)0)->applied_moment_body_n_m,
    bbtc_eb_vec3_float_t: 1,
    default: 0
));
static_assert(_Generic(
    ((bbtc_eb_force_moment_double_t*)0)->applied_force_reference_n,
    bbtc_eb_vec3_double_t: 1,
    default: 0
));
static_assert(_Generic(
    ((bbtc_eb_force_moment_double_t*)0)->applied_moment_body_n_m,
    bbtc_eb_vec3_double_t: 1,
    default: 0
));
static_assert(_Generic(
    ((bbtc_eb_force_moment_long_double_t*)0)->applied_force_reference_n,
    bbtc_eb_vec3_long_double_t: 1,
    default: 0
));
static_assert(_Generic(
    ((bbtc_eb_force_moment_long_double_t*)0)->applied_moment_body_n_m,
    bbtc_eb_vec3_long_double_t: 1,
    default: 0
));

#define BBTC_EB_ASSERT_FORCE_MOMENT_ORDER(TYPE)                            \
    static_assert(                                                        \
        offsetof(TYPE, applied_force_reference_n)                         \
            < offsetof(TYPE, applied_moment_body_n_m)                     \
    )

BBTC_EB_ASSERT_FORCE_MOMENT_ORDER(bbtc_eb_force_moment_float_t);
BBTC_EB_ASSERT_FORCE_MOMENT_ORDER(bbtc_eb_force_moment_double_t);
BBTC_EB_ASSERT_FORCE_MOMENT_ORDER(bbtc_eb_force_moment_long_double_t);

#undef BBTC_EB_ASSERT_FORCE_MOMENT_ORDER

/**
 * @brief Builds one representative finite mixed-frame native-double record.
 *
 * The numerical values are intentionally unrelated. B8 validation must not
 * infer a physical relationship between reference-frame force and body-frame
 * moment.
 */
static bbtc_eb_force_moment_double_t
representative_double_value(void)
{
    return (bbtc_eb_force_moment_double_t)
    {
        .applied_force_reference_n =
        {
            .x = -125.0,
            .y = 4.5,
            .z = 0.25
        },
        .applied_moment_body_n_m =
        {
            .x = 0.001,
            .y = -7.0,
            .z = 42.0
        }
    };
}

/**
 * @brief Verifies the exact all-zero B8 record is valid.
 */
static int
test_zero_is_valid(void)
{
    const bbtc_eb_force_moment_double_t value = {0};

    return bbtc_eb_force_moment_validate_double(&value)
            == BBTC_STATUS_SUCCESS
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}

/**
 * @brief Verifies arbitrary finite signed vectors are accepted independently.
 */
static int
test_arbitrary_finite_values(void)
{
    bbtc_eb_force_moment_double_t value = representative_double_value();

    if (bbtc_eb_force_moment_validate_double(&value) != BBTC_STATUS_SUCCESS)
        return EXIT_FAILURE;

    value.applied_force_reference_n =
        (bbtc_eb_vec3_double_t){.x = 1.0e300, .y = -1.0e-300, .z = 17.0};
    value.applied_moment_body_n_m =
        (bbtc_eb_vec3_double_t){.x = -2.0e200, .y = 0.0, .z = 3.0e-200};

    return bbtc_eb_force_moment_validate_double(&value)
            == BBTC_STATUS_SUCCESS
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}

/**
 * @brief Verifies null, NaN, infinity, and full-record precedence semantics.
 */
static int
test_diagnostic_precedence(void)
{
    if (bbtc_eb_force_moment_validate_double(NULL)
        != BBTC_STATUS_INVALID_ARGUMENT)
    {
        return EXIT_FAILURE;
    }

    bbtc_eb_force_moment_double_t value = representative_double_value();

    value.applied_force_reference_n.x = INFINITY;
    value.applied_moment_body_n_m.z = NAN;

    if (bbtc_eb_force_moment_validate_double(&value)
        != BBTC_STATUS_NAN_INPUT)
    {
        return EXIT_FAILURE;
    }

    value = representative_double_value();
    value.applied_moment_body_n_m.y = NAN;

    if (bbtc_eb_force_moment_validate_double(&value)
        != BBTC_STATUS_NAN_INPUT)
    {
        return EXIT_FAILURE;
    }

    value = representative_double_value();
    value.applied_force_reference_n.z = -INFINITY;

    if (bbtc_eb_force_moment_validate_double(&value)
        != BBTC_STATUS_NONFINITE_INPUT)
    {
        return EXIT_FAILURE;
    }

    value = representative_double_value();
    value.applied_moment_body_n_m.x = INFINITY;

    return bbtc_eb_force_moment_validate_double(&value)
            == BBTC_STATUS_NONFINITE_INPUT
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}

/**
 * @brief Verifies successful validation does not modify caller-owned storage.
 */
static int
test_validation_is_read_only(void)
{
    bbtc_eb_force_moment_double_t value = representative_double_value();

    const double force_x = value.applied_force_reference_n.x;
    const double force_y = value.applied_force_reference_n.y;
    const double force_z = value.applied_force_reference_n.z;
    const double moment_x = value.applied_moment_body_n_m.x;
    const double moment_y = value.applied_moment_body_n_m.y;
    const double moment_z = value.applied_moment_body_n_m.z;

    if (bbtc_eb_force_moment_validate_double(&value) != BBTC_STATUS_SUCCESS)
        return EXIT_FAILURE;

    return value.applied_force_reference_n.x == force_x
            && value.applied_force_reference_n.y == force_y
            && value.applied_force_reference_n.z == force_z
            && value.applied_moment_body_n_m.x == moment_x
            && value.applied_moment_body_n_m.y == moment_y
            && value.applied_moment_body_n_m.z == moment_z
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}

/**
 * @brief Exercises the native-float and native-long-double B8 validators.
 */
static int
test_other_precision_families(void)
{
    const bbtc_eb_force_moment_float_t float_value =
    {
        .applied_force_reference_n = {.x = -1.0f, .y = 2.0f, .z = -3.0f},
        .applied_moment_body_n_m = {.x = 4.0f, .y = -5.0f, .z = 6.0f}
    };

    const bbtc_eb_force_moment_long_double_t long_double_value =
    {
        .applied_force_reference_n =
        {
            .x = -1.0L,
            .y = 2.0L,
            .z = -3.0L
        },
        .applied_moment_body_n_m =
        {
            .x = 4.0L,
            .y = -5.0L,
            .z = 6.0L
        }
    };

    return bbtc_eb_force_moment_validate_float(&float_value)
                == BBTC_STATUS_SUCCESS
            && bbtc_eb_force_moment_validate_long_double(&long_double_value)
                == BBTC_STATUS_SUCCESS
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}

/**
 * @brief Runs the complete EB0.1-B8 applied-force/moment test suite.
 */
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
