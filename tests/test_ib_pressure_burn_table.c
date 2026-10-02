/**
 * @file
 * @brief Tests tabulated absolute-pressure propellant burn kinetics.
 *
 * @details
 * These tests cover the IB0.4d-B2 pressure-burn table backend across native
 * `float`, `double`, and `long double`. They verify structural validation,
 * complete-table NaN/infinity precedence, finite data-domain rules, strict
 * pressure ordering, deliberately nonmonotonic burn-rate support, exact-knot
 * recovery, log/log interpolation, represented-domain rejection, deterministic
 * output clearing, caller-storage immutability, and extreme-scale numerical
 * behavior without using commercial propellant data or making a firing
 * prediction.
 */
#include <float.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <bbtc/bbtc.h>


/**
 * @brief Reports one failed test condition.
 *
 * @param expression Failed expression text.
 * @param line Source line of the failure.
 *
 * @return `EXIT_FAILURE`.
 */
static int
fail(const char* const expression, const int line)
{
    fprintf(stderr, "FAIL line %d: %s\n", line, expression);
    return EXIT_FAILURE;
}


#define CHECK(expression)                       \
    do                                          \
    {                                           \
        if (!(expression))                      \
            return fail(#expression, __LINE__); \
    } while (0)


/**
 * @brief Verifies valid table records in every native scalar family.
 *
 * @return `EXIT_SUCCESS` on success; otherwise `EXIT_FAILURE`.
 */
static int
test_validation_all_precisions(void)
{
    const bbtc_ib_pressure_burn_point_float_t points_f[] =
    {
        { .pressure_pa = 1.0f, .burn_rate_m_per_s = 2.0f },
        { .pressure_pa = 4.0f, .burn_rate_m_per_s = 8.0f }
    };

    const bbtc_ib_pressure_burn_point_double_t points_d[] =
    {
        { .pressure_pa = 1.0, .burn_rate_m_per_s = 2.0 },
        { .pressure_pa = 4.0, .burn_rate_m_per_s = 8.0 }
    };

    const bbtc_ib_pressure_burn_point_long_double_t points_l[] =
    {
        { .pressure_pa = 1.0L, .burn_rate_m_per_s = 2.0L },
        { .pressure_pa = 4.0L, .burn_rate_m_per_s = 8.0L }
    };

    const bbtc_ib_pressure_burn_table_float_t model_f =
    {
        .points = points_f,
        .point_count = 2U
    };

    const bbtc_ib_pressure_burn_table_double_t model_d =
    {
        .points = points_d,
        .point_count = 2U
    };

    const bbtc_ib_pressure_burn_table_long_double_t model_l =
    {
        .points = points_l,
        .point_count = 2U
    };

    CHECK(
        bbtc_ib_pressure_burn_table_validate_float(&model_f)
            == BBTC_STATUS_SUCCESS
    );

    CHECK(
        bbtc_ib_pressure_burn_table_validate_double(&model_d)
            == BBTC_STATUS_SUCCESS
    );

    CHECK(
        bbtc_ib_pressure_burn_table_validate_long_double(&model_l)
            == BBTC_STATUS_SUCCESS
    );

    return EXIT_SUCCESS;
}


/**
 * @brief Verifies structural table validation and point-count semantics.
 *
 * @return `EXIT_SUCCESS` on success; otherwise `EXIT_FAILURE`.
 */
static int
test_structural_validation(void)
{
    const bbtc_ib_pressure_burn_point_double_t one_point[] =
    {
        { .pressure_pa = 1.0, .burn_rate_m_per_s = 1.0 }
    };

    const bbtc_ib_pressure_burn_table_double_t null_points =
    {
        .points = NULL,
        .point_count = 2U
    };

    const bbtc_ib_pressure_burn_table_double_t zero_points =
    {
        .points = one_point,
        .point_count = 0U
    };

    const bbtc_ib_pressure_burn_table_double_t one_point_model =
    {
        .points = one_point,
        .point_count = 1U
    };

    CHECK(
        bbtc_ib_pressure_burn_table_validate_double(NULL)
            == BBTC_STATUS_INVALID_ARGUMENT
    );

    CHECK(
        bbtc_ib_pressure_burn_table_validate_double(&null_points)
            == BBTC_STATUS_INVALID_ARGUMENT
    );

    CHECK(
        bbtc_ib_pressure_burn_table_validate_double(&zero_points)
            == BBTC_STATUS_OUTSIDE_DOMAIN
    );

    CHECK(
        bbtc_ib_pressure_burn_table_validate_double(&one_point_model)
            == BBTC_STATUS_OUTSIDE_DOMAIN
    );

    return EXIT_SUCCESS;
}


/**
 * @brief Verifies complete-table NaN-before-infinity status precedence.
 *
 * @details
 * Malformed values are intentionally placed in different points so a validator
 * that simply returns the first problem encountered would fail these tests.
 *
 * @return `EXIT_SUCCESS` on success; otherwise `EXIT_FAILURE`.
 */
static int
test_nonfinite_precedence(void)
{
    bbtc_ib_pressure_burn_point_double_t points[] =
    {
        { .pressure_pa = -1.0, .burn_rate_m_per_s = 1.0 },
        { .pressure_pa = 2.0,  .burn_rate_m_per_s = INFINITY },
        { .pressure_pa = 3.0,  .burn_rate_m_per_s = NAN }
    };

    const bbtc_ib_pressure_burn_table_double_t model =
    {
        .points = points,
        .point_count = 3U
    };

    CHECK(
        bbtc_ib_pressure_burn_table_validate_double(&model)
            == BBTC_STATUS_NAN_INPUT
    );

    points[2].burn_rate_m_per_s = 1.0;

    CHECK(
        bbtc_ib_pressure_burn_table_validate_double(&model)
            == BBTC_STATUS_NONFINITE_INPUT
    );

    points[1].burn_rate_m_per_s = 1.0;

    CHECK(
        bbtc_ib_pressure_burn_table_validate_double(&model)
            == BBTC_STATUS_OUTSIDE_DOMAIN
    );

    points[0].pressure_pa = INFINITY;
    points[2].burn_rate_m_per_s = NAN;

    CHECK(
        bbtc_ib_pressure_burn_table_validate_double(&model)
            == BBTC_STATUS_NAN_INPUT
    );

    return EXIT_SUCCESS;
}


/**
 * @brief Verifies finite positivity, pressure ordering, and rate-shape rules.
 *
 * @return `EXIT_SUCCESS` on success; otherwise `EXIT_FAILURE`.
 */
static int
test_finite_domain_and_ordering(void)
{
    const bbtc_ib_pressure_burn_point_double_t valid_points[] =
    {
        { .pressure_pa = 1.0, .burn_rate_m_per_s = 4.0 },
        { .pressure_pa = 2.0, .burn_rate_m_per_s = 2.0 },
        { .pressure_pa = 4.0, .burn_rate_m_per_s = 8.0 }
    };

    bbtc_ib_pressure_burn_point_double_t points[3] =
    {
        { .pressure_pa = 1.0, .burn_rate_m_per_s = 4.0 },
        { .pressure_pa = 2.0, .burn_rate_m_per_s = 2.0 },
        { .pressure_pa = 4.0, .burn_rate_m_per_s = 8.0 }
    };

    const bbtc_ib_pressure_burn_table_double_t valid_model =
    {
        .points = valid_points,
        .point_count = 3U
    };

    const bbtc_ib_pressure_burn_table_double_t model =
    {
        .points = points,
        .point_count = 3U
    };

    /*
     * Burn rate is deliberately nonmonotonic here: 4 -> 2 -> 8. The backend
     * must accept such a table because only pressure is required to be strictly
     * increasing.
     */
    CHECK(
        bbtc_ib_pressure_burn_table_validate_double(&valid_model)
            == BBTC_STATUS_SUCCESS
    );

    points[0].pressure_pa = 0.0;
    CHECK(
        bbtc_ib_pressure_burn_table_validate_double(&model)
            == BBTC_STATUS_OUTSIDE_DOMAIN
    );

    points[0].pressure_pa = -1.0;
    CHECK(
        bbtc_ib_pressure_burn_table_validate_double(&model)
            == BBTC_STATUS_OUTSIDE_DOMAIN
    );

    points[0] = valid_points[0];
    points[1].burn_rate_m_per_s = 0.0;
    CHECK(
        bbtc_ib_pressure_burn_table_validate_double(&model)
            == BBTC_STATUS_OUTSIDE_DOMAIN
    );

    points[1] = valid_points[1];
    points[1].pressure_pa = points[0].pressure_pa;
    CHECK(
        bbtc_ib_pressure_burn_table_validate_double(&model)
            == BBTC_STATUS_OUTSIDE_DOMAIN
    );

    points[1] = valid_points[1];
    points[2].pressure_pa = 1.5;
    CHECK(
        bbtc_ib_pressure_burn_table_validate_double(&model)
            == BBTC_STATUS_OUTSIDE_DOMAIN
    );

    points[2] = valid_points[2];
    points[2].burn_rate_m_per_s = -1.0;
    CHECK(
        bbtc_ib_pressure_burn_table_validate_double(&model)
            == BBTC_STATUS_OUTSIDE_DOMAIN
    );

    return EXIT_SUCCESS;
}


/**
 * @brief Verifies exact stored-knot recovery in every native scalar family.
 *
 * @return `EXIT_SUCCESS` on success; otherwise `EXIT_FAILURE`.
 */
static int
test_exact_knots_all_precisions(void)
{
    const bbtc_ib_pressure_burn_point_float_t points_f[] =
    {
        { .pressure_pa = 1.0f, .burn_rate_m_per_s = 3.0f },
        { .pressure_pa = 2.0f, .burn_rate_m_per_s = 5.0f },
        { .pressure_pa = 4.0f, .burn_rate_m_per_s = 7.0f }
    };

    const bbtc_ib_pressure_burn_point_double_t points_d[] =
    {
        { .pressure_pa = 1.0, .burn_rate_m_per_s = 3.0 },
        { .pressure_pa = 2.0, .burn_rate_m_per_s = 5.0 },
        { .pressure_pa = 4.0, .burn_rate_m_per_s = 7.0 }
    };

    const bbtc_ib_pressure_burn_point_long_double_t points_l[] =
    {
        { .pressure_pa = 1.0L, .burn_rate_m_per_s = 3.0L },
        { .pressure_pa = 2.0L, .burn_rate_m_per_s = 5.0L },
        { .pressure_pa = 4.0L, .burn_rate_m_per_s = 7.0L }
    };

    const bbtc_ib_pressure_burn_table_float_t model_f =
    {
        .points = points_f,
        .point_count = 3U
    };

    const bbtc_ib_pressure_burn_table_double_t model_d =
    {
        .points = points_d,
        .point_count = 3U
    };

    const bbtc_ib_pressure_burn_table_long_double_t model_l =
    {
        .points = points_l,
        .point_count = 3U
    };

    bbtc_ib_propellant_burn_kinetics_result_float_t result_f = {0};
    bbtc_ib_propellant_burn_kinetics_result_double_t result_d = {0};
    bbtc_ib_propellant_burn_kinetics_result_long_double_t result_l = {0};

    CHECK(
        bbtc_ib_pressure_burn_table_evaluate_float(
            &model_f,
            points_f[0].pressure_pa,
            &result_f
        ) == BBTC_STATUS_SUCCESS
    );
    CHECK(result_f.burn_rate_m_per_s == points_f[0].burn_rate_m_per_s);
    CHECK(result_f.applicability_flags == BBTC_APPLICABILITY_NONE_REPORTED);

    CHECK(
        bbtc_ib_pressure_burn_table_evaluate_float(
            &model_f,
            points_f[1].pressure_pa,
            &result_f
        ) == BBTC_STATUS_SUCCESS
    );
    CHECK(result_f.burn_rate_m_per_s == points_f[1].burn_rate_m_per_s);

    CHECK(
        bbtc_ib_pressure_burn_table_evaluate_float(
            &model_f,
            points_f[2].pressure_pa,
            &result_f
        ) == BBTC_STATUS_SUCCESS
    );
    CHECK(result_f.burn_rate_m_per_s == points_f[2].burn_rate_m_per_s);

    CHECK(
        bbtc_ib_pressure_burn_table_evaluate_double(
            &model_d,
            points_d[0].pressure_pa,
            &result_d
        ) == BBTC_STATUS_SUCCESS
    );
    CHECK(result_d.burn_rate_m_per_s == points_d[0].burn_rate_m_per_s);

    CHECK(
        bbtc_ib_pressure_burn_table_evaluate_double(
            &model_d,
            points_d[1].pressure_pa,
            &result_d
        ) == BBTC_STATUS_SUCCESS
    );
    CHECK(result_d.burn_rate_m_per_s == points_d[1].burn_rate_m_per_s);

    CHECK(
        bbtc_ib_pressure_burn_table_evaluate_double(
            &model_d,
            points_d[2].pressure_pa,
            &result_d
        ) == BBTC_STATUS_SUCCESS
    );
    CHECK(result_d.burn_rate_m_per_s == points_d[2].burn_rate_m_per_s);
    CHECK(result_d.applicability_flags == BBTC_APPLICABILITY_NONE_REPORTED);

    CHECK(
        bbtc_ib_pressure_burn_table_evaluate_long_double(
            &model_l,
            points_l[0].pressure_pa,
            &result_l
        ) == BBTC_STATUS_SUCCESS
    );
    CHECK(result_l.burn_rate_m_per_s == points_l[0].burn_rate_m_per_s);

    CHECK(
        bbtc_ib_pressure_burn_table_evaluate_long_double(
            &model_l,
            points_l[1].pressure_pa,
            &result_l
        ) == BBTC_STATUS_SUCCESS
    );
    CHECK(result_l.burn_rate_m_per_s == points_l[1].burn_rate_m_per_s);

    CHECK(
        bbtc_ib_pressure_burn_table_evaluate_long_double(
            &model_l,
            points_l[2].pressure_pa,
            &result_l
        ) == BBTC_STATUS_SUCCESS
    );
    CHECK(result_l.burn_rate_m_per_s == points_l[2].burn_rate_m_per_s);
    CHECK(result_l.applicability_flags == BBTC_APPLICABILITY_NONE_REPORTED);

    return EXIT_SUCCESS;
}


/**
 * @brief Verifies the specified piecewise log/log interpolation relation.
 *
 * @details
 * The first segment follows `r = P^2` through `(1, 1)` and `(4, 16)`, so the
 * geometric midpoint in pressure, `P = 2`, must evaluate to approximately
 * `r = 4`. This distinguishes the required relation from ordinary linear
 * interpolation in pressure/rate coordinates.
 *
 * @return `EXIT_SUCCESS` on success; otherwise `EXIT_FAILURE`.
 */
static int
test_log_log_interpolation_all_precisions(void)
{
    const bbtc_ib_pressure_burn_point_float_t points_f[] =
    {
        { .pressure_pa = 1.0f, .burn_rate_m_per_s = 1.0f },
        { .pressure_pa = 4.0f, .burn_rate_m_per_s = 16.0f }
    };

    const bbtc_ib_pressure_burn_point_double_t points_d[] =
    {
        { .pressure_pa = 1.0, .burn_rate_m_per_s = 1.0 },
        { .pressure_pa = 4.0, .burn_rate_m_per_s = 16.0 }
    };

    const bbtc_ib_pressure_burn_point_long_double_t points_l[] =
    {
        { .pressure_pa = 1.0L, .burn_rate_m_per_s = 1.0L },
        { .pressure_pa = 4.0L, .burn_rate_m_per_s = 16.0L }
    };

    const bbtc_ib_pressure_burn_table_float_t model_f =
    {
        .points = points_f,
        .point_count = 2U
    };

    const bbtc_ib_pressure_burn_table_double_t model_d =
    {
        .points = points_d,
        .point_count = 2U
    };

    const bbtc_ib_pressure_burn_table_long_double_t model_l =
    {
        .points = points_l,
        .point_count = 2U
    };

    bbtc_ib_propellant_burn_kinetics_result_float_t result_f = {0};
    bbtc_ib_propellant_burn_kinetics_result_double_t result_d = {0};
    bbtc_ib_propellant_burn_kinetics_result_long_double_t result_l = {0};

    CHECK(
        bbtc_ib_pressure_burn_table_evaluate_float(
            &model_f,
            2.0f,
            &result_f
        ) == BBTC_STATUS_SUCCESS
    );
    CHECK(
        fabsf(result_f.burn_rate_m_per_s - 4.0f)
            <= 256.0f * FLT_EPSILON
    );

    CHECK(
        bbtc_ib_pressure_burn_table_evaluate_double(
            &model_d,
            2.0,
            &result_d
        ) == BBTC_STATUS_SUCCESS
    );
    CHECK(
        fabs(result_d.burn_rate_m_per_s - 4.0)
            <= 512.0 * DBL_EPSILON
    );

    CHECK(
        bbtc_ib_pressure_burn_table_evaluate_long_double(
            &model_l,
            2.0L,
            &result_l
        ) == BBTC_STATUS_SUCCESS
    );
    CHECK(
        fabsl(result_l.burn_rate_m_per_s - 4.0L)
            <= 1024.0L * LDBL_EPSILON
    );

    return EXIT_SUCCESS;
}


/**
 * @brief Verifies model-before-state ordering and deterministic failure output.
 *
 * @return `EXIT_SUCCESS` on success; otherwise `EXIT_FAILURE`.
 */
static int
test_evaluator_failure_semantics(void)
{
    const bbtc_ib_pressure_burn_point_double_t valid_points[] =
    {
        { .pressure_pa = 10.0, .burn_rate_m_per_s = 1.0 },
        { .pressure_pa = 20.0, .burn_rate_m_per_s = 2.0 },
        { .pressure_pa = 40.0, .burn_rate_m_per_s = 4.0 }
    };

    const bbtc_ib_pressure_burn_table_double_t valid_model =
    {
        .points = valid_points,
        .point_count = 3U
    };

    const bbtc_ib_pressure_burn_table_double_t invalid_model =
    {
        .points = valid_points,
        .point_count = 1U
    };

    bbtc_ib_propellant_burn_kinetics_result_double_t result =
    {
        .applicability_flags = UINT64_MAX,
        .burn_rate_m_per_s = 9.0
    };

    CHECK(
        bbtc_ib_pressure_burn_table_evaluate_double(
            &valid_model,
            20.0,
            NULL
        ) == BBTC_STATUS_INVALID_ARGUMENT
    );

    CHECK(
        bbtc_ib_pressure_burn_table_evaluate_double(
            NULL,
            20.0,
            &result
        ) == BBTC_STATUS_INVALID_ARGUMENT
    );
    CHECK(result.applicability_flags == BBTC_APPLICABILITY_NONE_REPORTED);
    CHECK(result.burn_rate_m_per_s == 0.0);

    result.applicability_flags = UINT64_MAX;
    result.burn_rate_m_per_s = 9.0;

    /*
     * Complete model validation has precedence over later direct-pressure
     * classification.
     */
    CHECK(
        bbtc_ib_pressure_burn_table_evaluate_double(
            &invalid_model,
            NAN,
            &result
        ) == BBTC_STATUS_OUTSIDE_DOMAIN
    );
    CHECK(result.applicability_flags == BBTC_APPLICABILITY_NONE_REPORTED);
    CHECK(result.burn_rate_m_per_s == 0.0);

    result.applicability_flags = UINT64_MAX;
    result.burn_rate_m_per_s = 9.0;

    CHECK(
        bbtc_ib_pressure_burn_table_evaluate_double(
            &valid_model,
            NAN,
            &result
        ) == BBTC_STATUS_NAN_INPUT
    );
    CHECK(result.applicability_flags == BBTC_APPLICABILITY_NONE_REPORTED);
    CHECK(result.burn_rate_m_per_s == 0.0);

    result.applicability_flags = UINT64_MAX;
    result.burn_rate_m_per_s = 9.0;

    CHECK(
        bbtc_ib_pressure_burn_table_evaluate_double(
            &valid_model,
            INFINITY,
            &result
        ) == BBTC_STATUS_NONFINITE_INPUT
    );
    CHECK(result.applicability_flags == BBTC_APPLICABILITY_NONE_REPORTED);
    CHECK(result.burn_rate_m_per_s == 0.0);

    result.applicability_flags = UINT64_MAX;
    result.burn_rate_m_per_s = 9.0;

    CHECK(
        bbtc_ib_pressure_burn_table_evaluate_double(
            &valid_model,
            -INFINITY,
            &result
        ) == BBTC_STATUS_NONFINITE_INPUT
    );
    CHECK(result.applicability_flags == BBTC_APPLICABILITY_NONE_REPORTED);
    CHECK(result.burn_rate_m_per_s == 0.0);

    result.applicability_flags = UINT64_MAX;
    result.burn_rate_m_per_s = 9.0;

    CHECK(
        bbtc_ib_pressure_burn_table_evaluate_double(
            &valid_model,
            0.0,
            &result
        ) == BBTC_STATUS_OUTSIDE_DOMAIN
    );
    CHECK(result.applicability_flags == BBTC_APPLICABILITY_NONE_REPORTED);
    CHECK(result.burn_rate_m_per_s == 0.0);

    result.applicability_flags = UINT64_MAX;
    result.burn_rate_m_per_s = 9.0;

    CHECK(
        bbtc_ib_pressure_burn_table_evaluate_double(
            &valid_model,
            9.0,
            &result
        ) == BBTC_STATUS_OUTSIDE_DOMAIN
    );
    CHECK(result.applicability_flags == BBTC_APPLICABILITY_NONE_REPORTED);
    CHECK(result.burn_rate_m_per_s == 0.0);

    result.applicability_flags = UINT64_MAX;
    result.burn_rate_m_per_s = 9.0;

    CHECK(
        bbtc_ib_pressure_burn_table_evaluate_double(
            &valid_model,
            41.0,
            &result
        ) == BBTC_STATUS_OUTSIDE_DOMAIN
    );
    CHECK(result.applicability_flags == BBTC_APPLICABILITY_NONE_REPORTED);
    CHECK(result.burn_rate_m_per_s == 0.0);

    return EXIT_SUCCESS;
}


/**
 * @brief Verifies interpolation across an explicitly nonmonotonic rate table.
 *
 * @return `EXIT_SUCCESS` on success; otherwise `EXIT_FAILURE`.
 */
static int
test_nonmonotonic_rate_evaluation(void)
{
    const bbtc_ib_pressure_burn_point_double_t points[] =
    {
        { .pressure_pa = 1.0, .burn_rate_m_per_s = 4.0 },
        { .pressure_pa = 4.0, .burn_rate_m_per_s = 1.0 },
        { .pressure_pa = 16.0, .burn_rate_m_per_s = 4.0 }
    };

    const bbtc_ib_pressure_burn_table_double_t model =
    {
        .points = points,
        .point_count = 3U
    };

    bbtc_ib_propellant_burn_kinetics_result_double_t result = {0};

    CHECK(
        bbtc_ib_pressure_burn_table_evaluate_double(
            &model,
            2.0,
            &result
        ) == BBTC_STATUS_SUCCESS
    );
    CHECK(
        fabs(result.burn_rate_m_per_s - 2.0)
            <= 512.0 * DBL_EPSILON
    );

    CHECK(
        bbtc_ib_pressure_burn_table_evaluate_double(
            &model,
            8.0,
            &result
        ) == BBTC_STATUS_SUCCESS
    );
    CHECK(
        fabs(result.burn_rate_m_per_s - 2.0)
            <= 512.0 * DBL_EPSILON
    );

    return EXIT_SUCCESS;
}


/**
 * @brief Verifies exact evaluation across a constant-rate table segment.
 *
 * @details
 * Equal adjacent burn-rate knots represent a flat segment in log/log space.
 * The evaluator preserves the stored rate exactly rather than needlessly
 * reconstructing the same value through logarithm and exponential operations.
 *
 * @return `EXIT_SUCCESS` on success; otherwise `EXIT_FAILURE`.
 */
static int
test_flat_segment_exactness(void)
{
    const bbtc_ib_pressure_burn_point_double_t points[] =
    {
        { .pressure_pa = 1.0, .burn_rate_m_per_s = DBL_TRUE_MIN },
        { .pressure_pa = 4.0, .burn_rate_m_per_s = DBL_TRUE_MIN }
    };

    const bbtc_ib_pressure_burn_table_double_t model =
    {
        .points = points,
        .point_count = 2U
    };

    bbtc_ib_propellant_burn_kinetics_result_double_t result = {0};

    CHECK(
        bbtc_ib_pressure_burn_table_evaluate_double(
            &model,
            2.0,
            &result
        ) == BBTC_STATUS_SUCCESS
    );
    CHECK(result.burn_rate_m_per_s == DBL_TRUE_MIN);
    CHECK(result.applicability_flags == BBTC_APPLICABILITY_NONE_REPORTED);

    return EXIT_SUCCESS;
}


/**
 * @brief Verifies robust behavior when naive explicit ratios would overflow.
 *
 * @details
 * The first table spans `DBL_TRUE_MIN` to unity in pressure. Forming
 * `1.0 / DBL_TRUE_MIN` directly overflows, but the specified logarithmic
 * relation remains finite. The second table spans `DBL_TRUE_MIN` to `DBL_MAX`
 * in burn rate, where a direct endpoint-rate ratio likewise overflows.
 *
 * @return `EXIT_SUCCESS` on success; otherwise `EXIT_FAILURE`.
 */
static int
test_extreme_scale_interpolation(void)
{
    const bbtc_ib_pressure_burn_point_double_t pressure_points[] =
    {
        { .pressure_pa = DBL_TRUE_MIN, .burn_rate_m_per_s = DBL_TRUE_MIN },
        { .pressure_pa = 1.0,          .burn_rate_m_per_s = 1.0 }
    };

    const bbtc_ib_pressure_burn_table_double_t pressure_model =
    {
        .points = pressure_points,
        .point_count = 2U
    };

    const bbtc_ib_pressure_burn_point_double_t rate_points[] =
    {
        { .pressure_pa = 1.0, .burn_rate_m_per_s = DBL_TRUE_MIN },
        { .pressure_pa = 4.0, .burn_rate_m_per_s = DBL_MAX }
    };

    const bbtc_ib_pressure_burn_table_double_t rate_model =
    {
        .points = rate_points,
        .point_count = 2U
    };

    bbtc_ib_propellant_burn_kinetics_result_double_t result = {0};

    CHECK(
        bbtc_ib_pressure_burn_table_evaluate_double(
            &pressure_model,
            DBL_MIN,
            &result
        ) == BBTC_STATUS_SUCCESS
    );
    CHECK(isfinite(result.burn_rate_m_per_s));
    CHECK(result.burn_rate_m_per_s > 0.0);
    CHECK(result.burn_rate_m_per_s <= 1.0);

    CHECK(
        bbtc_ib_pressure_burn_table_evaluate_double(
            &rate_model,
            2.0,
            &result
        ) == BBTC_STATUS_SUCCESS
    );
    CHECK(isfinite(result.burn_rate_m_per_s));
    CHECK(result.burn_rate_m_per_s > 0.0);

    return EXIT_SUCCESS;
}


/**
 * @brief Verifies that validation and evaluation leave caller tables unchanged.
 *
 * @return `EXIT_SUCCESS` on success; otherwise `EXIT_FAILURE`.
 */
static int
test_caller_storage_immutability(void)
{
    const bbtc_ib_pressure_burn_point_double_t points[] =
    {
        { .pressure_pa = 1.0, .burn_rate_m_per_s = 2.0 },
        { .pressure_pa = 4.0, .burn_rate_m_per_s = 8.0 }
    };

    const bbtc_ib_pressure_burn_table_double_t model =
    {
        .points = points,
        .point_count = 2U
    };

    bbtc_ib_propellant_burn_kinetics_result_double_t result = {0};

    CHECK(
        bbtc_ib_pressure_burn_table_validate_double(&model)
            == BBTC_STATUS_SUCCESS
    );

    CHECK(
        bbtc_ib_pressure_burn_table_evaluate_double(
            &model,
            2.0,
            &result
        ) == BBTC_STATUS_SUCCESS
    );

    CHECK(model.points == points);
    CHECK(model.point_count == 2U);
    CHECK(points[0].pressure_pa == 1.0);
    CHECK(points[0].burn_rate_m_per_s == 2.0);
    CHECK(points[1].pressure_pa == 4.0);
    CHECK(points[1].burn_rate_m_per_s == 8.0);

    return EXIT_SUCCESS;
}


/**
 * @brief Runs the tabulated pressure-burn kinetics test suite.
 *
 * @return `EXIT_SUCCESS` when every test passes; otherwise `EXIT_FAILURE`.
 */
int
main(void)
{
    if (test_validation_all_precisions() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_structural_validation() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_nonfinite_precedence() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_finite_domain_and_ordering() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_exact_knots_all_precisions() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_log_log_interpolation_all_precisions() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_evaluator_failure_semantics() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_nonmonotonic_rate_evaluation() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_flat_segment_exactness() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_extreme_scale_interpolation() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_caller_storage_immutability() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    return EXIT_SUCCESS;
}
