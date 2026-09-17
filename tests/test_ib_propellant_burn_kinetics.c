/**
 * @file
 * @brief Tests normalized pressure-power propellant burn kinetics.
 *
 * @details
 * These tests cover the first concrete IB0.4d burn-kinetics backend across all
 * three native scalar families. They verify model validation, caller-nonfinite
 * precedence, exact mathematical boundaries, calibration applicability,
 * deterministic output clearing, and representability failures without using
 * any commercial propellant data or making a firing prediction.
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <float.h>
#include <math.h>

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
 * @brief Verifies valid model records in every native scalar family.
 *
 * @return `EXIT_SUCCESS` on success; otherwise `EXIT_FAILURE`.
 */
static int
test_validation_all_precisions(void)
{
    const bbtc_ib_pressure_power_burn_kinetics_float_t model_f =
    {
        .reference_burn_rate_m_per_s    = 0.01f,
        .reference_pressure_pa          = 100.0f,
        .pressure_exponent              = 0.8f,
        .minimum_calibrated_pressure_pa = 50.0f,
        .maximum_calibrated_pressure_pa = 200.0f
    };

    const bbtc_ib_pressure_power_burn_kinetics_double_t model_d =
    {
        .reference_burn_rate_m_per_s    = 0.01,
        .reference_pressure_pa          = 100.0,
        .pressure_exponent              = 0.8,
        .minimum_calibrated_pressure_pa = 50.0,
        .maximum_calibrated_pressure_pa = 200.0
    };

    const bbtc_ib_pressure_power_burn_kinetics_long_double_t model_l =
    {
        .reference_burn_rate_m_per_s    = 0.01L,
        .reference_pressure_pa          = 100.0L,
        .pressure_exponent              = 0.8L,
        .minimum_calibrated_pressure_pa = 50.0L,
        .maximum_calibrated_pressure_pa = 200.0L
    };

    CHECK(
        bbtc_ib_pressure_power_burn_kinetics_validate_float(&model_f)
            == BBTC_STATUS_SUCCESS
    );

    CHECK(
        bbtc_ib_pressure_power_burn_kinetics_validate_double(&model_d)
            == BBTC_STATUS_SUCCESS
    );

    CHECK(
        bbtc_ib_pressure_power_burn_kinetics_validate_long_double(&model_l)
            == BBTC_STATUS_SUCCESS
    );

    CHECK(
        bbtc_ib_pressure_power_burn_kinetics_validate_double(NULL)
            == BBTC_STATUS_INVALID_ARGUMENT
    );

    return EXIT_SUCCESS;
}


/**
 * @brief Verifies NaN/infinity precedence and finite model-domain rejection.
 *
 * @return `EXIT_SUCCESS` on success; otherwise `EXIT_FAILURE`.
 */
static int
test_validation_precedence_and_domain(void)
{
    const bbtc_ib_pressure_power_burn_kinetics_double_t valid =
    {
        .reference_burn_rate_m_per_s    = 0.01,
        .reference_pressure_pa          = 100.0,
        .pressure_exponent              = 0.8,
        .minimum_calibrated_pressure_pa = 50.0,
        .maximum_calibrated_pressure_pa = 200.0
    };

    bbtc_ib_pressure_power_burn_kinetics_double_t model = valid;

    model.reference_burn_rate_m_per_s = INFINITY;
    model.maximum_calibrated_pressure_pa = NAN;

    CHECK(
        bbtc_ib_pressure_power_burn_kinetics_validate_double(&model)
            == BBTC_STATUS_NAN_INPUT
    );

    model = valid;
    model.reference_burn_rate_m_per_s = NAN;
    model.pressure_exponent = -INFINITY;

    CHECK(
        bbtc_ib_pressure_power_burn_kinetics_validate_double(&model)
            == BBTC_STATUS_NAN_INPUT
    );

    model = valid;
    model.reference_pressure_pa = INFINITY;

    CHECK(
        bbtc_ib_pressure_power_burn_kinetics_validate_double(&model)
            == BBTC_STATUS_NONFINITE_INPUT
    );

    model = valid;
    model.reference_burn_rate_m_per_s = 0.0;

    CHECK(
        bbtc_ib_pressure_power_burn_kinetics_validate_double(&model)
            == BBTC_STATUS_OUTSIDE_DOMAIN
    );

    model = valid;
    model.reference_pressure_pa = 0.0;

    CHECK(
        bbtc_ib_pressure_power_burn_kinetics_validate_double(&model)
            == BBTC_STATUS_OUTSIDE_DOMAIN
    );

    model = valid;
    model.pressure_exponent = 0.0;

    CHECK(
        bbtc_ib_pressure_power_burn_kinetics_validate_double(&model)
            == BBTC_STATUS_OUTSIDE_DOMAIN
    );

    model = valid;
    model.pressure_exponent = -0.5;

    CHECK(
        bbtc_ib_pressure_power_burn_kinetics_validate_double(&model)
            == BBTC_STATUS_OUTSIDE_DOMAIN
    );

    model = valid;
    model.minimum_calibrated_pressure_pa = 0.0;

    CHECK(
        bbtc_ib_pressure_power_burn_kinetics_validate_double(&model)
            == BBTC_STATUS_OUTSIDE_DOMAIN
    );

    model = valid;
    model.maximum_calibrated_pressure_pa =
        model.minimum_calibrated_pressure_pa;

    CHECK(
        bbtc_ib_pressure_power_burn_kinetics_validate_double(&model)
            == BBTC_STATUS_OUTSIDE_DOMAIN
    );

    model = valid;
    model.reference_pressure_pa = 49.0;

    CHECK(
        bbtc_ib_pressure_power_burn_kinetics_validate_double(&model)
            == BBTC_STATUS_OUTSIDE_DOMAIN
    );

    model = valid;
    model.reference_pressure_pa = 201.0;

    CHECK(
        bbtc_ib_pressure_power_burn_kinetics_validate_double(&model)
            == BBTC_STATUS_OUTSIDE_DOMAIN
    );

    model = valid;
    model.reference_pressure_pa = model.minimum_calibrated_pressure_pa;

    CHECK(
        bbtc_ib_pressure_power_burn_kinetics_validate_double(&model)
            == BBTC_STATUS_SUCCESS
    );

    model = valid;
    model.reference_pressure_pa = model.maximum_calibrated_pressure_pa;

    CHECK(
        bbtc_ib_pressure_power_burn_kinetics_validate_double(&model)
            == BBTC_STATUS_SUCCESS
    );

    return EXIT_SUCCESS;
}


/**
 * @brief Verifies exact zero- and reference-pressure boundaries in every family.
 *
 * @return `EXIT_SUCCESS` on success; otherwise `EXIT_FAILURE`.
 */
static int
test_exact_boundaries_all_precisions(void)
{
    const bbtc_ib_pressure_power_burn_kinetics_float_t model_f =
    {
        .reference_burn_rate_m_per_s    = 0.25f,
        .reference_pressure_pa          = 100.0f,
        .pressure_exponent              = 1.0f,
        .minimum_calibrated_pressure_pa = 50.0f,
        .maximum_calibrated_pressure_pa = 200.0f
    };

    const bbtc_ib_pressure_power_burn_kinetics_double_t model_d =
    {
        .reference_burn_rate_m_per_s    = 0.25,
        .reference_pressure_pa          = 100.0,
        .pressure_exponent              = 1.0,
        .minimum_calibrated_pressure_pa = 50.0,
        .maximum_calibrated_pressure_pa = 200.0
    };

    const bbtc_ib_pressure_power_burn_kinetics_long_double_t model_l =
    {
        .reference_burn_rate_m_per_s    = 0.25L,
        .reference_pressure_pa          = 100.0L,
        .pressure_exponent              = 1.0L,
        .minimum_calibrated_pressure_pa = 50.0L,
        .maximum_calibrated_pressure_pa = 200.0L
    };

    bbtc_ib_propellant_burn_kinetics_result_float_t result_f = {0};
    bbtc_ib_propellant_burn_kinetics_result_double_t result_d = {0};
    bbtc_ib_propellant_burn_kinetics_result_long_double_t result_l = {0};

    CHECK(
        bbtc_ib_pressure_power_burn_kinetics_evaluate_float(
            &model_f,
            model_f.reference_pressure_pa,
            &result_f
        ) == BBTC_STATUS_SUCCESS
    );
    CHECK(result_f.burn_rate_m_per_s == model_f.reference_burn_rate_m_per_s);
    CHECK(result_f.applicability_flags == BBTC_APPLICABILITY_NONE_REPORTED);

    CHECK(
        bbtc_ib_pressure_power_burn_kinetics_evaluate_double(
            &model_d,
            model_d.reference_pressure_pa,
            &result_d
        ) == BBTC_STATUS_SUCCESS
    );
    CHECK(result_d.burn_rate_m_per_s == model_d.reference_burn_rate_m_per_s);
    CHECK(result_d.applicability_flags == BBTC_APPLICABILITY_NONE_REPORTED);

    CHECK(
        bbtc_ib_pressure_power_burn_kinetics_evaluate_long_double(
            &model_l,
            model_l.reference_pressure_pa,
            &result_l
        ) == BBTC_STATUS_SUCCESS
    );
    CHECK(result_l.burn_rate_m_per_s == model_l.reference_burn_rate_m_per_s);
    CHECK(result_l.applicability_flags == BBTC_APPLICABILITY_NONE_REPORTED);

    result_f.burn_rate_m_per_s = 9.0f;
    result_d.burn_rate_m_per_s = 9.0;
    result_l.burn_rate_m_per_s = 9.0L;

    CHECK(
        bbtc_ib_pressure_power_burn_kinetics_evaluate_float(
            &model_f,
            0.0f,
            &result_f
        ) == BBTC_STATUS_SUCCESS
    );
    CHECK(result_f.burn_rate_m_per_s == 0.0f);
    CHECK(
        result_f.applicability_flags
            == BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN
    );

    CHECK(
        bbtc_ib_pressure_power_burn_kinetics_evaluate_double(
            &model_d,
            -0.0,
            &result_d
        ) == BBTC_STATUS_SUCCESS
    );
    CHECK(result_d.burn_rate_m_per_s == 0.0);
    CHECK(
        result_d.applicability_flags
            == BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN
    );

    CHECK(
        bbtc_ib_pressure_power_burn_kinetics_evaluate_long_double(
            &model_l,
            0.0L,
            &result_l
        ) == BBTC_STATUS_SUCCESS
    );
    CHECK(result_l.burn_rate_m_per_s == 0.0L);
    CHECK(
        result_l.applicability_flags
            == BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN
    );

    return EXIT_SUCCESS;
}


/**
 * @brief Verifies calibration flags and representative power-law values.
 *
 * @return `EXIT_SUCCESS` on success; otherwise `EXIT_FAILURE`.
 */
static int
test_calibration_and_relation(void)
{
    const bbtc_ib_pressure_power_burn_kinetics_double_t model =
    {
        .reference_burn_rate_m_per_s    = 2.0,
        .reference_pressure_pa          = 100.0,
        .pressure_exponent              = 1.0,
        .minimum_calibrated_pressure_pa = 50.0,
        .maximum_calibrated_pressure_pa = 200.0
    };

    bbtc_ib_propellant_burn_kinetics_result_double_t result = {0};

    CHECK(
        bbtc_ib_pressure_power_burn_kinetics_evaluate_double(
            &model,
            50.0,
            &result
        ) == BBTC_STATUS_SUCCESS
    );
    CHECK(result.applicability_flags == BBTC_APPLICABILITY_NONE_REPORTED);
    CHECK(fabs(result.burn_rate_m_per_s - 1.0) <= 16.0 * DBL_EPSILON);

    CHECK(
        bbtc_ib_pressure_power_burn_kinetics_evaluate_double(
            &model,
            200.0,
            &result
        ) == BBTC_STATUS_SUCCESS
    );
    CHECK(result.applicability_flags == BBTC_APPLICABILITY_NONE_REPORTED);
    CHECK(fabs(result.burn_rate_m_per_s - 4.0) <= 64.0 * DBL_EPSILON);

    CHECK(
        bbtc_ib_pressure_power_burn_kinetics_evaluate_double(
            &model,
            25.0,
            &result
        ) == BBTC_STATUS_SUCCESS
    );
    CHECK(
        result.applicability_flags
            == BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN
    );
    CHECK(fabs(result.burn_rate_m_per_s - 0.5) <= 16.0 * DBL_EPSILON);

    CHECK(
        bbtc_ib_pressure_power_burn_kinetics_evaluate_double(
            &model,
            400.0,
            &result
        ) == BBTC_STATUS_SUCCESS
    );
    CHECK(
        result.applicability_flags
            == BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN
    );
    CHECK(fabs(result.burn_rate_m_per_s - 8.0) <= 128.0 * DBL_EPSILON);

    return EXIT_SUCCESS;
}


/**
 * @brief Verifies evaluator failure ordering and deterministic output clearing.
 *
 * @return `EXIT_SUCCESS` on success; otherwise `EXIT_FAILURE`.
 */
static int
test_evaluator_failure_semantics(void)
{
    const bbtc_ib_pressure_power_burn_kinetics_double_t valid =
    {
        .reference_burn_rate_m_per_s    = 0.01,
        .reference_pressure_pa          = 100.0,
        .pressure_exponent              = 0.8,
        .minimum_calibrated_pressure_pa = 50.0,
        .maximum_calibrated_pressure_pa = 200.0
    };

    bbtc_ib_pressure_power_burn_kinetics_double_t invalid = valid;
    bbtc_ib_propellant_burn_kinetics_result_double_t result =
    {
        .applicability_flags = UINT64_MAX,
        .burn_rate_m_per_s = 9.0
    };

    CHECK(
        bbtc_ib_pressure_power_burn_kinetics_evaluate_double(
            &valid,
            100.0,
            NULL
        ) == BBTC_STATUS_INVALID_ARGUMENT
    );

    CHECK(
        bbtc_ib_pressure_power_burn_kinetics_evaluate_double(
            NULL,
            100.0,
            &result
        ) == BBTC_STATUS_INVALID_ARGUMENT
    );
    CHECK(result.applicability_flags == BBTC_APPLICABILITY_NONE_REPORTED);
    CHECK(result.burn_rate_m_per_s == 0.0);

    /* Model validation must precede classification of the direct pressure. */
    invalid.reference_burn_rate_m_per_s = 0.0;
    result.applicability_flags = UINT64_MAX;
    result.burn_rate_m_per_s = 9.0;

    CHECK(
        bbtc_ib_pressure_power_burn_kinetics_evaluate_double(
            &invalid,
            NAN,
            &result
        ) == BBTC_STATUS_OUTSIDE_DOMAIN
    );
    CHECK(result.applicability_flags == BBTC_APPLICABILITY_NONE_REPORTED);
    CHECK(result.burn_rate_m_per_s == 0.0);

    result.applicability_flags = UINT64_MAX;
    result.burn_rate_m_per_s = 9.0;

    CHECK(
        bbtc_ib_pressure_power_burn_kinetics_evaluate_double(
            &valid,
            NAN,
            &result
        ) == BBTC_STATUS_NAN_INPUT
    );
    CHECK(result.applicability_flags == BBTC_APPLICABILITY_NONE_REPORTED);
    CHECK(result.burn_rate_m_per_s == 0.0);

    result.applicability_flags = UINT64_MAX;
    result.burn_rate_m_per_s = 9.0;

    CHECK(
        bbtc_ib_pressure_power_burn_kinetics_evaluate_double(
            &valid,
            INFINITY,
            &result
        ) == BBTC_STATUS_NONFINITE_INPUT
    );
    CHECK(result.applicability_flags == BBTC_APPLICABILITY_NONE_REPORTED);
    CHECK(result.burn_rate_m_per_s == 0.0);

    result.applicability_flags = UINT64_MAX;
    result.burn_rate_m_per_s = 9.0;

    CHECK(
        bbtc_ib_pressure_power_burn_kinetics_evaluate_double(
            &valid,
            -1.0,
            &result
        ) == BBTC_STATUS_OUTSIDE_DOMAIN
    );
    CHECK(result.applicability_flags == BBTC_APPLICABILITY_NONE_REPORTED);
    CHECK(result.burn_rate_m_per_s == 0.0);

    return EXIT_SUCCESS;
}


/**
 * @brief Verifies positive representable reference rates near each scalar floor.
 *
 * @details
 * Exact evaluation at the reference pressure must preserve a caller-supplied
 * positive representable reference burn rate rather than mistaking a subnormal
 * value for physical zero or a malformed model.
 *
 * @return `EXIT_SUCCESS` on success; otherwise `EXIT_FAILURE`.
 */
static int
test_positive_representable_floors(void)
{
    const bbtc_ib_pressure_power_burn_kinetics_float_t model_f =
    {
        .reference_burn_rate_m_per_s    = FLT_TRUE_MIN,
        .reference_pressure_pa          = 1.0f,
        .pressure_exponent              = 1.0f,
        .minimum_calibrated_pressure_pa = 1.0f,
        .maximum_calibrated_pressure_pa = 2.0f
    };

    const bbtc_ib_pressure_power_burn_kinetics_double_t model_d =
    {
        .reference_burn_rate_m_per_s    = DBL_TRUE_MIN,
        .reference_pressure_pa          = 1.0,
        .pressure_exponent              = 1.0,
        .minimum_calibrated_pressure_pa = 1.0,
        .maximum_calibrated_pressure_pa = 2.0
    };

    const bbtc_ib_pressure_power_burn_kinetics_long_double_t model_l =
    {
        .reference_burn_rate_m_per_s    = LDBL_TRUE_MIN,
        .reference_pressure_pa          = 1.0L,
        .pressure_exponent              = 1.0L,
        .minimum_calibrated_pressure_pa = 1.0L,
        .maximum_calibrated_pressure_pa = 2.0L
    };

    bbtc_ib_propellant_burn_kinetics_result_float_t result_f = {0};
    bbtc_ib_propellant_burn_kinetics_result_double_t result_d = {0};
    bbtc_ib_propellant_burn_kinetics_result_long_double_t result_l = {0};

    CHECK(
        bbtc_ib_pressure_power_burn_kinetics_evaluate_float(
            &model_f,
            1.0f,
            &result_f
        ) == BBTC_STATUS_SUCCESS
    );
    CHECK(result_f.burn_rate_m_per_s == FLT_TRUE_MIN);

    CHECK(
        bbtc_ib_pressure_power_burn_kinetics_evaluate_double(
            &model_d,
            1.0,
            &result_d
        ) == BBTC_STATUS_SUCCESS
    );
    CHECK(result_d.burn_rate_m_per_s == DBL_TRUE_MIN);

    CHECK(
        bbtc_ib_pressure_power_burn_kinetics_evaluate_long_double(
            &model_l,
            1.0L,
            &result_l
        ) == BBTC_STATUS_SUCCESS
    );
    CHECK(result_l.burn_rate_m_per_s == LDBL_TRUE_MIN);

    return EXIT_SUCCESS;
}


/**
 * @brief Verifies overflow and underflow of required positive results.
 *
 * @return `EXIT_SUCCESS` on success; otherwise `EXIT_FAILURE`.
 */
static int
test_numerical_failure(void)
{
    const bbtc_ib_pressure_power_burn_kinetics_double_t overflow_model =
    {
        .reference_burn_rate_m_per_s    = DBL_MAX,
        .reference_pressure_pa          = 1.0,
        .pressure_exponent              = 1.0,
        .minimum_calibrated_pressure_pa = 1.0,
        .maximum_calibrated_pressure_pa = 2.0
    };

    const bbtc_ib_pressure_power_burn_kinetics_double_t underflow_model =
    {
        .reference_burn_rate_m_per_s    = DBL_TRUE_MIN,
        .reference_pressure_pa          = 1.0,
        .pressure_exponent              = 1.0,
        .minimum_calibrated_pressure_pa = 0.25,
        .maximum_calibrated_pressure_pa = 2.0
    };

    bbtc_ib_propellant_burn_kinetics_result_double_t result =
    {
        .applicability_flags = UINT64_MAX,
        .burn_rate_m_per_s = 9.0
    };

    CHECK(
        bbtc_ib_pressure_power_burn_kinetics_evaluate_double(
            &overflow_model,
            2.0,
            &result
        ) == BBTC_STATUS_NUMERICAL_FAILURE
    );
    CHECK(result.applicability_flags == BBTC_APPLICABILITY_NONE_REPORTED);
    CHECK(result.burn_rate_m_per_s == 0.0);

    result.applicability_flags = UINT64_MAX;
    result.burn_rate_m_per_s = 9.0;

    CHECK(
        bbtc_ib_pressure_power_burn_kinetics_evaluate_double(
            &underflow_model,
            0.25,
            &result
        ) == BBTC_STATUS_NUMERICAL_FAILURE
    );
    CHECK(result.applicability_flags == BBTC_APPLICABILITY_NONE_REPORTED);
    CHECK(result.burn_rate_m_per_s == 0.0);

    return EXIT_SUCCESS;
}


/**
 * @brief Runs the pressure-power burn-kinetics test suite.
 *
 * @return `EXIT_SUCCESS` when every test passes; otherwise `EXIT_FAILURE`.
 */
int
main(void)
{
    if (test_validation_all_precisions() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_validation_precedence_and_domain() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_exact_boundaries_all_precisions() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_calibration_and_relation() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_evaluator_failure_semantics() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_positive_representable_floors() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_numerical_failure() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    return EXIT_SUCCESS;
}
