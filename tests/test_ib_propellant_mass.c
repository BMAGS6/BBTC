/**
 * @file
 * @brief Tests whole-charge propellant regression-to-mass coupling.
 *
 * @details
 * These tests exercise the first concrete IB0.4e coupling primitive across all
 * three native scalar families. They deliberately separate software-contract
 * behavior from physical-model uncertainty: exact endpoints, validation
 * precedence, output clearing, applicability propagation, and representability
 * failures are tested exactly, while ordinary interior arithmetic uses values
 * chosen to be exactly representable in binary.
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
static int fail(const char* const expression, const int line)
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
 * @brief Verifies one exactly representable interior state in every precision.
 */
static int test_interior_all_precisions(void)
{
    const bbtc_ib_propellant_charge_float_t charge_f =
    {
        .charge_mass_kg = 8.0f,
        .condensed_phase_density_kg_per_m3 = 2.0f
    };
    const bbtc_ib_propellant_grain_state_float_t grain_f =
    {
        .remaining_volume_m3 = 1.0f,
        .burning_surface_area_m2 = 3.0f,
        .remaining_regression_to_burnout_m = 0.5f,
        .consumed_volume_fraction = 0.5f
    };
    const bbtc_ib_propellant_burn_kinetics_result_float_t kinetics_f =
    {
        .applicability_flags = BBTC_APPLICABILITY_NONE_REPORTED,
        .burn_rate_m_per_s = 0.25f
    };
    bbtc_ib_propellant_mass_result_float_t result_f = {0};

    const bbtc_ib_propellant_charge_double_t charge_d =
    {
        .charge_mass_kg = 8.0,
        .condensed_phase_density_kg_per_m3 = 2.0
    };
    const bbtc_ib_propellant_grain_state_double_t grain_d =
    {
        .remaining_volume_m3 = 1.0,
        .burning_surface_area_m2 = 3.0,
        .remaining_regression_to_burnout_m = 0.5,
        .consumed_volume_fraction = 0.5
    };
    const bbtc_ib_propellant_burn_kinetics_result_double_t kinetics_d =
    {
        .applicability_flags = BBTC_APPLICABILITY_NONE_REPORTED,
        .burn_rate_m_per_s = 0.25
    };
    bbtc_ib_propellant_mass_result_double_t result_d = {0};

    const bbtc_ib_propellant_charge_long_double_t charge_l =
    {
        .charge_mass_kg = 8.0L,
        .condensed_phase_density_kg_per_m3 = 2.0L
    };
    const bbtc_ib_propellant_grain_state_long_double_t grain_l =
    {
        .remaining_volume_m3 = 1.0L,
        .burning_surface_area_m2 = 3.0L,
        .remaining_regression_to_burnout_m = 0.5L,
        .consumed_volume_fraction = 0.5L
    };
    const bbtc_ib_propellant_burn_kinetics_result_long_double_t kinetics_l =
    {
        .applicability_flags = BBTC_APPLICABILITY_NONE_REPORTED,
        .burn_rate_m_per_s = 0.25L
    };
    bbtc_ib_propellant_mass_result_long_double_t result_l = {0};

    CHECK(
        bbtc_ib_propellant_mass_evaluate_float(
            &charge_f,
            2.0f,
            &grain_f,
            &kinetics_f,
            &result_f
        ) == BBTC_STATUS_SUCCESS
    );
    CHECK(result_f.applicability_flags == BBTC_APPLICABILITY_NONE_REPORTED);
    CHECK(result_f.equivalent_population_scale == 2.0f);
    CHECK(result_f.remaining_volume_m3 == 2.0f);
    CHECK(result_f.burning_surface_area_m2 == 6.0f);
    CHECK(result_f.remaining_mass_kg == 4.0f);
    CHECK(result_f.reacted_mass_kg == 4.0f);
    CHECK(result_f.reacted_mass_rate_kg_per_s == 3.0f);

    CHECK(
        bbtc_ib_propellant_mass_evaluate_double(
            &charge_d,
            2.0,
            &grain_d,
            &kinetics_d,
            &result_d
        ) == BBTC_STATUS_SUCCESS
    );
    CHECK(result_d.applicability_flags == BBTC_APPLICABILITY_NONE_REPORTED);
    CHECK(result_d.equivalent_population_scale == 2.0);
    CHECK(result_d.remaining_volume_m3 == 2.0);
    CHECK(result_d.burning_surface_area_m2 == 6.0);
    CHECK(result_d.remaining_mass_kg == 4.0);
    CHECK(result_d.reacted_mass_kg == 4.0);
    CHECK(result_d.reacted_mass_rate_kg_per_s == 3.0);

    CHECK(
        bbtc_ib_propellant_mass_evaluate_long_double(
            &charge_l,
            2.0L,
            &grain_l,
            &kinetics_l,
            &result_l
        ) == BBTC_STATUS_SUCCESS
    );
    CHECK(result_l.applicability_flags == BBTC_APPLICABILITY_NONE_REPORTED);
    CHECK(result_l.equivalent_population_scale == 2.0L);
    CHECK(result_l.remaining_volume_m3 == 2.0L);
    CHECK(result_l.burning_surface_area_m2 == 6.0L);
    CHECK(result_l.remaining_mass_kg == 4.0L);
    CHECK(result_l.reacted_mass_kg == 4.0L);
    CHECK(result_l.reacted_mass_rate_kg_per_s == 3.0L);

    return EXIT_SUCCESS;
}


/**
 * @brief Verifies exact initial, zero-rate, and burnout semantics.
 */
static int test_exact_boundaries(void)
{
    const bbtc_ib_propellant_charge_double_t charge =
    {
        .charge_mass_kg = 8.0,
        .condensed_phase_density_kg_per_m3 = 2.0
    };
    const bbtc_ib_propellant_grain_state_double_t initial =
    {
        .remaining_volume_m3 = 2.0,
        .burning_surface_area_m2 = 3.0,
        .remaining_regression_to_burnout_m = 1.0,
        .consumed_volume_fraction = 0.0
    };
    const bbtc_ib_propellant_grain_state_double_t interior =
    {
        .remaining_volume_m3 = 1.0,
        .burning_surface_area_m2 = 3.0,
        .remaining_regression_to_burnout_m = 0.5,
        .consumed_volume_fraction = 0.5
    };
    const bbtc_ib_propellant_grain_state_double_t burnout =
    {
        .remaining_volume_m3 = 0.0,
        .burning_surface_area_m2 = 0.0,
        .remaining_regression_to_burnout_m = 0.0,
        .consumed_volume_fraction = 1.0
    };
    const bbtc_ib_propellant_burn_kinetics_result_double_t positive_rate =
    {
        .applicability_flags = BBTC_APPLICABILITY_NONE_REPORTED,
        .burn_rate_m_per_s = 0.25
    };
    const bbtc_ib_propellant_burn_kinetics_result_double_t zero_rate =
    {
        .applicability_flags = BBTC_APPLICABILITY_NONE_REPORTED,
        .burn_rate_m_per_s = 0.0
    };
    bbtc_ib_propellant_mass_result_double_t result = {0};

    CHECK(
        bbtc_ib_propellant_mass_evaluate_double(
            &charge,
            2.0,
            &initial,
            &positive_rate,
            &result
        ) == BBTC_STATUS_SUCCESS
    );
    CHECK(result.equivalent_population_scale == 2.0);
    CHECK(result.remaining_volume_m3 == 4.0);
    CHECK(result.burning_surface_area_m2 == 6.0);
    CHECK(result.remaining_mass_kg == 8.0);
    CHECK(result.reacted_mass_kg == 0.0);
    CHECK(result.reacted_mass_rate_kg_per_s == 3.0);

    CHECK(
        bbtc_ib_propellant_mass_evaluate_double(
            &charge,
            2.0,
            &interior,
            &zero_rate,
            &result
        ) == BBTC_STATUS_SUCCESS
    );
    CHECK(result.reacted_mass_rate_kg_per_s == 0.0);

    CHECK(
        bbtc_ib_propellant_mass_evaluate_double(
            &charge,
            2.0,
            &burnout,
            &positive_rate,
            &result
        ) == BBTC_STATUS_SUCCESS
    );
    CHECK(result.equivalent_population_scale == 2.0);
    CHECK(result.remaining_volume_m3 == 0.0);
    CHECK(result.burning_surface_area_m2 == 0.0);
    CHECK(result.remaining_mass_kg == 0.0);
    CHECK(result.reacted_mass_kg == 8.0);
    CHECK(result.reacted_mass_rate_kg_per_s == 0.0);

    return EXIT_SUCCESS;
}


/**
 * @brief Verifies that the equivalent population is a real scaling factor.
 */
static int test_fractional_population(void)
{
    const bbtc_ib_propellant_charge_double_t charge =
    {
        .charge_mass_kg = 1.0,
        .condensed_phase_density_kg_per_m3 = 2.0
    };
    const bbtc_ib_propellant_grain_state_double_t grain =
    {
        .remaining_volume_m3 = 1.0,
        .burning_surface_area_m2 = 4.0,
        .remaining_regression_to_burnout_m = 0.5,
        .consumed_volume_fraction = 0.5
    };
    const bbtc_ib_propellant_burn_kinetics_result_double_t kinetics =
    {
        .applicability_flags = BBTC_APPLICABILITY_NONE_REPORTED,
        .burn_rate_m_per_s = 0.5
    };
    bbtc_ib_propellant_mass_result_double_t result = {0};

    CHECK(
        bbtc_ib_propellant_mass_evaluate_double(
            &charge,
            2.0,
            &grain,
            &kinetics,
            &result
        ) == BBTC_STATUS_SUCCESS
    );
    CHECK(result.equivalent_population_scale == 0.25);
    CHECK(result.remaining_volume_m3 == 0.25);
    CHECK(result.burning_surface_area_m2 == 1.0);
    CHECK(result.remaining_mass_kg == 0.5);
    CHECK(result.reacted_mass_kg == 0.5);
    CHECK(result.reacted_mass_rate_kg_per_s == 1.0);

    return EXIT_SUCCESS;
}


/**
 * @brief Verifies structural argument handling and deterministic result clearing.
 */
static int test_arguments_and_output_clearing(void)
{
    const bbtc_ib_propellant_charge_double_t charge =
    {
        .charge_mass_kg = 8.0,
        .condensed_phase_density_kg_per_m3 = 2.0
    };
    const bbtc_ib_propellant_grain_state_double_t grain =
    {
        .remaining_volume_m3 = 1.0,
        .burning_surface_area_m2 = 3.0,
        .remaining_regression_to_burnout_m = 0.5,
        .consumed_volume_fraction = 0.5
    };
    const bbtc_ib_propellant_burn_kinetics_result_double_t kinetics =
    {
        .applicability_flags = BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN,
        .burn_rate_m_per_s = 0.25
    };
    bbtc_ib_propellant_mass_result_double_t result =
    {
        .applicability_flags = UINT64_MAX,
        .equivalent_population_scale = 1.0,
        .remaining_volume_m3 = 1.0,
        .burning_surface_area_m2 = 1.0,
        .remaining_mass_kg = 1.0,
        .reacted_mass_kg = 1.0,
        .reacted_mass_rate_kg_per_s = 1.0
    };

    CHECK(
        bbtc_ib_propellant_mass_evaluate_double(
            &charge,
            2.0,
            &grain,
            &kinetics,
            NULL
        ) == BBTC_STATUS_INVALID_ARGUMENT
    );

    CHECK(
        bbtc_ib_propellant_mass_evaluate_double(
            NULL,
            2.0,
            &grain,
            &kinetics,
            &result
        ) == BBTC_STATUS_INVALID_ARGUMENT
    );
    CHECK(result.applicability_flags == 0U);
    CHECK(result.equivalent_population_scale == 0.0);
    CHECK(result.remaining_volume_m3 == 0.0);
    CHECK(result.burning_surface_area_m2 == 0.0);
    CHECK(result.remaining_mass_kg == 0.0);
    CHECK(result.reacted_mass_kg == 0.0);
    CHECK(result.reacted_mass_rate_kg_per_s == 0.0);

    CHECK(
        bbtc_ib_propellant_mass_evaluate_double(
            &charge,
            2.0,
            NULL,
            &kinetics,
            &result
        ) == BBTC_STATUS_INVALID_ARGUMENT
    );

    CHECK(
        bbtc_ib_propellant_mass_evaluate_double(
            &charge,
            2.0,
            &grain,
            NULL,
            &result
        ) == BBTC_STATUS_INVALID_ARGUMENT
    );

    return EXIT_SUCCESS;
}


/**
 * @brief Verifies explicit initial-grain-volume scalar classification.
 */
static int test_initial_volume_validation(void)
{
    const bbtc_ib_propellant_charge_double_t charge =
    {
        .charge_mass_kg = 8.0,
        .condensed_phase_density_kg_per_m3 = 2.0
    };
    const bbtc_ib_propellant_grain_state_double_t grain =
    {
        .remaining_volume_m3 = 1.0,
        .burning_surface_area_m2 = 3.0,
        .remaining_regression_to_burnout_m = 0.5,
        .consumed_volume_fraction = 0.5
    };
    const bbtc_ib_propellant_burn_kinetics_result_double_t kinetics =
    {
        .applicability_flags = 0U,
        .burn_rate_m_per_s = 0.25
    };
    bbtc_ib_propellant_mass_result_double_t result = {0};

    /* Charge validation precedes classification of the direct volume scalar. */
    {
        bbtc_ib_propellant_charge_double_t invalid_charge = charge;
        invalid_charge.charge_mass_kg = 0.0;

        CHECK(
            bbtc_ib_propellant_mass_evaluate_double(
                &invalid_charge,
                NAN,
                &grain,
                &kinetics,
                &result
            ) == BBTC_STATUS_OUTSIDE_DOMAIN
        );
    }

    CHECK(
        bbtc_ib_propellant_mass_evaluate_double(
            &charge,
            NAN,
            &grain,
            &kinetics,
            &result
        ) == BBTC_STATUS_NAN_INPUT
    );
    CHECK(
        bbtc_ib_propellant_mass_evaluate_double(
            &charge,
            INFINITY,
            &grain,
            &kinetics,
            &result
        ) == BBTC_STATUS_NONFINITE_INPUT
    );
    CHECK(
        bbtc_ib_propellant_mass_evaluate_double(
            &charge,
            0.0,
            &grain,
            &kinetics,
            &result
        ) == BBTC_STATUS_OUTSIDE_DOMAIN
    );
    CHECK(
        bbtc_ib_propellant_mass_evaluate_double(
            &charge,
            -1.0,
            &grain,
            &kinetics,
            &result
        ) == BBTC_STATUS_OUTSIDE_DOMAIN
    );

    return EXIT_SUCCESS;
}


/**
 * @brief Verifies whole-record grain-state nonfinite precedence and domain checks.
 */
static int test_grain_state_validation(void)
{
    const bbtc_ib_propellant_charge_double_t charge =
    {
        .charge_mass_kg = 8.0,
        .condensed_phase_density_kg_per_m3 = 2.0
    };
    const bbtc_ib_propellant_grain_state_double_t valid =
    {
        .remaining_volume_m3 = 1.0,
        .burning_surface_area_m2 = 3.0,
        .remaining_regression_to_burnout_m = 0.5,
        .consumed_volume_fraction = 0.5
    };
    const bbtc_ib_propellant_burn_kinetics_result_double_t kinetics =
    {
        .applicability_flags = 0U,
        .burn_rate_m_per_s = 0.25
    };
    bbtc_ib_propellant_grain_state_double_t grain = valid;
    bbtc_ib_propellant_mass_result_double_t result = {0};

    grain.remaining_volume_m3 = INFINITY;
    grain.burning_surface_area_m2 = NAN;
    CHECK(
        bbtc_ib_propellant_mass_evaluate_double(
            &charge,
            2.0,
            &grain,
            &kinetics,
            &result
        ) == BBTC_STATUS_NAN_INPUT
    );

    grain = valid;
    grain.remaining_regression_to_burnout_m = -INFINITY;
    CHECK(
        bbtc_ib_propellant_mass_evaluate_double(
            &charge,
            2.0,
            &grain,
            &kinetics,
            &result
        ) == BBTC_STATUS_NONFINITE_INPUT
    );

    grain = valid;
    grain.remaining_volume_m3 = -1.0;
    CHECK(
        bbtc_ib_propellant_mass_evaluate_double(
            &charge,
            2.0,
            &grain,
            &kinetics,
            &result
        ) == BBTC_STATUS_OUTSIDE_DOMAIN
    );

    grain = valid;
    grain.consumed_volume_fraction = 1.25;
    CHECK(
        bbtc_ib_propellant_mass_evaluate_double(
            &charge,
            2.0,
            &grain,
            &kinetics,
            &result
        ) == BBTC_STATUS_OUTSIDE_DOMAIN
    );

    return EXIT_SUCCESS;
}


/**
 * @brief Verifies the broad cross-record invariants frozen by contract 0.1.20.
 */
static int test_grain_state_consistency(void)
{
    const bbtc_ib_propellant_charge_double_t charge =
    {
        .charge_mass_kg = 8.0,
        .condensed_phase_density_kg_per_m3 = 2.0
    };
    const bbtc_ib_propellant_burn_kinetics_result_double_t kinetics =
    {
        .applicability_flags = 0U,
        .burn_rate_m_per_s = 0.25
    };
    bbtc_ib_propellant_grain_state_double_t grain =
    {
        .remaining_volume_m3 = 1.0,
        .burning_surface_area_m2 = 3.0,
        .remaining_regression_to_burnout_m = 0.5,
        .consumed_volume_fraction = 0.5
    };
    bbtc_ib_propellant_mass_result_double_t result = {0};

    grain.remaining_volume_m3 = 3.0;
    CHECK(
        bbtc_ib_propellant_mass_evaluate_double(
            &charge, 2.0, &grain, &kinetics, &result
        ) == BBTC_STATUS_INCONSISTENT_CONFIGURATION
    );

    grain = (bbtc_ib_propellant_grain_state_double_t)
    {
        .remaining_volume_m3 = 1.0,
        .burning_surface_area_m2 = 3.0,
        .remaining_regression_to_burnout_m = 1.0,
        .consumed_volume_fraction = 0.0
    };
    CHECK(
        bbtc_ib_propellant_mass_evaluate_double(
            &charge, 2.0, &grain, &kinetics, &result
        ) == BBTC_STATUS_INCONSISTENT_CONFIGURATION
    );

    grain = (bbtc_ib_propellant_grain_state_double_t)
    {
        .remaining_volume_m3 = 0.0,
        .burning_surface_area_m2 = 1.0,
        .remaining_regression_to_burnout_m = 0.0,
        .consumed_volume_fraction = 1.0
    };
    CHECK(
        bbtc_ib_propellant_mass_evaluate_double(
            &charge, 2.0, &grain, &kinetics, &result
        ) == BBTC_STATUS_INCONSISTENT_CONFIGURATION
    );

    grain = (bbtc_ib_propellant_grain_state_double_t)
    {
        .remaining_volume_m3 = 1.0,
        .burning_surface_area_m2 = 0.0,
        .remaining_regression_to_burnout_m = 0.5,
        .consumed_volume_fraction = 0.5
    };
    CHECK(
        bbtc_ib_propellant_mass_evaluate_double(
            &charge, 2.0, &grain, &kinetics, &result
        ) == BBTC_STATUS_INCONSISTENT_CONFIGURATION
    );

    return EXIT_SUCCESS;
}


/**
 * @brief Verifies burn-rate scalar classification after grain validation.
 */
static int test_kinetics_rate_validation(void)
{
    const bbtc_ib_propellant_charge_double_t charge =
    {
        .charge_mass_kg = 8.0,
        .condensed_phase_density_kg_per_m3 = 2.0
    };
    const bbtc_ib_propellant_grain_state_double_t grain =
    {
        .remaining_volume_m3 = 1.0,
        .burning_surface_area_m2 = 3.0,
        .remaining_regression_to_burnout_m = 0.5,
        .consumed_volume_fraction = 0.5
    };
    bbtc_ib_propellant_burn_kinetics_result_double_t kinetics =
    {
        .applicability_flags = 0U,
        .burn_rate_m_per_s = 0.25
    };
    bbtc_ib_propellant_mass_result_double_t result = {0};

    /* Grain-state failure precedes classification of the later kinetics rate. */
    {
        bbtc_ib_propellant_grain_state_double_t invalid_grain = grain;
        invalid_grain.remaining_volume_m3 = -1.0;
        kinetics.burn_rate_m_per_s = NAN;

        CHECK(
            bbtc_ib_propellant_mass_evaluate_double(
                &charge, 2.0, &invalid_grain, &kinetics, &result
            ) == BBTC_STATUS_OUTSIDE_DOMAIN
        );
    }

    kinetics.burn_rate_m_per_s = NAN;
    CHECK(
        bbtc_ib_propellant_mass_evaluate_double(
            &charge, 2.0, &grain, &kinetics, &result
        ) == BBTC_STATUS_NAN_INPUT
    );

    kinetics.burn_rate_m_per_s = INFINITY;
    CHECK(
        bbtc_ib_propellant_mass_evaluate_double(
            &charge, 2.0, &grain, &kinetics, &result
        ) == BBTC_STATUS_NONFINITE_INPUT
    );

    kinetics.burn_rate_m_per_s = -0.25;
    CHECK(
        bbtc_ib_propellant_mass_evaluate_double(
            &charge, 2.0, &grain, &kinetics, &result
        ) == BBTC_STATUS_OUTSIDE_DOMAIN
    );

    return EXIT_SUCCESS;
}


/**
 * @brief Verifies that all applicability bits survive successful coupling.
 */
static int test_applicability_propagation(void)
{
    const bbtc_ib_propellant_charge_double_t charge =
    {
        .charge_mass_kg = 8.0,
        .condensed_phase_density_kg_per_m3 = 2.0
    };
    const bbtc_ib_propellant_grain_state_double_t grain =
    {
        .remaining_volume_m3 = 1.0,
        .burning_surface_area_m2 = 3.0,
        .remaining_regression_to_burnout_m = 0.5,
        .consumed_volume_fraction = 0.5
    };
    const bbtc_applicability_flags_t unknown_bit = UINT64_C(1) << 63;
    const bbtc_applicability_flags_t expected =
        BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN | unknown_bit;
    const bbtc_ib_propellant_burn_kinetics_result_double_t kinetics =
    {
        .applicability_flags = expected,
        .burn_rate_m_per_s = 0.25
    };
    bbtc_ib_propellant_mass_result_double_t result = {0};

    CHECK(
        bbtc_ib_propellant_mass_evaluate_double(
            &charge, 2.0, &grain, &kinetics, &result
        ) == BBTC_STATUS_SUCCESS
    );
    CHECK(result.applicability_flags == expected);

    return EXIT_SUCCESS;
}


/**
 * @brief Verifies exponent-separated arithmetic avoids unnecessary range loss.
 */
static int test_extreme_scale_arithmetic(void)
{
    bbtc_ib_propellant_mass_result_double_t result = {0};

    /*
     * A naive `rho * V_g0` overflows here, even though the equivalent population
     * scale is exactly 0.5 and the complete initial state is representable.
     */
    {
        const bbtc_ib_propellant_charge_double_t charge =
        {
            .charge_mass_kg = DBL_MAX,
            .condensed_phase_density_kg_per_m3 = DBL_MAX
        };
        const bbtc_ib_propellant_grain_state_double_t grain =
        {
            .remaining_volume_m3 = 2.0,
            .burning_surface_area_m2 = 1.0,
            .remaining_regression_to_burnout_m = 1.0,
            .consumed_volume_fraction = 0.0
        };
        const bbtc_ib_propellant_burn_kinetics_result_double_t kinetics =
        {
            .applicability_flags = 0U,
            .burn_rate_m_per_s = 0.0
        };

        CHECK(
            bbtc_ib_propellant_mass_evaluate_double(
                &charge, 2.0, &grain, &kinetics, &result
            ) == BBTC_STATUS_SUCCESS
        );
        CHECK(result.equivalent_population_scale == 0.5);
        CHECK(result.remaining_volume_m3 == 1.0);
        CHECK(result.burning_surface_area_m2 == 0.5);
        CHECK(result.remaining_mass_kg == DBL_MAX);
        CHECK(result.reacted_mass_kg == 0.0);
        CHECK(result.reacted_mass_rate_kg_per_s == 0.0);
    }

    /*
     * A naive `m0 * V_g` overflows before division by `V_g0`. The scaled ratio
     * must instead recover the representable half-mass result.
     */
    {
        const bbtc_ib_propellant_charge_double_t charge =
        {
            .charge_mass_kg = DBL_MAX,
            .condensed_phase_density_kg_per_m3 = DBL_MAX
        };
        const bbtc_ib_propellant_grain_state_double_t grain =
        {
            .remaining_volume_m3 = 2.0,
            .burning_surface_area_m2 = 1.0,
            .remaining_regression_to_burnout_m = 1.0,
            .consumed_volume_fraction = 0.5
        };
        const bbtc_ib_propellant_burn_kinetics_result_double_t kinetics =
        {
            .applicability_flags = 0U,
            .burn_rate_m_per_s = 0.0
        };

        CHECK(
            bbtc_ib_propellant_mass_evaluate_double(
                &charge, 4.0, &grain, &kinetics, &result
            ) == BBTC_STATUS_SUCCESS
        );
        CHECK(result.equivalent_population_scale == 0.25);
        CHECK(isfinite(result.remaining_mass_kg));
        CHECK(result.remaining_mass_kg > 0.0);
        CHECK(isfinite(result.reacted_mass_kg));
        CHECK(result.reacted_mass_kg > 0.0);
    }

    /*
     * The mass-rate expression also has a representable answer after an
     * otherwise overflowing intermediate product.
     */
    {
        const bbtc_ib_propellant_charge_double_t charge =
        {
            .charge_mass_kg = DBL_MAX,
            .condensed_phase_density_kg_per_m3 = 1.0
        };
        const bbtc_ib_propellant_grain_state_double_t grain =
        {
            .remaining_volume_m3 = DBL_MAX / 2.0,
            .burning_surface_area_m2 = 2.0,
            .remaining_regression_to_burnout_m = 1.0,
            .consumed_volume_fraction = 0.5
        };
        const bbtc_ib_propellant_burn_kinetics_result_double_t kinetics =
        {
            .applicability_flags = 0U,
            .burn_rate_m_per_s = 0.25
        };

        CHECK(
            bbtc_ib_propellant_mass_evaluate_double(
                &charge, DBL_MAX, &grain, &kinetics, &result
            ) == BBTC_STATUS_SUCCESS
        );
        CHECK(result.equivalent_population_scale == 1.0);
        CHECK(result.burning_surface_area_m2 == 2.0);
        CHECK(result.reacted_mass_rate_kg_per_s == 0.5);
    }

    return EXIT_SUCCESS;
}


/**
 * @brief Verifies that genuinely unrepresentable positive outputs fail cleanly.
 */
static int test_numerical_failure(void)
{
    bbtc_ib_propellant_mass_result_double_t result = {0};

    {
        const bbtc_ib_propellant_charge_double_t charge =
        {
            .charge_mass_kg = DBL_TRUE_MIN,
            .condensed_phase_density_kg_per_m3 = DBL_MAX
        };
        const bbtc_ib_propellant_grain_state_double_t grain =
        {
            .remaining_volume_m3 = DBL_MAX,
            .burning_surface_area_m2 = 1.0,
            .remaining_regression_to_burnout_m = 1.0,
            .consumed_volume_fraction = 0.0
        };
        const bbtc_ib_propellant_burn_kinetics_result_double_t kinetics =
        {
            .applicability_flags = 0U,
            .burn_rate_m_per_s = 0.0
        };

        CHECK(
            bbtc_ib_propellant_mass_evaluate_double(
                &charge, DBL_MAX, &grain, &kinetics, &result
            ) == BBTC_STATUS_NUMERICAL_FAILURE
        );
    }

    {
        const bbtc_ib_propellant_charge_double_t charge =
        {
            .charge_mass_kg = DBL_MAX,
            .condensed_phase_density_kg_per_m3 = 1.0
        };
        const bbtc_ib_propellant_grain_state_double_t grain =
        {
            .remaining_volume_m3 = 1.0,
            .burning_surface_area_m2 = 2.0,
            .remaining_regression_to_burnout_m = 1.0,
            .consumed_volume_fraction = 0.0
        };
        const bbtc_ib_propellant_burn_kinetics_result_double_t kinetics =
        {
            .applicability_flags = 0U,
            .burn_rate_m_per_s = 0.0
        };

        CHECK(
            bbtc_ib_propellant_mass_evaluate_double(
                &charge, 1.0, &grain, &kinetics, &result
            ) == BBTC_STATUS_NUMERICAL_FAILURE
        );
    }

    return EXIT_SUCCESS;
}


int main(void)
{
    if (test_interior_all_precisions() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_exact_boundaries() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_fractional_population() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_arguments_and_output_clearing() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_initial_volume_validation() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_grain_state_validation() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_grain_state_consistency() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_kinetics_rate_validation() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_applicability_propagation() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_extreme_scale_arithmetic() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    return test_numerical_failure();
}
