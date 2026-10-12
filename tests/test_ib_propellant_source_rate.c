/**
 * @file
 * @brief Tests IB0.4f thermochemical source-rate coupling.
 *
 * @details
 * These tests treat the source-rate evaluator as a narrow algebraic and
 * metadata-preserving coupling primitive. Exact software semantics are tested
 * exactly: output clearing, validation precedence, the zero-rate identity,
 * all-gas behavior, applicability propagation, and representability failures.
 *
 * Ordinary source arithmetic uses binary-exact values where possible. A
 * separate residual test checks the reduced mass-rate partition without
 * imposing a bit-for-bit sum identity that the public contract does not
 * promise.
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
 * @brief Verifies one exactly representable split-product state in all families.
 *
 * @return `EXIT_SUCCESS` when every native family produces the same physical
 *         source rates; otherwise `EXIT_FAILURE`.
 */
static int
test_split_source_all_precisions(void)
{
    const bbtc_ib_propellant_thermochemistry_float_t thermo_f =
    {
        .gas_product_mass_fraction = 0.75f,
        .specific_reaction_internal_energy_release_j_per_kg = 4.0f
    };
    const bbtc_ib_propellant_mass_result_float_t mass_f =
    {
        .applicability_flags = BBTC_APPLICABILITY_NONE_REPORTED,
        .reacted_mass_rate_kg_per_s = 2.0f
    };
    bbtc_ib_propellant_source_rate_float_t result_f = {0};

    const bbtc_ib_propellant_thermochemistry_double_t thermo_d =
    {
        .gas_product_mass_fraction = 0.75,
        .specific_reaction_internal_energy_release_j_per_kg = 4.0
    };
    const bbtc_ib_propellant_mass_result_double_t mass_d =
    {
        .applicability_flags = BBTC_APPLICABILITY_NONE_REPORTED,
        .reacted_mass_rate_kg_per_s = 2.0
    };
    bbtc_ib_propellant_source_rate_double_t result_d = {0};

    const bbtc_ib_propellant_thermochemistry_long_double_t thermo_l =
    {
        .gas_product_mass_fraction = 0.75L,
        .specific_reaction_internal_energy_release_j_per_kg = 4.0L
    };
    const bbtc_ib_propellant_mass_result_long_double_t mass_l =
    {
        .applicability_flags = BBTC_APPLICABILITY_NONE_REPORTED,
        .reacted_mass_rate_kg_per_s = 2.0L
    };
    bbtc_ib_propellant_source_rate_long_double_t result_l = {0};

    CHECK(
        bbtc_ib_propellant_source_rate_evaluate_float(
            &thermo_f,
            &mass_f,
            &result_f
        ) == BBTC_STATUS_SUCCESS
    );
    CHECK(result_f.applicability_flags == BBTC_APPLICABILITY_NONE_REPORTED);
    CHECK(result_f.gas_product_mass_rate_kg_per_s == 1.5f);
    CHECK(result_f.condensed_product_mass_rate_kg_per_s == 0.5f);
    CHECK(result_f.reaction_energy_release_rate_w == 8.0f);

    CHECK(
        bbtc_ib_propellant_source_rate_evaluate_double(
            &thermo_d,
            &mass_d,
            &result_d
        ) == BBTC_STATUS_SUCCESS
    );
    CHECK(result_d.applicability_flags == BBTC_APPLICABILITY_NONE_REPORTED);
    CHECK(result_d.gas_product_mass_rate_kg_per_s == 1.5);
    CHECK(result_d.condensed_product_mass_rate_kg_per_s == 0.5);
    CHECK(result_d.reaction_energy_release_rate_w == 8.0);

    CHECK(
        bbtc_ib_propellant_source_rate_evaluate_long_double(
            &thermo_l,
            &mass_l,
            &result_l
        ) == BBTC_STATUS_SUCCESS
    );
    CHECK(result_l.applicability_flags == BBTC_APPLICABILITY_NONE_REPORTED);
    CHECK(result_l.gas_product_mass_rate_kg_per_s == 1.5L);
    CHECK(result_l.condensed_product_mass_rate_kg_per_s == 0.5L);
    CHECK(result_l.reaction_energy_release_rate_w == 8.0L);

    return EXIT_SUCCESS;
}


/**
 * @brief Verifies zero-rate identity behavior and the exact all-gas boundary.
 */
static int
test_zero_rate_and_all_gas(void)
{
    const bbtc_applicability_flags_t unknown_bit = UINT64_C(1) << 63;
    const bbtc_applicability_flags_t flags =
        BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN | unknown_bit;
    const bbtc_ib_propellant_thermochemistry_double_t thermo =
    {
        .gas_product_mass_fraction = 1.0,
        .specific_reaction_internal_energy_release_j_per_kg = 5.0
    };
    bbtc_ib_propellant_mass_result_double_t mass =
    {
        .applicability_flags = flags,
        .reacted_mass_rate_kg_per_s = 0.0
    };
    bbtc_ib_propellant_source_rate_double_t result =
    {
        .applicability_flags = UINT64_MAX,
        .gas_product_mass_rate_kg_per_s = 9.0,
        .condensed_product_mass_rate_kg_per_s = 9.0,
        .reaction_energy_release_rate_w = 9.0
    };

    CHECK(
        bbtc_ib_propellant_source_rate_evaluate_double(
            &thermo,
            &mass,
            &result
        ) == BBTC_STATUS_SUCCESS
    );
    CHECK(result.applicability_flags == flags);
    CHECK(result.gas_product_mass_rate_kg_per_s == 0.0);
    CHECK(result.condensed_product_mass_rate_kg_per_s == 0.0);
    CHECK(result.reaction_energy_release_rate_w == 0.0);

    mass.reacted_mass_rate_kg_per_s = 3.0;

    CHECK(
        bbtc_ib_propellant_source_rate_evaluate_double(
            &thermo,
            &mass,
            &result
        ) == BBTC_STATUS_SUCCESS
    );
    CHECK(result.applicability_flags == flags);
    CHECK(result.gas_product_mass_rate_kg_per_s == 3.0);
    CHECK(result.condensed_product_mass_rate_kg_per_s == 0.0);
    CHECK(result.reaction_energy_release_rate_w == 15.0);

    return EXIT_SUCCESS;
}


/**
 * @brief Verifies deterministic validation precedence and output clearing.
 *
 * @details
 * The thermochemistry model is validated before the mass-result pointer and
 * reacted-mass-rate scalar. Every failure after a nonnull result pointer has
 * been accepted must leave the complete output cleared.
 */
static int
test_failure_semantics(void)
{
    const bbtc_ib_propellant_thermochemistry_double_t valid_thermo =
    {
        .gas_product_mass_fraction = 0.5,
        .specific_reaction_internal_energy_release_j_per_kg = 4.0
    };
    bbtc_ib_propellant_thermochemistry_double_t invalid_thermo = valid_thermo;
    bbtc_ib_propellant_mass_result_double_t mass =
    {
        .applicability_flags = UINT64_MAX,
        .reacted_mass_rate_kg_per_s = 1.0
    };
    bbtc_ib_propellant_source_rate_double_t result =
    {
        .applicability_flags = UINT64_MAX,
        .gas_product_mass_rate_kg_per_s = 7.0,
        .condensed_product_mass_rate_kg_per_s = 7.0,
        .reaction_energy_release_rate_w = 7.0
    };

    CHECK(
        bbtc_ib_propellant_source_rate_evaluate_double(
            &valid_thermo,
            &mass,
            NULL
        ) == BBTC_STATUS_INVALID_ARGUMENT
    );

    CHECK(
        bbtc_ib_propellant_source_rate_evaluate_double(
            NULL,
            &mass,
            &result
        ) == BBTC_STATUS_INVALID_ARGUMENT
    );
    CHECK(result.applicability_flags == 0U);
    CHECK(result.gas_product_mass_rate_kg_per_s == 0.0);
    CHECK(result.condensed_product_mass_rate_kg_per_s == 0.0);
    CHECK(result.reaction_energy_release_rate_w == 0.0);

    invalid_thermo.gas_product_mass_fraction = NAN;

    CHECK(
        bbtc_ib_propellant_source_rate_evaluate_double(
            &invalid_thermo,
            NULL,
            &result
        ) == BBTC_STATUS_NAN_INPUT
    );

    CHECK(
        bbtc_ib_propellant_source_rate_evaluate_double(
            &valid_thermo,
            NULL,
            &result
        ) == BBTC_STATUS_INVALID_ARGUMENT
    );

    mass.reacted_mass_rate_kg_per_s = NAN;
    CHECK(
        bbtc_ib_propellant_source_rate_evaluate_double(
            &valid_thermo,
            &mass,
            &result
        ) == BBTC_STATUS_NAN_INPUT
    );

    mass.reacted_mass_rate_kg_per_s = INFINITY;
    CHECK(
        bbtc_ib_propellant_source_rate_evaluate_double(
            &valid_thermo,
            &mass,
            &result
        ) == BBTC_STATUS_NONFINITE_INPUT
    );

    mass.reacted_mass_rate_kg_per_s = -INFINITY;
    CHECK(
        bbtc_ib_propellant_source_rate_evaluate_double(
            &valid_thermo,
            &mass,
            &result
        ) == BBTC_STATUS_NONFINITE_INPUT
    );

    mass.reacted_mass_rate_kg_per_s = -1.0;
    CHECK(
        bbtc_ib_propellant_source_rate_evaluate_double(
            &valid_thermo,
            &mass,
            &result
        ) == BBTC_STATUS_OUTSIDE_DOMAIN
    );

    CHECK(result.applicability_flags == 0U);
    CHECK(result.gas_product_mass_rate_kg_per_s == 0.0);
    CHECK(result.condensed_product_mass_rate_kg_per_s == 0.0);
    CHECK(result.reaction_energy_release_rate_w == 0.0);

    return EXIT_SUCCESS;
}


/**
 * @brief Verifies that applicability metadata is copied without interpretation.
 */
static int
test_applicability_propagation(void)
{
    const bbtc_applicability_flags_t unknown_bit = UINT64_C(1) << 63;
    const bbtc_applicability_flags_t expected =
        BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN | unknown_bit;
    const bbtc_ib_propellant_thermochemistry_double_t thermo =
    {
        .gas_product_mass_fraction = 0.75,
        .specific_reaction_internal_energy_release_j_per_kg = 4.0
    };
    const bbtc_ib_propellant_mass_result_double_t mass =
    {
        .applicability_flags = expected,
        .reacted_mass_rate_kg_per_s = 2.0
    };
    bbtc_ib_propellant_source_rate_double_t result = {0};

    CHECK(
        bbtc_ib_propellant_source_rate_evaluate_double(
            &thermo,
            &mass,
            &result
        ) == BBTC_STATUS_SUCCESS
    );
    CHECK(result.applicability_flags == expected);

    return EXIT_SUCCESS;
}


/**
 * @brief Verifies that unrelated upstream diagnostic fields are not revalidated.
 *
 * @details
 * IB0.4f has no access to the original charge, initial grain volume, grain
 * geometry, or kinetics inputs needed to repeat IB0.4e validation. The rate
 * layer therefore consumes only the reacted-mass-rate scalar and applicability
 * mask. This test intentionally poisons unrelated fields in a hand-constructed
 * record and verifies that they do not create a fabricated downstream failure.
 */
static int
test_unrelated_mass_fields_are_not_revalidated(void)
{
    const bbtc_ib_propellant_thermochemistry_double_t thermo =
    {
        .gas_product_mass_fraction = 0.75,
        .specific_reaction_internal_energy_release_j_per_kg = 4.0
    };
    const bbtc_ib_propellant_mass_result_double_t mass =
    {
        .applicability_flags = BBTC_APPLICABILITY_NONE_REPORTED,
        .equivalent_population_scale = NAN,
        .remaining_volume_m3 = -INFINITY,
        .burning_surface_area_m2 = -1.0,
        .remaining_mass_kg = NAN,
        .reacted_mass_kg = INFINITY,
        .reacted_mass_rate_kg_per_s = 2.0
    };
    bbtc_ib_propellant_source_rate_double_t result = {0};

    CHECK(
        bbtc_ib_propellant_source_rate_evaluate_double(
            &thermo,
            &mass,
            &result
        ) == BBTC_STATUS_SUCCESS
    );
    CHECK(result.gas_product_mass_rate_kg_per_s == 1.5);
    CHECK(result.condensed_product_mass_rate_kg_per_s == 0.5);
    CHECK(result.reaction_energy_release_rate_w == 8.0);

    return EXIT_SUCCESS;
}


/**
 * @brief Verifies failure when a required positive source rate is unrepresentable.
 */
static int
test_representability_failures(void)
{
    bbtc_ib_propellant_source_rate_double_t result =
    {
        .applicability_flags = UINT64_MAX,
        .gas_product_mass_rate_kg_per_s = 7.0,
        .condensed_product_mass_rate_kg_per_s = 7.0,
        .reaction_energy_release_rate_w = 7.0
    };

    {
        const bbtc_ib_propellant_thermochemistry_double_t thermo =
        {
            .gas_product_mass_fraction = 0.5,
            .specific_reaction_internal_energy_release_j_per_kg = DBL_MAX
        };
        const bbtc_ib_propellant_mass_result_double_t mass =
        {
            .applicability_flags = 0U,
            .reacted_mass_rate_kg_per_s = 2.0
        };

        CHECK(
            bbtc_ib_propellant_source_rate_evaluate_double(
                &thermo,
                &mass,
                &result
            ) == BBTC_STATUS_NUMERICAL_FAILURE
        );
        CHECK(result.applicability_flags == 0U);
        CHECK(result.gas_product_mass_rate_kg_per_s == 0.0);
        CHECK(result.condensed_product_mass_rate_kg_per_s == 0.0);
        CHECK(result.reaction_energy_release_rate_w == 0.0);
    }

    {
        const bbtc_ib_propellant_thermochemistry_double_t thermo =
        {
            .gas_product_mass_fraction = DBL_TRUE_MIN,
            .specific_reaction_internal_energy_release_j_per_kg = 1.0
        };
        const bbtc_ib_propellant_mass_result_double_t mass =
        {
            .applicability_flags = 0U,
            .reacted_mass_rate_kg_per_s = 0.5
        };

        CHECK(
            bbtc_ib_propellant_source_rate_evaluate_double(
                &thermo,
                &mass,
                &result
            ) == BBTC_STATUS_NUMERICAL_FAILURE
        );
    }

    {
        const bbtc_ib_propellant_thermochemistry_double_t thermo =
        {
            .gas_product_mass_fraction = nextafter(1.0, 0.0),
            .specific_reaction_internal_energy_release_j_per_kg = 1.0
        };
        const bbtc_ib_propellant_mass_result_double_t mass =
        {
            .applicability_flags = 0U,
            .reacted_mass_rate_kg_per_s = DBL_TRUE_MIN
        };

        CHECK(
            bbtc_ib_propellant_source_rate_evaluate_double(
                &thermo,
                &mass,
                &result
            ) == BBTC_STATUS_NUMERICAL_FAILURE
        );
    }

    {
        const bbtc_ib_propellant_thermochemistry_double_t thermo =
        {
            .gas_product_mass_fraction = 0.5,
            .specific_reaction_internal_energy_release_j_per_kg = DBL_TRUE_MIN
        };
        const bbtc_ib_propellant_mass_result_double_t mass =
        {
            .applicability_flags = 0U,
            .reacted_mass_rate_kg_per_s = 0.5
        };

        CHECK(
            bbtc_ib_propellant_source_rate_evaluate_double(
                &thermo,
                &mass,
                &result
            ) == BBTC_STATUS_NUMERICAL_FAILURE
        );
    }

    return EXIT_SUCCESS;
}


/**
 * @brief Verifies the algebraic mass-rate partition with a rounding residual.
 */
static int
test_mass_rate_partition_rounding(void)
{
    const bbtc_ib_propellant_thermochemistry_double_t thermo =
    {
        .gas_product_mass_fraction = 0.3,
        .specific_reaction_internal_energy_release_j_per_kg = 7.0
    };
    const bbtc_ib_propellant_mass_result_double_t mass =
    {
        .applicability_flags = 0U,
        .reacted_mass_rate_kg_per_s = 0.1
    };
    bbtc_ib_propellant_source_rate_double_t result = {0};
    double residual;

    CHECK(
        bbtc_ib_propellant_source_rate_evaluate_double(
            &thermo,
            &mass,
            &result
        ) == BBTC_STATUS_SUCCESS
    );

    residual = fabs(
        (result.gas_product_mass_rate_kg_per_s
         + result.condensed_product_mass_rate_kg_per_s)
        - mass.reacted_mass_rate_kg_per_s
    );

    CHECK(
        residual <=
            4.0 * DBL_EPSILON * fabs(mass.reacted_mass_rate_kg_per_s)
    );

    return EXIT_SUCCESS;
}


/**
 * @brief Cross-checks differential source rates against the IB0.4a cumulative source.
 *
 * @details
 * This is a consistency test only; the library performs no time integration
 * here. For constant thermochemistry and a fixed interval `dt`, multiplying the
 * instantaneous IB0.4f rates by `dt` must agree with the IB0.4a cumulative
 * source evaluated at `m_r = mdot_r * dt`, subject only to native arithmetic.
 */
static int
test_cumulative_rate_consistency(void)
{
    const bbtc_ib_propellant_thermochemistry_double_t thermo =
    {
        .gas_product_mass_fraction = 0.75,
        .specific_reaction_internal_energy_release_j_per_kg = 4.0
    };
    const bbtc_ib_propellant_mass_result_double_t mass =
    {
        .applicability_flags = 0U,
        .reacted_mass_rate_kg_per_s = 2.0
    };
    const double dt_s = 0.25;
    const double reacted_mass_kg = mass.reacted_mass_rate_kg_per_s * dt_s;
    bbtc_ib_propellant_source_rate_double_t rate = {0};
    bbtc_ib_propellant_thermochemical_source_double_t cumulative = {0};

    CHECK(
        bbtc_ib_propellant_source_rate_evaluate_double(
            &thermo,
            &mass,
            &rate
        ) == BBTC_STATUS_SUCCESS
    );

    CHECK(
        bbtc_ib_propellant_thermochemical_source_evaluate_double(
            &thermo,
            reacted_mass_kg,
            &cumulative
        ) == BBTC_STATUS_SUCCESS
    );

    CHECK(rate.gas_product_mass_rate_kg_per_s * dt_s
          == cumulative.gas_product_mass_kg);
    CHECK(rate.condensed_product_mass_rate_kg_per_s * dt_s
          == cumulative.condensed_product_mass_kg);
    CHECK(rate.reaction_energy_release_rate_w * dt_s
          == cumulative.reaction_internal_energy_release_j);

    return EXIT_SUCCESS;
}


/**
 * @brief Verifies that successful evaluation does not mutate either input.
 */
static int
test_input_immutability(void)
{
    const bbtc_ib_propellant_thermochemistry_double_t thermo =
    {
        .gas_product_mass_fraction = 0.75,
        .specific_reaction_internal_energy_release_j_per_kg = 4.0
    };
    const bbtc_ib_propellant_mass_result_double_t mass =
    {
        .applicability_flags = BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN,
        .equivalent_population_scale = 2.0,
        .remaining_volume_m3 = 1.0,
        .burning_surface_area_m2 = 3.0,
        .remaining_mass_kg = 4.0,
        .reacted_mass_kg = 4.0,
        .reacted_mass_rate_kg_per_s = 2.0
    };
    bbtc_ib_propellant_source_rate_double_t result = {0};

    CHECK(
        bbtc_ib_propellant_source_rate_evaluate_double(
            &thermo,
            &mass,
            &result
        ) == BBTC_STATUS_SUCCESS
    );

    CHECK(thermo.gas_product_mass_fraction == 0.75);
    CHECK(thermo.specific_reaction_internal_energy_release_j_per_kg == 4.0);
    CHECK(
        mass.applicability_flags
        == BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN
    );
    CHECK(mass.equivalent_population_scale == 2.0);
    CHECK(mass.remaining_volume_m3 == 1.0);
    CHECK(mass.burning_surface_area_m2 == 3.0);
    CHECK(mass.remaining_mass_kg == 4.0);
    CHECK(mass.reacted_mass_kg == 4.0);
    CHECK(mass.reacted_mass_rate_kg_per_s == 2.0);

    return EXIT_SUCCESS;
}


/**
 * @brief Runs the IB0.4f-B1 propellant source-rate tests.
 *
 * @return `EXIT_SUCCESS` when every check passes; otherwise `EXIT_FAILURE`.
 */
int main(void)
{
    if (test_split_source_all_precisions() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_zero_rate_and_all_gas() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_failure_semantics() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_applicability_propagation() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_unrelated_mass_fields_are_not_revalidated() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_representability_failures() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_mass_rate_partition_rounding() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_cumulative_rate_consistency() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    return test_input_immutability();
}
