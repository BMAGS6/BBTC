/**
 * @file
 * @brief Tests reduced propellant thermochemistry and reaction-source evaluation.
 */
#include <stdio.h>
#include <stdlib.h>
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
 * @brief Verifies model validation across all scalar families.
 *
 * @return `EXIT_SUCCESS` on success; otherwise `EXIT_FAILURE`.
 */
static int
test_model_validation_all_precisions(void)
{
    const bbtc_ib_propellant_thermochemistry_float_t model_f =
    {
        .gas_product_mass_fraction = 0.8f,
        .specific_reaction_internal_energy_release_j_per_kg = 3.0e6f
    };

    const bbtc_ib_propellant_thermochemistry_double_t model_d =
    {
        .gas_product_mass_fraction = 0.8,
        .specific_reaction_internal_energy_release_j_per_kg = 3.0e6
    };

    const bbtc_ib_propellant_thermochemistry_long_double_t model_l =
    {
        .gas_product_mass_fraction = 0.8L,
        .specific_reaction_internal_energy_release_j_per_kg = 3.0e6L
    };

    CHECK(
        bbtc_ib_propellant_thermochemistry_validate_float(&model_f)
            == BBTC_STATUS_SUCCESS
    );

    CHECK(
        bbtc_ib_propellant_thermochemistry_validate_double(&model_d)
            == BBTC_STATUS_SUCCESS
    );

    CHECK(
        bbtc_ib_propellant_thermochemistry_validate_long_double(&model_l)
            == BBTC_STATUS_SUCCESS
    );

    CHECK(
        bbtc_ib_propellant_thermochemistry_validate_double(NULL)
            == BBTC_STATUS_INVALID_ARGUMENT
    );

    {
        bbtc_ib_propellant_thermochemistry_double_t invalid = model_d;

        invalid.gas_product_mass_fraction = NAN;

        CHECK(
            bbtc_ib_propellant_thermochemistry_validate_double(&invalid)
                == BBTC_STATUS_NONFINITE_INPUT
        );

        invalid = model_d;
        invalid.gas_product_mass_fraction = 0.0;

        CHECK(
            bbtc_ib_propellant_thermochemistry_validate_double(&invalid)
                == BBTC_STATUS_OUTSIDE_DOMAIN
        );

        invalid = model_d;
        invalid.gas_product_mass_fraction = 1.0001;

        CHECK(
            bbtc_ib_propellant_thermochemistry_validate_double(&invalid)
                == BBTC_STATUS_OUTSIDE_DOMAIN
        );

        invalid = model_d;
        invalid.specific_reaction_internal_energy_release_j_per_kg = 0.0;

        CHECK(
            bbtc_ib_propellant_thermochemistry_validate_double(&invalid)
                == BBTC_STATUS_OUTSIDE_DOMAIN
        );
    }

    return EXIT_SUCCESS;
}


/**
 * @brief Verifies finite split-product source evaluation in every precision.
 *
 * @return `EXIT_SUCCESS` on success; otherwise `EXIT_FAILURE`.
 */
static int
test_source_split_all_precisions(void)
{
    const bbtc_ib_propellant_thermochemistry_float_t model_f =
    {
        .gas_product_mass_fraction = 0.75f,
        .specific_reaction_internal_energy_release_j_per_kg = 4.0f
    };

    const bbtc_ib_propellant_thermochemistry_double_t model_d =
    {
        .gas_product_mass_fraction = 0.75,
        .specific_reaction_internal_energy_release_j_per_kg = 4.0
    };

    const bbtc_ib_propellant_thermochemistry_long_double_t model_l =
    {
        .gas_product_mass_fraction = 0.75L,
        .specific_reaction_internal_energy_release_j_per_kg = 4.0L
    };

    bbtc_ib_propellant_thermochemical_source_float_t source_f = {0};
    bbtc_ib_propellant_thermochemical_source_double_t source_d = {0};
    bbtc_ib_propellant_thermochemical_source_long_double_t source_l = {0};

    CHECK(
        bbtc_ib_propellant_thermochemical_source_evaluate_float(
            &model_f,
            2.0f,
            &source_f
        ) == BBTC_STATUS_SUCCESS
    );

    CHECK(source_f.gas_product_mass_kg == 1.5f);
    CHECK(source_f.condensed_product_mass_kg == 0.5f);
    CHECK(source_f.reaction_internal_energy_release_j == 8.0f);

    CHECK(
        bbtc_ib_propellant_thermochemical_source_evaluate_double(
            &model_d,
            2.0,
            &source_d
        ) == BBTC_STATUS_SUCCESS
    );

    CHECK(source_d.gas_product_mass_kg == 1.5);
    CHECK(source_d.condensed_product_mass_kg == 0.5);
    CHECK(source_d.reaction_internal_energy_release_j == 8.0);

    CHECK(
        bbtc_ib_propellant_thermochemical_source_evaluate_long_double(
            &model_l,
            2.0L,
            &source_l
        ) == BBTC_STATUS_SUCCESS
    );

    CHECK(source_l.gas_product_mass_kg == 1.5L);
    CHECK(source_l.condensed_product_mass_kg == 0.5L);
    CHECK(source_l.reaction_internal_energy_release_j == 8.0L);

    return EXIT_SUCCESS;
}


/**
 * @brief Verifies the all-gas limit and zero-reacted-mass identity state.
 *
 * @return `EXIT_SUCCESS` on success; otherwise `EXIT_FAILURE`.
 */
static int
test_all_gas_and_zero_mass(void)
{
    const bbtc_ib_propellant_thermochemistry_double_t model =
    {
        .gas_product_mass_fraction = 1.0,
        .specific_reaction_internal_energy_release_j_per_kg = 5.0
    };

    bbtc_ib_propellant_thermochemical_source_double_t source =
    {
        .gas_product_mass_kg = 9.0,
        .condensed_product_mass_kg = 9.0,
        .reaction_internal_energy_release_j = 9.0
    };

    CHECK(
        bbtc_ib_propellant_thermochemical_source_evaluate_double(
            &model,
            3.0,
            &source
        ) == BBTC_STATUS_SUCCESS
    );

    CHECK(source.gas_product_mass_kg == 3.0);
    CHECK(source.condensed_product_mass_kg == 0.0);
    CHECK(source.reaction_internal_energy_release_j == 15.0);

    source.gas_product_mass_kg = 9.0;
    source.condensed_product_mass_kg = 9.0;
    source.reaction_internal_energy_release_j = 9.0;

    CHECK(
        bbtc_ib_propellant_thermochemical_source_evaluate_double(
            &model,
            0.0,
            &source
        ) == BBTC_STATUS_SUCCESS
    );

    CHECK(source.gas_product_mass_kg == 0.0);
    CHECK(source.condensed_product_mass_kg == 0.0);
    CHECK(source.reaction_internal_energy_release_j == 0.0);

    return EXIT_SUCCESS;
}


/**
 * @brief Verifies validation ordering and deterministic output clearing.
 *
 * @return `EXIT_SUCCESS` on success; otherwise `EXIT_FAILURE`.
 */
static int
test_failure_semantics(void)
{
    const bbtc_ib_propellant_thermochemistry_double_t model =
    {
        .gas_product_mass_fraction = 0.5,
        .specific_reaction_internal_energy_release_j_per_kg = 4.0
    };

    bbtc_ib_propellant_thermochemical_source_double_t source =
    {
        .gas_product_mass_kg = 7.0,
        .condensed_product_mass_kg = 7.0,
        .reaction_internal_energy_release_j = 7.0
    };

    CHECK(
        bbtc_ib_propellant_thermochemical_source_evaluate_double(
            &model,
            1.0,
            NULL
        ) == BBTC_STATUS_INVALID_ARGUMENT
    );

    CHECK(
        bbtc_ib_propellant_thermochemical_source_evaluate_double(
            NULL,
            1.0,
            &source
        ) == BBTC_STATUS_INVALID_ARGUMENT
    );

    CHECK(source.gas_product_mass_kg == 0.0);
    CHECK(source.condensed_product_mass_kg == 0.0);
    CHECK(source.reaction_internal_energy_release_j == 0.0);

    source.gas_product_mass_kg = 7.0;
    source.condensed_product_mass_kg = 7.0;
    source.reaction_internal_energy_release_j = 7.0;

    CHECK(
        bbtc_ib_propellant_thermochemical_source_evaluate_double(
            &model,
            INFINITY,
            &source
        ) == BBTC_STATUS_NONFINITE_INPUT
    );

    CHECK(source.gas_product_mass_kg == 0.0);
    CHECK(source.condensed_product_mass_kg == 0.0);
    CHECK(source.reaction_internal_energy_release_j == 0.0);

    source.gas_product_mass_kg = 7.0;
    source.condensed_product_mass_kg = 7.0;
    source.reaction_internal_energy_release_j = 7.0;

    CHECK(
        bbtc_ib_propellant_thermochemical_source_evaluate_double(
            &model,
            -1.0,
            &source
        ) == BBTC_STATUS_OUTSIDE_DOMAIN
    );

    CHECK(source.gas_product_mass_kg == 0.0);
    CHECK(source.condensed_product_mass_kg == 0.0);
    CHECK(source.reaction_internal_energy_release_j == 0.0);

    return EXIT_SUCCESS;
}


/**
 * @brief Verifies numerical-failure reporting for unrepresentable source terms.
 *
 * @return `EXIT_SUCCESS` on success; otherwise `EXIT_FAILURE`.
 */
static int
test_numerical_failure(void)
{
    const bbtc_ib_propellant_thermochemistry_double_t model =
    {
        .gas_product_mass_fraction = 0.5,
        .specific_reaction_internal_energy_release_j_per_kg = DBL_MAX
    };

    bbtc_ib_propellant_thermochemical_source_double_t source =
    {
        .gas_product_mass_kg = 1.0,
        .condensed_product_mass_kg = 1.0,
        .reaction_internal_energy_release_j = 1.0
    };

    CHECK(
        bbtc_ib_propellant_thermochemical_source_evaluate_double(
            &model,
            2.0,
            &source
        ) == BBTC_STATUS_NUMERICAL_FAILURE
    );

    CHECK(source.gas_product_mass_kg == 0.0);
    CHECK(source.condensed_product_mass_kg == 0.0);
    CHECK(source.reaction_internal_energy_release_j == 0.0);

    return EXIT_SUCCESS;
}


/**
 * @brief Verifies representational underflow failures for required source terms.
 *
 * @details
 * Each case leaves the other required quantities representable so the test
 * isolates one source-term representability boundary at a time.
 *
 * @return `EXIT_SUCCESS` on success; otherwise `EXIT_FAILURE`.
 */
static int
test_source_underflow_failures_double(void)
{
    bbtc_ib_propellant_thermochemical_source_double_t source =
    {
        .gas_product_mass_kg = 7.0,
        .condensed_product_mass_kg = 7.0,
        .reaction_internal_energy_release_j = 7.0
    };

    {
        const bbtc_ib_propellant_thermochemistry_double_t model =
        {
            .gas_product_mass_fraction = DBL_TRUE_MIN,
            .specific_reaction_internal_energy_release_j_per_kg = 1.0
        };

        CHECK(
            bbtc_ib_propellant_thermochemical_source_evaluate_double(
                &model,
                0.5,
                &source
            ) == BBTC_STATUS_NUMERICAL_FAILURE
        );

        CHECK(source.gas_product_mass_kg == 0.0);
        CHECK(source.condensed_product_mass_kg == 0.0);
        CHECK(source.reaction_internal_energy_release_j == 0.0);
    }

    source = (bbtc_ib_propellant_thermochemical_source_double_t)
    {
        .gas_product_mass_kg = 7.0,
        .condensed_product_mass_kg = 7.0,
        .reaction_internal_energy_release_j = 7.0
    };

    {
        const bbtc_ib_propellant_thermochemistry_double_t model =
        {
            .gas_product_mass_fraction = nextafter(1.0, 0.0),
            .specific_reaction_internal_energy_release_j_per_kg = 1.0
        };

        CHECK(
            bbtc_ib_propellant_thermochemical_source_evaluate_double(
                &model,
                DBL_TRUE_MIN,
                &source
            ) == BBTC_STATUS_NUMERICAL_FAILURE
        );

        CHECK(source.gas_product_mass_kg == 0.0);
        CHECK(source.condensed_product_mass_kg == 0.0);
        CHECK(source.reaction_internal_energy_release_j == 0.0);
    }

    source = (bbtc_ib_propellant_thermochemical_source_double_t)
    {
        .gas_product_mass_kg = 7.0,
        .condensed_product_mass_kg = 7.0,
        .reaction_internal_energy_release_j = 7.0
    };

    {
        const bbtc_ib_propellant_thermochemistry_double_t model =
        {
            .gas_product_mass_fraction = 0.5,
            .specific_reaction_internal_energy_release_j_per_kg = DBL_TRUE_MIN
        };

        CHECK(
            bbtc_ib_propellant_thermochemical_source_evaluate_double(
                &model,
                0.5,
                &source
            ) == BBTC_STATUS_NUMERICAL_FAILURE
        );

        CHECK(source.gas_product_mass_kg == 0.0);
        CHECK(source.condensed_product_mass_kg == 0.0);
        CHECK(source.reaction_internal_energy_release_j == 0.0);
    }

    return EXIT_SUCCESS;
}


/**
 * @brief Verifies exact and near-exact gas-yield boundaries.
 *
 * @details
 * The exact unity yield must produce no condensed phase in every scalar
 * family. The immediately smaller representable double must remain a valid
 * split-product model when its condensed source is representable.
 *
 * @return `EXIT_SUCCESS` on success; otherwise `EXIT_FAILURE`.
 */
static int
test_gas_fraction_boundaries_all_precisions(void)
{
    const bbtc_ib_propellant_thermochemistry_float_t model_f =
    {
        .gas_product_mass_fraction = 1.0f,
        .specific_reaction_internal_energy_release_j_per_kg = 2.0f
    };

    const bbtc_ib_propellant_thermochemistry_double_t model_d =
    {
        .gas_product_mass_fraction = 1.0,
        .specific_reaction_internal_energy_release_j_per_kg = 2.0
    };

    const bbtc_ib_propellant_thermochemistry_long_double_t model_l =
    {
        .gas_product_mass_fraction = 1.0L,
        .specific_reaction_internal_energy_release_j_per_kg = 2.0L
    };

    bbtc_ib_propellant_thermochemical_source_float_t source_f = {0};
    bbtc_ib_propellant_thermochemical_source_double_t source_d = {0};
    bbtc_ib_propellant_thermochemical_source_long_double_t source_l = {0};

    CHECK(
        bbtc_ib_propellant_thermochemical_source_evaluate_float(
            &model_f,
            3.0f,
            &source_f
        ) == BBTC_STATUS_SUCCESS
    );

    CHECK(source_f.gas_product_mass_kg == 3.0f);
    CHECK(source_f.condensed_product_mass_kg == 0.0f);
    CHECK(source_f.reaction_internal_energy_release_j == 6.0f);

    CHECK(
        bbtc_ib_propellant_thermochemical_source_evaluate_double(
            &model_d,
            3.0,
            &source_d
        ) == BBTC_STATUS_SUCCESS
    );

    CHECK(source_d.gas_product_mass_kg == 3.0);
    CHECK(source_d.condensed_product_mass_kg == 0.0);
    CHECK(source_d.reaction_internal_energy_release_j == 6.0);

    CHECK(
        bbtc_ib_propellant_thermochemical_source_evaluate_long_double(
            &model_l,
            3.0L,
            &source_l
        ) == BBTC_STATUS_SUCCESS
    );

    CHECK(source_l.gas_product_mass_kg == 3.0L);
    CHECK(source_l.condensed_product_mass_kg == 0.0L);
    CHECK(source_l.reaction_internal_energy_release_j == 6.0L);

    {
        const bbtc_ib_propellant_thermochemistry_double_t near_unity =
        {
            .gas_product_mass_fraction = nextafter(1.0, 0.0),
            .specific_reaction_internal_energy_release_j_per_kg = 2.0
        };

        CHECK(
            bbtc_ib_propellant_thermochemical_source_evaluate_double(
                &near_unity,
                1.0,
                &source_d
            ) == BBTC_STATUS_SUCCESS
        );

        CHECK(source_d.gas_product_mass_kg > 0.0);
        CHECK(source_d.gas_product_mass_kg < 1.0);
        CHECK(source_d.condensed_product_mass_kg > 0.0);
        CHECK(source_d.reaction_internal_energy_release_j == 2.0);
    }

    return EXIT_SUCCESS;
}


/**
 * @brief Verifies the zero-reacted-mass identity in every scalar family.
 *
 * @return `EXIT_SUCCESS` on success; otherwise `EXIT_FAILURE`.
 */
static int
test_zero_reacted_mass_all_precisions(void)
{
    const bbtc_ib_propellant_thermochemistry_float_t model_f =
    {
        .gas_product_mass_fraction = 0.75f,
        .specific_reaction_internal_energy_release_j_per_kg = 4.0f
    };

    const bbtc_ib_propellant_thermochemistry_double_t model_d =
    {
        .gas_product_mass_fraction = 0.75,
        .specific_reaction_internal_energy_release_j_per_kg = 4.0
    };

    const bbtc_ib_propellant_thermochemistry_long_double_t model_l =
    {
        .gas_product_mass_fraction = 0.75L,
        .specific_reaction_internal_energy_release_j_per_kg = 4.0L
    };

    bbtc_ib_propellant_thermochemical_source_float_t source_f =
    {
        .gas_product_mass_kg = 9.0f,
        .condensed_product_mass_kg = 9.0f,
        .reaction_internal_energy_release_j = 9.0f
    };

    bbtc_ib_propellant_thermochemical_source_double_t source_d =
    {
        .gas_product_mass_kg = 9.0,
        .condensed_product_mass_kg = 9.0,
        .reaction_internal_energy_release_j = 9.0
    };

    bbtc_ib_propellant_thermochemical_source_long_double_t source_l =
    {
        .gas_product_mass_kg = 9.0L,
        .condensed_product_mass_kg = 9.0L,
        .reaction_internal_energy_release_j = 9.0L
    };

    CHECK(
        bbtc_ib_propellant_thermochemical_source_evaluate_float(
            &model_f,
            0.0f,
            &source_f
        ) == BBTC_STATUS_SUCCESS
    );

    CHECK(source_f.gas_product_mass_kg == 0.0f);
    CHECK(source_f.condensed_product_mass_kg == 0.0f);
    CHECK(source_f.reaction_internal_energy_release_j == 0.0f);

    CHECK(
        bbtc_ib_propellant_thermochemical_source_evaluate_double(
            &model_d,
            0.0,
            &source_d
        ) == BBTC_STATUS_SUCCESS
    );

    CHECK(source_d.gas_product_mass_kg == 0.0);
    CHECK(source_d.condensed_product_mass_kg == 0.0);
    CHECK(source_d.reaction_internal_energy_release_j == 0.0);

    CHECK(
        bbtc_ib_propellant_thermochemical_source_evaluate_long_double(
            &model_l,
            0.0L,
            &source_l
        ) == BBTC_STATUS_SUCCESS
    );

    CHECK(source_l.gas_product_mass_kg == 0.0L);
    CHECK(source_l.condensed_product_mass_kg == 0.0L);
    CHECK(source_l.reaction_internal_energy_release_j == 0.0L);

    return EXIT_SUCCESS;
}


/**
 * @brief Verifies deterministic model-before-mass validation ordering.
 *
 * @return `EXIT_SUCCESS` on success; otherwise `EXIT_FAILURE`.
 */
static int
test_validation_ordering_and_nonfinite_inputs(void)
{
    const bbtc_ib_propellant_thermochemistry_double_t valid_model =
    {
        .gas_product_mass_fraction = 0.5,
        .specific_reaction_internal_energy_release_j_per_kg = 4.0
    };

    bbtc_ib_propellant_thermochemistry_double_t invalid_model = valid_model;

    bbtc_ib_propellant_thermochemical_source_double_t source =
    {
        .gas_product_mass_kg = 5.0,
        .condensed_product_mass_kg = 5.0,
        .reaction_internal_energy_release_j = 5.0
    };

    CHECK(
        bbtc_ib_propellant_thermochemical_source_evaluate_double(
            NULL,
            NAN,
            &source
        ) == BBTC_STATUS_INVALID_ARGUMENT
    );

    CHECK(source.gas_product_mass_kg == 0.0);
    CHECK(source.condensed_product_mass_kg == 0.0);
    CHECK(source.reaction_internal_energy_release_j == 0.0);

    invalid_model.gas_product_mass_fraction = NAN;

    CHECK(
        bbtc_ib_propellant_thermochemical_source_evaluate_double(
            &invalid_model,
            -1.0,
            &source
        ) == BBTC_STATUS_NONFINITE_INPUT
    );

    CHECK(source.gas_product_mass_kg == 0.0);
    CHECK(source.condensed_product_mass_kg == 0.0);
    CHECK(source.reaction_internal_energy_release_j == 0.0);

    CHECK(
        bbtc_ib_propellant_thermochemical_source_evaluate_double(
            &valid_model,
            NAN,
            &source
        ) == BBTC_STATUS_NONFINITE_INPUT
    );

    CHECK(
        bbtc_ib_propellant_thermochemical_source_evaluate_double(
            &valid_model,
            INFINITY,
            &source
        ) == BBTC_STATUS_NONFINITE_INPUT
    );

    return EXIT_SUCCESS;
}


/**
 * @brief Verifies mass partition within floating-point rounding error.
 *
 * @details
 * The public contract conserves reacted mass algebraically, but does not
 * require the separately rounded source fields to sum bit-for-bit to the input
 * mass. This test therefore checks a small numerical residual rather than
 * imposing a false exact-arithmetic ABI promise.
 *
 * @return `EXIT_SUCCESS` on success; otherwise `EXIT_FAILURE`.
 */
static int
test_mass_partition_rounding_double(void)
{
    const bbtc_ib_propellant_thermochemistry_double_t model =
    {
        .gas_product_mass_fraction = 0.3,
        .specific_reaction_internal_energy_release_j_per_kg = 7.0
    };

    const double reacted_propellant_mass_kg = 0.1;
    bbtc_ib_propellant_thermochemical_source_double_t source = {0};
    double partition_residual;

    CHECK(
        bbtc_ib_propellant_thermochemical_source_evaluate_double(
            &model,
            reacted_propellant_mass_kg,
            &source
        ) == BBTC_STATUS_SUCCESS
    );

    partition_residual = fabs(
        (source.gas_product_mass_kg + source.condensed_product_mass_kg) -
        reacted_propellant_mass_kg
    );

    CHECK(
        partition_residual <=
            4.0 * DBL_EPSILON * fabs(reacted_propellant_mass_kg)
    );

    return EXIT_SUCCESS;
}


/**
 * @brief Runs the IB0.4a reduced propellant-thermochemistry tests.
 *
 * @return `EXIT_SUCCESS` when every check passes; otherwise `EXIT_FAILURE`.
 */
int main(void)
{

    if (test_source_underflow_failures_double() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_gas_fraction_boundaries_all_precisions() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_zero_reacted_mass_all_precisions() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_validation_ordering_and_nonfinite_inputs() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_mass_partition_rounding_double() != EXIT_SUCCESS)
        return EXIT_FAILURE;


    if (test_model_validation_all_precisions() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_source_split_all_precisions() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_all_gas_and_zero_mass() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_failure_semantics() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    return test_numerical_failure();
}
