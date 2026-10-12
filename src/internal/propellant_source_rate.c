/**
 * @file
 * @brief Reduced thermochemical source-rate coupling implementation.
 *
 * @details
 * This translation unit intentionally performs a narrow algebraic transform.
 * It consumes thermochemical coefficients that have already been selected by
 * the caller and a reacted-propellant mass rate that has already been produced
 * by the IB0.4e mass-coupling layer. It neither evaluates nor hides any burn
 * law, grain geometry, time integration, chamber state, or gas equation of
 * state.
 *
 * The three scalar families are implemented directly in their native C types.
 * The float and long-double paths do not convert through double.
 */
#include <math.h>
#include <stddef.h>

#include "bbtc/internal_ballistics/propellant_source_rate.h"


bbtc_status_e
bbtc_ib_propellant_source_rate_evaluate_float(
    const bbtc_ib_propellant_thermochemistry_float_t* const thermochemistry,
    const bbtc_ib_propellant_mass_result_float_t* const mass,
    bbtc_ib_propellant_source_rate_float_t* const result
)
{
    bbtc_status_e status;
    float reacted_mass_rate_kg_per_s;
    float gas_product_mass_rate_kg_per_s;
    float condensed_product_mass_rate_kg_per_s;
    float reaction_energy_release_rate_w;

    if (result == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    /*
     * Clearing happens before every later validation step. This keeps failure
     * output deterministic and, critically, prevents stale applicability bits
     * from surviving a failed evaluation.
     */
    *result = (bbtc_ib_propellant_source_rate_float_t){0};

    /*
     * The thermochemistry record is the model input for this layer, so its
     * established IB0.4a validation precedence remains ahead of the upstream
     * mass-result pointer and rate scalar.
     */
    status = bbtc_ib_propellant_thermochemistry_validate_float(thermochemistry);

    if (status != BBTC_STATUS_SUCCESS)
        return status;

    if (mass == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    reacted_mass_rate_kg_per_s = mass->reacted_mass_rate_kg_per_s;

    if (isnan(reacted_mass_rate_kg_per_s))
        return BBTC_STATUS_NAN_INPUT;

    if (isinf(reacted_mass_rate_kg_per_s))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (reacted_mass_rate_kg_per_s < 0.0f)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    /*
     * Zero is a first-class identity state. The rates remain exactly zero, but
     * upstream applicability metadata still belongs to this successful state.
     */
    if (reacted_mass_rate_kg_per_s == 0.0f)
    {
        result->applicability_flags = mass->applicability_flags;
        return BBTC_STATUS_SUCCESS;
    }

    gas_product_mass_rate_kg_per_s =
        thermochemistry->gas_product_mass_fraction
        * reacted_mass_rate_kg_per_s;

    /*
     * The all-gas boundary is made explicit rather than being left to an
     * incidental subtraction. For every strict split-product state, the
     * subtraction is positive and its product must remain representable.
     */
    if (thermochemistry->gas_product_mass_fraction == 1.0f)
        condensed_product_mass_rate_kg_per_s = 0.0f;
    else
    {
        condensed_product_mass_rate_kg_per_s =
            (1.0f - thermochemistry->gas_product_mass_fraction)
            * reacted_mass_rate_kg_per_s;
    }

    reaction_energy_release_rate_w =
        thermochemistry->specific_reaction_internal_energy_release_j_per_kg
        * reacted_mass_rate_kg_per_s;

    /*
     * Every positive reacted-mass-rate state requires positive finite gaseous
     * product and reaction-energy rates. A result that underflows to zero or
     * overflows to infinity is a numerical failure, never a clamped success.
     */
    if (!isfinite(gas_product_mass_rate_kg_per_s) ||
        gas_product_mass_rate_kg_per_s <= 0.0f    ||
        !isfinite(reaction_energy_release_rate_w) ||
        reaction_energy_release_rate_w <= 0.0f)
    {
        return BBTC_STATUS_NUMERICAL_FAILURE;
    }

    if (thermochemistry->gas_product_mass_fraction < 1.0f)
    {
        if (!isfinite(condensed_product_mass_rate_kg_per_s) ||
            condensed_product_mass_rate_kg_per_s <= 0.0f)
        {
            return BBTC_STATUS_NUMERICAL_FAILURE;
        }
    }
    else if (condensed_product_mass_rate_kg_per_s != 0.0f)
        return BBTC_STATUS_INTERNAL_INVARIANT_FAILURE;

    /*
     * Commit only after every source term has passed validation. Unknown or
     * future applicability bits are deliberately copied without filtering.
     */
    result->applicability_flags = mass->applicability_flags;
    result->gas_product_mass_rate_kg_per_s = gas_product_mass_rate_kg_per_s;
    result->condensed_product_mass_rate_kg_per_s =
        condensed_product_mass_rate_kg_per_s;
    result->reaction_energy_release_rate_w = reaction_energy_release_rate_w;

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e
bbtc_ib_propellant_source_rate_evaluate_double(
    const bbtc_ib_propellant_thermochemistry_double_t* const thermochemistry,
    const bbtc_ib_propellant_mass_result_double_t* const mass,
    bbtc_ib_propellant_source_rate_double_t* const result
)
{
    bbtc_status_e status;
    double reacted_mass_rate_kg_per_s;
    double gas_product_mass_rate_kg_per_s;
    double condensed_product_mass_rate_kg_per_s;
    double reaction_energy_release_rate_w;

    if (result == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    *result = (bbtc_ib_propellant_source_rate_double_t){0};

    status = bbtc_ib_propellant_thermochemistry_validate_double(thermochemistry);

    if (status != BBTC_STATUS_SUCCESS)
        return status;

    if (mass == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    reacted_mass_rate_kg_per_s = mass->reacted_mass_rate_kg_per_s;

    if (isnan(reacted_mass_rate_kg_per_s))
        return BBTC_STATUS_NAN_INPUT;

    if (isinf(reacted_mass_rate_kg_per_s))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (reacted_mass_rate_kg_per_s < 0.0)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    if (reacted_mass_rate_kg_per_s == 0.0)
    {
        result->applicability_flags = mass->applicability_flags;
        return BBTC_STATUS_SUCCESS;
    }

    gas_product_mass_rate_kg_per_s =
        thermochemistry->gas_product_mass_fraction
        * reacted_mass_rate_kg_per_s;

    if (thermochemistry->gas_product_mass_fraction == 1.0)
        condensed_product_mass_rate_kg_per_s = 0.0;
    else
    {
        condensed_product_mass_rate_kg_per_s =
            (1.0 - thermochemistry->gas_product_mass_fraction)
            * reacted_mass_rate_kg_per_s;
    }

    reaction_energy_release_rate_w =
        thermochemistry->specific_reaction_internal_energy_release_j_per_kg
        * reacted_mass_rate_kg_per_s;

    if (!isfinite(gas_product_mass_rate_kg_per_s) ||
        gas_product_mass_rate_kg_per_s <= 0.0     ||
        !isfinite(reaction_energy_release_rate_w) ||
        reaction_energy_release_rate_w <= 0.0)
    {
        return BBTC_STATUS_NUMERICAL_FAILURE;
    }

    if (thermochemistry->gas_product_mass_fraction < 1.0)
    {
        if (!isfinite(condensed_product_mass_rate_kg_per_s) ||
            condensed_product_mass_rate_kg_per_s <= 0.0)
        {
            return BBTC_STATUS_NUMERICAL_FAILURE;
        }
    }
    else if (condensed_product_mass_rate_kg_per_s != 0.0)
        return BBTC_STATUS_INTERNAL_INVARIANT_FAILURE;

    result->applicability_flags = mass->applicability_flags;
    result->gas_product_mass_rate_kg_per_s = gas_product_mass_rate_kg_per_s;
    result->condensed_product_mass_rate_kg_per_s =
        condensed_product_mass_rate_kg_per_s;
    result->reaction_energy_release_rate_w = reaction_energy_release_rate_w;

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e
bbtc_ib_propellant_source_rate_evaluate_long_double(
    const bbtc_ib_propellant_thermochemistry_long_double_t* const thermochemistry,
    const bbtc_ib_propellant_mass_result_long_double_t* const mass,
    bbtc_ib_propellant_source_rate_long_double_t* const result
)
{
    bbtc_status_e status;
    long double reacted_mass_rate_kg_per_s;
    long double gas_product_mass_rate_kg_per_s;
    long double condensed_product_mass_rate_kg_per_s;
    long double reaction_energy_release_rate_w;

    if (result == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    *result = (bbtc_ib_propellant_source_rate_long_double_t){0};

    status =
        bbtc_ib_propellant_thermochemistry_validate_long_double(thermochemistry);

    if (status != BBTC_STATUS_SUCCESS)
        return status;

    if (mass == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    reacted_mass_rate_kg_per_s = mass->reacted_mass_rate_kg_per_s;

    if (isnan(reacted_mass_rate_kg_per_s))
        return BBTC_STATUS_NAN_INPUT;

    if (isinf(reacted_mass_rate_kg_per_s))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (reacted_mass_rate_kg_per_s < 0.0L)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    if (reacted_mass_rate_kg_per_s == 0.0L)
    {
        result->applicability_flags = mass->applicability_flags;
        return BBTC_STATUS_SUCCESS;
    }

    gas_product_mass_rate_kg_per_s =
        thermochemistry->gas_product_mass_fraction
        * reacted_mass_rate_kg_per_s;

    if (thermochemistry->gas_product_mass_fraction == 1.0L)
        condensed_product_mass_rate_kg_per_s = 0.0L;
    else
    {
        condensed_product_mass_rate_kg_per_s =
            (1.0L - thermochemistry->gas_product_mass_fraction)
            * reacted_mass_rate_kg_per_s;
    }

    reaction_energy_release_rate_w =
        thermochemistry->specific_reaction_internal_energy_release_j_per_kg
        * reacted_mass_rate_kg_per_s;

    if (!isfinite(gas_product_mass_rate_kg_per_s) ||
        gas_product_mass_rate_kg_per_s <= 0.0L    ||
        !isfinite(reaction_energy_release_rate_w) ||
        reaction_energy_release_rate_w <= 0.0L)
    {
        return BBTC_STATUS_NUMERICAL_FAILURE;
    }

    if (thermochemistry->gas_product_mass_fraction < 1.0L)
    {
        if (!isfinite(condensed_product_mass_rate_kg_per_s) ||
            condensed_product_mass_rate_kg_per_s <= 0.0L)
        {
            return BBTC_STATUS_NUMERICAL_FAILURE;
        }
    }
    else if (condensed_product_mass_rate_kg_per_s != 0.0L)
        return BBTC_STATUS_INTERNAL_INVARIANT_FAILURE;

    result->applicability_flags = mass->applicability_flags;
    result->gas_product_mass_rate_kg_per_s = gas_product_mass_rate_kg_per_s;
    result->condensed_product_mass_rate_kg_per_s =
        condensed_product_mass_rate_kg_per_s;
    result->reaction_energy_release_rate_w = reaction_energy_release_rate_w;

    return BBTC_STATUS_SUCCESS;
}
