/**
 * @file
 * @brief Reduced propellant thermochemistry validation and source evaluation.
 */
#include <stddef.h>
#include <math.h>

#include "bbtc/internal_ballistics/propellant_thermochemistry.h"

bbtc_status_e
bbtc_ib_propellant_thermochemistry_validate_float(
    const bbtc_ib_propellant_thermochemistry_float_t* const thermochemistry
)
{
    if (thermochemistry == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (isnan(thermochemistry->gas_product_mass_fraction) ||
        isnan(thermochemistry->specific_reaction_internal_energy_release_j_per_kg))
    {
        return BBTC_STATUS_NAN_INPUT;
    }

    if (isinf(thermochemistry->gas_product_mass_fraction) ||
        isinf(thermochemistry->specific_reaction_internal_energy_release_j_per_kg))
    {
        return BBTC_STATUS_NONFINITE_INPUT;
    }

    if (thermochemistry->gas_product_mass_fraction                          <= 0.0f ||
        thermochemistry->gas_product_mass_fraction                          >  1.0f ||
        thermochemistry->specific_reaction_internal_energy_release_j_per_kg <= 0.0f)
    {
        return BBTC_STATUS_OUTSIDE_DOMAIN;
    }

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e
bbtc_ib_propellant_thermochemistry_validate_double(
    const bbtc_ib_propellant_thermochemistry_double_t* const thermochemistry
)
{
    if (thermochemistry == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (isnan(thermochemistry->gas_product_mass_fraction) ||
        isnan(thermochemistry->specific_reaction_internal_energy_release_j_per_kg))
    {
        return BBTC_STATUS_NAN_INPUT;
    }

    if (isinf(thermochemistry->gas_product_mass_fraction) ||
        isinf(thermochemistry->specific_reaction_internal_energy_release_j_per_kg))
    {
        return BBTC_STATUS_NONFINITE_INPUT;
    }

    if (thermochemistry->gas_product_mass_fraction                          <= 0.0 ||
        thermochemistry->gas_product_mass_fraction                          >  1.0 ||
        thermochemistry->specific_reaction_internal_energy_release_j_per_kg <= 0.0)
    {
        return BBTC_STATUS_OUTSIDE_DOMAIN;
    }

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e
bbtc_ib_propellant_thermochemistry_validate_long_double(
    const bbtc_ib_propellant_thermochemistry_long_double_t* const thermochemistry
)
{
    if (thermochemistry == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (isnan(thermochemistry->gas_product_mass_fraction) ||
        isnan(thermochemistry->specific_reaction_internal_energy_release_j_per_kg))
    {
        return BBTC_STATUS_NAN_INPUT;
    }

    if (isinf(thermochemistry->gas_product_mass_fraction) ||
        isinf(thermochemistry->specific_reaction_internal_energy_release_j_per_kg))
    {
        return BBTC_STATUS_NONFINITE_INPUT;
    }

    if (thermochemistry->gas_product_mass_fraction                          <= 0.0L ||
        thermochemistry->gas_product_mass_fraction                          >  1.0L ||
        thermochemistry->specific_reaction_internal_energy_release_j_per_kg <= 0.0L)
    {
        return BBTC_STATUS_OUTSIDE_DOMAIN;
    }

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e
bbtc_ib_propellant_thermochemical_source_evaluate_float(
    const bbtc_ib_propellant_thermochemistry_float_t* const thermochemistry,
    const float reacted_propellant_mass_kg,
    bbtc_ib_propellant_thermochemical_source_float_t* const result
)
{
    bbtc_status_e status;
    float         gas_product_mass_kg;
    float         condensed_product_mass_kg;
    float         reaction_internal_energy_release_j;

    if (result == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    *result = (bbtc_ib_propellant_thermochemical_source_float_t){0};

    status = bbtc_ib_propellant_thermochemistry_validate_float(thermochemistry);

    if (status != BBTC_STATUS_SUCCESS)
        return status;

    if (isnan(reacted_propellant_mass_kg))
        return BBTC_STATUS_NAN_INPUT;

    if (isinf(reacted_propellant_mass_kg))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (reacted_propellant_mass_kg < 0.0f)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    if (reacted_propellant_mass_kg == 0.0f)
        return BBTC_STATUS_SUCCESS;

    gas_product_mass_kg =
        thermochemistry->gas_product_mass_fraction
        * reacted_propellant_mass_kg;

    condensed_product_mass_kg =
        (1.0f - thermochemistry->gas_product_mass_fraction)
        * reacted_propellant_mass_kg;

    reaction_internal_energy_release_j =
        thermochemistry->specific_reaction_internal_energy_release_j_per_kg
        * reacted_propellant_mass_kg;

    if (!isfinite(gas_product_mass_kg)                ||
        gas_product_mass_kg <= 0.0f                   ||
        !isfinite(reaction_internal_energy_release_j) ||
        reaction_internal_energy_release_j <= 0.0f)
    {
        return BBTC_STATUS_NUMERICAL_FAILURE;
    }

    if (thermochemistry->gas_product_mass_fraction < 1.0f)
    {
        if (!isfinite(condensed_product_mass_kg) ||
            condensed_product_mass_kg <= 0.0f)
        {
            return BBTC_STATUS_NUMERICAL_FAILURE;
        }
    }
    else if (condensed_product_mass_kg != 0.0f)
        return BBTC_STATUS_INTERNAL_INVARIANT_FAILURE;

    result->gas_product_mass_kg                = gas_product_mass_kg;
    result->condensed_product_mass_kg          = condensed_product_mass_kg;
    result->reaction_internal_energy_release_j = reaction_internal_energy_release_j;

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e
bbtc_ib_propellant_thermochemical_source_evaluate_double(
    const bbtc_ib_propellant_thermochemistry_double_t* const thermochemistry,
    const double reacted_propellant_mass_kg,
    bbtc_ib_propellant_thermochemical_source_double_t* const result
)
{
    bbtc_status_e status;
    double        gas_product_mass_kg;
    double        condensed_product_mass_kg;
    double        reaction_internal_energy_release_j;

    if (result == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    *result = (bbtc_ib_propellant_thermochemical_source_double_t){0};

    status = bbtc_ib_propellant_thermochemistry_validate_double(thermochemistry);

    if (status != BBTC_STATUS_SUCCESS)
        return status;

    if (isnan(reacted_propellant_mass_kg))
        return BBTC_STATUS_NAN_INPUT;

    if (isinf(reacted_propellant_mass_kg))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (reacted_propellant_mass_kg < 0.0)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    if (reacted_propellant_mass_kg == 0.0)
        return BBTC_STATUS_SUCCESS;

    gas_product_mass_kg =
        thermochemistry->gas_product_mass_fraction
        * reacted_propellant_mass_kg;

    condensed_product_mass_kg =
        (1.0 - thermochemistry->gas_product_mass_fraction)
        * reacted_propellant_mass_kg;

    reaction_internal_energy_release_j =
        thermochemistry->specific_reaction_internal_energy_release_j_per_kg
        * reacted_propellant_mass_kg;

    if (!isfinite(gas_product_mass_kg) ||
        gas_product_mass_kg <= 0.0 ||
        !isfinite(reaction_internal_energy_release_j) ||
        reaction_internal_energy_release_j <= 0.0)
    {
        return BBTC_STATUS_NUMERICAL_FAILURE;
    }

    if (thermochemistry->gas_product_mass_fraction < 1.0)
    {
        if (!isfinite(condensed_product_mass_kg) ||
            condensed_product_mass_kg <= 0.0)
        {
            return BBTC_STATUS_NUMERICAL_FAILURE;
        }
    }
    else if (condensed_product_mass_kg != 0.0)
        return BBTC_STATUS_INTERNAL_INVARIANT_FAILURE;

    result->gas_product_mass_kg                = gas_product_mass_kg;
    result->condensed_product_mass_kg          = condensed_product_mass_kg;
    result->reaction_internal_energy_release_j = reaction_internal_energy_release_j;

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e
bbtc_ib_propellant_thermochemical_source_evaluate_long_double(
    const bbtc_ib_propellant_thermochemistry_long_double_t* const thermochemistry,
    const long double reacted_propellant_mass_kg,
    bbtc_ib_propellant_thermochemical_source_long_double_t* const result
)
{
    bbtc_status_e status;
    long double   gas_product_mass_kg;
    long double   condensed_product_mass_kg;
    long double   reaction_internal_energy_release_j;

    if (result == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    *result = (bbtc_ib_propellant_thermochemical_source_long_double_t){0};

    status = bbtc_ib_propellant_thermochemistry_validate_long_double(thermochemistry);

    if (status != BBTC_STATUS_SUCCESS)
        return status;

    if (isnan(reacted_propellant_mass_kg))
        return BBTC_STATUS_NAN_INPUT;

    if (isinf(reacted_propellant_mass_kg))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (reacted_propellant_mass_kg < 0.0L)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    if (reacted_propellant_mass_kg == 0.0L)
        return BBTC_STATUS_SUCCESS;

    gas_product_mass_kg =
        thermochemistry->gas_product_mass_fraction
        * reacted_propellant_mass_kg;

    condensed_product_mass_kg =
        (1.0L - thermochemistry->gas_product_mass_fraction)
        * reacted_propellant_mass_kg;

    reaction_internal_energy_release_j =
        thermochemistry->specific_reaction_internal_energy_release_j_per_kg
        * reacted_propellant_mass_kg;

    if (!isfinite(gas_product_mass_kg)                ||
        gas_product_mass_kg <= 0.0L                   ||
        !isfinite(reaction_internal_energy_release_j) ||
        reaction_internal_energy_release_j <= 0.0L)
    {
        return BBTC_STATUS_NUMERICAL_FAILURE;
    }

    if (thermochemistry->gas_product_mass_fraction < 1.0L)
    {
        if (!isfinite(condensed_product_mass_kg) ||
            condensed_product_mass_kg <= 0.0L)
        {
            return BBTC_STATUS_NUMERICAL_FAILURE;
        }
    }
    else if (condensed_product_mass_kg != 0.0L)
        return BBTC_STATUS_INTERNAL_INVARIANT_FAILURE;

    result->gas_product_mass_kg                = gas_product_mass_kg;
    result->condensed_product_mass_kg          = condensed_product_mass_kg;
    result->reaction_internal_energy_release_j = reaction_internal_energy_release_j;

    return BBTC_STATUS_SUCCESS;
}
