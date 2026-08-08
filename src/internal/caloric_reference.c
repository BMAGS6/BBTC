/**
 * @file
 * @brief Validation for explicit reduced-gas caloric reference records.
 */

#include "bbtc/internal_ballistics/caloric_reference.h"

#include <stddef.h>
#include <math.h>

bbtc_status_e
bbtc_ib_caloric_reference_validate_float(const bbtc_ib_caloric_reference_float_t* const reference)
{
    if (reference == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (!isfinite(reference->reference_temperature_k)                      ||
        !isfinite(reference->reference_specific_internal_energy_j_per_kg))
    {
        return BBTC_STATUS_NONFINITE_INPUT;
    }

    if (reference->reference_temperature_k <= 0.0f)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e
bbtc_ib_caloric_reference_validate_double(const bbtc_ib_caloric_reference_double_t* const reference)
{
    if (reference == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (!isfinite(reference->reference_temperature_k)                      ||
        !isfinite(reference->reference_specific_internal_energy_j_per_kg))
    {
        return BBTC_STATUS_NONFINITE_INPUT;
    }

    if (reference->reference_temperature_k <= 0.0)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e
bbtc_ib_caloric_reference_validate_long_double(
    const bbtc_ib_caloric_reference_long_double_t* const reference
)
{
    if (reference == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (!isfinite(reference->reference_temperature_k)                      ||
        !isfinite(reference->reference_specific_internal_energy_j_per_kg))
    {
        return BBTC_STATUS_NONFINITE_INPUT;
    }

    if (reference->reference_temperature_k <= 0.0L)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    return BBTC_STATUS_SUCCESS;
}
