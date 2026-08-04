/**
 * @file
 * @brief Validation for precision-qualified propellant-charge records.
 */

#include "bbtc/internal_ballistics/propellant_charge.h"

#include <math.h>
#include <stddef.h>

bbtc_status_e
bbtc_ib_propellant_charge_validate_float(const bbtc_ib_propellant_charge_float_t* const charge)
{
    if (charge == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (!isfinite(charge->charge_mass_kg)                    ||
        !isfinite(charge->condensed_phase_density_kg_per_m3))
    {
        return BBTC_STATUS_NONFINITE_INPUT;
    }

    if (charge->charge_mass_kg                    <= 0.0f ||
        charge->condensed_phase_density_kg_per_m3 <= 0.0f)
    {
        return BBTC_STATUS_OUTSIDE_DOMAIN;
    }

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e
bbtc_ib_propellant_charge_validate_double(const bbtc_ib_propellant_charge_double_t* const charge)
{
    if (charge == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (!isfinite(charge->charge_mass_kg)                    ||
        !isfinite(charge->condensed_phase_density_kg_per_m3))
    {
        return BBTC_STATUS_NONFINITE_INPUT;
    }

    if (charge->charge_mass_kg                    <= 0.0 ||
        charge->condensed_phase_density_kg_per_m3 <= 0.0)
    {
        return BBTC_STATUS_OUTSIDE_DOMAIN;
    }

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e
bbtc_ib_propellant_charge_validate_long_double(const bbtc_ib_propellant_charge_long_double_t* const charge)
{
    if (charge == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (!isfinite(charge->charge_mass_kg)                    ||
        !isfinite(charge->condensed_phase_density_kg_per_m3))
    {
        return BBTC_STATUS_NONFINITE_INPUT;
    }

    if (charge->charge_mass_kg                    <= 0.0L ||
        charge->condensed_phase_density_kg_per_m3 <= 0.0L)
    {
        return BBTC_STATUS_OUTSIDE_DOMAIN;
    }

    return BBTC_STATUS_SUCCESS;
}
