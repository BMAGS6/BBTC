/**
 * @file
 * @brief Validation for precision-qualified propellant-charge records.
 */

#include "bbtc/internal_ballistics/propellant_charge.h"

#include <stddef.h>
#include <math.h>

bbtc_status_e
bbtc_ib_propellant_charge_validate_float(const bbtc_ib_propellant_charge_float_t* const charge)
{
    if (charge == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (isnan(charge->charge_mass_kg) ||
        isnan(charge->condensed_phase_density_kg_per_m3))
    {
        return BBTC_STATUS_NAN_INPUT;
    }

    if (isinf(charge->charge_mass_kg) ||
        isinf(charge->condensed_phase_density_kg_per_m3))
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

    if (isnan(charge->charge_mass_kg) ||
        isnan(charge->condensed_phase_density_kg_per_m3))
    {
        return BBTC_STATUS_NAN_INPUT;
    }

    if (isinf(charge->charge_mass_kg) ||
        isinf(charge->condensed_phase_density_kg_per_m3))
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

    if (isnan(charge->charge_mass_kg) ||
        isnan(charge->condensed_phase_density_kg_per_m3))
    {
        return BBTC_STATUS_NAN_INPUT;
    }

    if (isinf(charge->charge_mass_kg) ||
        isinf(charge->condensed_phase_density_kg_per_m3))
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
