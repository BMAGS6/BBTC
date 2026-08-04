/**
 * @file
 * @brief Validation and derived volumes for composed loading states.
 */

#include "bbtc/internal_ballistics/loading_state.h"

#include <stddef.h>
#include <math.h>

bbtc_status_e
bbtc_ib_loading_state_evaluate_float(const bbtc_ib_loading_state_float_t* const   loading_state,
                                     bbtc_ib_loading_state_volumes_float_t* const out_volumes)
{
    bbtc_status_e status;
    float         condensed_volume;
    float         free_gas_volume;

    if (out_volumes == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    *out_volumes = (bbtc_ib_loading_state_volumes_float_t){0};

    if (loading_state == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    status = bbtc_ib_geometry_validate_float(&loading_state->geometry);
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    status = bbtc_ib_projectile_validate_float(&loading_state->projectile);
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    status = bbtc_ib_propellant_charge_validate_float(&loading_state->propellant_charge);
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    condensed_volume = loading_state->propellant_charge.charge_mass_kg
                     / loading_state->propellant_charge.condensed_phase_density_kg_per_m3;

    if (!isfinite(condensed_volume) || condensed_volume <= 0.0f)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    if (condensed_volume >= loading_state->geometry.initial_behind_projectile_volume_m3)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    free_gas_volume = loading_state->geometry.initial_behind_projectile_volume_m3
                    - condensed_volume;

    if (!isfinite(free_gas_volume) || free_gas_volume <= 0.0f)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    out_volumes->condensed_propellant_volume_m3 = condensed_volume;
    out_volumes->initial_free_gas_volume_m3     = free_gas_volume;

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e
bbtc_ib_loading_state_evaluate_double(const bbtc_ib_loading_state_double_t* const   loading_state,
                                      bbtc_ib_loading_state_volumes_double_t* const out_volumes)
{
    bbtc_status_e status;
    double         condensed_volume;
    double         free_gas_volume;

    if (out_volumes == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    *out_volumes = (bbtc_ib_loading_state_volumes_double_t){0};

    if (loading_state == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    status = bbtc_ib_geometry_validate_double(&loading_state->geometry);
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    status = bbtc_ib_projectile_validate_double(&loading_state->projectile);
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    status = bbtc_ib_propellant_charge_validate_double(
        &loading_state->propellant_charge);
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    condensed_volume = loading_state->propellant_charge.charge_mass_kg
                     / loading_state->propellant_charge.condensed_phase_density_kg_per_m3;

    if (!isfinite(condensed_volume) || condensed_volume <= 0.0)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    if (condensed_volume >= loading_state->geometry.initial_behind_projectile_volume_m3)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    free_gas_volume = loading_state->geometry.initial_behind_projectile_volume_m3
                    - condensed_volume;

    if (!isfinite(free_gas_volume) || free_gas_volume <= 0.0)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    out_volumes->condensed_propellant_volume_m3 = condensed_volume;
    out_volumes->initial_free_gas_volume_m3     = free_gas_volume;

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e
bbtc_ib_loading_state_evaluate_long_double(const bbtc_ib_loading_state_long_double_t* const   loading_state,
                                           bbtc_ib_loading_state_volumes_long_double_t* const out_volumes)
{
    bbtc_status_e status;
    long double   condensed_volume;
    long double   free_gas_volume;

    if (out_volumes == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    *out_volumes = (bbtc_ib_loading_state_volumes_long_double_t){0};

    if (loading_state == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    status = bbtc_ib_geometry_validate_long_double(&loading_state->geometry);
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    status = bbtc_ib_projectile_validate_long_double(&loading_state->projectile);
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    status = bbtc_ib_propellant_charge_validate_long_double(&loading_state->propellant_charge);
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    condensed_volume = loading_state->propellant_charge.charge_mass_kg
                     / loading_state->propellant_charge.condensed_phase_density_kg_per_m3;

    if (!isfinite(condensed_volume) || condensed_volume <= 0.0L)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    if (condensed_volume >= loading_state->geometry.initial_behind_projectile_volume_m3)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    free_gas_volume = loading_state->geometry.initial_behind_projectile_volume_m3
                    - condensed_volume;

    if (!isfinite(free_gas_volume) || free_gas_volume <= 0.0L)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    out_volumes->condensed_propellant_volume_m3 = condensed_volume;
    out_volumes->initial_free_gas_volume_m3 = free_gas_volume;

    return BBTC_STATUS_SUCCESS;
}
