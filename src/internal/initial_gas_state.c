/**
 * @file
 * @brief Validation for precision-qualified initial gas-state records.
 */

#include "bbtc/internal_ballistics/initial_gas_state.h"

#include <stddef.h>
#include <math.h>

bbtc_status_e
bbtc_ib_initial_gas_state_validate_float(const bbtc_ib_initial_gas_state_float_t* const state)
{
    if (state == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (isnan(state->absolute_pressure_pa) ||
        isnan(state->temperature_k))
    {
        return BBTC_STATUS_NAN_INPUT;
    }

    if (isinf(state->absolute_pressure_pa) ||
        isinf(state->temperature_k))
    {
        return BBTC_STATUS_NONFINITE_INPUT;
    }

    if (state->absolute_pressure_pa <= 0.0f ||
        state->temperature_k        <= 0.0f)
    {
        return BBTC_STATUS_OUTSIDE_DOMAIN;
    }

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e
bbtc_ib_initial_gas_state_validate_double(const bbtc_ib_initial_gas_state_double_t* const state)
{
    if (state == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (isnan(state->absolute_pressure_pa) ||
        isnan(state->temperature_k))
    {
        return BBTC_STATUS_NAN_INPUT;
    }

    if (isinf(state->absolute_pressure_pa) ||
        isinf(state->temperature_k))
    {
        return BBTC_STATUS_NONFINITE_INPUT;
    }

    if (state->absolute_pressure_pa <= 0.0 ||
        state->temperature_k        <= 0.0)
    {
        return BBTC_STATUS_OUTSIDE_DOMAIN;
    }

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e
bbtc_ib_initial_gas_state_validate_long_double(const bbtc_ib_initial_gas_state_long_double_t* const state)
{
    if (state == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (isnan(state->absolute_pressure_pa) ||
        isnan(state->temperature_k))
    {
        return BBTC_STATUS_NAN_INPUT;
    }

    if (isinf(state->absolute_pressure_pa) ||
        isinf(state->temperature_k))
    {
        return BBTC_STATUS_NONFINITE_INPUT;
    }

    if (state->absolute_pressure_pa <= 0.0L ||
        state->temperature_k        <= 0.0L)
    {
        return BBTC_STATUS_OUTSIDE_DOMAIN;
    }

    return BBTC_STATUS_SUCCESS;
}
