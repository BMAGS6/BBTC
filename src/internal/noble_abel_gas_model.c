/**
 * @file
 * @brief Validation for precision-qualified Noble-Abel gas-model records.
 */

#include "bbtc/internal_ballistics/noble_abel_gas_model.h"

#include <stddef.h>
#include <math.h>

bbtc_status_e
bbtc_ib_noble_abel_gas_model_validate_float(const bbtc_ib_noble_abel_gas_model_float_t* const model)
{
    if (model == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (!isfinite(model->specific_gas_constant_j_per_kg_k)         ||
        !isfinite(model->constant_volume_specific_heat_j_per_kg_k) ||
        !isfinite(model->covolume_m3_per_kg))
    {
        return BBTC_STATUS_NONFINITE_INPUT;
    }

    if (model->specific_gas_constant_j_per_kg_k         <= 0.0f ||
        model->constant_volume_specific_heat_j_per_kg_k <= 0.0f ||
        model->covolume_m3_per_kg                       <  0.0f)
    {
        return BBTC_STATUS_OUTSIDE_DOMAIN;
    }

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e
bbtc_ib_noble_abel_gas_model_validate_double(const bbtc_ib_noble_abel_gas_model_double_t* const model)
{
    if (model == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (!isfinite(model->specific_gas_constant_j_per_kg_k)         ||
        !isfinite(model->constant_volume_specific_heat_j_per_kg_k) ||
        !isfinite(model->covolume_m3_per_kg))
    {
        return BBTC_STATUS_NONFINITE_INPUT;
    }

    if (model->specific_gas_constant_j_per_kg_k         <= 0.0 ||
        model->constant_volume_specific_heat_j_per_kg_k <= 0.0 ||
        model->covolume_m3_per_kg                       <  0.0)
    {
        return BBTC_STATUS_OUTSIDE_DOMAIN;
    }

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e
bbtc_ib_noble_abel_gas_model_validate_long_double(const bbtc_ib_noble_abel_gas_model_long_double_t* const model)
{
    if (model == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (!isfinite(model->specific_gas_constant_j_per_kg_k)         ||
        !isfinite(model->constant_volume_specific_heat_j_per_kg_k) ||
        !isfinite(model->covolume_m3_per_kg))
    {
        return BBTC_STATUS_NONFINITE_INPUT;
    }

    if (model->specific_gas_constant_j_per_kg_k         <= 0.0L ||
        model->constant_volume_specific_heat_j_per_kg_k <= 0.0L ||
        model->covolume_m3_per_kg                       <  0.0L)
    {
        return BBTC_STATUS_OUTSIDE_DOMAIN;
    }

    return BBTC_STATUS_SUCCESS;
}
