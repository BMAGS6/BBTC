/**
 * @file virial_adapter.c
 * @brief Thin, native-family invocation adapter for VV0.1-B2b.
 *
 * @details
 * These functions intentionally do almost nothing: their purpose is to
 * obtain unmodified observations from BBTC's existing public API. The
 * caller's future runner is responsible for reference identity, units,
 * expected values, B1 comparison, missing-case accounting, and evidence.
 *
 * The raw BBTC status is returned directly. No adapter-level error enum or
 * surrogate result is introduced, and cleared terms on failure must not be
 * treated as a valid result. This translation unit has no B1 dependency.
 */
#include <stddef.h>

#include "virial_adapter.h"

/**
 * @brief Forward one float evaluation without transforming its inputs.
 * @see bbtc_vv_virial_observe_float
 */
bbtc_status_e bbtc_vv_virial_observe_float(
    const bbtc_ib_first_order_virial_temperature_law_float_t* law,
    float temperature_k,
    bbtc_ib_first_order_virial_temperature_terms_float_t* out_terms,
    bool* out_terms_valid
)
{
    if (out_terms_valid != NULL)
        *out_terms_valid = false;

    /* Preserve the production null-output check and validation precedence. */
    const bbtc_status_e status =
        bbtc_ib_first_order_virial_temperature_law_evaluate_float(law, temperature_k, out_terms);

    if (out_terms_valid != NULL)
        *out_terms_valid = (status == BBTC_STATUS_SUCCESS);

    return status;
}


/**
 * @brief Forward one double evaluation without transforming its inputs.
 * @see bbtc_vv_virial_observe_double
 */
bbtc_status_e bbtc_vv_virial_observe_double(
    const bbtc_ib_first_order_virial_temperature_law_double_t* law,
    double temperature_k,
    bbtc_ib_first_order_virial_temperature_terms_double_t* out_terms,
    bool* out_terms_valid
)
{
    if (out_terms_valid != NULL)
        *out_terms_valid = false;

    const bbtc_status_e status =
        bbtc_ib_first_order_virial_temperature_law_evaluate_double(law, temperature_k, out_terms);

    if (out_terms_valid != NULL)
        *out_terms_valid = (status == BBTC_STATUS_SUCCESS);

    return status;
}


/**
 * @brief Forward one long-double evaluation without narrowing its precision.
 * @see bbtc_vv_virial_observe_long_double
 */
bbtc_status_e bbtc_vv_virial_observe_long_double(
    const bbtc_ib_first_order_virial_temperature_law_long_double_t* law,
    long double temperature_k,
    bbtc_ib_first_order_virial_temperature_terms_long_double_t* out_terms,
    bool* out_terms_valid
)
{
    if (out_terms_valid != NULL)
        *out_terms_valid = false;

    const bbtc_status_e status =
        bbtc_ib_first_order_virial_temperature_law_evaluate_long_double(law, temperature_k, out_terms);

    if (out_terms_valid != NULL)
        *out_terms_valid = (status == BBTC_STATUS_SUCCESS);

    return status;
}
