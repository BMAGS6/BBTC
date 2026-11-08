/**
 * @file virial_adapter.h
 * @brief Developer-only observation adapter for BBTC's virial temperature law.
 *
 * @details
 * VV0.1-B2b introduces a deliberately narrow boundary between existing
 * production evaluator calls and future independent reference comparisons.
 * This adapter does not implement a polynomial, calculate an expectation,
 * choose a tolerance, invoke the B1 comparator, or produce an evidentiary
 * verdict. It forwards each call to the matching public BBTC entry point.
 *
 * All three output quantities share the production evaluator's validity:
 * they are meaningful for numerical comparisons only when its status is
 * BBTC_STATUS_SUCCESS. A failure-cleared zero is NOT a valid observation.
 *
 * @warning This header is a developer-tool interface, not an installed BBTC
 *          public API. These functions do not validate model accuracy,
 *          ammunition, firearms, materials, or physical safety.
 */
#ifndef BBTC_VV_VIRIAL_ADAPTER_H
#define BBTC_VV_VIRIAL_ADAPTER_H

#include <stdbool.h>

#include <bbtc/internal_ballistics/first_order_virial_gas_model.h>
#include <bbtc/status.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Obtain a native-float observation from the production BBTC API.
 *
 * @details
 * The complete law, including its coefficient array, is borrowed solely for
 * this call. The adapter never retains, changes, copies, or frees the array.
 * The native float output is passed directly to the public evaluator, without
 * conversion to double or long double.
 *
 * A nonnull validity pointer is set false before invoking BBTC and set true
 * only if the EXACT returned production status is BBTC_STATUS_SUCCESS. The
 * validity pointer may be null when the caller needs only the raw status.
 *
 * When out_terms is nonnull, BBTC's documented output clearing and field
 * values are exposed unchanged. A null out_terms is deliberately forwarded
 * to BBTC so the public API's own required-output status can be tested.
 * out_terms_valid, if provided, must not overlap the out_terms object.
 *
 * @param law Caller-owned temperature law; may be null for a status test.
 * @param temperature_k Native float absolute temperature, in kelvins.
 * @param out_terms Caller-owned terms storage; may be null for a status test.
 * @param out_terms_valid Optional caller-owned output validity flag.
 *
 * @return Raw bbtc_status_e returned by the production evaluator, unchanged.
 *
 * @note This call does not establish scientific reference admissibility or
 *       select a numerical comparison policy.
 */
bbtc_status_e bbtc_vv_virial_observe_float(
    const bbtc_ib_first_order_virial_temperature_law_float_t* law,
    float temperature_k,
    bbtc_ib_first_order_virial_temperature_terms_float_t* out_terms,
    bool* out_terms_valid
);

/**
 * @brief Native-double counterpart of bbtc_vv_virial_observe_float().
 *
 * @details
 * Preserves the double law, evaluation temperature, and all three result
 * fields without converting through a different scalar family. The borrowed
 * pointer, output-clearing, validity and raw-status contracts are identical.
 *
 * @param law Caller-owned double temperature law.
 * @param temperature_k Native double temperature, in kelvins.
 * @param out_terms Caller-owned double output; may be null.
 * @param out_terms_valid Optional validity flag; must not overlap out_terms.
 * @return The exact production bbtc_status_e for this call.
 */
bbtc_status_e bbtc_vv_virial_observe_double(
    const bbtc_ib_first_order_virial_temperature_law_double_t* law,
    double temperature_k,
    bbtc_ib_first_order_virial_temperature_terms_double_t* out_terms,
    bool* out_terms_valid
);

/**
 * @brief Native-long-double counterpart of bbtc_vv_virial_observe_float().
 *
 * @details
 * Dispatches directly to the public long-double entry point. It never
 * narrows the input, borrowed coefficients, or result through double.
 * The borrowed pointer, output-clearing, validity and raw-status contracts
 * are the same as those documented for the float entry point.
 *
 * @param law Caller-owned long-double temperature law.
 * @param temperature_k Native long-double temperature, in kelvins.
 * @param out_terms Caller-owned long-double output; may be null.
 * @param out_terms_valid Optional validity flag; must not overlap out_terms.
 * @return The exact production bbtc_status_e for this call.
 */
bbtc_status_e bbtc_vv_virial_observe_long_double(
    const bbtc_ib_first_order_virial_temperature_law_long_double_t* law,
    long double temperature_k,
    bbtc_ib_first_order_virial_temperature_terms_long_double_t* out_terms,
    bool* out_terms_valid
);

#ifdef __cplusplus
} /* extern "C" */
#endif
#endif /* BBTC_VV_VIRIAL_ADAPTER_H */
