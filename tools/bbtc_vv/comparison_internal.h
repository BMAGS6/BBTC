/**
 * @file comparison_internal.h
 * @brief B1b.3b-only, non-production comparison decision preparation layer.
 *
 * @details
 * This interface exists to independently qualify the exact decision, status
 * translation, structural validation precedence, and floating-environment
 * handling required by the *future* complete comparison API.
 *
 * Deliberate B1b.3b boundary:
 *   - These operations return an operation status and an exact finding.
 *   - They DO NOT return a numerical metric or a metric-state classification.
 *   - Callers must not present these functions as the accepted compare_* API.
 *
 * The complete B1 API will first be exported after B1b.3c has implemented
 * correctly rounded scalar diagnostics for every applicable method.
 */
#ifndef BBTC_VV_COMPARISON_INTERNAL_H
#define BBTC_VV_COMPARISON_INTERNAL_H

#include "comparison.h"

/**
 * @brief Validate policy and decide an exact mathematical comparison (float).
 *
 * @param reference Exact mathematical value of the native `float` input.
 * @param observation Observed native `float` input.
 * @param policy Non-null selection of exact/absolute/reference-relative mode.
 * @param out_finding Required output, set to NOT_EVALUATED before examination.
 * @return SUCCESS with PASS/FAIL, or a classified failure with cleared output.
 *
 * Precedence follows B1a section 5, except capability qualification may
 * precede NaN/infinity classification where required for safe inspection.
 * The dyadic kernel remains the sole authority for the finding.
 */
bbtc_vv_operation_status_e bbtc_vv_prepare_float(
    float reference,
    float observation,
    const bbtc_vv_policy_float_t* policy,
    bbtc_vv_finding_e* out_finding
);

/** @brief Native-double decision preparation; no float/double narrowing. */
bbtc_vv_operation_status_e bbtc_vv_prepare_double(
    double reference,
    double observation,
    const bbtc_vv_policy_double_t* policy,
    bbtc_vv_finding_e* out_finding
);

/** @brief Native-long-double decision preparation; no double narrowing. */
bbtc_vv_operation_status_e bbtc_vv_prepare_long_double(
    long double reference,
    long double observation,
    const bbtc_vv_policy_long_double_t* policy,
    bbtc_vv_finding_e* out_finding
);

#endif /* BBTC_VV_COMPARISON_INTERNAL_H */
