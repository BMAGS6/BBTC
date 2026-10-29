/**
 * @file exact_round.h
 * @brief Deterministic exact-rational to native-binary rounding (B1b.3c-2).
 *
 * @details
 * Converts an exact, nonnegative mathematical magnitude from exact_metric.h
 * to one of the three separately qualified native floating-point families.
 * The input rational is NEVER evaluated by an intermediate native division.
 * All representability, midway, and overflow decisions are integer-exact.
 *
 * This is a developer-internal conversion interface, NOT a complete VVUQ
 * comparison. In particular it does not compute the exact PASS/FAIL finding,
 * and cannot be substituted for the forthcoming bbtc_vv_compare_* API.
 *
 * The output policy is the accepted B1b.3a round-to-nearest/ties-to-even,
 * independent of the caller's rounding mode. For normal returns in a
 * qualified environment, the caller's rounding mode and sticky exception
 * flags are restored using feholdexcept()/fesetenv(). Unmasked FP traps,
 * fast-math, noncanonical x87 values, and unqualified formats are excluded.
 */
#ifndef BBTC_VV_EXACT_ROUND_H
#define BBTC_VV_EXACT_ROUND_H

#include "exact_metric.h"

/**
 * @brief Round the exact rational metric into native float without narrowing.
 * @param exact Required exact metric, constructed by bbtc_vv_metric_exact_*.
 * @param out_state Required metric fidelity classification; zeroed on error.
 * @param out_value Required native metric; positive zero on error or sentinel.
 * @return SUCCESS if rounding/classification was established, otherwise a
 *         classified failure and canonical cleared outputs.
 *
 * A successful return with EXCEEDS_RANGE, ROUNDED_TO_ZERO, or UNRESOLVED is
 * possible by contract, but UNRESOLVED is never emitted as an ordinary
 * rounding shortcut by this implementation. Mathematically overflowing values
 * use +0, not infinity; positive underflows rounding to zero likewise use +0.
 */
bbtc_vv_operation_status_e bbtc_vv_metric_round_float(
    const bbtc_vv_exact_metric_t* exact,
    bbtc_vv_metric_state_e* out_state,
    float* out_value
);

/** @brief Native double conversion, preserving binary64 precision. */
bbtc_vv_operation_status_e bbtc_vv_metric_round_double(
    const bbtc_vv_exact_metric_t* exact,
    bbtc_vv_metric_state_e* out_state,
    double* out_value
);

/** @brief Native long double conversion, preserving qualified x87 precision. */
bbtc_vv_operation_status_e bbtc_vv_metric_round_long_double(
    const bbtc_vv_exact_metric_t* exact,
    bbtc_vv_metric_state_e* out_state,
    long double* out_value
);

#endif /* BBTC_VV_EXACT_ROUND_H */
