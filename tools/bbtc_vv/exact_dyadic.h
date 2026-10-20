/**
 * @file exact_dyadic.h
 * @brief Private, exact dyadic inequality decision kernel for BBTC VVUQ.
 *
 * @details
 * This developer-only interface tests mathematical relations between the
 * **exact real values represented** by the supplied C floating-point objects.
 * It is NOT the public B1 comparison/metric interface. The latter will wrap
 * this kernel and compute separately classified diagnostic metrics.
 *
 * The functions do not link against or include any production BBTC code.
 * Supported binary environments are intentionally restricted; unsupported
 * environments fail closed rather than switching to rounded comparisons.
 *
 * On supported environments and normal returns, the caller's rounding mode
 * and sticky floating-point exception flags are preserved. Unmasked traps,
 * noncanonical floating representations, and unsafe fast-math builds are
 * outside the initial certified execution domain.
 */
#ifndef BBTC_VV_EXACT_DYADIC_H
#define BBTC_VV_EXACT_DYADIC_H

#include <stdbool.h>
#include <stdint.h>

/** @brief Selects one mathematically specified comparison relation. */
typedef enum bbtc_vv_dyadic_method : uint8_t
{
    /** Test reference == observation; tolerance must be zero. */
    BBTC_VV_DYADIC_EXACT = 1,

    /** Test |observation - reference| <= tolerance. */
    BBTC_VV_DYADIC_ABSOLUTE = 2,

    /** Test |observation - reference| <= tolerance * |reference|. */
    BBTC_VV_DYADIC_REFERENCE_RELATIVE = 3
} bbtc_vv_dyadic_method_e;

/**
 * @brief Kernel execution result; NOT a physical validation verdict.
 *
 * Only SUCCESS makes `out_pass` meaningful. All other statuses leave it false.
 */
typedef enum bbtc_vv_dyadic_status : uint8_t
{
    BBTC_VV_DYADIC_SUCCESS = 0,
    BBTC_VV_DYADIC_INVALID_ARGUMENT = 1,
    BBTC_VV_DYADIC_NAN_INPUT = 2,
    BBTC_VV_DYADIC_NONFINITE_INPUT = 3,
    BBTC_VV_DYADIC_OUTSIDE_DOMAIN = 4,
    BBTC_VV_DYADIC_UNSUPPORTED_ENVIRONMENT = 5,
    BBTC_VV_DYADIC_INTERNAL_FAILURE = 6
} bbtc_vv_dyadic_status_e;

/**
 * @brief Decide an exact relation for native `float` values.
 *
 * @param method Relation to evaluate.
 * @param reference Exact represented reference value.
 * @param observation Exact represented observation value.
 * @param tolerance Exact represented tolerance (zero for Exact mode).
 * @param out_pass Required output; cleared before processing.
 * @return SUCCESS with mathematically exact Boolean; otherwise classified
 *         failure with `*out_pass == false`.
 */
bbtc_vv_dyadic_status_e
bbtc_vv_dyadic_compare_float(
    bbtc_vv_dyadic_method_e method,
    float reference,
    float observation,
    float tolerance,
    bool* out_pass
);

/** @brief Native `double` counterpart of bbtc_vv_dyadic_compare_float(). */
bbtc_vv_dyadic_status_e
bbtc_vv_dyadic_compare_double(
    bbtc_vv_dyadic_method_e method,
    double reference,
    double observation,
    double tolerance,
    bool* out_pass
);

/** @brief Native `long double` counterpart; never narrowed to `double`. */
bbtc_vv_dyadic_status_e
bbtc_vv_dyadic_compare_long_double(
    bbtc_vv_dyadic_method_e method,
    long double reference,
    long double observation,
    long double tolerance,
    bool* out_pass
);

#endif /* BBTC_VV_EXACT_DYADIC_H */
