/**
 * @file exact_metric.h
 * @brief Private, allocation-free *exact* metric arithmetic for VV0.1-B1b.3c-1.
 *
 * @details
 * This header describes an exact, NON-ROUNDED mathematical magnitude. It is
 * deliberately distinct from comparison.h's native floating-point metric:
 * B1b.3c-2 must still implement independently verified correctly rounded
 * rational-to-native conversion and classify its representability.
 *
 * For every successful operation the exact mathematical result is
 *
 *     (sum_{i=0}^{used-1} word[i] * 2^(64*i)) * 2^numerator_exponent
 *     ------------------------------------------------------------- .
 *                    denominator * 2^denominator_exponent
 *
 * This form is intentionally NOT necessarily reduced. It is unambiguous,
 * uses no floating-point arithmetic to construct the difference, and permits
 * exact numerator/denominator comparisons in the subsequent rounding stage.
 * Zero uses used=0, numerator_exponent=0, with a nonzero denominator.
 *
 * This is a developer-only VVUQ interface; do not install it as a production
 * BBTC header or treat a metric as scientific model validation.
 */
#ifndef BBTC_VV_EXACT_METRIC_H
#define BBTC_VV_EXACT_METRIC_H

#include "comparison.h"

#include <stdint.h>

/**
 * @brief Conservative ceiling for aligned input differences on certified
 * binary32/binary64/x87 extended scalar families (see proof document).
 */
#define BBTC_VV_METRIC_LIMBS 520

/**
 * @brief Signed-free exact rational representation with a dyadic denominator.
 *
 * @note `used` bounds every meaningful word; all remaining words are zero.
 *       `denominator` is nonzero on success. Callers MUST NOT infer rounding
 *       state, representability, or numerical validity from this structure.
 */
typedef struct bbtc_vv_exact_metric_t
{
    uint64_t word[BBTC_VV_METRIC_LIMBS];
    uint32_t used;
    int32_t numerator_exponent;
    uint64_t denominator;
    int32_t denominator_exponent;
} bbtc_vv_exact_metric_t;

/**
 * @brief Construct an exact absolute or reference-relative error (float).
 *
 * @param method ABSOLUTE or REFERENCE_RELATIVE; EXACT is not a metric mode.
 * @param reference Native scalar value, interpreted as its exact dyadic value.
 * @param observation Native scalar value, interpreted exactly.
 * @param out_metric Required output, cleared before all other validation.
 * @return Accepted B1 status; on failure the output is entirely zero.
 *
 * Valid numeric input must be finite. Relative mode requires reference != 0.
 * The caller's rounding mode and existing sticky exception flags are preserved
 * on all normal supported returns. Unmasked traps, noncanonical formats and
 * fast-math builds remain outside the qualified execution domain.
 */
bbtc_vv_operation_status_e bbtc_vv_metric_exact_float(
    bbtc_vv_comparison_method_e method,
    float reference,
    float observation,
    bbtc_vv_exact_metric_t* out_metric
);

/** @brief Native `double` variant, without float or long-double coercion. */
bbtc_vv_operation_status_e bbtc_vv_metric_exact_double(
    bbtc_vv_comparison_method_e method,
    double reference,
    double observation,
    bbtc_vv_exact_metric_t* out_metric
);

/** @brief Native `long double` variant; never narrowed to `double`. */
bbtc_vv_operation_status_e bbtc_vv_metric_exact_long_double(
    bbtc_vv_comparison_method_e method,
    long double reference,
    long double observation,
    bbtc_vv_exact_metric_t* out_metric
);

#endif /* BBTC_VV_EXACT_METRIC_H */
