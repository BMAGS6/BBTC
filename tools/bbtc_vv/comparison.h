/**
 * @file comparison.h
 * @brief Accepted VV0.1-B1 comparison data model (developer-only).
 *
 * @details
 * This header freezes the internal comparison vocabulary established by
 * VVUQ_COMPARISON_CORE_CONTRACT.md, version 0.1.0. It deliberately does NOT
 * declare the final bbtc_vv_compare_* entry points during B1b.3b: those
 * entry points promise correctly rounded diagnostic metrics for Absolute and
 * Reference-relative comparisons, which are the subject of B1b.3c.
 *
 * Until that mathematical implementation is qualified, the only callable
 * B1b.3b interface is comparison_internal.h. Its decision preparation
 * functions are NOT substitutes for the promised public-facing developer API.
 *
 * No type here belongs to BBTC's installed public ABI. Consumers must not
 * install this header as a production BBTC include file.
 *
 * @warning A comparison result is a numerical finding, not a physical safety
 *          finding or scientific model-validation verdict.
 */
#ifndef BBTC_VV_COMPARISON_H
#define BBTC_VV_COMPARISON_H

#include <stdint.h>

#ifdef __cplusplus
    extern "C"{
#endif

/**
 * @brief Determines the meaning and units of a comparison's tolerance.
 *
 * A policy selects exactly one method; no fallback from relative to absolute
 * tolerance is permitted. Exact mode requires a numerical zero tolerance.
 */
typedef enum bbtc_vv_comparison_method : uint8_t
{
    BBTC_VV_COMPARISON_EXACT              = 1,
    BBTC_VV_COMPARISON_ABSOLUTE           = 2,
    BBTC_VV_COMPARISON_REFERENCE_RELATIVE = 3
}
bbtc_vv_comparison_method_e;


/**
 * @brief API execution status, independent of the scientific comparison.
 *
 * Only SUCCESS authorizes consumption of a full comparison result. Failure
 * output is always cleared to the canonical all-zero representation.
 */
typedef enum bbtc_vv_operation_status : uint8_t
{
    BBTC_VV_STATUS_SUCCESS                    = 0,
    BBTC_VV_STATUS_INVALID_ARGUMENT           = 1,
    BBTC_VV_STATUS_NAN_INPUT                  = 2,
    BBTC_VV_STATUS_NONFINITE_INPUT            = 3,
    BBTC_VV_STATUS_OUTSIDE_DOMAIN             = 4,
    BBTC_VV_STATUS_UNSUPPORTED_FP_ENVIRONMENT = 5,
    BBTC_VV_STATUS_NUMERICAL_FAILURE          = 6
}
bbtc_vv_operation_status_e;


/**
 * @brief Mathematical finding, separate from operation execution status.
 *
 * For every admissible B1 comparison on a supported platform, the eventual
 * public API MUST return PASS or FAIL, never INDETERMINATE. The latter value
 * is reserved for explicitly contracted future comparison methods.
 */
typedef enum bbtc_vv_finding : uint8_t
{
    BBTC_VV_FINDING_NOT_EVALUATED = 0,
    BBTC_VV_FINDING_PASS          = 1,
    BBTC_VV_FINDING_FAIL          = 2,
    BBTC_VV_FINDING_INDETERMINATE = 3
}
bbtc_vv_finding_e;


/**
 * @brief Representability and rounding fidelity of a scalar diagnostic.
 *
 * The diagnostic is NEVER the authority for a mathematical PASS/FAIL. In
 * B1b.3b these enumerators are data-model definitions only. B1b.3c must
 * compute the exact mathematical metric before selecting one of these states.
 */
typedef enum bbtc_vv_metric_state : uint8_t
{
    /** No diagnostic was computed (Exact mode or operation failure). */
    BBTC_VV_METRIC_NOT_COMPUTED = 0,

    /** Exact mathematical metric is representable, including exact +0. */
    BBTC_VV_METRIC_FINITE_EXACT = 1,

    /** Correct nearest-even nonzero finite approximation to an inexact value. */
    BBTC_VV_METRIC_FINITE_ROUNDED = 2,

    /** Mathematical magnitude exceeds the native type's finite maximum. */
    BBTC_VV_METRIC_EXCEEDS_RANGE = 3,

    /** Strictly positive mathematical magnitude rounds to numerical zero. */
    BBTC_VV_METRIC_ROUNDED_TO_ZERO = 4,

    /** Unable to produce a defensible metric despite an exact finding. */
    BBTC_VV_METRIC_UNRESOLVED = 5
}
bbtc_vv_metric_state_e;


/** @brief Native-float method and tolerance, with no implicit defaults. */
typedef struct bbtc_vv_policy_float_t
{
    bbtc_vv_comparison_method_e method;
    float tolerance;
}
bbtc_vv_policy_float_t;


/** @brief Native-double method and tolerance, with no precision narrowing. */
typedef struct bbtc_vv_policy_double_t
{
    bbtc_vv_comparison_method_e method;
    double tolerance;
}
bbtc_vv_policy_double_t;


/** @brief Native-long-double policy; long double is never routed through double. */
typedef struct bbtc_vv_policy_long_double_t
{
    bbtc_vv_comparison_method_e method;
    long double tolerance;
}
bbtc_vv_policy_long_double_t;


/**
 * @brief Final native-float result shape, reserved for the complete B1b.3 API.
 *
 * On operation failure, every field will be zero (NOT_EVALUATED,
 * NOT_COMPUTED, and +0 respectively). On success in Absolute or relative
 * mode, `metric` must be correctly rounded, not merely computed by fabsf.
 */
typedef struct bbtc_vv_result_float_t
{
    bbtc_vv_finding_e finding;
    bbtc_vv_metric_state_e metric_state;
    float metric;
} bbtc_vv_result_float_t;

/** @brief Result shape for native-double comparisons. */
typedef struct bbtc_vv_result_double_t
{
    bbtc_vv_finding_e finding;
    bbtc_vv_metric_state_e metric_state;
    double metric;
} bbtc_vv_result_double_t;

/** @brief Result shape for native-long-double comparisons. */
typedef struct bbtc_vv_result_long_double_t
{
    bbtc_vv_finding_e finding;
    bbtc_vv_metric_state_e metric_state;
    long double metric;
} bbtc_vv_result_long_double_t;


#ifdef __cplusplus
    }   /* extern "C" */
#endif
#endif /* BBTC_VV_COMPARISON_H */
