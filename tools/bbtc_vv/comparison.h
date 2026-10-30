/**
 * @file comparison.h
 * @brief Complete VV0.1-B1 comparison API and data model (developer-only).
 *
 * @details
 * The types and three typed `bbtc_vv_compare_*` entry points implement the
 * accepted numerical contract in VVUQ_COMPARISON_CORE_CONTRACT.md, version
 * 0.1.0. The exact dyadic decision and independently rounded diagnostic are
 * separate obligations; a rounded metric never determines a PASS/FAIL.
 *
 * `comparison_internal.h` remains a developer-private preparation adapter.
 * Its decision-only functions are not substitutes for the full API.
 *
 * No type or function here belongs to BBTC's installed public ABI. Consumers
 * must not install this header as a production BBTC include file. The final
 * source-level acceptance decision remains subject to the A01-A14 audit.
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
/**
 * @brief Compare native-float inputs under the accepted B1 policy.
 *
 * @param reference Native reference, interpreted as its exact binary value.
 * @param observation Native observation, interpreted the same way.
 * @param policy Required method and native tolerance (no implicit defaults).
 * @param out_result Required result; all fields are cleared before any other
 *                   argument validation. No write occurs when it is null.
 * @return Execution status, separate from the comparison PASS/FAIL finding.
 *
 * EXACT returns NOT_COMPUTED and +0. ABSOLUTE computes |o-r| in native
 * units; REFERENCE_RELATIVE computes |o-r|/|r| (reference must be nonzero).
 * Both metrics are rounded nearest/ties-even from *exact rational arithmetic*
 * and receive an independent exact/rounded/out-of-range classification.
 * All successful findings are mathematically exact PASS/FAIL, regardless of
 * diagnostic representability or the caller's rounding mode. Any failure
 * leaves finding=NOT_EVALUATED, metric_state=NOT_COMPUTED, metric=+0.
 *
 * Supported caller rounding mode and sticky flags are unchanged on return.
 * Unmasked traps, fast-math, and unqualified numerical formats are excluded.
 *
 * @warning Numerical comparison is NOT a physical safety or model verdict.
 */
bbtc_vv_operation_status_e bbtc_vv_compare_float(
    float reference,
    float observation,
    const bbtc_vv_policy_float_t* policy,
    bbtc_vv_result_float_t* out_result
);

/** @brief Native binary64 counterpart; does not narrow through float. */
bbtc_vv_operation_status_e bbtc_vv_compare_double(
    double reference,
    double observation,
    const bbtc_vv_policy_double_t* policy,
    bbtc_vv_result_double_t* out_result
);

/** @brief Native extended long-double counterpart; never narrows to double. */
bbtc_vv_operation_status_e bbtc_vv_compare_long_double(
    long double reference,
    long double observation,
    const bbtc_vv_policy_long_double_t* policy,
    bbtc_vv_result_long_double_t* out_result
);


#ifdef __cplusplus
    }   /* extern "C" */
#endif
#endif /* BBTC_VV_COMPARISON_H */
