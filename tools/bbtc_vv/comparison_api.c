/**
 * @file comparison_api.c
 * @brief Complete developer-only B1 numerical comparison API.
 *
 * @details
 * The accepted B1 comparison contract requires an exact mathematical
 * PASS/FAIL and a separately classified, correctly rounded scalar metric.
 * This implementation delegates to three independently qualified kernels:
 * the exact dyadic decision, exact rational magnitude and nearest-even
 * native scalar rounding. It never uses the metric to decide PASS or FAIL.
 *
 * The public-facing result is published only after successful preservation
 * of the entire caller floating-point environment. All component calls are
 * also guarded individually; nested fenv save/restore is intentional.
 *
 * This file is a DEVELOPER-ONLY VVUQ module. It does not modify the BBTC
 * production API or claim physical/experimental model validation.
 */
#include "comparison.h"
#include "comparison_internal.h"
#include "exact_metric.h"
#include "exact_round.h"

#include <fenv.h>
#include <stddef.h>

#if defined(__FAST_MATH__)
    #error "BBTC VVUQ complete comparison API forbids -ffast-math"
#endif
#if defined(__FINITE_MATH_ONLY__)
    #if __FINITE_MATH_ONLY__ != 0
        #error "BBTC VVUQ comparisons require finite-math-only disabled"
    #endif
#endif
#if defined(__clang__)
    #pragma STDC FENV_ACCESS ON
#endif

/**
 * @brief Expand three complete, native-precision comparison entry points.
 *
 * This implementation intentionally reuses the already qualified
 * `bbtc_vv_prepare_*` layer for EVERY initial comparison method. No rounded
 * metric is consulted while choosing the mathematical finding. Only after
 * a successful exact finding is an ABSOLUTE or REFERENCE_RELATIVE diagnostic
 * constructed; EXACT requires no diagnostic or additional kernel calls.
 *
 * STRUCTURAL PRECEDENCE: a null output returns without a write; otherwise
 * a compound-literal initialization clears every result field before policy
 * or numeric inspection. The internal preparation adapter then implements
 * B1a's method, NaN-before-infinity, tolerance, and reference-domain rules.
 *
 * FENV ISOLATION: the outer feholdexcept/fesetenv pair protects the *whole*
 * operation. Component calls may independently hold/restore their nested
 * environments, but the caller's original state is restored only when this
 * outer guard completes. We do not publish provisional findings or metrics
 * when any component or restoration fails. The macro has precisely one
 * out_result commit site and one outer restoration site.
 *
 * INTERNAL FAILURE: after an exact-success finding, the input values are
 * already validated. Any unexpected metric/round non-success (other than an
 * environment rejection) is therefore an internal failure, not a user input
 * error. EXCEEDS_RANGE and ROUNDED_TO_ZERO are legitimate successful metric
 * classifications: neither changes the exact finding.
 */
#define BBTC_VV_DEFINE_COMPARE(NAME, TYPE)                                      \
    bbtc_vv_operation_status_e bbtc_vv_compare_##NAME(                          \
        TYPE reference, TYPE observation,                                      \
        const bbtc_vv_policy_##NAME##_t* policy,                               \
        bbtc_vv_result_##NAME##_t* out_result                                  \
    )                                                                          \
    {                                                                          \
        if (out_result == NULL)                                                \
            return BBTC_VV_STATUS_INVALID_ARGUMENT;                            \
        *out_result = (bbtc_vv_result_##NAME##_t){0};                          \
        if (policy == NULL)                                                    \
            return BBTC_VV_STATUS_INVALID_ARGUMENT;                            \
        if (policy->method != BBTC_VV_COMPARISON_EXACT &&                       \
            policy->method != BBTC_VV_COMPARISON_ABSOLUTE &&                    \
            policy->method != BBTC_VV_COMPARISON_REFERENCE_RELATIVE)           \
            return BBTC_VV_STATUS_INVALID_ARGUMENT;                            \
                                                                               \
        /* Do not classify an FP input before capturing its environment. */   \
        fenv_t initial_environment;                                            \
        if (feholdexcept(&initial_environment) != 0)                           \
            return BBTC_VV_STATUS_UNSUPPORTED_FP_ENVIRONMENT;                  \
                                                                               \
        bbtc_vv_finding_e finding = BBTC_VV_FINDING_NOT_EVALUATED;             \
        bbtc_vv_metric_state_e metric_state = BBTC_VV_METRIC_NOT_COMPUTED;     \
        TYPE rounded_metric = (TYPE)0;                                         \
        bbtc_vv_operation_status_e status = bbtc_vv_prepare_##NAME(            \
            reference, observation, policy, &finding);                         \
                                                                               \
        if (status == BBTC_VV_STATUS_SUCCESS &&                                \
            policy->method != BBTC_VV_COMPARISON_EXACT)                        \
        {                                                                      \
            bbtc_vv_exact_metric_t exact = {0};                                \
            const bbtc_vv_operation_status_e metric_status =                  \
                bbtc_vv_metric_exact_##NAME(                                   \
                    policy->method, reference, observation, &exact);           \
            if (metric_status != BBTC_VV_STATUS_SUCCESS)                       \
            {                                                                  \
                status = metric_status ==                                      \
                    BBTC_VV_STATUS_UNSUPPORTED_FP_ENVIRONMENT ?               \
                    metric_status : BBTC_VV_STATUS_NUMERICAL_FAILURE;          \
            }                                                                  \
            else                                                               \
            {                                                                  \
                const bbtc_vv_operation_status_e round_status =               \
                    bbtc_vv_metric_round_##NAME(                               \
                        &exact, &metric_state, &rounded_metric);               \
                if (round_status != BBTC_VV_STATUS_SUCCESS)                    \
                    status = round_status ==                                   \
                        BBTC_VV_STATUS_UNSUPPORTED_FP_ENVIRONMENT ?           \
                        round_status : BBTC_VV_STATUS_NUMERICAL_FAILURE;       \
            }                                                                  \
        }                                                                      \
                                                                               \
        /* Restoration is mandatory before any successful output commit. */  \
        if (fesetenv(&initial_environment) != 0)                               \
            return BBTC_VV_STATUS_NUMERICAL_FAILURE;                           \
        if (status != BBTC_VV_STATUS_SUCCESS)                                  \
            return status;                                                     \
                                                                               \
        out_result->finding = finding;                                         \
        out_result->metric_state = metric_state;                               \
        out_result->metric = rounded_metric;                                   \
        return BBTC_VV_STATUS_SUCCESS;                                         \
    }

BBTC_VV_DEFINE_COMPARE(float, float)
BBTC_VV_DEFINE_COMPARE(double, double)
BBTC_VV_DEFINE_COMPARE(long_double, long double)

#undef BBTC_VV_DEFINE_COMPARE
