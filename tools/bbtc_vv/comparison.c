/**
 * @file comparison.c
 * @brief Isolated B1b.3b validation/decision adapters over exact_dyadic.c.
 *
 * @details
 * The accepted B1a interface requires BOTH a mathematically exact finding
 * and a correctly rounded, classified scalar diagnostic. This file currently
 * implements ONLY the independent foundation for that API:
 *
 *  1. Structural pointer and method validation ahead of numeric inspection.
 *  2. Mapping between the accepted VVUQ methods and the internal dyadic ones.
 *  3. Preserving the environment over the entire numeric path, including
 *     nested calls to the existing exact dyadic kernel.
 *  4. Translating the kernel's execution statuses without ever mapping
 *     a failed operation to an apparent successful comparison.
 *  5. Canonical NOT_EVALUATED output on every failing return.
 *
 * There is NO exported bbtc_vv_compare_* symbol in this increment: its
 * contractual success results for Absolute/relative modes cannot be honored
 * until B1b.3c implements exact-to-native metric rounding.
 *
 * No production BBTC headers or objects are referenced.
 */
#include <stddef.h>
#include <fenv.h>

#if __STDC_VERSION__ < 202311L
#   include <stdbool.h>
#endif

#include "comparison_internal.h"
#include "exact_dyadic.h"

#if defined(__FAST_MATH__)
    #error "BBTC VVUQ comparison foundation does not support -ffast-math"
#endif

#if defined(__FINITE_MATH_ONLY__)
    #if __FINITE_MATH_ONLY__ != 0
        #error "BBTC VVUQ comparison foundation needs full NaN/Inf semantics"
    #endif
#endif

#if defined(__clang__)
    #pragma STDC FENV_ACCESS ON
#endif

/**
 * @brief Translate the contract's method into the private dyadic method.
 *
 * @return True if the supplied enumerator is supported, false otherwise.
 *         On true, `*out_method` is initialized to a valid dyadic method.
 *
 * This switch intentionally avoids casting enums by their numerical values.
 * The two enumerations are independently contracted; an implicit equality of
 * their values would be accidental coupling and hard to audit.
 */
static bool translate_method(bbtc_vv_comparison_method_e method,
                             bbtc_vv_dyadic_method_e* out_method)
{
    switch (method)
    {
        case BBTC_VV_COMPARISON_EXACT:
            *out_method = BBTC_VV_DYADIC_EXACT;
            return true;

        case BBTC_VV_COMPARISON_ABSOLUTE:
            *out_method = BBTC_VV_DYADIC_ABSOLUTE;
            return true;

        case BBTC_VV_COMPARISON_REFERENCE_RELATIVE:
            *out_method = BBTC_VV_DYADIC_REFERENCE_RELATIVE;
            return true;

        default:
            return false;
    }
}

/**
 * @brief Translate a dyadic *operation* status to its B1a API counterpart.
 *
 * Statuses, findings, and metric states are deliberately separate types. A
 * non-success dyadic status NEVER yields a public SUCCESS status. An unknown
 * enumerator also fails closed as NUMERICAL_FAILURE, not as an accepted
 * mathematical finding.
 */
static bbtc_vv_operation_status_e translate_status(bbtc_vv_dyadic_status_e status)
{
    switch (status)
    {
        case BBTC_VV_DYADIC_SUCCESS:
            return BBTC_VV_STATUS_SUCCESS;

        case BBTC_VV_DYADIC_INVALID_ARGUMENT:
            return BBTC_VV_STATUS_INVALID_ARGUMENT;

        case BBTC_VV_DYADIC_NAN_INPUT:
            return BBTC_VV_STATUS_NAN_INPUT;

        case BBTC_VV_DYADIC_NONFINITE_INPUT:
            return BBTC_VV_STATUS_NONFINITE_INPUT;

        case BBTC_VV_DYADIC_OUTSIDE_DOMAIN:
            return BBTC_VV_STATUS_OUTSIDE_DOMAIN;

        case BBTC_VV_DYADIC_UNSUPPORTED_ENVIRONMENT:
            return BBTC_VV_STATUS_UNSUPPORTED_FP_ENVIRONMENT;

        case BBTC_VV_DYADIC_INTERNAL_FAILURE:

        default:
            return BBTC_VV_STATUS_NUMERICAL_FAILURE;
    }
}

/**
 * @brief Instantiate identically ordered checks for three native C families.
 *
 * @details
 * This macro is intentionally short and confined to the three typed entry
 * points; status and method mapping remain non-macro functions for review.
 * The kernel performs input classification in NaN-before-infinity order,
 * followed by tolerance and reference-domain checks. Repeating these floating
 * checks in this layer would risk raising FE_INVALID before environment
 * capture, especially for signaling NaNs.
 *
 * `feholdexcept()` is called only AFTER structural checks, because a null
 * output or policy must not require floating-point work. It saves both the
 * incoming rounding mode and sticky exception state, and masks supported
 * traps while nested operations execute. `fesetenv()` restores the state
 * even after kernel rejection. Unmasked-trap environments are not certified.
 *
 * An unsuccessful restore overrides any apparent SUCCESS. Since the caller's
 * state cannot be guaranteed after such failure, NUMERICAL_FAILURE is used
 * and no mathematical finding is returned.
 */
#define BBTC_VV_DEFINE_PREPARE(NAME, TYPE)                                     \
    bbtc_vv_operation_status_e bbtc_vv_prepare_##NAME(                         \
        TYPE reference, TYPE observation,                                      \
        const bbtc_vv_policy_##NAME##_t* policy,                               \
        bbtc_vv_finding_e* out_finding                                         \
    )                                                                          \
    {                                                                          \
        if (out_finding == NULL)                                               \
            return BBTC_VV_STATUS_INVALID_ARGUMENT;                            \
        *out_finding = BBTC_VV_FINDING_NOT_EVALUATED;                          \
        if (policy == NULL)                                                    \
            return BBTC_VV_STATUS_INVALID_ARGUMENT;                            \
        bbtc_vv_dyadic_method_e method;                                        \
        if (!translate_method(policy->method, &method))                        \
            return BBTC_VV_STATUS_INVALID_ARGUMENT;                            \
        fenv_t saved_environment;                                              \
        if (feholdexcept(&saved_environment) != 0)                             \
            return BBTC_VV_STATUS_UNSUPPORTED_FP_ENVIRONMENT;                  \
        bool passed = false;                                                   \
        const bbtc_vv_dyadic_status_e core_status =                            \
            bbtc_vv_dyadic_compare_##NAME(                                     \
                method, reference, observation, policy->tolerance, &passed     \
            );                                                                 \
        const bbtc_vv_operation_status_e status =                              \
            translate_status(core_status);                                     \
        if (fesetenv(&saved_environment) != 0)                                 \
            return BBTC_VV_STATUS_NUMERICAL_FAILURE;                           \
        if (status == BBTC_VV_STATUS_SUCCESS)                                  \
            *out_finding = passed ? BBTC_VV_FINDING_PASS : BBTC_VV_FINDING_FAIL;\
        return status;                                                          \
    }

BBTC_VV_DEFINE_PREPARE(float, float)
BBTC_VV_DEFINE_PREPARE(double, double)
BBTC_VV_DEFINE_PREPARE(long_double, long double)

#undef BBTC_VV_DEFINE_PREPARE
