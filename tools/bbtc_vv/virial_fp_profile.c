/**
 * @file virial_fp_profile.c
 * @brief Source-level B2b.3c.2 host profile observation and rejection logic.
 *
 * @details
 * This module performs a restrictive, developer-only PRECHECK for the
 * M-01..M-04 exact-arithmetic argument in the B2b.3c.1 proof record. A green
 * result means only that selected runtime/ABI properties are coherent. It
 * never says that the production virial evaluator was compiled with a known
 * set of flags or that its generated instructions have been reviewed.
 *
 * The compiler's `<float.h>` metadata is kept separate from metadata
 * retrieved from `bbtc_precision_info()` in the *linked production library*.
 * Compiler options for this module cannot establish options for a different
 * static-library object. The separate compile-command capture must be read
 * and reviewed before declaring any target-specific profile qualified.
 */
#include "virial_fp_profile.h"

#include <stddef.h>
#include <fenv.h>
#include <float.h>

#if defined(__FAST_MATH__)
    #error "B2b.3c.2 profile code forbids fast-math"
#endif
#if defined(__FINITE_MATH_ONLY__) && __FINITE_MATH_ONLY__ != 0
    #error "B2b.3c.2 profile code requires finite-math-only disabled"
#endif
#if defined(__clang__)
    #pragma STDC FENV_ACCESS ON
#endif

/**
 * @brief Require exactly the compilation-unit properties reported by BBTC.
 * @param a Compiler metadata expected for this scalar family.
 * @param b Linked-production-library metadata actually observed.
 * @return true iff identity and all reported ABI fields agree.
 */
static bool equal_precision_info(const bbtc_precision_info_t* a,
                                 const bbtc_precision_info_t* b)
{
    return a->precision               == b->precision               &&
           a->radix                   == b->radix                   &&
           a->mantissa_digits         == b->mantissa_digits         &&
           a->minimum_normal_exponent == b->minimum_normal_exponent &&
           a->maximum_finite_exponent == b->maximum_finite_exponent &&
           a->decimal_digits          == b->decimal_digits          &&
           a->storage_bytes           == b->storage_bytes;
}

/**
 * @brief Examine only the deliberately admitted binary-format subset.
 *
 * @details
 * The proof's nonzero values lie between 2^-24 and 2^9. These conservative
 * bounds also reject unexpected implementation formats rather than implying
 * general support from a numerically correct output. A mantissa size alone
 * cannot establish IEEE encoding or hardware exception-mask semantics.
 *
 * @param o Compiler-side metadata for all three scalar families.
 * @return true iff the currently declared numerical format is in the subset.
 */
static bool supported_compiler_formats(const bbtc_vv_virial_profile_observation_t* o)
{
    const bbtc_precision_info_t* f = &o->compiler_info[0];
    const bbtc_precision_info_t* d = &o->compiler_info[1];
    const bbtc_precision_info_t* l = &o->compiler_info[2];

    return f->precision               == BBTC_PRECISION_FLOAT                   &&
           d->precision               == BBTC_PRECISION_DOUBLE                  &&
           l->precision               == BBTC_PRECISION_LONG_DOUBLE             &&
           f->radix                   == 2u && d->radix == 2u && l->radix == 2u &&
           f->mantissa_digits         == 24u && d->mantissa_digits == 53u       &&
           (l->mantissa_digits        == 53u || l->mantissa_digits == 64u ||
                l->mantissa_digits    == 113u)                                  &&
           f->minimum_normal_exponent <= -24                                    &&
           d->minimum_normal_exponent <= -24                                    &&
           l->minimum_normal_exponent <= -24                                    &&
           f->maximum_finite_exponent >= 128                                    &&
           d->maximum_finite_exponent >= 1024                                   &&
           l->maximum_finite_exponent >= 1024                                   &&
           f->storage_bytes           == (uint32_t)sizeof(float)                &&
           d->storage_bytes           == (uint32_t)sizeof(double)               &&
           l->storage_bytes           == (uint32_t)sizeof(long double);
}

/**
 * @brief Deterministically classify one observation without touching fenv.
 *
 * @details
 * A missing result pointer cannot be written to and returns UNSUPPORTED.
 * Otherwise the result is cleared before checking the observation pointer.
 * Multiple simultaneously invalid properties contribute separate issue bits.
 * Unsupported operation status is *never* a B1 numerical FAIL or a scientific
 * contradiction: it means the execution environment is not admitted.
 */
bbtc_vv_virial_profile_state_e
bbtc_vv_virial_profile_classify(const bbtc_vv_virial_profile_observation_t* observation,
                                bbtc_vv_virial_profile_result_t* out_result
)
{
    if (out_result == NULL)
        return BBTC_VV_VIRIAL_PROFILE_UNSUPPORTED;

    *out_result = (bbtc_vv_virial_profile_result_t){0};

    if (observation == NULL)
    {
        out_result->issue_mask = BBTC_VV_VIRIAL_ISSUE_INVALID_OBSERVATION;
        out_result->state      = BBTC_VV_VIRIAL_PROFILE_UNSUPPORTED;
        return out_result->state;
    }

    uint32_t issues = 0u;

    if (observation->rounding_mode != FE_TONEAREST)
        issues |= BBTC_VV_VIRIAL_ISSUE_ROUNDING_MODE;

    /* C99/C23 FLT_EVAL_METHOD: other/unbounded modes need separate review. */
    if (observation->evaluation_method != 0 &&
        observation->evaluation_method != 1 &&
        observation->evaluation_method != 2)
    {
        issues |= BBTC_VV_VIRIAL_ISSUE_EVALUATION_METHOD;
    }

    for (size_t i = 0u; i < BBTC_VV_VIRIAL_NATIVE_FAMILIES; ++i)
    {
        if (observation->compiler_info[i].radix != 2u)
            issues |= BBTC_VV_VIRIAL_ISSUE_RADIX;

        if (observation->production_status[i] != BBTC_STATUS_SUCCESS ||
            !equal_precision_info(&observation->compiler_info[i],
                                  &observation->production_info[i]))
        {
            issues |= BBTC_VV_VIRIAL_ISSUE_PRODUCTION_METADATA;
        }
    }

    if (!supported_compiler_formats(observation))
        issues |= BBTC_VV_VIRIAL_ISSUE_NATIVE_FORMAT;

    out_result->issue_mask = issues;

    out_result->state = issues == 0u ? BBTC_VV_VIRIAL_PROFILE_SCREENED_UNQUALIFIED
                                     : BBTC_VV_VIRIAL_PROFILE_UNSUPPORTED;
    return out_result->state;
}

/**
 * @brief Collect compiler metadata, production ABI metadata, and rounding.
 *
 * @details
 * The compiler-side array contains an explicit precision identity in each
 * slot. The linked-library query is not assumed to succeed; it is recorded
 * independently so a failed metadata query cannot masquerade as valid zeros.
 * No floating-point operation is performed to calculate reference results.
 */
bool
bbtc_vv_virial_profile_observe(bbtc_vv_virial_profile_observation_t* out_observation)
{
    if (out_observation == NULL)
        return false;

    *out_observation = (bbtc_vv_virial_profile_observation_t){0};
    out_observation->rounding_mode = fegetround();
    out_observation->evaluation_method = FLT_EVAL_METHOD;

    out_observation->compiler_info[0] = (bbtc_precision_info_t)
    {
        .precision               = BBTC_PRECISION_FLOAT,
        .radix                   = FLT_RADIX,
        .mantissa_digits         = FLT_MANT_DIG,
        .minimum_normal_exponent = FLT_MIN_EXP,
        .maximum_finite_exponent = FLT_MAX_EXP,
        .decimal_digits          = FLT_DECIMAL_DIG,
        .storage_bytes           = sizeof(float)
    };

    out_observation->compiler_info[1] = (bbtc_precision_info_t)
    {
        .precision               = BBTC_PRECISION_DOUBLE,
        .radix                   = FLT_RADIX,
        .mantissa_digits         = DBL_MANT_DIG,
        .minimum_normal_exponent = DBL_MIN_EXP,
        .maximum_finite_exponent = DBL_MAX_EXP,
        .decimal_digits          = DBL_DECIMAL_DIG,
        .storage_bytes           = sizeof(double)
    };

    out_observation->compiler_info[2] = (bbtc_precision_info_t)
    {
        .precision               = BBTC_PRECISION_LONG_DOUBLE,
        .radix                   = FLT_RADIX,
        .mantissa_digits         = LDBL_MANT_DIG,
        .minimum_normal_exponent = LDBL_MIN_EXP,
        .maximum_finite_exponent = LDBL_MAX_EXP,
        .decimal_digits          = LDBL_DECIMAL_DIG,
        .storage_bytes           = sizeof(long double)
    };

    for (size_t i = 0u; i < BBTC_VV_VIRIAL_NATIVE_FAMILIES; ++i)
    {
        const bbtc_precision_e precision = out_observation->compiler_info[i].precision;

        out_observation->production_status[i] = bbtc_precision_info(
            precision, &out_observation->production_info[i]);
    }

    return true;
}

/**
 * @brief Observe and classify, preserving the complete input and decision.
 *
 * A null pointer is an unsupported invocation. Both caller-owned structures
 * are cleared independently of any subsequent observation or classification
 * failure. The function cannot return QUALIFIED by construction.
 */
bbtc_vv_virial_profile_state_e
bbtc_vv_virial_profile_screen(
    bbtc_vv_virial_profile_observation_t* out_observation,
    bbtc_vv_virial_profile_result_t* out_result
)
{
    if (out_result != NULL)
        *out_result = (bbtc_vv_virial_profile_result_t){0};

    if (out_observation != NULL)
        *out_observation = (bbtc_vv_virial_profile_observation_t){0};

    if (out_observation == NULL || out_result == NULL)
    {
        if (out_result != NULL)
        {
            out_result->state      = BBTC_VV_VIRIAL_PROFILE_UNSUPPORTED;
            out_result->issue_mask = BBTC_VV_VIRIAL_ISSUE_INVALID_OBSERVATION;
        }
        return BBTC_VV_VIRIAL_PROFILE_UNSUPPORTED;
    }

    (void)bbtc_vv_virial_profile_observe(out_observation);
    return bbtc_vv_virial_profile_classify(out_observation, out_result);
}
