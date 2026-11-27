/**
 * @file
 * @brief Fail-closed dispatch for EB0.1-B7 reference-frame providers.
 *
 * The dispatch layer centralizes request validation and successful B6 sample
 * validation so future solver code does not duplicate provider-boundary rules.
 *
 * A non-null output is zeroed before any remaining validation. Every later
 * failure restores that same deterministic zero state. Because an all-zero B6
 * sample is physically valid, callers must use the returned status rather than
 * the output contents to determine whether dispatch succeeded.
 *
 * Providers are deliberately treated as sampling interfaces only. This module
 * does not cache, interpolate, construct Earth geometry, evaluate Coriolis or
 * centrifugal acceleration, add Eötvös corrections, derive gravity, or
 * interpret caller context.
 */

#include <bbtc/external_ballistics/reference_frame_provider.h>

#include <math.h>
#include <stddef.h>

#define BBTC_EB_DEFINE_REFERENCE_FRAME_PROVIDER_DISPATCH(                  \
    SUFFIX, SCALAR, VEC_T, SAMPLE_T, PROVIDER_T, VALIDATE_SAMPLE          \
)                                                                          \
    bbtc_status_e                                                          \
    bbtc_eb_reference_frame_provider_sample_##SUFFIX(                     \
        const PROVIDER_T* const provider,                                 \
        const VEC_T* const position_reference_m,                          \
        const SCALAR time_s,                                               \
        SAMPLE_T* const out_sample                                         \
    )                                                                      \
    {                                                                      \
        if (out_sample == NULL)                                            \
            return BBTC_STATUS_INVALID_ARGUMENT;                           \
                                                                           \
        *out_sample = (SAMPLE_T){0};                                       \
                                                                           \
        if (provider == NULL                                               \
            || provider->sample == NULL                                    \
            || position_reference_m == NULL)                               \
        {                                                                  \
            return BBTC_STATUS_INVALID_ARGUMENT;                           \
        }                                                                  \
                                                                           \
        const SCALAR request_values[] =                                    \
        {                                                                  \
            position_reference_m->x,                                       \
            position_reference_m->y,                                       \
            position_reference_m->z,                                       \
            time_s                                                         \
        };                                                                 \
                                                                           \
        for (size_t i = 0;                                                 \
             i < sizeof(request_values) / sizeof(request_values[0]);       \
             ++i)                                                          \
        {                                                                  \
            if (isnan(request_values[i]))                                  \
                return BBTC_STATUS_NAN_INPUT;                              \
        }                                                                  \
                                                                           \
        for (size_t i = 0;                                                 \
             i < sizeof(request_values) / sizeof(request_values[0]);       \
             ++i)                                                          \
        {                                                                  \
            if (isinf(request_values[i]))                                  \
                return BBTC_STATUS_NONFINITE_INPUT;                        \
        }                                                                  \
                                                                           \
        if (time_s < (SCALAR)0)                                            \
            return BBTC_STATUS_OUTSIDE_DOMAIN;                             \
                                                                           \
        const bbtc_status_e provider_status = provider->sample(            \
            provider->context,                                             \
            position_reference_m,                                          \
            time_s,                                                        \
            out_sample                                                     \
        );                                                                 \
                                                                           \
        if (provider_status != BBTC_STATUS_SUCCESS)                        \
        {                                                                  \
            *out_sample = (SAMPLE_T){0};                                   \
            return provider_status;                                        \
        }                                                                  \
                                                                           \
        const bbtc_status_e sample_status = VALIDATE_SAMPLE(out_sample);   \
                                                                           \
        if (sample_status != BBTC_STATUS_SUCCESS)                          \
        {                                                                  \
            *out_sample = (SAMPLE_T){0};                                   \
            return sample_status;                                          \
        }                                                                  \
                                                                           \
        return BBTC_STATUS_SUCCESS;                                        \
    }

BBTC_EB_DEFINE_REFERENCE_FRAME_PROVIDER_DISPATCH(
    float,
    float,
    bbtc_eb_vec3_float_t,
    bbtc_eb_reference_frame_sample_float_t,
    bbtc_eb_reference_frame_provider_float_t,
    bbtc_eb_reference_frame_sample_validate_float
)

BBTC_EB_DEFINE_REFERENCE_FRAME_PROVIDER_DISPATCH(
    double,
    double,
    bbtc_eb_vec3_double_t,
    bbtc_eb_reference_frame_sample_double_t,
    bbtc_eb_reference_frame_provider_double_t,
    bbtc_eb_reference_frame_sample_validate_double
)

BBTC_EB_DEFINE_REFERENCE_FRAME_PROVIDER_DISPATCH(
    long_double,
    long double,
    bbtc_eb_vec3_long_double_t,
    bbtc_eb_reference_frame_sample_long_double_t,
    bbtc_eb_reference_frame_provider_long_double_t,
    bbtc_eb_reference_frame_sample_validate_long_double
)

#undef BBTC_EB_DEFINE_REFERENCE_FRAME_PROVIDER_DISPATCH
