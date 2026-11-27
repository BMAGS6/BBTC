/**
 * @file
 * @brief Fail-closed dispatch for caller-controlled EB environment providers.
 *
 * The dispatch layer centralizes request validation and successful-sample
 * validation so future solver code does not need to duplicate provider-boundary
 * rules. A non-null output is cleared before any remaining validation, and it
 * is restored to that deterministic zero state whenever callback execution or
 * B4 sample validation fails.
 *
 * Providers are deliberately treated as pure sampling interfaces at this
 * boundary. Dispatch does not cache, interpolate, derive atmosphere quantities,
 * impose a standard atmosphere, or interpret caller context.
 */

#include <bbtc/external_ballistics/environment_provider.h>

#include <math.h>
#include <stddef.h>

#define BBTC_EB_DEFINE_ENVIRONMENT_PROVIDER_DISPATCH(                      \
    SUFFIX, SCALAR, VEC_T, SAMPLE_T, PROVIDER_T, VALIDATE_SAMPLE          \
)                                                                          \
    bbtc_status_e                                                          \
    bbtc_eb_environment_provider_sample_##SUFFIX(                         \
        const PROVIDER_T* const provider,                                 \
        const VEC_T* const position_world_m,                              \
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
            || position_world_m == NULL)                                   \
        {                                                                  \
            return BBTC_STATUS_INVALID_ARGUMENT;                           \
        }                                                                  \
                                                                           \
        const SCALAR request_values[] =                                    \
        {                                                                  \
            position_world_m->x,                                           \
            position_world_m->y,                                           \
            position_world_m->z,                                           \
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
            position_world_m,                                              \
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

BBTC_EB_DEFINE_ENVIRONMENT_PROVIDER_DISPATCH(
    float,
    float,
    bbtc_eb_vec3_float_t,
    bbtc_eb_environment_sample_float_t,
    bbtc_eb_environment_provider_float_t,
    bbtc_eb_environment_sample_validate_float
)

BBTC_EB_DEFINE_ENVIRONMENT_PROVIDER_DISPATCH(
    double,
    double,
    bbtc_eb_vec3_double_t,
    bbtc_eb_environment_sample_double_t,
    bbtc_eb_environment_provider_double_t,
    bbtc_eb_environment_sample_validate_double
)

BBTC_EB_DEFINE_ENVIRONMENT_PROVIDER_DISPATCH(
    long_double,
    long double,
    bbtc_eb_vec3_long_double_t,
    bbtc_eb_environment_sample_long_double_t,
    bbtc_eb_environment_provider_long_double_t,
    bbtc_eb_environment_sample_validate_long_double
)

#undef BBTC_EB_DEFINE_ENVIRONMENT_PROVIDER_DISPATCH
