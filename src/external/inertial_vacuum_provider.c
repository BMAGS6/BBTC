/**
 * @file
 * @brief EB1.2 inertial-vacuum B7/B10 provider composition.
 */

#include <bbtc/external_ballistics/inertial_vacuum_provider.h>

#include <bbtc/external_ballistics/inertial_vacuum.h>

#include <stddef.h>

#define BBTC_EB_DEFINE_INERTIAL_VACUUM_PROVIDER(SUFFIX)                     \
    bbtc_status_e                                                           \
    bbtc_eb_inertial_vacuum_provider_config_validate_##SUFFIX(             \
        const bbtc_eb_inertial_vacuum_provider_config_##SUFFIX##_t* const  \
            config                                                          \
    )                                                                       \
    {                                                                       \
        if (config == NULL)                                                 \
            return BBTC_STATUS_INVALID_ARGUMENT;                            \
                                                                            \
        bbtc_status_e status =                                              \
            bbtc_eb_projectile_properties_validate_##SUFFIX(                \
                &config->projectile                                         \
            );                                                              \
        if (status != BBTC_STATUS_SUCCESS)                                  \
            return status;                                                  \
                                                                            \
        status = bbtc_eb_force_moment_validate_##SUFFIX(                    \
            &config->constant_force_moment                                  \
        );                                                                  \
        if (status != BBTC_STATUS_SUCCESS)                                  \
            return status;                                                  \
                                                                            \
        if (config->reference_frame_provider.sample == NULL)                \
            return BBTC_STATUS_INVALID_ARGUMENT;                            \
                                                                            \
        return BBTC_STATUS_SUCCESS;                                         \
    }                                                                       \
                                                                            \
    static bbtc_status_e                                                    \
    bbtc_eb_inertial_vacuum_provider_evaluate_##SUFFIX(                    \
        const void* const context,                                          \
        const bbtc_eb_rigid_body_state_##SUFFIX##_t* const state,          \
        bbtc_eb_rigid_body_state_derivative_##SUFFIX##_t* const            \
            out_derivative                                                  \
    )                                                                       \
    {                                                                       \
        const bbtc_eb_inertial_vacuum_provider_config_##SUFFIX##_t* const  \
            config = context;                                               \
                                                                            \
        const bbtc_status_e config_status =                                 \
            bbtc_eb_inertial_vacuum_provider_config_validate_##SUFFIX(     \
                config                                                      \
            );                                                              \
        if (config_status != BBTC_STATUS_SUCCESS)                           \
            return config_status;                                           \
                                                                            \
        bbtc_eb_reference_frame_sample_##SUFFIX##_t reference_frame = {0}; \
        const bbtc_status_e frame_status =                                  \
            bbtc_eb_reference_frame_provider_sample_##SUFFIX(               \
                &config->reference_frame_provider,                          \
                &state->position_m,                                         \
                state->time_s,                                              \
                &reference_frame                                            \
            );                                                              \
        if (frame_status != BBTC_STATUS_SUCCESS)                            \
            return frame_status;                                            \
                                                                            \
        return bbtc_eb_inertial_vacuum_derivative_evaluate_##SUFFIX(       \
            state,                                                          \
            &config->projectile,                                            \
            &reference_frame,                                               \
            &config->constant_force_moment,                                 \
            out_derivative                                                  \
        );                                                                  \
    }                                                                       \
                                                                            \
    bbtc_status_e                                                           \
    bbtc_eb_inertial_vacuum_provider_bind_##SUFFIX(                        \
        const bbtc_eb_inertial_vacuum_provider_config_##SUFFIX##_t* const  \
            config,                                                         \
        bbtc_eb_state_derivative_provider_##SUFFIX##_t* const out_provider \
    )                                                                       \
    {                                                                       \
        if (out_provider == NULL)                                           \
            return BBTC_STATUS_INVALID_ARGUMENT;                            \
                                                                            \
        *out_provider =                                                     \
            (bbtc_eb_state_derivative_provider_##SUFFIX##_t){0};           \
                                                                            \
        if (config == NULL)                                                 \
            return BBTC_STATUS_INVALID_ARGUMENT;                            \
                                                                            \
        const bbtc_status_e status =                                        \
            bbtc_eb_inertial_vacuum_provider_config_validate_##SUFFIX(     \
                config                                                      \
            );                                                              \
        if (status != BBTC_STATUS_SUCCESS)                                  \
            return status;                                                  \
                                                                            \
        out_provider->evaluate =                                            \
            bbtc_eb_inertial_vacuum_provider_evaluate_##SUFFIX;            \
        out_provider->context = config;                                     \
        return BBTC_STATUS_SUCCESS;                                         \
    }

BBTC_EB_DEFINE_INERTIAL_VACUUM_PROVIDER(float)
BBTC_EB_DEFINE_INERTIAL_VACUUM_PROVIDER(double)
BBTC_EB_DEFINE_INERTIAL_VACUUM_PROVIDER(long_double)

#undef BBTC_EB_DEFINE_INERTIAL_VACUUM_PROVIDER
