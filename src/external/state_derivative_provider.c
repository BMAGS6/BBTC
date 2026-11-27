/**
 * @file
 * @brief Fail-closed dispatch for EB0.1-B10 state-derivative providers.
 */

#include <bbtc/external_ballistics/state_derivative_provider.h>

#include <stddef.h>

#define BBTC_EB_DEFINE_STATE_DERIVATIVE_PROVIDER_DISPATCH(                  \
    SUFFIX, STATE_T, DERIVATIVE_T, PROVIDER_T, VALIDATE_STATE,              \
    VALIDATE_DERIVATIVE                                                      \
)                                                                           \
    bbtc_status_e                                                           \
    bbtc_eb_state_derivative_provider_evaluate_##SUFFIX(                   \
        const PROVIDER_T* const provider,                                   \
        const STATE_T* const state,                                         \
        DERIVATIVE_T* const out_derivative                                  \
    )                                                                       \
    {                                                                       \
        if (out_derivative == NULL)                                         \
            return BBTC_STATUS_INVALID_ARGUMENT;                            \
                                                                            \
        *out_derivative = (DERIVATIVE_T){0};                                \
                                                                            \
        if (provider == NULL                                                \
            || provider->evaluate == NULL                                   \
            || state == NULL)                                               \
        {                                                                   \
            return BBTC_STATUS_INVALID_ARGUMENT;                            \
        }                                                                   \
                                                                            \
        const bbtc_status_e state_status = VALIDATE_STATE(state);           \
                                                                            \
        if (state_status != BBTC_STATUS_SUCCESS)                            \
            return state_status;                                            \
                                                                            \
        const bbtc_status_e callback_status = provider->evaluate(           \
            provider->context, state, out_derivative                        \
        );                                                                  \
                                                                            \
        if (callback_status != BBTC_STATUS_SUCCESS)                         \
        {                                                                   \
            *out_derivative = (DERIVATIVE_T){0};                            \
            return callback_status;                                         \
        }                                                                   \
                                                                            \
        const bbtc_status_e derivative_status =                             \
            VALIDATE_DERIVATIVE(out_derivative);                            \
                                                                            \
        if (derivative_status != BBTC_STATUS_SUCCESS)                       \
        {                                                                   \
            *out_derivative = (DERIVATIVE_T){0};                            \
            return derivative_status;                                       \
        }                                                                   \
                                                                            \
        return BBTC_STATUS_SUCCESS;                                         \
    }

BBTC_EB_DEFINE_STATE_DERIVATIVE_PROVIDER_DISPATCH(
    float,
    bbtc_eb_rigid_body_state_float_t,
    bbtc_eb_rigid_body_state_derivative_float_t,
    bbtc_eb_state_derivative_provider_float_t,
    bbtc_eb_rigid_body_state_validate_float,
    bbtc_eb_rigid_body_state_derivative_validate_float
)

BBTC_EB_DEFINE_STATE_DERIVATIVE_PROVIDER_DISPATCH(
    double,
    bbtc_eb_rigid_body_state_double_t,
    bbtc_eb_rigid_body_state_derivative_double_t,
    bbtc_eb_state_derivative_provider_double_t,
    bbtc_eb_rigid_body_state_validate_double,
    bbtc_eb_rigid_body_state_derivative_validate_double
)

BBTC_EB_DEFINE_STATE_DERIVATIVE_PROVIDER_DISPATCH(
    long_double,
    bbtc_eb_rigid_body_state_long_double_t,
    bbtc_eb_rigid_body_state_derivative_long_double_t,
    bbtc_eb_state_derivative_provider_long_double_t,
    bbtc_eb_rigid_body_state_validate_long_double,
    bbtc_eb_rigid_body_state_derivative_validate_long_double
)

#undef BBTC_EB_DEFINE_STATE_DERIVATIVE_PROVIDER_DISPATCH
