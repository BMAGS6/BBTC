/**
 * @file
 * @brief Caller-controlled rigid-body state-derivative provider interface.
 *
 * EB0.1-B10 defines the generic boundary through which later numerical
 * integration code requests one validated B9 rigid-body state derivative for
 * one validated B1/B2 candidate state. The provider is model-agnostic:
 * simulation/stage time is carried only by `state->time_s`, provider context is
 * borrowed caller-owned data and may be null, and dispatch is fail-closed.
 */

#ifndef BBTC_EXTERNAL_BALLISTICS_STATE_DERIVATIVE_PROVIDER_H
#define BBTC_EXTERNAL_BALLISTICS_STATE_DERIVATIVE_PROVIDER_H

#include <bbtc/external_ballistics/state.h>
#include <bbtc/external_ballistics/state_derivative.h>
#include <bbtc/status.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Native-float callback that evaluates one rigid-body state derivative.
 *
 * The dispatcher validates `state` before callback invocation and presents
 * `out_derivative` as the exact all-zero B9 derivative on callback entry.
 */
typedef bbtc_status_e
(*bbtc_eb_state_derivative_provider_float_fn)(
    const void* context,
    const bbtc_eb_rigid_body_state_float_t* state,
    bbtc_eb_rigid_body_state_derivative_float_t* out_derivative
);

/** @brief Native-double derivative-evaluation callback. */
typedef bbtc_status_e
(*bbtc_eb_state_derivative_provider_double_fn)(
    const void* context,
    const bbtc_eb_rigid_body_state_double_t* state,
    bbtc_eb_rigid_body_state_derivative_double_t* out_derivative
);

/** @brief Native-long-double derivative-evaluation callback. */
typedef bbtc_status_e
(*bbtc_eb_state_derivative_provider_long_double_fn)(
    const void* context,
    const bbtc_eb_rigid_body_state_long_double_t* state,
    bbtc_eb_rigid_body_state_derivative_long_double_t* out_derivative
);

/** @brief Borrowed native-float rigid-body derivative provider. */
typedef struct bbtc_eb_state_derivative_provider_float_t
{
    /** Matching native-float callback. */
    bbtc_eb_state_derivative_provider_float_fn evaluate;

    /** Borrowed opaque caller context; may be null. */
    const void* context;
}
bbtc_eb_state_derivative_provider_float_t;

/** @brief Borrowed native-double rigid-body derivative provider. */
typedef struct bbtc_eb_state_derivative_provider_double_t
{
    /** Matching native-double callback. */
    bbtc_eb_state_derivative_provider_double_fn evaluate;

    /** Borrowed opaque caller context; may be null. */
    const void* context;
}
bbtc_eb_state_derivative_provider_double_t;

/** @brief Borrowed native-long-double rigid-body derivative provider. */
typedef struct bbtc_eb_state_derivative_provider_long_double_t
{
    /** Matching native-long-double callback. */
    bbtc_eb_state_derivative_provider_long_double_fn evaluate;

    /** Borrowed opaque caller context; may be null. */
    const void* context;
}
bbtc_eb_state_derivative_provider_long_double_t;

/**
 * @brief Dispatches one native-float derivative request.
 *
 * Failure precedence:
 * 1. null output -> `BBTC_STATUS_INVALID_ARGUMENT`;
 * 2. otherwise zero output;
 * 3. null provider/callback/state -> `BBTC_STATUS_INVALID_ARGUMENT`;
 * 4. validate state with the matching B2 validator;
 * 5. propagate state validation failure unchanged;
 * 6. invoke callback exactly once;
 * 7. propagate callback failure unchanged after restoring zero output;
 * 8. after callback success, validate output with the matching B9 validator;
 * 9. invalid derivative -> zero output and return validator status;
 * 10. otherwise preserve callback output and return success.
 *
 * Callback failure status is authoritative over any derivative contents the
 * callback wrote. The exact all-zero derivative is valid success data.
 */
bbtc_status_e
bbtc_eb_state_derivative_provider_evaluate_float(
    const bbtc_eb_state_derivative_provider_float_t* provider,
    const bbtc_eb_rigid_body_state_float_t* state,
    bbtc_eb_rigid_body_state_derivative_float_t* out_derivative
);

/** @brief Native-double dispatch with identical B10 semantics. */
bbtc_status_e
bbtc_eb_state_derivative_provider_evaluate_double(
    const bbtc_eb_state_derivative_provider_double_t* provider,
    const bbtc_eb_rigid_body_state_double_t* state,
    bbtc_eb_rigid_body_state_derivative_double_t* out_derivative
);

/** @brief Native-long-double dispatch with identical B10 semantics. */
bbtc_status_e
bbtc_eb_state_derivative_provider_evaluate_long_double(
    const bbtc_eb_state_derivative_provider_long_double_t* provider,
    const bbtc_eb_rigid_body_state_long_double_t* state,
    bbtc_eb_rigid_body_state_derivative_long_double_t* out_derivative
);

#ifdef __cplusplus
}   /* extern "C" */
#endif

#endif /* BBTC_EXTERNAL_BALLISTICS_STATE_DERIVATIVE_PROVIDER_H */
