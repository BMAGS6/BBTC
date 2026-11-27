/**
 * @file
 * @brief Fixed-step Runge-Kutta integration for external-ballistics state.
 *
 * EB0.1-B11 defines the first public numerical-integration operation in the
 * reconstructed external-ballistics subsystem. The integrator consumes only
 * the model-agnostic B10 state-derivative provider boundary and therefore
 * remains independent of vacuum, aerodynamic, gravity, force, and moment
 * equations.
 *
 * One successful step uses classical fourth-order Runge-Kutta (RK4) for the
 * stored dependent-variable components. Internally generated quaternion stages
 * and the final quaternion are normalized as explicit constraint control.
 *
 * Stepping is transactional with respect to the caller-owned rigid-body state:
 * all work occurs in local storage and `*state` is replaced only after the
 * complete step succeeds. Provider-context side effects are caller-owned and
 * are not rolled back.
 */

#ifndef BBTC_EXTERNAL_BALLISTICS_INTEGRATOR_H
#define BBTC_EXTERNAL_BALLISTICS_INTEGRATOR_H

#include <bbtc/external_ballistics/state_derivative_provider.h>
#include <bbtc/status.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Advances one native-float rigid-body state by one fixed RK4 step.
 *
 * `step_s` must be finite and strictly positive. The initial `state` must
 * already satisfy the public B2 state contract; B11 never repairs an invalid
 * caller-supplied orientation.
 *
 * Stage times are exactly `t`, `t + h/2`, `t + h/2`, and `t + h` in native
 * float arithmetic. K2 and K3 use half-step derivative increments; K4 uses the
 * full-step state `Y0 + h*K3`. A successful step performs exactly four B10
 * provider dispatches.
 *
 * Every callback-visible state is local/internal storage rather than the
 * caller's `state` object. Provider context used during stepping must not
 * provide writable aliasing access to that caller-owned state.
 *
 * On any failure, this function leaves `*state` bit-for-bit unchanged. Provider
 * callback status is propagated unchanged; failures caused by internally
 * generated stage/final state arithmetic are reported as
 * `BBTC_STATUS_NUMERICAL_FAILURE`.
 *
 * @param provider Borrowed B10 derivative provider.
 * @param step_s Finite, strictly positive step duration in seconds.
 * @param state Required in/out rigid-body state.
 *
 * @return `BBTC_STATUS_SUCCESS` on committed advancement, otherwise the
 *         deterministic B11 request, provider, or numerical-failure status.
 */
bbtc_status_e
bbtc_eb_rk4_step_float(
    const bbtc_eb_state_derivative_provider_float_t* provider,
    float step_s,
    bbtc_eb_rigid_body_state_float_t* state
);

/**
 * @brief Advances one native-double rigid-body state by one fixed RK4 step.
 *
 * Semantics are identical to `bbtc_eb_rk4_step_float()` while all state,
 * derivative, time, stage, and quaternion-normalization arithmetic remains
 * native `double`.
 */
bbtc_status_e
bbtc_eb_rk4_step_double(
    const bbtc_eb_state_derivative_provider_double_t* provider,
    double step_s,
    bbtc_eb_rigid_body_state_double_t* state
);

/**
 * @brief Advances one native-long-double rigid-body state by one fixed RK4 step.
 *
 * Semantics are identical to `bbtc_eb_rk4_step_float()` while all arithmetic
 * remains native `long double`.
 */
bbtc_status_e
bbtc_eb_rk4_step_long_double(
    const bbtc_eb_state_derivative_provider_long_double_t* provider,
    long double step_s,
    bbtc_eb_rigid_body_state_long_double_t* state
);

#ifdef __cplusplus
}   /* extern "C" */
#endif

#endif /* BBTC_EXTERNAL_BALLISTICS_INTEGRATOR_H */
