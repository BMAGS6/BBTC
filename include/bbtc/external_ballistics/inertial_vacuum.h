/**
 * @file
 * @brief Inertial-vacuum six-degree-of-freedom derivative evaluation.
 *
 * EB1.1 evaluates rigid-projectile equations of motion in an inertial,
 * nonaccelerating, nonrotating active BBTC reference frame. Vacuum excludes
 * atmospheric and aerodynamic interaction but does not exclude physical
 * gravitational acceleration or other caller-supplied physical force/moment.
 */

#ifndef BBTC_EXTERNAL_BALLISTICS_INERTIAL_VACUUM_H
#define BBTC_EXTERNAL_BALLISTICS_INERTIAL_VACUUM_H

#include <bbtc/external_ballistics/force_moment.h>
#include <bbtc/external_ballistics/projectile.h>
#include <bbtc/external_ballistics/reference_frame.h>
#include <bbtc/external_ballistics/state.h>
#include <bbtc/external_ballistics/state_derivative.h>
#include <bbtc/status.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Evaluates the native-float EB1.1 inertial-vacuum rigid-body derivative.
 *
 * The evaluator applies `dr/dt = v`, `dv/dt = F/m + g`,
 * `dq_WB/dt = 0.5 * q_WB * omega_B_quat`, and principal-axis Euler rotational
 * dynamics. The B6 reference-frame origin acceleration, angular velocity, and
 * angular acceleration must all be exactly zero (positive or negative zero are
 * equivalent); otherwise the model returns
 * `BBTC_STATUS_UNSUPPORTED_MODEL_OR_OPTION`.
 *
 * Input validation order is state, projectile, reference-frame sample, then
 * force/moment after the common null-input check. Generated derivative data is
 * B9-validated before commit. Any generated nonfinite result maps to
 * `BBTC_STATUS_NUMERICAL_FAILURE`. With a nonnull output pointer, every failure
 * leaves the exact all-zero B9 derivative in caller storage.
 *
 * @param state Valid B2 rigid-body state.
 * @param projectile Valid B3 rigid-projectile physical properties.
 * @param reference_frame Valid B6 already-sampled reference-frame value.
 * @param force_moment Valid B8 applied physical force/moment value.
 * @param out_derivative Caller-owned B9 derivative output.
 *
 * @return Deterministic EB1.1 status defined by the external-ballistics design
 *         contract.
 */
bbtc_status_e
bbtc_eb_inertial_vacuum_derivative_evaluate_float(
    const bbtc_eb_rigid_body_state_float_t* state,
    const bbtc_eb_projectile_properties_float_t* projectile,
    const bbtc_eb_reference_frame_sample_float_t* reference_frame,
    const bbtc_eb_force_moment_float_t* force_moment,
    bbtc_eb_rigid_body_state_derivative_float_t* out_derivative
);

/**
 * @brief Evaluates the native-double EB1.1 inertial-vacuum rigid-body derivative.
 *
 * Semantics, validation precedence, inertial-frame restrictions, fail-closed
 * output behavior, and equations of motion match the float-family evaluator.
 * All arithmetic and storage remain native `double`.
 *
 * @param state Valid B2 rigid-body state.
 * @param projectile Valid B3 rigid-projectile physical properties.
 * @param reference_frame Valid B6 already-sampled reference-frame value.
 * @param force_moment Valid B8 applied physical force/moment value.
 * @param out_derivative Caller-owned B9 derivative output.
 *
 * @return Deterministic EB1.1 status defined by the external-ballistics design
 *         contract.
 */
bbtc_status_e
bbtc_eb_inertial_vacuum_derivative_evaluate_double(
    const bbtc_eb_rigid_body_state_double_t* state,
    const bbtc_eb_projectile_properties_double_t* projectile,
    const bbtc_eb_reference_frame_sample_double_t* reference_frame,
    const bbtc_eb_force_moment_double_t* force_moment,
    bbtc_eb_rigid_body_state_derivative_double_t* out_derivative
);

/**
 * @brief Evaluates the native-long-double EB1.1 inertial-vacuum derivative.
 *
 * Semantics, validation precedence, inertial-frame restrictions, fail-closed
 * output behavior, and equations of motion match the float-family evaluator.
 * All arithmetic and storage remain native `long double`.
 *
 * @param state Valid B2 rigid-body state.
 * @param projectile Valid B3 rigid-projectile physical properties.
 * @param reference_frame Valid B6 already-sampled reference-frame value.
 * @param force_moment Valid B8 applied physical force/moment value.
 * @param out_derivative Caller-owned B9 derivative output.
 *
 * @return Deterministic EB1.1 status defined by the external-ballistics design
 *         contract.
 */
bbtc_status_e
bbtc_eb_inertial_vacuum_derivative_evaluate_long_double(
    const bbtc_eb_rigid_body_state_long_double_t* state,
    const bbtc_eb_projectile_properties_long_double_t* projectile,
    const bbtc_eb_reference_frame_sample_long_double_t* reference_frame,
    const bbtc_eb_force_moment_long_double_t* force_moment,
    bbtc_eb_rigid_body_state_derivative_long_double_t* out_derivative
);

#ifdef __cplusplus
}   /* extern "C" */
#endif

#endif /* BBTC_EXTERNAL_BALLISTICS_INERTIAL_VACUUM_H */
