/**
 * @file
 * @brief Precision-qualified external-ballistics rigid-body state derivatives.
 *
 * EB0.1-B9 defines the public value representation for derivatives of the four
 * physical dependent-variable groups stored by the BBTC rigid-body state:
 *
 * - world/reference-from-body orientation;
 * - active-reference-frame position;
 * - active-reference-frame translational velocity; and
 * - projectile-body-frame angular velocity.
 *
 * Simulation time is deliberately not represented by a derivative member.
 * Time is the independent integration variable; a future integrator advances
 * `state.time_s` explicitly and assigns intermediate stage times directly.
 *
 * A quaternion derivative is not an orientation quaternion. It is therefore
 * not required to be unit length, and its norm carries no B9 validity
 * requirement. Zero, unit-norm, non-unit, positive, and negative finite
 * derivative components are all representable and valid.
 *
 * EB0.1-B9 provides representation and validation only. It does not implement
 * quaternion kinematics, equations of motion, force/mass acceleration,
 * gravitational coupling, rigid-body Euler dynamics, or numerical integration.
 */

#ifndef BBTC_EXTERNAL_BALLISTICS_STATE_DERIVATIVE_H
#define BBTC_EXTERNAL_BALLISTICS_STATE_DERIVATIVE_H

#include <bbtc/external_ballistics/quaternion.h>
#include <bbtc/external_ballistics/vector.h>
#include <bbtc/status.h>

/**
 * @brief Native-float derivative of BBTC rigid-body dependent variables.
 *
 * The exact all-zero record is valid. No physical cross-record relationship is
 * implied or checked by this value type.
 */
typedef struct bbtc_eb_rigid_body_state_derivative_float_t
{
    /**
     * @brief Component-wise derivative of world/reference-from-body orientation.
     *
     * Components have reciprocal-second semantics. This quaternion-shaped value
     * is a derivative, not a unit orientation, and MUST NOT be normalized merely
     * to satisfy orientation-state rules.
     */
    bbtc_eb_quat_float_t orientation_world_from_body_derivative_per_s;

    /** @brief Position derivative in m/s, in active-reference axes. */
    bbtc_eb_vec3_float_t position_derivative_reference_m_per_s;

    /** @brief Translational-velocity derivative in m/s^2, in active-reference axes. */
    bbtc_eb_vec3_float_t velocity_derivative_reference_m_per_s2;

    /** @brief Body angular-velocity derivative in rad/s^2, in projectile body axes. */
    bbtc_eb_vec3_float_t angular_velocity_derivative_body_rad_per_s2;
}
bbtc_eb_rigid_body_state_derivative_float_t;

/** @brief Native-double derivative of BBTC rigid-body dependent variables. */
typedef struct bbtc_eb_rigid_body_state_derivative_double_t
{
    /** World/reference-from-body quaternion component derivative, in 1/s. */
    bbtc_eb_quat_double_t orientation_world_from_body_derivative_per_s;

    /** Position derivative in m/s, expressed in active-reference axes. */
    bbtc_eb_vec3_double_t position_derivative_reference_m_per_s;

    /** Translational-velocity derivative in m/s^2, in active-reference axes. */
    bbtc_eb_vec3_double_t velocity_derivative_reference_m_per_s2;

    /** Body angular-velocity derivative in rad/s^2, in projectile body axes. */
    bbtc_eb_vec3_double_t angular_velocity_derivative_body_rad_per_s2;
}
bbtc_eb_rigid_body_state_derivative_double_t;

/** @brief Native-long-double derivative of BBTC rigid-body dependent variables. */
typedef struct bbtc_eb_rigid_body_state_derivative_long_double_t
{
    /** World/reference-from-body quaternion component derivative, in 1/s. */
    bbtc_eb_quat_long_double_t orientation_world_from_body_derivative_per_s;

    /** Position derivative in m/s, expressed in active-reference axes. */
    bbtc_eb_vec3_long_double_t position_derivative_reference_m_per_s;

    /** Translational-velocity derivative in m/s^2, in active-reference axes. */
    bbtc_eb_vec3_long_double_t velocity_derivative_reference_m_per_s2;

    /** Body angular-velocity derivative in rad/s^2, in projectile body axes. */
    bbtc_eb_vec3_long_double_t angular_velocity_derivative_body_rad_per_s2;
}
bbtc_eb_rigid_body_state_derivative_long_double_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Validates one native-float rigid-body state derivative.
 *
 * Validation is read-only and deliberately representational. It does not
 * normalize the quaternion derivative, compare this derivative against a
 * rigid-body state, derive acceleration from force/mass, or derive angular
 * acceleration from moment/inertia.
 *
 * Deterministic failure precedence is:
 *
 * 1. null pointer -> `BBTC_STATUS_INVALID_ARGUMENT`;
 * 2. any NaN among all thirteen stored scalars -> `BBTC_STATUS_NAN_INPUT`;
 * 3. otherwise any positive or negative infinity ->
 *    `BBTC_STATUS_NONFINITE_INPUT`; and
 * 4. otherwise -> `BBTC_STATUS_SUCCESS`.
 *
 * The entire record is scanned for NaN before any infinity result is returned.
 * No additional domain failure is defined by EB0.1-B9.
 *
 * @param derivative Derivative record to inspect.
 * @return Deterministic validation status described above.
 */
bbtc_status_e
bbtc_eb_rigid_body_state_derivative_validate_float(
    const bbtc_eb_rigid_body_state_derivative_float_t* derivative
);

/** @brief Validates one native-double rigid-body state derivative. */
bbtc_status_e
bbtc_eb_rigid_body_state_derivative_validate_double(
    const bbtc_eb_rigid_body_state_derivative_double_t* derivative
);

/** @brief Validates one native-long-double rigid-body state derivative. */
bbtc_status_e
bbtc_eb_rigid_body_state_derivative_validate_long_double(
    const bbtc_eb_rigid_body_state_derivative_long_double_t* derivative
);

#ifdef __cplusplus
}   /* extern "C" */
#endif

#endif /* BBTC_EXTERNAL_BALLISTICS_STATE_DERIVATIVE_H */
