/**
 * @file
 * @brief Precision-qualified external-ballistics rigid-body state records.
 *
 * The records in this header are the first public six-degree-of-freedom state
 * representation in the reconstructed BBTC external-ballistics subsystem.
 * EB0.1-B2 adds fail-closed validation while still deliberately omitting
 * equations of motion, force models, environment providers, and integration.
 *
 * Frame distinctions are part of the physical contract:
 *
 * - position and translational velocity are in the active world/reference
 *   frame;
 * - orientation is an active world/reference-from-body unit quaternion when
 *   the state is valid; and
 * - angular velocity is expressed in the BBTC projectile body frame.
 *
 * The canonical BBTC body frame is right-handed: +X points forward toward the
 * projectile nose, +Y points body-right, and +Z points body-down.
 */

#ifndef BBTC_EXTERNAL_BALLISTICS_STATE_H
#define BBTC_EXTERNAL_BALLISTICS_STATE_H

#include <bbtc/external_ballistics/quaternion.h>
#include <bbtc/external_ballistics/vector.h>
#include <bbtc/status.h>

/**
 * @brief Native-float external-ballistics rigid-body state.
 *
 * `orientation_world_from_body` is declared first, followed by the three
 * vector-valued members and scalar time. The declaration order is part of the
 * EB0.1-B1 representation contract, but callers must not assume one universal
 * byte size or padding pattern.
 *
 * A zero-initialized record is deterministic storage, not a valid projectile
 * state: in particular, its all-zero quaternion is not a unit orientation.
 */
typedef struct bbtc_eb_rigid_body_state_float_t
{
    /**
     * @brief Active world/reference-from-body orientation.
     *
     * When valid and unit length, this quaternion rotates a vector from the
     * BBTC projectile body frame into the active world/reference frame.
     */
    bbtc_eb_quat_float_t orientation_world_from_body;

    /**
     * @brief Projectile center-of-mass position, in metres.
     *
     * Components are expressed in the active world/reference frame. For the
     * intended Cubes of Honor adapter this is normally displacement from the
     * muzzle-local integration origin rather than a large absolute world
     * coordinate.
     */
    bbtc_eb_vec3_float_t position_m;

    /**
     * @brief Projectile center-of-mass translational velocity, in m/s.
     *
     * Components are expressed in the active world/reference frame.
     */
    bbtc_eb_vec3_float_t velocity_m_per_s;

    /**
     * @brief Projectile angular velocity, in rad/s, expressed in body axes.
     *
     * Components correspond to angular velocity about the canonical BBTC body
     * +X forward, +Y right, and +Z down axes. Positive rotation follows the
     * right-hand rule.
     */
    bbtc_eb_vec3_float_t angular_velocity_body_rad_per_s;

    /** Simulation time in seconds measured from the EB initial state. */
    float time_s;
}
bbtc_eb_rigid_body_state_float_t;


/**
 * @brief Native-double external-ballistics rigid-body state.
 *
 * This record has exactly the same physical and frame semantics as
 * `bbtc_eb_rigid_body_state_float_t` while preserving native `double`
 * representation throughout the state.
 *
 * The double family is the initial recommended integration family for Cubes of
 * Honor, but the host remains responsible for explicit conversion to and from
 * its own engine-math types.
 */
typedef struct bbtc_eb_rigid_body_state_double_t
{
    /**
     * @brief Active world/reference-from-body orientation.
     *
     * When valid and unit length, this quaternion rotates a vector from the
     * BBTC projectile body frame into the active world/reference frame.
     */
    bbtc_eb_quat_double_t orientation_world_from_body;

    /**
     * @brief Projectile center-of-mass position, in metres.
     *
     * Components are expressed in the active world/reference frame. A host may
     * use a local integration origin and reconstruct its absolute world
     * position at the adapter boundary.
     */
    bbtc_eb_vec3_double_t position_m;

    /**
     * @brief Projectile center-of-mass translational velocity, in m/s.
     *
     * Components are expressed in the active world/reference frame.
     */
    bbtc_eb_vec3_double_t velocity_m_per_s;

    /**
     * @brief Projectile angular velocity, in rad/s, expressed in body axes.
     *
     * Components correspond to angular velocity about the canonical BBTC body
     * +X forward, +Y right, and +Z down axes. Positive rotation follows the
     * right-hand rule.
     */
    bbtc_eb_vec3_double_t angular_velocity_body_rad_per_s;

    /** Simulation time in seconds measured from the EB initial state. */
    double time_s;
}
bbtc_eb_rigid_body_state_double_t;


/**
 * @brief Native-long-double external-ballistics rigid-body state.
 *
 * This record has the same physical and frame semantics as the other state
 * families while retaining native `long double` values. No assumption is made
 * that `long double` is wider than `double` on every supported platform.
 */
typedef struct bbtc_eb_rigid_body_state_long_double_t
{
    /**
     * @brief Active world/reference-from-body orientation.
     *
     * When valid and unit length, this quaternion rotates a vector from the
     * BBTC projectile body frame into the active world/reference frame.
     */
    bbtc_eb_quat_long_double_t orientation_world_from_body;

    /**
     * @brief Projectile center-of-mass position, in metres.
     *
     * Components are expressed in the active world/reference frame.
     */
    bbtc_eb_vec3_long_double_t position_m;

    /**
     * @brief Projectile center-of-mass translational velocity, in m/s.
     *
     * Components are expressed in the active world/reference frame.
     */
    bbtc_eb_vec3_long_double_t velocity_m_per_s;

    /**
     * @brief Projectile angular velocity, in rad/s, expressed in body axes.
     *
     * Components correspond to angular velocity about the canonical BBTC body
     * +X forward, +Y right, and +Z down axes. Positive rotation follows the
     * right-hand rule.
     */
    bbtc_eb_vec3_long_double_t angular_velocity_body_rad_per_s;

    /** Simulation time in seconds measured from the EB initial state. */
    long double time_s;
}
bbtc_eb_rigid_body_state_long_double_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Validates one native-float external-ballistics rigid-body state.
 *
 * Validation is read-only and fail-closed. The state pointer must be nonnull;
 * every quaternion, position, velocity, angular-velocity, and time component
 * must be finite; time must be nonnegative; and the world-from-body
 * orientation must be nonzero and sufficiently close to unit length.
 *
 * The orientation acceptance test is
 *
 * `abs(norm_squared - 1) <= 64 * FLT_EPSILON`.
 *
 * The common float-sized acceptance floor is intentional so a quaternion that
 * was normalized in a float-oriented host such as Cubes of Honor can be
 * converted component-by-component into another BBTC precision family without
 * being rejected solely because the destination family has a smaller epsilon.
 *
 * When multiple invalid conditions exist, NaN takes precedence over infinity,
 * and infinity takes precedence over domain failures. The function never
 * normalizes, repairs, clamps, or otherwise modifies the caller-owned state.
 *
 * @param state State to validate.
 *
 * @return `BBTC_STATUS_SUCCESS` when the complete state is valid;
 *         `BBTC_STATUS_INVALID_ARGUMENT` for a null pointer;
 *         `BBTC_STATUS_NAN_INPUT` when any inspected scalar is NaN;
 *         `BBTC_STATUS_NONFINITE_INPUT` when no NaN exists but an infinity
 *         does; otherwise `BBTC_STATUS_OUTSIDE_DOMAIN` for negative time or an
 *         invalid orientation.
 */
bbtc_status_e
bbtc_eb_rigid_body_state_validate_float(
    const bbtc_eb_rigid_body_state_float_t* state
);

/**
 * @brief Validates one native-double external-ballistics rigid-body state.
 *
 * The physical requirements, validation precedence, orientation tolerance,
 * ownership rules, and read-only behavior are identical to
 * `bbtc_eb_rigid_body_state_validate_float()`. All arithmetic used to inspect
 * the double state remains native `double` arithmetic.
 *
 * @param state State to validate.
 *
 * @return `BBTC_STATUS_SUCCESS` when valid; otherwise the deterministic
 *         validation status described by the float-family function.
 */
bbtc_status_e
bbtc_eb_rigid_body_state_validate_double(
    const bbtc_eb_rigid_body_state_double_t* state
);

/**
 * @brief Validates one native-long-double external-ballistics rigid-body state.
 *
 * The physical requirements, validation precedence, orientation tolerance,
 * ownership rules, and read-only behavior are identical to
 * `bbtc_eb_rigid_body_state_validate_float()`. All arithmetic used to inspect
 * the long-double state remains native `long double` arithmetic.
 *
 * @param state State to validate.
 *
 * @return `BBTC_STATUS_SUCCESS` when valid; otherwise the deterministic
 *         validation status described by the float-family function.
 */
bbtc_status_e
bbtc_eb_rigid_body_state_validate_long_double(
    const bbtc_eb_rigid_body_state_long_double_t* state
);

#ifdef __cplusplus
}   /* extern "C" */
#endif

#endif /* BBTC_EXTERNAL_BALLISTICS_STATE_H */
