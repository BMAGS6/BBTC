/**
 * @file
 * @brief Precision-qualified external-ballistics applied force/moment records.
 *
 * EB0.1-B8 defines the public value boundary used to carry resultant physical
 * mechanical loading into later rigid-body equations of motion. The record is
 * deliberately small: it contains one applied-force vector and one
 * applied-moment vector, with their frames and reference point fixed by the
 * external-ballistics design contract.
 *
 * Frame semantics are intentionally mixed because the rigid-body state already
 * uses different natural frames for translation and rotation:
 *
 * - applied force is expressed in the active BBTC reference frame, matching
 *   translational position and velocity; and
 * - applied moment is expressed in the canonical BBTC projectile body frame
 *   about the projectile center of mass, matching body-frame angular velocity
 *   and principal-axis inertia.
 *
 * The B8 applied force contains physical applied force only. It must not
 * duplicate gravity or future non-inertial reference-frame accelerations by
 * converting them to force-equivalent values. Likewise, the B8 applied moment
 * contains physical applied torque only and does not carry fictitious
 * rotational correction terms.
 *
 * EB0.1-B8 introduces representation and validation only. It does not evaluate
 * aerodynamic force, propulsion, gravity-gradient torque, rigid-body
 * derivatives, equations of motion, or numerical integration.
 */

#ifndef BBTC_EXTERNAL_BALLISTICS_FORCE_MOMENT_H
#define BBTC_EXTERNAL_BALLISTICS_FORCE_MOMENT_H

#include <bbtc/external_ballistics/vector.h>
#include <bbtc/status.h>

/**
 * @brief Native-float resultant applied force and moment.
 *
 * An all-zero record is valid and represents no applied force and no applied
 * moment. Arbitrary finite positive, negative, and zero components are valid.
 */
typedef struct bbtc_eb_force_moment_float_t
{
    /**
     * @brief Resultant applied physical force, in newtons.
     *
     * Components are expressed in the active BBTC reference frame. Gravity and
     * reference-frame non-inertial accelerations are not represented here.
     */
    bbtc_eb_vec3_float_t applied_force_reference_n;

    /**
     * @brief Resultant applied physical moment about the center of mass.
     *
     * Components are in newton-metres and are expressed in the canonical BBTC
     * projectile body frame: +X forward, +Y right, +Z down.
     */
    bbtc_eb_vec3_float_t applied_moment_body_n_m;
}
bbtc_eb_force_moment_float_t;

/**
 * @brief Native-double resultant applied force and moment.
 *
 * Physical meaning is identical to `bbtc_eb_force_moment_float_t`; only the
 * native scalar family differs.
 */
typedef struct bbtc_eb_force_moment_double_t
{
    /** Resultant physical force in newtons, in active-reference axes. */
    bbtc_eb_vec3_double_t applied_force_reference_n;

    /** Resultant physical moment in N*m about COM, in projectile body axes. */
    bbtc_eb_vec3_double_t applied_moment_body_n_m;
}
bbtc_eb_force_moment_double_t;

/**
 * @brief Native-long-double resultant applied force and moment.
 *
 * Physical meaning is identical to the other B8 records while preserving
 * native `long double` components throughout.
 */
typedef struct bbtc_eb_force_moment_long_double_t
{
    /** Resultant physical force in newtons, in active-reference axes. */
    bbtc_eb_vec3_long_double_t applied_force_reference_n;

    /** Resultant physical moment in N*m about COM, in projectile body axes. */
    bbtc_eb_vec3_long_double_t applied_moment_body_n_m;
}
bbtc_eb_force_moment_long_double_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Validates one native-float B8 applied force/moment record.
 *
 * Validation is read-only and deliberately limited to representational
 * finiteness. It does not rotate, accumulate, normalize, clamp, derive, or
 * otherwise modify either vector, and it does not impose a relationship
 * between force and moment.
 *
 * Deterministic failure precedence is:
 *
 * 1. null pointer -> `BBTC_STATUS_INVALID_ARGUMENT`;
 * 2. any NaN in the complete six-scalar record -> `BBTC_STATUS_NAN_INPUT`;
 * 3. otherwise any positive or negative infinity ->
 *    `BBTC_STATUS_NONFINITE_INPUT`; and
 * 4. otherwise -> `BBTC_STATUS_SUCCESS`.
 *
 * The complete record is scanned for NaN before any infinity result is
 * returned. Zero, negative, positive, and arbitrarily large finite components
 * are all valid.
 *
 * @param force_moment Applied force/moment record to inspect.
 *
 * @return Deterministic validation status described above.
 */
bbtc_status_e
bbtc_eb_force_moment_validate_float(
    const bbtc_eb_force_moment_float_t* force_moment
);

/**
 * @brief Validates one native-double B8 applied force/moment record.
 *
 * Semantics, failure precedence, and read-only behavior match
 * `bbtc_eb_force_moment_validate_float()`. Inspection remains native `double`.
 *
 * @param force_moment Applied force/moment record to inspect.
 *
 * @return Deterministic validation status described by the float-family API.
 */
bbtc_status_e
bbtc_eb_force_moment_validate_double(
    const bbtc_eb_force_moment_double_t* force_moment
);

/**
 * @brief Validates one native-long-double B8 applied force/moment record.
 *
 * Semantics, failure precedence, and read-only behavior match
 * `bbtc_eb_force_moment_validate_float()`. Inspection remains native
 * `long double`.
 *
 * @param force_moment Applied force/moment record to inspect.
 *
 * @return Deterministic validation status described by the float-family API.
 */
bbtc_status_e
bbtc_eb_force_moment_validate_long_double(
    const bbtc_eb_force_moment_long_double_t* force_moment
);

#ifdef __cplusplus
}   /* extern "C" */
#endif

#endif /* BBTC_EXTERNAL_BALLISTICS_FORCE_MOMENT_H */
