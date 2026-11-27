/**
 * @file
 * @brief Precision-qualified external-ballistics reference-frame samples.
 *
 * EB0.1-B6 defines one already-sampled local gravity/reference-frame value.
 * The sample contains true gravitational-field acceleration plus the complete
 * translational and rotational kinematics of the active BBTC reference frame
 * relative to inertial space. It deliberately does not evaluate Coriolis,
 * centrifugal, Euler, Eötvös, atmosphere, aerodynamic, or integration effects.
 *
 * All vectors are expressed in the axes of the active BBTC reference frame.
 * The all-zero sample is valid and represents an inertial, nonaccelerating,
 * nonrotating, zero-gravity condition.
 */

#ifndef BBTC_EXTERNAL_BALLISTICS_REFERENCE_FRAME_H
#define BBTC_EXTERNAL_BALLISTICS_REFERENCE_FRAME_H

#include <bbtc/external_ballistics/vector.h>
#include <bbtc/status.h>

/**
 * @brief Native-float sampled gravity/reference-frame state.
 */
typedef struct bbtc_eb_reference_frame_sample_float_t
{
    /** Physical gravitational-field acceleration at the sample point. */
    bbtc_eb_vec3_float_t gravitational_acceleration_reference_m_per_s2;

    /** Complete inertial acceleration of the active reference-frame origin. */
    bbtc_eb_vec3_float_t
        reference_frame_origin_acceleration_reference_m_per_s2;

    /**
     * @brief Active-frame angular velocity relative to inertial space.
     *
     * Components are expressed in active-reference axes, in radians per second.
     */
    bbtc_eb_vec3_float_t
        reference_frame_angular_velocity_reference_rad_per_s;

    /**
     * @brief Inertial derivative of active-frame angular velocity.
     *
     * Components are expressed in active-reference axes, in radians per second
     * squared.
     */
    bbtc_eb_vec3_float_t
        reference_frame_angular_acceleration_reference_rad_per_s2;
}
bbtc_eb_reference_frame_sample_float_t;


/**
 * @brief Native-double sampled gravity/reference-frame state.
 *
 * Semantics are identical to the native-float family while every vector
 * component remains native `double`.
 */
typedef struct bbtc_eb_reference_frame_sample_double_t
{
    /** Physical gravitational-field acceleration at the sample point. */
    bbtc_eb_vec3_double_t gravitational_acceleration_reference_m_per_s2;

    /** Complete inertial acceleration of the active reference-frame origin. */
    bbtc_eb_vec3_double_t
        reference_frame_origin_acceleration_reference_m_per_s2;

    /** Active-frame angular velocity relative to inertial space. */
    bbtc_eb_vec3_double_t
        reference_frame_angular_velocity_reference_rad_per_s;

    /** Inertial derivative of active-frame angular velocity. */
    bbtc_eb_vec3_double_t
        reference_frame_angular_acceleration_reference_rad_per_s2;
}
bbtc_eb_reference_frame_sample_double_t;


/**
 * @brief Native-long-double sampled gravity/reference-frame state.
 *
 * Semantics are identical to the other precision families while every vector
 * component remains native `long double`.
 */
typedef struct bbtc_eb_reference_frame_sample_long_double_t
{
    /** Physical gravitational-field acceleration at the sample point. */
    bbtc_eb_vec3_long_double_t gravitational_acceleration_reference_m_per_s2;

    /** Complete inertial acceleration of the active reference-frame origin. */
    bbtc_eb_vec3_long_double_t
        reference_frame_origin_acceleration_reference_m_per_s2;

    /** Active-frame angular velocity relative to inertial space. */
    bbtc_eb_vec3_long_double_t
        reference_frame_angular_velocity_reference_rad_per_s;

    /** Inertial derivative of active-frame angular velocity. */
    bbtc_eb_vec3_long_double_t
        reference_frame_angular_acceleration_reference_rad_per_s2;
}
bbtc_eb_reference_frame_sample_long_double_t;

#ifdef __cplusplus
    extern "C" {
#endif

/**
 * @brief Validates one native-float B6 reference-frame sample.
 *
 * Validation is read-only. A null pointer is invalid. Otherwise, NaN anywhere
 * in the complete record takes precedence over infinity anywhere in the
 * complete record. Every finite vector value is accepted; no Earth-specific,
 * magnitude, sign, direction, or cross-field relation is imposed.
 *
 * @param sample Reference-frame sample to validate.
 *
 * @return `BBTC_STATUS_SUCCESS` when every component is finite;
 *         `BBTC_STATUS_INVALID_ARGUMENT` for a null pointer;
 *         `BBTC_STATUS_NAN_INPUT` when any component is NaN; otherwise
 *         `BBTC_STATUS_NONFINITE_INPUT` when an infinity exists.
 */
bbtc_status_e
bbtc_eb_reference_frame_sample_validate_float(
    const bbtc_eb_reference_frame_sample_float_t* sample
);

/**
 * @brief Validates one native-double B6 reference-frame sample.
 *
 * Semantics and diagnostic precedence are identical to the native-float
 * validator while inspection remains native `double`.
 *
 * @param sample Reference-frame sample to validate.
 *
 * @return The deterministic validation status described by the float family.
 */
bbtc_status_e
bbtc_eb_reference_frame_sample_validate_double(
    const bbtc_eb_reference_frame_sample_double_t* sample
);

/**
 * @brief Validates one native-long-double B6 reference-frame sample.
 *
 * Semantics and diagnostic precedence are identical to the native-float
 * validator while inspection remains native `long double`.
 *
 * @param sample Reference-frame sample to validate.
 *
 * @return The deterministic validation status described by the float family.
 */
bbtc_status_e
bbtc_eb_reference_frame_sample_validate_long_double(
    const bbtc_eb_reference_frame_sample_long_double_t* sample
);

#ifdef __cplusplus
    }    /* extern "C" */
#endif

#endif /* BBTC_EXTERNAL_BALLISTICS_REFERENCE_FRAME_H */
