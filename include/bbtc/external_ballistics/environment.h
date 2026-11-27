/**
 * @file
 * @brief Precision-qualified external-ballistics local environment samples.
 *
 * EB0.1-B4 defines one already-sampled local atmospheric/environment value.
 * The record deliberately does not define how the sample is obtained. A future
 * provider interface may evaluate a constant atmosphere, standard atmosphere,
 * table, host weather system, or another model at a projectile position and
 * simulation time, but all such providers feed the same value contract defined
 * here.
 *
 * The environment sample is also deliberately distinct from projectile state.
 * It contains properties of the local medium, not projectile-relative air
 * velocity, aerodynamic coefficients, forces, moments, gravity, or integration
 * state.
 */

#ifndef BBTC_EXTERNAL_BALLISTICS_ENVIRONMENT_H
#define BBTC_EXTERNAL_BALLISTICS_ENVIRONMENT_H

#include <bbtc/external_ballistics/vector.h>
#include <bbtc/status.h>

/**
 * @brief Native-float sampled local environment.
 *
 * Air velocity is expressed in the active BBTC world/reference frame. The
 * remaining fields are caller-supplied local thermodynamic/acoustic quantities.
 * BBTC does not derive one stored quantity from another at this boundary.
 */
typedef struct bbtc_eb_environment_sample_float_t
{
    /**
     * @brief Velocity of the local atmospheric medium, in metres per second.
     *
     * Components are expressed in the active BBTC world/reference frame used
     * by rigid-body translational velocity. This is not projectile-relative air
     * velocity; a later physics layer derives that quantity from projectile and
     * medium velocities.
     */
    bbtc_eb_vec3_float_t air_velocity_world_m_per_s;

    /** Local absolute thermodynamic temperature, in kelvin. */
    float temperature_k;

    /** Local absolute static pressure, in pascals. */
    float absolute_pressure_pa;

    /** Local atmospheric-medium mass density, in kilograms per cubic metre. */
    float air_density_kg_per_m3;

    /**
     * @brief Local acoustic propagation speed, in metres per second.
     *
     * Zero is permitted only when medium density is zero. Future aerodynamic
     * code must recognize zero-density vacuum before attempting Mach-number
     * normalization and must not divide relative-air speed by zero.
     */
    float speed_of_sound_m_per_s;
}
bbtc_eb_environment_sample_float_t;


/**
 * @brief Native-double sampled local environment.
 *
 * Semantics are identical to the native-float family while all vector and
 * scalar storage remains native `double`.
 */
typedef struct bbtc_eb_environment_sample_double_t
{
    /**
     * @brief Velocity of the local atmospheric medium, in metres per second.
     *
     * Components are expressed in the active BBTC world/reference frame used
     * by rigid-body translational velocity. Positive, negative, and zero
     * components are all physically admissible.
     */
    bbtc_eb_vec3_double_t air_velocity_world_m_per_s;

    /** Local absolute thermodynamic temperature, in kelvin. */
    double temperature_k;

    /** Local absolute static pressure, in pascals. */
    double absolute_pressure_pa;

    /** Local atmospheric-medium mass density, in kilograms per cubic metre. */
    double air_density_kg_per_m3;

    /**
     * @brief Local acoustic propagation speed, in metres per second.
     *
     * Positive-density media require a positive value. A zero-density sample
     * may use zero so exact vacuum is representable without epsilon-valued
     * atmosphere fields.
     */
    double speed_of_sound_m_per_s;
}
bbtc_eb_environment_sample_double_t;


/**
 * @brief Native-long-double sampled local environment.
 *
 * Semantics are identical to the other precision families while all vector and
 * scalar storage remains native `long double`.
 */
typedef struct bbtc_eb_environment_sample_long_double_t
{
    /**
     * @brief Velocity of the local atmospheric medium, in metres per second.
     *
     * Components are expressed in the active BBTC world/reference frame and
     * carry no hidden geographic axis convention.
     */
    bbtc_eb_vec3_long_double_t air_velocity_world_m_per_s;

    /** Local absolute thermodynamic temperature, in kelvin. */
    long double temperature_k;

    /** Local absolute static pressure, in pascals. */
    long double absolute_pressure_pa;

    /** Local atmospheric-medium mass density, in kilograms per cubic metre. */
    long double air_density_kg_per_m3;

    /** Local acoustic propagation speed, in metres per second. */
    long double speed_of_sound_m_per_s;
}
bbtc_eb_environment_sample_long_double_t;

#ifdef __cplusplus
    extern "C" {
#endif

/**
 * @brief Validates one native-float local environment sample.
 *
 * Validation is read-only and fail-closed. All vector components and scalar
 * fields must be finite. Temperature must be strictly positive; absolute
 * pressure, density, and sound speed must be nonnegative; and any
 * positive-density medium must have strictly positive sound speed.
 *
 * Exact vacuum is representable with zero pressure, zero density, and zero
 * sound speed while retaining a positive finite temperature. Validation does
 * not impose an ideal-gas law, standard-atmosphere relation, or another hidden
 * equation of state, and it never derives or repairs caller values.
 *
 * Deterministic failure precedence is null pointer, NaN anywhere in the record,
 * infinity anywhere in the record, then physical-domain failure.
 *
 * @param sample Local environment sample to validate.
 *
 * @return `BBTC_STATUS_SUCCESS` when valid;
 *         `BBTC_STATUS_INVALID_ARGUMENT` for a null pointer;
 *         `BBTC_STATUS_NAN_INPUT` when any inspected scalar is NaN;
 *         `BBTC_STATUS_NONFINITE_INPUT` when no NaN exists but an infinity
 *         does; otherwise `BBTC_STATUS_OUTSIDE_DOMAIN` for a physical-domain
 *         violation.
 */
bbtc_status_e
bbtc_eb_environment_sample_validate_float(
    const bbtc_eb_environment_sample_float_t* sample
);

/**
 * @brief Validates one native-double local environment sample.
 *
 * The domain, vacuum, read-only, non-derivation, and deterministic-diagnostic
 * rules are identical to `bbtc_eb_environment_sample_validate_float()`.
 * Inspection remains native `double`.
 *
 * @param sample Local environment sample to validate.
 *
 * @return The deterministic validation status described by the float-family
 *         function.
 */
bbtc_status_e
bbtc_eb_environment_sample_validate_double(
    const bbtc_eb_environment_sample_double_t* sample
);

/**
 * @brief Validates one native-long-double local environment sample.
 *
 * The domain, vacuum, read-only, non-derivation, and deterministic-diagnostic
 * rules are identical to `bbtc_eb_environment_sample_validate_float()`.
 * Inspection remains native `long double`.
 *
 * @param sample Local environment sample to validate.
 *
 * @return The deterministic validation status described by the float-family
 *         function.
 */
bbtc_status_e
bbtc_eb_environment_sample_validate_long_double(
    const bbtc_eb_environment_sample_long_double_t* sample
);

#ifdef __cplusplus
    }    /* extern "C" */
#endif
#endif /* BBTC_EXTERNAL_BALLISTICS_ENVIRONMENT_H */
