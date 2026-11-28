/**
 * @file
 * @brief Derived projectile-relative-air quantities for external ballistics.
 *
 * EB2.1 establishes the kinematic and scalar-flow boundary between a projectile
 * translational velocity and one already sampled B4 environment value. The
 * evaluator defined here does not sample an environment provider, evaluate a
 * drag coefficient, produce aerodynamic force/moment data, or integrate state.
 *
 * Relative-air velocity is explicitly projectile velocity minus local air
 * velocity, with both vectors expressed in the active BBTC world/reference
 * frame. Later drag models therefore oppose this stored vector.
 *
 * Zero-density environment is treated as aerodynamic vacuum: relative velocity
 * and relative speed are still derived, while Mach number and dynamic pressure
 * are exactly zero. This finite Mach convention is an aerodynamic no-medium
 * sentinel and is not a claim that acoustic Mach number is physically defined
 * in vacuum.
 */

#ifndef BBTC_EXTERNAL_BALLISTICS_RELATIVE_AIR_H
#define BBTC_EXTERNAL_BALLISTICS_RELATIVE_AIR_H

#include <bbtc/external_ballistics/environment.h>
#include <bbtc/external_ballistics/vector.h>
#include <bbtc/status.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Native-float projectile-relative-air derived-flow result.
 *
 * The vector is expressed in the active world/reference frame and is defined as
 * projectile velocity minus local air velocity. Scalar speed is its Euclidean
 * norm. Mach and dynamic pressure follow the EB2.1 zero-density and
 * positive-density conventions.
 */
typedef struct bbtc_eb_relative_air_float_t
{
    /** Projectile velocity relative to air, in active world axes, m/s. */
    bbtc_eb_vec3_float_t projectile_relative_to_air_velocity_world_m_per_s;

    /** Euclidean magnitude of the relative-air velocity, in m/s. */
    float projectile_relative_to_air_speed_m_per_s;

    /** Aerodynamic Mach number; exactly zero for zero-density medium. */
    float mach_number;

    /** Aerodynamic dynamic pressure, in pascals. */
    float dynamic_pressure_pa;
}
bbtc_eb_relative_air_float_t;


/**
 * @brief Native-double projectile-relative-air derived-flow result.
 *
 * Semantics are identical to `bbtc_eb_relative_air_float_t` while all vector
 * and scalar storage remains native `double`.
 */
typedef struct bbtc_eb_relative_air_double_t
{
    /** Projectile velocity relative to air, in active world axes, m/s. */
    bbtc_eb_vec3_double_t projectile_relative_to_air_velocity_world_m_per_s;

    /** Euclidean magnitude of the relative-air velocity, in m/s. */
    double projectile_relative_to_air_speed_m_per_s;

    /** Aerodynamic Mach number; exactly zero for zero-density medium. */
    double mach_number;

    /** Aerodynamic dynamic pressure, in pascals. */
    double dynamic_pressure_pa;
}
bbtc_eb_relative_air_double_t;


/**
 * @brief Native-long-double projectile-relative-air derived-flow result.
 *
 * Semantics are identical to the other precision families while all vector and
 * scalar storage remains native `long double`.
 */
typedef struct bbtc_eb_relative_air_long_double_t
{
    /** Projectile velocity relative to air, in active world axes, m/s. */
    bbtc_eb_vec3_long_double_t projectile_relative_to_air_velocity_world_m_per_s;

    /** Euclidean magnitude of the relative-air velocity, in m/s. */
    long double projectile_relative_to_air_speed_m_per_s;

    /** Aerodynamic Mach number; exactly zero for zero-density medium. */
    long double mach_number;

    /** Aerodynamic dynamic pressure, in pascals. */
    long double dynamic_pressure_pa;
}
bbtc_eb_relative_air_long_double_t;


/**
 * @brief Evaluate native-float projectile-relative-air derived flow.
 *
 * Relative velocity is:
 *
 * `v_relative = projectile_velocity_world - environment.air_velocity_world`.
 *
 * Positive-density Mach is relative speed divided by the B4 speed of sound.
 * Zero-density Mach is exactly zero without sound-speed division. Dynamic
 * pressure is `0.5 * rho * speed^2`, evaluated in an overflow/underflow-aware
 * form so representable results are not lost to avoidable intermediate
 * arithmetic.
 *
 * A nonnull output is zeroed before input validation and remains exact-zero on
 * every failure. NaN anywhere across both complete inputs has precedence over
 * infinity anywhere across both inputs; B4 physical-domain validation follows.
 * Nonfinite values generated from otherwise valid finite inputs return
 * `BBTC_STATUS_NUMERICAL_FAILURE`.
 *
 * @param projectile_velocity_world_m_per_s Finite projectile velocity in the
 *        active BBTC world/reference frame, in metres per second.
 * @param environment Already sampled B4 environment value; validated here.
 * @param out_relative_air Required destination result.
 *
 * @return `BBTC_STATUS_SUCCESS` on a complete finite result, or the deterministic
 *         EB2.1 validation/numerical status on failure.
 */
bbtc_status_e
bbtc_eb_relative_air_evaluate_float(
    const bbtc_eb_vec3_float_t* projectile_velocity_world_m_per_s,
    const bbtc_eb_environment_sample_float_t* environment,
    bbtc_eb_relative_air_float_t* out_relative_air
);


/**
 * @brief Native-double relative-air evaluation with identical EB2.1 semantics.
 */
bbtc_status_e
bbtc_eb_relative_air_evaluate_double(
    const bbtc_eb_vec3_double_t* projectile_velocity_world_m_per_s,
    const bbtc_eb_environment_sample_double_t* environment,
    bbtc_eb_relative_air_double_t* out_relative_air
);


/**
 * @brief Native-long-double relative-air evaluation with identical semantics.
 */
bbtc_status_e
bbtc_eb_relative_air_evaluate_long_double(
    const bbtc_eb_vec3_long_double_t* projectile_velocity_world_m_per_s,
    const bbtc_eb_environment_sample_long_double_t* environment,
    bbtc_eb_relative_air_long_double_t* out_relative_air
);

#ifdef __cplusplus
}   /* extern "C" */
#endif

#endif /* BBTC_EXTERNAL_BALLISTICS_RELATIVE_AIR_H */
