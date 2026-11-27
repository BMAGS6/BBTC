/**
 * @file
 * @brief Precision-qualified external-ballistics projectile physical properties.
 *
 * EB0.1-B3 separates immutable rigid-projectile identity from the integrated
 * six-degree-of-freedom state. The records in this header describe projectile
 * mass, principal-axis inertia about the center of mass, and aerodynamic
 * reference geometry. They do not contain aerodynamic coefficients,
 * environment state, force/moment data, or integration state.
 *
 * The canonical BBTC projectile body frame is right-handed:
 *
 * - +X points forward toward the projectile nose and is the longitudinal/roll
 *   axis;
 * - +Y points body-right and is the pitch axis; and
 * - +Z points body-down and is the yaw axis.
 *
 * For EB0.1-B3 the body-frame origin is the projectile center of mass. The
 * inertia records therefore contain principal moments about axes through the
 * center of mass.
 */

#ifndef BBTC_EXTERNAL_BALLISTICS_PROJECTILE_H
#define BBTC_EXTERNAL_BALLISTICS_PROJECTILE_H

#include <bbtc/status.h>

/**
 * @brief Native-float diagonal principal moments of inertia.
 *
 * The canonical BBTC body axes must be principal axes for this representation.
 * Axisymmetry is not required: the three moments may differ.
 */
typedef struct bbtc_eb_principal_inertia_float_t
{
    /** Moment of inertia about body +X, in kg*m^2. */
    float moment_about_x_kg_m2;

    /** Moment of inertia about body +Y, in kg*m^2. */
    float moment_about_y_kg_m2;

    /** Moment of inertia about body +Z, in kg*m^2. */
    float moment_about_z_kg_m2;
}
bbtc_eb_principal_inertia_float_t;


/**
 * @brief Native-double diagonal principal moments of inertia.
 *
 * The canonical BBTC body axes must be principal axes for this representation.
 * Axisymmetry is not required: the three moments may differ.
 */
typedef struct bbtc_eb_principal_inertia_double_t
{
    /** Moment of inertia about body +X, in kg*m^2. */
    double moment_about_x_kg_m2;

    /** Moment of inertia about body +Y, in kg*m^2. */
    double moment_about_y_kg_m2;

    /** Moment of inertia about body +Z, in kg*m^2. */
    double moment_about_z_kg_m2;
}
bbtc_eb_principal_inertia_double_t;


/**
 * @brief Native-long-double diagonal principal moments of inertia.
 *
 * The canonical BBTC body axes must be principal axes for this representation.
 * Axisymmetry is not required: the three moments may differ.
 */
typedef struct bbtc_eb_principal_inertia_long_double_t
{
    /** Moment of inertia about body +X, in kg*m^2. */
    long double moment_about_x_kg_m2;

    /** Moment of inertia about body +Y, in kg*m^2. */
    long double moment_about_y_kg_m2;

    /** Moment of inertia about body +Z, in kg*m^2. */
    long double moment_about_z_kg_m2;
}
bbtc_eb_principal_inertia_long_double_t;


/**
 * @brief Native-float rigid-projectile physical properties.
 *
 * Reference diameter and area are explicit aerodynamic reference quantities.
 * BBTC does not silently derive either one from the other and does not assume
 * the circular relation pi*d^2/4 at this validation boundary.
 */
typedef struct bbtc_eb_projectile_properties_float_t
{
    /** Principal moments about the center of mass and canonical body axes. */
    bbtc_eb_principal_inertia_float_t principal_inertia;

    /** Projectile mass, in kilograms. */
    float mass_kg;

    /** Positive aerodynamic/reference diameter, in metres. */
    float reference_diameter_m;

    /** Positive aerodynamic/reference area, in square metres. */
    float reference_area_m2;
}
bbtc_eb_projectile_properties_float_t;


/**
 * @brief Native-double rigid-projectile physical properties.
 *
 * Reference diameter and area are explicit aerodynamic reference quantities.
 * BBTC does not silently derive either one from the other and does not assume
 * the circular relation pi*d^2/4 at this validation boundary.
 */
typedef struct bbtc_eb_projectile_properties_double_t
{
    /** Principal moments about the center of mass and canonical body axes. */
    bbtc_eb_principal_inertia_double_t principal_inertia;

    /** Projectile mass, in kilograms. */
    double mass_kg;

    /** Positive aerodynamic/reference diameter, in metres. */
    double reference_diameter_m;

    /** Positive aerodynamic/reference area, in square metres. */
    double reference_area_m2;
}
bbtc_eb_projectile_properties_double_t;


/**
 * @brief Native-long-double rigid-projectile physical properties.
 *
 * Reference diameter and area are explicit aerodynamic reference quantities.
 * BBTC does not silently derive either one from the other and does not assume
 * the circular relation pi*d^2/4 at this validation boundary.
 */
typedef struct bbtc_eb_projectile_properties_long_double_t
{
    /** Principal moments about the center of mass and canonical body axes. */
    bbtc_eb_principal_inertia_long_double_t principal_inertia;

    /** Projectile mass, in kilograms. */
    long double mass_kg;

    /** Positive aerodynamic/reference diameter, in metres. */
    long double reference_diameter_m;

    /** Positive aerodynamic/reference area, in square metres. */
    long double reference_area_m2;
}
bbtc_eb_projectile_properties_long_double_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Validates one native-float EB projectile-property record.
 *
 * Validation is read-only and fail-closed. All stored scalars must be finite;
 * mass, each principal moment, reference diameter, and reference area must be
 * strictly positive; and the three principal moments must satisfy the rigid-
 * body inertia triangle inequalities. Axisymmetry is not required.
 *
 * Deterministic failure precedence is null pointer, NaN, infinity, nonpositive
 * physical quantities, then inertia-triangle violation. Validation never
 * derives reference geometry, repairs inertia, or mutates caller storage.
 *
 * @param properties Projectile properties to validate.
 *
 * @return `BBTC_STATUS_SUCCESS` when valid;
 *         `BBTC_STATUS_INVALID_ARGUMENT` for a null pointer;
 *         `BBTC_STATUS_NAN_INPUT` when any inspected scalar is NaN;
 *         `BBTC_STATUS_NONFINITE_INPUT` when no NaN exists but an infinity
 *         does; otherwise `BBTC_STATUS_OUTSIDE_DOMAIN` for a physical-domain
 *         violation.
 */
bbtc_status_e
bbtc_eb_projectile_properties_validate_float(
    const bbtc_eb_projectile_properties_float_t* properties
);

/**
 * @brief Validates one native-double EB projectile-property record.
 *
 * The physical requirements, read-only behavior, and failure precedence are
 * identical to `bbtc_eb_projectile_properties_validate_float()`. Inspection
 * remains native `double`.
 *
 * @param properties Projectile properties to validate.
 *
 * @return The deterministic validation status described by the float-family
 *         function.
 */
bbtc_status_e
bbtc_eb_projectile_properties_validate_double(
    const bbtc_eb_projectile_properties_double_t* properties
);

/**
 * @brief Validates one native-long-double EB projectile-property record.
 *
 * The physical requirements, read-only behavior, and failure precedence are
 * identical to `bbtc_eb_projectile_properties_validate_float()`. Inspection
 * remains native `long double`.
 *
 * @param properties Projectile properties to validate.
 *
 * @return The deterministic validation status described by the float-family
 *         function.
 */
bbtc_status_e
bbtc_eb_projectile_properties_validate_long_double(
    const bbtc_eb_projectile_properties_long_double_t* properties
);

#ifdef __cplusplus
}   /* extern "C" */
#endif

#endif /* BBTC_EXTERNAL_BALLISTICS_PROJECTILE_H */
