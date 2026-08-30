/**
 * @file propellant_grain_geometry.h
 * @brief Canonical propellant-grain regression geometry.
 *
 * @details
 * This module evaluates the geometry of one idealized propellant grain after
 * an explicitly supplied uniform normal surface-regression depth. It reports
 * remaining solid volume, exposed burning surface area, remaining regression
 * distance to geometric burnout, and consumed-volume fraction.
 *
 * Geometry is chemically agnostic. The module does not determine burn rate,
 * ignition, thermochemistry, pressure, temperature, reaction-source rate,
 * grain population behavior, projectile motion, or firearm safety.
 */
#ifndef BBTC_INTERNAL_BALLISTICS_PROPELLANT_GRAIN_GEOMETRY_H
#define BBTC_INTERNAL_BALLISTICS_PROPELLANT_GRAIN_GEOMETRY_H

#include <bbtc/status.h>

#ifdef __cplusplus
    extern "C" {
#endif

/**
 * @struct bbtc_ib_spherical_grain_geometry_float_t
 * @brief Spherical grain geometry expressed with native `float`.
 */
typedef struct bbtc_ib_spherical_grain_geometry_float_t
{
    /** Initial grain radius, in meters. */
    float initial_radius_m;
}
bbtc_ib_spherical_grain_geometry_float_t;


/**
 * @struct bbtc_ib_spherical_grain_geometry_double_t
 * @brief Spherical grain geometry expressed with native `double`.
 */
typedef struct bbtc_ib_spherical_grain_geometry_double_t
{
    /** Initial grain radius, in meters. */
    double initial_radius_m;
}
bbtc_ib_spherical_grain_geometry_double_t;


/**
 * @struct bbtc_ib_spherical_grain_geometry_long_double_t
 * @brief Spherical grain geometry expressed with native `long double`.
 */
typedef struct bbtc_ib_spherical_grain_geometry_long_double_t
{
    /** Initial grain radius, in meters. */
    long double initial_radius_m;
}
bbtc_ib_spherical_grain_geometry_long_double_t;


/**
 * @struct bbtc_ib_solid_cylindrical_grain_geometry_float_t
 * @brief Solid finite cylindrical grain geometry expressed with native `float`.
 */
typedef struct bbtc_ib_solid_cylindrical_grain_geometry_float_t
{
    /** Initial cylinder radius, in meters. */
    float initial_radius_m;

    /** Initial cylinder length, in meters. */
    float initial_length_m;
}
bbtc_ib_solid_cylindrical_grain_geometry_float_t;


/**
 * @struct bbtc_ib_solid_cylindrical_grain_geometry_double_t
 * @brief Solid finite cylindrical grain geometry expressed with native `double`.
 */
typedef struct bbtc_ib_solid_cylindrical_grain_geometry_double_t
{
    /** Initial cylinder radius, in meters. */
    double initial_radius_m;

    /** Initial cylinder length, in meters. */
    double initial_length_m;
}
bbtc_ib_solid_cylindrical_grain_geometry_double_t;


/**
 * @struct bbtc_ib_solid_cylindrical_grain_geometry_long_double_t
 * @brief Solid finite cylindrical grain geometry expressed with native `long double`.
 */
typedef struct bbtc_ib_solid_cylindrical_grain_geometry_long_double_t
{
    /** Initial cylinder radius, in meters. */
    long double initial_radius_m;

    /** Initial cylinder length, in meters. */
    long double initial_length_m;
}
bbtc_ib_solid_cylindrical_grain_geometry_long_double_t;


/**
 * @struct bbtc_ib_rectangular_prismatic_grain_geometry_float_t
 * @brief Rectangular-prismatic grain geometry expressed with native `float`.
 */
typedef struct bbtc_ib_rectangular_prismatic_grain_geometry_float_t
{
    /** Initial prism length, in meters. */
    float initial_length_m;

    /** Initial prism width, in meters. */
    float initial_width_m;

    /** Initial prism thickness, in meters. */
    float initial_thickness_m;
}
bbtc_ib_rectangular_prismatic_grain_geometry_float_t;


/**
 * @struct bbtc_ib_rectangular_prismatic_grain_geometry_double_t
 * @brief Rectangular-prismatic grain geometry expressed with native `double`.
 */
typedef struct bbtc_ib_rectangular_prismatic_grain_geometry_double_t
{
    /** Initial prism length, in meters. */
    double initial_length_m;

    /** Initial prism width, in meters. */
    double initial_width_m;

    /** Initial prism thickness, in meters. */
    double initial_thickness_m;
}
bbtc_ib_rectangular_prismatic_grain_geometry_double_t;


/**
 * @struct bbtc_ib_rectangular_prismatic_grain_geometry_long_double_t
 * @brief Rectangular-prismatic grain geometry expressed with native `long double`.
 */
typedef struct bbtc_ib_rectangular_prismatic_grain_geometry_long_double_t
{
    /** Initial prism length, in meters. */
    long double initial_length_m;

    /** Initial prism width, in meters. */
    long double initial_width_m;

    /** Initial prism thickness, in meters. */
    long double initial_thickness_m;
}
bbtc_ib_rectangular_prismatic_grain_geometry_long_double_t;


/**
 * @struct bbtc_ib_single_perforated_cylindrical_grain_geometry_float_t
 * @brief Single-perforated finite cylindrical grain geometry in native `float`.
 */
typedef struct bbtc_ib_single_perforated_cylindrical_grain_geometry_float_t
{
    /** Initial outer radius, in meters. */
    float initial_outer_radius_m;

    /** Initial axial perforation radius, in meters. */
    float initial_inner_radius_m;

    /** Initial grain length, in meters. */
    float initial_length_m;
}
bbtc_ib_single_perforated_cylindrical_grain_geometry_float_t;


/**
 * @struct bbtc_ib_single_perforated_cylindrical_grain_geometry_double_t
 * @brief Single-perforated finite cylindrical grain geometry in native `double`.
 */
typedef struct bbtc_ib_single_perforated_cylindrical_grain_geometry_double_t
{
    /** Initial outer radius, in meters. */
    double initial_outer_radius_m;

    /** Initial axial perforation radius, in meters. */
    double initial_inner_radius_m;

    /** Initial grain length, in meters. */
    double initial_length_m;
}
bbtc_ib_single_perforated_cylindrical_grain_geometry_double_t;


/**
 * @struct bbtc_ib_single_perforated_cylindrical_grain_geometry_long_double_t
 * @brief Single-perforated finite cylindrical grain geometry in native `long double`.
 */
typedef struct bbtc_ib_single_perforated_cylindrical_grain_geometry_long_double_t
{
    /** Initial outer radius, in meters. */
    long double initial_outer_radius_m;

    /** Initial axial perforation radius, in meters. */
    long double initial_inner_radius_m;

    /** Initial grain length, in meters. */
    long double initial_length_m;
}
bbtc_ib_single_perforated_cylindrical_grain_geometry_long_double_t;


/**
 * @struct bbtc_ib_propellant_grain_state_float_t
 * @brief Evaluated one-grain regression state expressed with native `float`.
 */
typedef struct bbtc_ib_propellant_grain_state_float_t
{
    /** Remaining condensed-grain volume, in cubic meters. */
    float remaining_volume_m3;

    /** Geometrically exposed burning surface area, in square meters. */
    float burning_surface_area_m2;

    /** Additional uniform regression depth available before geometric burnout. */
    float remaining_regression_to_burnout_m;

    /** Fraction of the initial grain volume consumed, dimensionless. */
    float consumed_volume_fraction;
}
bbtc_ib_propellant_grain_state_float_t;


/**
 * @struct bbtc_ib_propellant_grain_state_double_t
 * @brief Evaluated one-grain regression state expressed with native `double`.
 */
typedef struct bbtc_ib_propellant_grain_state_double_t
{
    /** Remaining condensed-grain volume, in cubic meters. */
    double remaining_volume_m3;

    /** Geometrically exposed burning surface area, in square meters. */
    double burning_surface_area_m2;

    /** Additional uniform regression depth available before geometric burnout. */
    double remaining_regression_to_burnout_m;

    /** Fraction of the initial grain volume consumed, dimensionless. */
    double consumed_volume_fraction;
}
bbtc_ib_propellant_grain_state_double_t;


/**
 * @struct bbtc_ib_propellant_grain_state_long_double_t
 * @brief Evaluated one-grain regression state expressed with native `long double`.
 */
typedef struct bbtc_ib_propellant_grain_state_long_double_t
{
    /** Remaining condensed-grain volume, in cubic meters. */
    long double remaining_volume_m3;

    /** Geometrically exposed burning surface area, in square meters. */
    long double burning_surface_area_m2;

    /** Additional uniform regression depth available before geometric burnout. */
    long double remaining_regression_to_burnout_m;

    /** Fraction of the initial grain volume consumed, dimensionless. */
    long double consumed_volume_fraction;
}
bbtc_ib_propellant_grain_state_long_double_t;


/**
 * @brief Validates a spherical native-float grain geometry.
 *
 * The initial radius must be strictly positive.
 *
 * @param geometry Caller-owned spherical grain geometry to validate.
 *
 * @return `BBTC_STATUS_SUCCESS` when valid;
 *         `BBTC_STATUS_INVALID_ARGUMENT` for a null pointer;
 *         `BBTC_STATUS_NAN_INPUT` for a NaN radius;
 *         `BBTC_STATUS_NONFINITE_INPUT` for positive or negative infinity; or
 *         `BBTC_STATUS_OUTSIDE_DOMAIN` for a finite nonpositive radius.
 */
bbtc_status_e
bbtc_ib_spherical_grain_geometry_validate_float(
    const bbtc_ib_spherical_grain_geometry_float_t* geometry
);


/**
 * @brief Native-double counterpart of
 *        `bbtc_ib_spherical_grain_geometry_validate_float()`.
 *
 * @param geometry Caller-owned spherical grain geometry to validate.
 *
 * @return Status semantics match the native-float validator.
 */
bbtc_status_e
bbtc_ib_spherical_grain_geometry_validate_double(
    const bbtc_ib_spherical_grain_geometry_double_t* geometry
);


/**
 * @brief Native-long-double counterpart of
 *        `bbtc_ib_spherical_grain_geometry_validate_float()`.
 *
 * @param geometry Caller-owned spherical grain geometry to validate.
 *
 * @return Status semantics match the native-float validator.
 */
bbtc_status_e
bbtc_ib_spherical_grain_geometry_validate_long_double(
    const bbtc_ib_spherical_grain_geometry_long_double_t* geometry
);


/**
 * @brief Validates a solid finite cylindrical native-float grain geometry.
 *
 * The initial radius and length must both be strictly positive. NaN takes
 * precedence over infinity when multiple dimensions are nonfinite.
 *
 * @param geometry Caller-owned solid cylindrical grain geometry to validate.
 *
 * @return `BBTC_STATUS_SUCCESS` when valid;
 *         `BBTC_STATUS_INVALID_ARGUMENT` for a null pointer;
 *         `BBTC_STATUS_NAN_INPUT` for a NaN dimension;
 *         `BBTC_STATUS_NONFINITE_INPUT` for an infinite dimension; or
 *         `BBTC_STATUS_OUTSIDE_DOMAIN` for any finite nonpositive dimension.
 */
bbtc_status_e
bbtc_ib_solid_cylindrical_grain_geometry_validate_float(
    const bbtc_ib_solid_cylindrical_grain_geometry_float_t* geometry
);


/**
 * @brief Native-double counterpart of
 *        `bbtc_ib_solid_cylindrical_grain_geometry_validate_float()`.
 *
 * @param geometry Caller-owned solid cylindrical grain geometry to validate.
 *
 * @return Status semantics match the native-float validator.
 */
bbtc_status_e
bbtc_ib_solid_cylindrical_grain_geometry_validate_double(
    const bbtc_ib_solid_cylindrical_grain_geometry_double_t* geometry
);


/**
 * @brief Native-long-double counterpart of
 *        `bbtc_ib_solid_cylindrical_grain_geometry_validate_float()`.
 *
 * @param geometry Caller-owned solid cylindrical grain geometry to validate.
 *
 * @return Status semantics match the native-float validator.
 */
bbtc_status_e
bbtc_ib_solid_cylindrical_grain_geometry_validate_long_double(
    const bbtc_ib_solid_cylindrical_grain_geometry_long_double_t* geometry
);


/**
 * @brief Validates a rectangular-prismatic native-float grain geometry.
 *
 * Initial length, width, and thickness must each be strictly positive. NaN
 * takes precedence over infinity when multiple dimensions are nonfinite.
 *
 * @param geometry Caller-owned rectangular-prismatic geometry to validate.
 *
 * @return `BBTC_STATUS_SUCCESS` when valid;
 *         `BBTC_STATUS_INVALID_ARGUMENT` for a null pointer;
 *         `BBTC_STATUS_NAN_INPUT` for a NaN dimension;
 *         `BBTC_STATUS_NONFINITE_INPUT` for an infinite dimension; or
 *         `BBTC_STATUS_OUTSIDE_DOMAIN` for any finite nonpositive dimension.
 */
bbtc_status_e
bbtc_ib_rectangular_prismatic_grain_geometry_validate_float(
    const bbtc_ib_rectangular_prismatic_grain_geometry_float_t* geometry
);


/**
 * @brief Native-double counterpart of
 *        `bbtc_ib_rectangular_prismatic_grain_geometry_validate_float()`.
 *
 * @param geometry Caller-owned rectangular-prismatic geometry to validate.
 *
 * @return Status semantics match the native-float validator.
 */
bbtc_status_e
bbtc_ib_rectangular_prismatic_grain_geometry_validate_double(
    const bbtc_ib_rectangular_prismatic_grain_geometry_double_t* geometry
);

/**
 * @brief Native-long-double counterpart of
 *        `bbtc_ib_rectangular_prismatic_grain_geometry_validate_float()`.
 *
 * @param geometry Caller-owned rectangular-prismatic geometry to validate.
 *
 * @return Status semantics match the native-float validator.
 */
bbtc_status_e
bbtc_ib_rectangular_prismatic_grain_geometry_validate_long_double(
    const bbtc_ib_rectangular_prismatic_grain_geometry_long_double_t* geometry
);


/**
 * @brief Validates a single-perforated cylindrical native-float grain geometry.
 *
 * Initial outer radius, perforation radius, and length must each be strictly
 * positive. The outer radius must also be strictly greater than the perforation
 * radius. NaN takes precedence over infinity when multiple dimensions are
 * nonfinite.
 *
 * @param geometry Caller-owned single-perforated geometry to validate.
 *
 * @return `BBTC_STATUS_SUCCESS` when valid;
 *         `BBTC_STATUS_INVALID_ARGUMENT` for a null pointer;
 *         `BBTC_STATUS_NAN_INPUT` for a NaN dimension;
 *         `BBTC_STATUS_NONFINITE_INPUT` for an infinite dimension;
 *         `BBTC_STATUS_OUTSIDE_DOMAIN` for any finite nonpositive dimension; or
 *         `BBTC_STATUS_INCONSISTENT_CONFIGURATION` when the outer radius is not
 *         strictly greater than the perforation radius.
 */
bbtc_status_e
bbtc_ib_single_perforated_cylindrical_grain_geometry_validate_float(
    const bbtc_ib_single_perforated_cylindrical_grain_geometry_float_t* geometry
);


/**
 * @brief Native-double counterpart of
 *        `bbtc_ib_single_perforated_cylindrical_grain_geometry_validate_float()`.
 *
 * @param geometry Caller-owned single-perforated geometry to validate.
 *
 * @return Status semantics match the native-float validator.
 */
bbtc_status_e
bbtc_ib_single_perforated_cylindrical_grain_geometry_validate_double(
    const bbtc_ib_single_perforated_cylindrical_grain_geometry_double_t* geometry
);


/**
 * @brief Native-long-double counterpart of
 *        `bbtc_ib_single_perforated_cylindrical_grain_geometry_validate_float()`.
 *
 * @param geometry Caller-owned single-perforated geometry to validate.
 *
 * @return Status semantics match the native-float validator.
 */
bbtc_status_e
bbtc_ib_single_perforated_cylindrical_grain_geometry_validate_long_double(
    const bbtc_ib_single_perforated_cylindrical_grain_geometry_long_double_t* geometry
);


/**
 * @brief Evaluates spherical-grain regression with native `float`.
 *
 * `regression_depth_m` is a nonnegative uniform normal regression depth. The
 * exact burnout coordinate is the initial radius. Exact burnout is a valid
 * zero-volume, zero-area state with consumed-volume fraction one. Regression
 * beyond burnout is outside the model domain and is never clamped.
 *
 * Once `result` is known to be nonnull, it is cleared before geometry or
 * regression validation. Geometry validation therefore has precedence over
 * regression-scalar validation.
 *
 * @param geometry Valid spherical grain geometry.
 * @param regression_depth_m Uniform normal regression depth, in meters.
 * @param result Caller-owned grain-state output record.
 *
 * @return `BBTC_STATUS_SUCCESS` on successful evaluation; otherwise the
 *         relevant validation, domain, or numerical-failure status.
 */
bbtc_status_e
bbtc_ib_spherical_grain_evaluate_float(
    const bbtc_ib_spherical_grain_geometry_float_t* geometry,
    float regression_depth_m,
    bbtc_ib_propellant_grain_state_float_t* result
);


/**
 * @brief Native-double counterpart of `bbtc_ib_spherical_grain_evaluate_float()`.
 *
 * @param geometry Valid spherical grain geometry.
 * @param regression_depth_m Uniform normal regression depth, in meters.
 * @param result Caller-owned grain-state output record.
 *
 * @return Status semantics match the native-float evaluator.
 */
bbtc_status_e
bbtc_ib_spherical_grain_evaluate_double(
    const bbtc_ib_spherical_grain_geometry_double_t* geometry,
    double regression_depth_m,
    bbtc_ib_propellant_grain_state_double_t* result
);


/**
 * @brief Native-long-double counterpart of
 *        `bbtc_ib_spherical_grain_evaluate_float()`.
 *
 * @param geometry Valid spherical grain geometry.
 * @param regression_depth_m Uniform normal regression depth, in meters.
 * @param result Caller-owned grain-state output record.
 *
 * @return Status semantics match the native-float evaluator.
 */
bbtc_status_e
bbtc_ib_spherical_grain_evaluate_long_double(
    const bbtc_ib_spherical_grain_geometry_long_double_t* geometry,
    long double regression_depth_m,
    bbtc_ib_propellant_grain_state_long_double_t* result
);


/**
 * @brief Evaluates solid finite cylindrical grain regression with native `float`.
 *
 * All cylindrical surfaces are treated as exposed burning surfaces. Radial
 * regression reduces the cylinder radius by `s`, while the two burning end
 * faces reduce total length by `2*s`. The first burnout coordinate is
 * `min(initial_radius_m, initial_length_m / 2)`.
 *
 * @param geometry Valid solid cylindrical grain geometry.
 * @param regression_depth_m Uniform normal regression depth, in meters.
 * @param result Caller-owned grain-state output record.
 *
 * @return `BBTC_STATUS_SUCCESS` on successful evaluation; otherwise the
 *         relevant validation, domain, or numerical-failure status.
 */
bbtc_status_e
bbtc_ib_solid_cylindrical_grain_evaluate_float(
    const bbtc_ib_solid_cylindrical_grain_geometry_float_t* geometry,
    float regression_depth_m,
    bbtc_ib_propellant_grain_state_float_t* result
);


/**
 * @brief Native-double counterpart of
 *        `bbtc_ib_solid_cylindrical_grain_evaluate_float()`.
 *
 * @param geometry Valid solid cylindrical grain geometry.
 * @param regression_depth_m Uniform normal regression depth, in meters.
 * @param result Caller-owned grain-state output record.
 *
 * @return Status semantics match the native-float evaluator.
 */
bbtc_status_e
bbtc_ib_solid_cylindrical_grain_evaluate_double(
    const bbtc_ib_solid_cylindrical_grain_geometry_double_t* geometry,
    double regression_depth_m,
    bbtc_ib_propellant_grain_state_double_t* result
);


/**
 * @brief Native-long-double counterpart of
 *        `bbtc_ib_solid_cylindrical_grain_evaluate_float()`.
 *
 * @param geometry Valid solid cylindrical grain geometry.
 * @param regression_depth_m Uniform normal regression depth, in meters.
 * @param result Caller-owned grain-state output record.
 *
 * @return Status semantics match the native-float evaluator.
 */
bbtc_status_e
bbtc_ib_solid_cylindrical_grain_evaluate_long_double(
    const bbtc_ib_solid_cylindrical_grain_geometry_long_double_t* geometry,
    long double regression_depth_m,
    bbtc_ib_propellant_grain_state_long_double_t* result
);


/**
 * @brief Evaluates rectangular-prismatic grain regression with native `float`.
 *
 * All six prism faces are treated as exposed burning surfaces. Each opposing
 * face pair reduces its corresponding full dimension by `2*s`. The first
 * burnout coordinate is one half of the smallest initial dimension.
 *
 * @param geometry Valid rectangular-prismatic grain geometry.
 * @param regression_depth_m Uniform normal regression depth, in meters.
 * @param result Caller-owned grain-state output record.
 *
 * @return `BBTC_STATUS_SUCCESS` on successful evaluation; otherwise the
 *         relevant validation, domain, or numerical-failure status.
 */
bbtc_status_e
bbtc_ib_rectangular_prismatic_grain_evaluate_float(
    const bbtc_ib_rectangular_prismatic_grain_geometry_float_t* geometry,
    float regression_depth_m,
    bbtc_ib_propellant_grain_state_float_t* result
);


/**
 * @brief Native-double counterpart of
 *        `bbtc_ib_rectangular_prismatic_grain_evaluate_float()`.
 *
 * @param geometry Valid rectangular-prismatic grain geometry.
 * @param regression_depth_m Uniform normal regression depth, in meters.
 * @param result Caller-owned grain-state output record.
 *
 * @return Status semantics match the native-float evaluator.
 */
bbtc_status_e
bbtc_ib_rectangular_prismatic_grain_evaluate_double(
    const bbtc_ib_rectangular_prismatic_grain_geometry_double_t* geometry,
    double regression_depth_m,
    bbtc_ib_propellant_grain_state_double_t* result
);


/**
 * @brief Native-long-double counterpart of
 *        `bbtc_ib_rectangular_prismatic_grain_evaluate_float()`.
 *
 * @param geometry Valid rectangular-prismatic grain geometry.
 * @param regression_depth_m Uniform normal regression depth, in meters.
 * @param result Caller-owned grain-state output record.
 *
 * @return Status semantics match the native-float evaluator.
 */
bbtc_status_e
bbtc_ib_rectangular_prismatic_grain_evaluate_long_double(
    const bbtc_ib_rectangular_prismatic_grain_geometry_long_double_t* geometry,
    long double regression_depth_m,
    bbtc_ib_propellant_grain_state_long_double_t* result
);


/**
 * @brief Evaluates single-perforated cylindrical grain regression with native
 *        `float`.
 *
 * The outer cylindrical surface regresses inward, the perforation surface
 * regresses outward, and both end faces regress inward. The first burnout
 * coordinate is `min((R0 - r0) / 2, L0 / 2)`.
 *
 * @param geometry Valid single-perforated cylindrical grain geometry.
 * @param regression_depth_m Uniform normal regression depth, in meters.
 * @param result Caller-owned grain-state output record.
 *
 * @return `BBTC_STATUS_SUCCESS` on successful evaluation; otherwise the
 *         relevant validation, domain, inconsistent-configuration, or
 *         numerical-failure status.
 */
bbtc_status_e
bbtc_ib_single_perforated_cylindrical_grain_evaluate_float(
    const bbtc_ib_single_perforated_cylindrical_grain_geometry_float_t* geometry,
    float regression_depth_m,
    bbtc_ib_propellant_grain_state_float_t* result
);


/**
 * @brief Native-double counterpart of
 *        `bbtc_ib_single_perforated_cylindrical_grain_evaluate_float()`.
 *
 * @param geometry Valid single-perforated cylindrical grain geometry.
 * @param regression_depth_m Uniform normal regression depth, in meters.
 * @param result Caller-owned grain-state output record.
 *
 * @return Status semantics match the native-float evaluator.
 */
bbtc_status_e
bbtc_ib_single_perforated_cylindrical_grain_evaluate_double(
    const bbtc_ib_single_perforated_cylindrical_grain_geometry_double_t* geometry,
    double regression_depth_m,
    bbtc_ib_propellant_grain_state_double_t* result
);


/**
 * @brief Native-long-double counterpart of
 *        `bbtc_ib_single_perforated_cylindrical_grain_evaluate_float()`.
 *
 * @param geometry Valid single-perforated cylindrical grain geometry.
 * @param regression_depth_m Uniform normal regression depth, in meters.
 * @param result Caller-owned grain-state output record.
 *
 * @return Status semantics match the native-float evaluator.
 */
bbtc_status_e
bbtc_ib_single_perforated_cylindrical_grain_evaluate_long_double(
    const bbtc_ib_single_perforated_cylindrical_grain_geometry_long_double_t* geometry,
    long double regression_depth_m,
    bbtc_ib_propellant_grain_state_long_double_t* result
);


#ifdef __cplusplus
}   /* extern "C" */
#endif
#endif /* BBTC_INTERNAL_BALLISTICS_PROPELLANT_GRAIN_GEOMETRY_H */
