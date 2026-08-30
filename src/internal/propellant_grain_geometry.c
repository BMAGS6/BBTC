/**
 * @file
 * @brief Canonical propellant-grain regression geometry evaluation.
 */
#include <stddef.h>
#include <math.h>

#include "bbtc/internal_ballistics/propellant_grain_geometry.h"

static constexpr float       BBTC_PI_f  = 3.14159265358979323846f;
static constexpr double      BBTC_PI    = 3.141592653589793238462643383279502884;
static constexpr long double BBTC_PI_ld = 3.141592653589793238462643383279502884L;


/**
 * @brief Validates and commits one non-burnout native-float grain state.
 */
static bbtc_status_e grain_state_commit_float(
    const float remaining_volume_m3,
    const float burning_surface_area_m2,
    const float remaining_regression_to_burnout_m,
    const float consumed_volume_fraction,
    bbtc_ib_propellant_grain_state_float_t* const result)
{
    if (!isfinite(remaining_volume_m3)     || remaining_volume_m3       <= 0.0f ||
        !isfinite(burning_surface_area_m2) || burning_surface_area_m2   <= 0.0f ||
        !isfinite(remaining_regression_to_burnout_m)                            ||
        remaining_regression_to_burnout_m                               <= 0.0f ||
        !isfinite(consumed_volume_fraction) || consumed_volume_fraction < 0.0f  ||
        consumed_volume_fraction >= 1.0f)
    {
        return BBTC_STATUS_NUMERICAL_FAILURE;
    }

    result->remaining_volume_m3                 = remaining_volume_m3;
    result->burning_surface_area_m2             = burning_surface_area_m2;
    result->remaining_regression_to_burnout_m   = remaining_regression_to_burnout_m;
    result->consumed_volume_fraction            = consumed_volume_fraction;

    return BBTC_STATUS_SUCCESS;
}


/**
 * @brief Validates and commits one non-burnout native-double grain state.
 */
static bbtc_status_e grain_state_commit_double(
    const double remaining_volume_m3,
    const double burning_surface_area_m2,
    const double remaining_regression_to_burnout_m,
    const double consumed_volume_fraction,
    bbtc_ib_propellant_grain_state_double_t* const result)
{
    if (!isfinite(remaining_volume_m3)     || remaining_volume_m3       <= 0.0 ||
        !isfinite(burning_surface_area_m2) || burning_surface_area_m2   <= 0.0 ||
        !isfinite(remaining_regression_to_burnout_m)                           ||
        remaining_regression_to_burnout_m                               <= 0.0 ||
        !isfinite(consumed_volume_fraction) || consumed_volume_fraction < 0.0  ||
        consumed_volume_fraction >= 1.0)
    {
        return BBTC_STATUS_NUMERICAL_FAILURE;
    }

    result->remaining_volume_m3                 = remaining_volume_m3;
    result->burning_surface_area_m2             = burning_surface_area_m2;
    result->remaining_regression_to_burnout_m   = remaining_regression_to_burnout_m;
    result->consumed_volume_fraction            = consumed_volume_fraction;

    return BBTC_STATUS_SUCCESS;
}


/**
 * @brief Validates and commits one non-burnout native-long-double grain state.
 */
static bbtc_status_e grain_state_commit_long_double(
    const long double remaining_volume_m3,
    const long double burning_surface_area_m2,
    const long double remaining_regression_to_burnout_m,
    const long double consumed_volume_fraction,
    bbtc_ib_propellant_grain_state_long_double_t* const result)
{
    if (!isfinite(remaining_volume_m3)               || remaining_volume_m3     <= 0.0L ||
        !isfinite(burning_surface_area_m2)           || burning_surface_area_m2 <= 0.0L ||
        !isfinite(remaining_regression_to_burnout_m)                                    ||
        remaining_regression_to_burnout_m                                       <= 0.0L ||
        !isfinite(consumed_volume_fraction) || consumed_volume_fraction         <  0.0L ||
        consumed_volume_fraction >= 1.0L)
    {
        return BBTC_STATUS_NUMERICAL_FAILURE;
    }

    result->remaining_volume_m3                 = remaining_volume_m3;
    result->burning_surface_area_m2             = burning_surface_area_m2;
    result->remaining_regression_to_burnout_m   = remaining_regression_to_burnout_m;
    result->consumed_volume_fraction            = consumed_volume_fraction;

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e bbtc_ib_spherical_grain_geometry_validate_float(
    const bbtc_ib_spherical_grain_geometry_float_t* const geometry)
{
    if (geometry == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (isnan(geometry->initial_radius_m))
        return BBTC_STATUS_NAN_INPUT;

    if (isinf(geometry->initial_radius_m))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (geometry->initial_radius_m <= 0.0f)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e bbtc_ib_spherical_grain_geometry_validate_double(
    const bbtc_ib_spherical_grain_geometry_double_t* const geometry)
{
    if (geometry == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (isnan(geometry->initial_radius_m))
        return BBTC_STATUS_NAN_INPUT;

    if (isinf(geometry->initial_radius_m))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (geometry->initial_radius_m <= 0.0)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e bbtc_ib_spherical_grain_geometry_validate_long_double(
    const bbtc_ib_spherical_grain_geometry_long_double_t* const geometry)
{
    if (geometry == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (isnan(geometry->initial_radius_m))
        return BBTC_STATUS_NAN_INPUT;

    if (isinf(geometry->initial_radius_m))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (geometry->initial_radius_m <= 0.0L)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e bbtc_ib_solid_cylindrical_grain_geometry_validate_float(
    const bbtc_ib_solid_cylindrical_grain_geometry_float_t* const geometry)
{
    if (geometry == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (isnan(geometry->initial_radius_m) ||
        isnan(geometry->initial_length_m))
    {
        return BBTC_STATUS_NAN_INPUT;
    }

    if (isinf(geometry->initial_radius_m) ||
        isinf(geometry->initial_length_m))
    {
        return BBTC_STATUS_NONFINITE_INPUT;
    }

    if (geometry->initial_radius_m <= 0.0f ||
        geometry->initial_length_m <= 0.0f)
    {
        return BBTC_STATUS_OUTSIDE_DOMAIN;
    }

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e bbtc_ib_solid_cylindrical_grain_geometry_validate_double(
    const bbtc_ib_solid_cylindrical_grain_geometry_double_t* const geometry)
{
    if (geometry == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (isnan(geometry->initial_radius_m) ||
        isnan(geometry->initial_length_m))
    {
        return BBTC_STATUS_NAN_INPUT;
    }

    if (isinf(geometry->initial_radius_m) ||
        isinf(geometry->initial_length_m))
    {
        return BBTC_STATUS_NONFINITE_INPUT;
    }

    if (geometry->initial_radius_m <= 0.0 ||
        geometry->initial_length_m <= 0.0)
    {
        return BBTC_STATUS_OUTSIDE_DOMAIN;
    }

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e bbtc_ib_solid_cylindrical_grain_geometry_validate_long_double(
    const bbtc_ib_solid_cylindrical_grain_geometry_long_double_t* const geometry)
{
    if (geometry == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (isnan(geometry->initial_radius_m) ||
        isnan(geometry->initial_length_m))
    {
        return BBTC_STATUS_NAN_INPUT;
    }

    if (isinf(geometry->initial_radius_m) ||
        isinf(geometry->initial_length_m))
    {
        return BBTC_STATUS_NONFINITE_INPUT;
    }

    if (geometry->initial_radius_m <= 0.0L ||
        geometry->initial_length_m <= 0.0L)
    {
        return BBTC_STATUS_OUTSIDE_DOMAIN;
    }

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e bbtc_ib_rectangular_prismatic_grain_geometry_validate_float(
    const bbtc_ib_rectangular_prismatic_grain_geometry_float_t* const geometry)
{
    if (geometry == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (isnan(geometry->initial_length_m) ||
        isnan(geometry->initial_width_m) ||
        isnan(geometry->initial_thickness_m))
    {
        return BBTC_STATUS_NAN_INPUT;
    }

    if (isinf(geometry->initial_length_m) ||
        isinf(geometry->initial_width_m) ||
        isinf(geometry->initial_thickness_m))
    {
        return BBTC_STATUS_NONFINITE_INPUT;
    }

    if (geometry->initial_length_m <= 0.0f ||
        geometry->initial_width_m <= 0.0f ||
        geometry->initial_thickness_m <= 0.0f)
    {
        return BBTC_STATUS_OUTSIDE_DOMAIN;
    }

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e bbtc_ib_rectangular_prismatic_grain_geometry_validate_double(
    const bbtc_ib_rectangular_prismatic_grain_geometry_double_t* const geometry)
{
    if (geometry == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (isnan(geometry->initial_length_m) ||
        isnan(geometry->initial_width_m) ||
        isnan(geometry->initial_thickness_m))
    {
        return BBTC_STATUS_NAN_INPUT;
    }

    if (isinf(geometry->initial_length_m) ||
        isinf(geometry->initial_width_m) ||
        isinf(geometry->initial_thickness_m))
    {
        return BBTC_STATUS_NONFINITE_INPUT;
    }

    if (geometry->initial_length_m <= 0.0 ||
        geometry->initial_width_m <= 0.0 ||
        geometry->initial_thickness_m <= 0.0)
    {
        return BBTC_STATUS_OUTSIDE_DOMAIN;
    }

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e bbtc_ib_rectangular_prismatic_grain_geometry_validate_long_double(
    const bbtc_ib_rectangular_prismatic_grain_geometry_long_double_t* const geometry)
{
    if (geometry == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (isnan(geometry->initial_length_m) ||
        isnan(geometry->initial_width_m) ||
        isnan(geometry->initial_thickness_m))
    {
        return BBTC_STATUS_NAN_INPUT;
    }

    if (isinf(geometry->initial_length_m) ||
        isinf(geometry->initial_width_m) ||
        isinf(geometry->initial_thickness_m))
    {
        return BBTC_STATUS_NONFINITE_INPUT;
    }

    if (geometry->initial_length_m <= 0.0L ||
        geometry->initial_width_m <= 0.0L ||
        geometry->initial_thickness_m <= 0.0L)
    {
        return BBTC_STATUS_OUTSIDE_DOMAIN;
    }

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e bbtc_ib_single_perforated_cylindrical_grain_geometry_validate_float(
    const bbtc_ib_single_perforated_cylindrical_grain_geometry_float_t* const geometry)
{
    if (geometry == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (isnan(geometry->initial_outer_radius_m) ||
        isnan(geometry->initial_inner_radius_m) ||
        isnan(geometry->initial_length_m))
    {
        return BBTC_STATUS_NAN_INPUT;
    }

    if (isinf(geometry->initial_outer_radius_m) ||
        isinf(geometry->initial_inner_radius_m) ||
        isinf(geometry->initial_length_m))
    {
        return BBTC_STATUS_NONFINITE_INPUT;
    }

    if (geometry->initial_outer_radius_m <= 0.0f ||
        geometry->initial_inner_radius_m <= 0.0f ||
        geometry->initial_length_m <= 0.0f)
    {
        return BBTC_STATUS_OUTSIDE_DOMAIN;
    }

    if (geometry->initial_outer_radius_m <=
        geometry->initial_inner_radius_m)
    {
        return BBTC_STATUS_INCONSISTENT_CONFIGURATION;
    }

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e bbtc_ib_single_perforated_cylindrical_grain_geometry_validate_double(
    const bbtc_ib_single_perforated_cylindrical_grain_geometry_double_t* const geometry)
{
    if (geometry == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (isnan(geometry->initial_outer_radius_m) ||
        isnan(geometry->initial_inner_radius_m) ||
        isnan(geometry->initial_length_m))
    {
        return BBTC_STATUS_NAN_INPUT;
    }

    if (isinf(geometry->initial_outer_radius_m) ||
        isinf(geometry->initial_inner_radius_m) ||
        isinf(geometry->initial_length_m))
    {
        return BBTC_STATUS_NONFINITE_INPUT;
    }

    if (geometry->initial_outer_radius_m <= 0.0 ||
        geometry->initial_inner_radius_m <= 0.0 ||
        geometry->initial_length_m <= 0.0)
    {
        return BBTC_STATUS_OUTSIDE_DOMAIN;
    }

    if (geometry->initial_outer_radius_m <=
        geometry->initial_inner_radius_m)
    {
        return BBTC_STATUS_INCONSISTENT_CONFIGURATION;
    }

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e bbtc_ib_single_perforated_cylindrical_grain_geometry_validate_long_double(
    const bbtc_ib_single_perforated_cylindrical_grain_geometry_long_double_t* const geometry)
{
    if (geometry == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (isnan(geometry->initial_outer_radius_m) ||
        isnan(geometry->initial_inner_radius_m) ||
        isnan(geometry->initial_length_m))
    {
        return BBTC_STATUS_NAN_INPUT;
    }

    if (isinf(geometry->initial_outer_radius_m) ||
        isinf(geometry->initial_inner_radius_m) ||
        isinf(geometry->initial_length_m))
    {
        return BBTC_STATUS_NONFINITE_INPUT;
    }

    if (geometry->initial_outer_radius_m <= 0.0L ||
        geometry->initial_inner_radius_m <= 0.0L ||
        geometry->initial_length_m <= 0.0L)
    {
        return BBTC_STATUS_OUTSIDE_DOMAIN;
    }

    if (geometry->initial_outer_radius_m <=
        geometry->initial_inner_radius_m)
    {
        return BBTC_STATUS_INCONSISTENT_CONFIGURATION;
    }

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e bbtc_ib_spherical_grain_evaluate_float(
    const bbtc_ib_spherical_grain_geometry_float_t* const geometry,
    const float regression_depth_m,
    bbtc_ib_propellant_grain_state_float_t* const result)
{
    bbtc_status_e status;
    float         radius_m;
    float         remaining_volume_m3;
    float         burning_surface_area_m2;
    float         remaining_regression_m;
    float         normalized_regression;
    float         consumed_fraction;


    if (result == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    *result = (bbtc_ib_propellant_grain_state_float_t){0};

    status = bbtc_ib_spherical_grain_geometry_validate_float(geometry);
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    if (isnan(regression_depth_m))
        return BBTC_STATUS_NAN_INPUT;

    if (isinf(regression_depth_m))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (regression_depth_m < 0.0f || regression_depth_m > geometry->initial_radius_m)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    if (regression_depth_m == geometry->initial_radius_m)
    {
        result->consumed_volume_fraction = 1.0f;
        return BBTC_STATUS_SUCCESS;
    }

    radius_m = geometry->initial_radius_m - regression_depth_m;

    remaining_volume_m3 = (4.0f / 3.0f)
                        * BBTC_PI_f
                        * radius_m
                        * radius_m
                        * radius_m;

    burning_surface_area_m2 = 4.0f
                            * BBTC_PI_f
                            * radius_m
                            * radius_m;

    remaining_regression_m = geometry->initial_radius_m - regression_depth_m;

    normalized_regression  = regression_depth_m / geometry->initial_radius_m;

    consumed_fraction = normalized_regression
                      * (3.0f - normalized_regression * (3.0f - normalized_regression));

    return grain_state_commit_float(remaining_volume_m3,
                                    burning_surface_area_m2,
                                    remaining_regression_m,
                                    consumed_fraction,
                                    result);
}


bbtc_status_e bbtc_ib_spherical_grain_evaluate_double(
    const bbtc_ib_spherical_grain_geometry_double_t* const geometry,
    const double regression_depth_m,
    bbtc_ib_propellant_grain_state_double_t* const result)
{
    bbtc_status_e status;
    double        radius_m;
    double        remaining_volume_m3;
    double        burning_surface_area_m2;
    double        remaining_regression_m;
    double        normalized_regression;
    double        consumed_fraction;

    if (result == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    *result = (bbtc_ib_propellant_grain_state_double_t){0};

    status = bbtc_ib_spherical_grain_geometry_validate_double(geometry);
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    if (isnan(regression_depth_m))
        return BBTC_STATUS_NAN_INPUT;

    if (isinf(regression_depth_m))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (regression_depth_m < 0.0 || regression_depth_m > geometry->initial_radius_m)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    if (regression_depth_m == geometry->initial_radius_m)
    {
        result->consumed_volume_fraction = 1.0;
        return BBTC_STATUS_SUCCESS;
    }

    radius_m = geometry->initial_radius_m - regression_depth_m;

    remaining_volume_m3 = (4.0 / 3.0)
                        * BBTC_PI
                        * radius_m
                        * radius_m
                        * radius_m;

    burning_surface_area_m2 = 4.0
                            * BBTC_PI
                            * radius_m
                            * radius_m;

    remaining_regression_m = geometry->initial_radius_m - regression_depth_m;

    normalized_regression = regression_depth_m / geometry->initial_radius_m;

    consumed_fraction = normalized_regression
                      * (3.0 - normalized_regression * (3.0 - normalized_regression));

    return grain_state_commit_double(remaining_volume_m3,
                                     burning_surface_area_m2,
                                     remaining_regression_m,
                                     consumed_fraction,
                                     result);
}


bbtc_status_e bbtc_ib_spherical_grain_evaluate_long_double(
    const bbtc_ib_spherical_grain_geometry_long_double_t* const geometry,
    const long double regression_depth_m,
    bbtc_ib_propellant_grain_state_long_double_t* const result)
{
    bbtc_status_e status;
    long double   radius_m;
    long double   remaining_volume_m3;
    long double   burning_surface_area_m2;
    long double   remaining_regression_m;
    long double   normalized_regression;
    long double   consumed_fraction;

    if (result == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    *result = (bbtc_ib_propellant_grain_state_long_double_t){0};

    status = bbtc_ib_spherical_grain_geometry_validate_long_double(geometry);
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    if (isnan(regression_depth_m))
        return BBTC_STATUS_NAN_INPUT;

    if (isinf(regression_depth_m))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (regression_depth_m < 0.0L || regression_depth_m > geometry->initial_radius_m)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    if (regression_depth_m == geometry->initial_radius_m)
    {
        result->consumed_volume_fraction = 1.0L;
        return BBTC_STATUS_SUCCESS;
    }

    radius_m = geometry->initial_radius_m - regression_depth_m;

    remaining_volume_m3 = (4.0L / 3.0L)
                        * BBTC_PI_ld
                        * radius_m
                        * radius_m
                        * radius_m;

    burning_surface_area_m2 = 4.0L
                            * BBTC_PI_ld
                            * radius_m
                            * radius_m;

    remaining_regression_m = geometry->initial_radius_m - regression_depth_m;
    normalized_regression  = regression_depth_m / geometry->initial_radius_m;

    consumed_fraction = normalized_regression
                      * (3.0L - normalized_regression * (3.0L - normalized_regression));

    return grain_state_commit_long_double(remaining_volume_m3,
                                          burning_surface_area_m2,
                                          remaining_regression_m,
                                          consumed_fraction,
                                          result);
}


bbtc_status_e bbtc_ib_solid_cylindrical_grain_evaluate_float(
    const bbtc_ib_solid_cylindrical_grain_geometry_float_t* const geometry,
    const float regression_depth_m,
    bbtc_ib_propellant_grain_state_float_t* const result)
{
    bbtc_status_e status;
    float         half_length_m;
    float         maximum_regression_m;
    float         radius_m;
    float         length_m;
    float         remaining_volume_m3;
    float         burning_surface_area_m2;
    float         remaining_regression_m;
    float         radial_fraction;
    float         axial_fraction;
    float         radial_remaining_fraction;
    float         consumed_fraction;

    if (result == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    *result = (bbtc_ib_propellant_grain_state_float_t){0};

    status = bbtc_ib_solid_cylindrical_grain_geometry_validate_float(geometry);
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    if (isnan(regression_depth_m))
        return BBTC_STATUS_NAN_INPUT;

    if (isinf(regression_depth_m))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (regression_depth_m < 0.0f)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    half_length_m        = 0.5f * geometry->initial_length_m;
    maximum_regression_m = fminf(geometry->initial_radius_m, half_length_m);

    if (half_length_m <= 0.0f || maximum_regression_m <= 0.0f)
        return BBTC_STATUS_NUMERICAL_FAILURE;

    if (regression_depth_m > maximum_regression_m)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    if (regression_depth_m == maximum_regression_m)
    {
        result->consumed_volume_fraction = 1.0f;
        return BBTC_STATUS_SUCCESS;
    }

    radius_m = geometry->initial_radius_m - regression_depth_m;
    length_m = 2.0f * (half_length_m - regression_depth_m);

    remaining_volume_m3     = BBTC_PI_f
                            * radius_m
                            * radius_m
                            * length_m;

    burning_surface_area_m2 = 2.0f
                            * BBTC_PI_f
                            * radius_m
                            * (length_m + radius_m);

    remaining_regression_m  = maximum_regression_m - regression_depth_m;

    radial_fraction = regression_depth_m / geometry->initial_radius_m;
    axial_fraction  = regression_depth_m / half_length_m;

    radial_remaining_fraction = 1.0f - radial_fraction;

    consumed_fraction = radial_fraction * (2.0f - radial_fraction)
                      + radial_remaining_fraction * radial_remaining_fraction * axial_fraction;

    return grain_state_commit_float(remaining_volume_m3,
                                    burning_surface_area_m2,
                                    remaining_regression_m,
                                    consumed_fraction,
                                    result);
}


bbtc_status_e bbtc_ib_solid_cylindrical_grain_evaluate_double(
    const bbtc_ib_solid_cylindrical_grain_geometry_double_t* const geometry,
    const double regression_depth_m,
    bbtc_ib_propellant_grain_state_double_t* const result)
{
    bbtc_status_e status;
    double half_length_m;
    double maximum_regression_m;
    double radius_m;
    double length_m;
    double remaining_volume_m3;
    double burning_surface_area_m2;
    double remaining_regression_m;
    double radial_fraction;
    double axial_fraction;
    double radial_remaining_fraction;
    double consumed_fraction;

    if (result == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    *result = (bbtc_ib_propellant_grain_state_double_t){0};

    status = bbtc_ib_solid_cylindrical_grain_geometry_validate_double(geometry);
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    if (isnan(regression_depth_m))
        return BBTC_STATUS_NAN_INPUT;

    if (isinf(regression_depth_m))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (regression_depth_m < 0.0)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    half_length_m        = 0.5 * geometry->initial_length_m;
    maximum_regression_m = fmin(geometry->initial_radius_m, half_length_m);

    if (half_length_m <= 0.0 || maximum_regression_m <= 0.0)
        return BBTC_STATUS_NUMERICAL_FAILURE;

    if (regression_depth_m > maximum_regression_m)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    if (regression_depth_m == maximum_regression_m)
    {
        result->consumed_volume_fraction = 1.0;
        return BBTC_STATUS_SUCCESS;
    }

    radius_m = geometry->initial_radius_m - regression_depth_m;
    length_m = 2.0 * (half_length_m - regression_depth_m);

    remaining_volume_m3     = BBTC_PI
                            * radius_m
                            * radius_m
                            * length_m;

    burning_surface_area_m2 = 2.0
                            * BBTC_PI
                            * radius_m
                            * (length_m + radius_m);

    remaining_regression_m = maximum_regression_m - regression_depth_m;

    radial_fraction = regression_depth_m / geometry->initial_radius_m;
    axial_fraction  = regression_depth_m / half_length_m;

    radial_remaining_fraction = 1.0 - radial_fraction;

    consumed_fraction = radial_fraction * (2.0 - radial_fraction)
                      + radial_remaining_fraction * radial_remaining_fraction * axial_fraction;

    return grain_state_commit_double(remaining_volume_m3,
                                     burning_surface_area_m2,
                                     remaining_regression_m,
                                     consumed_fraction,
                                     result);
}

bbtc_status_e bbtc_ib_solid_cylindrical_grain_evaluate_long_double(
    const bbtc_ib_solid_cylindrical_grain_geometry_long_double_t* const geometry,
    const long double regression_depth_m,
    bbtc_ib_propellant_grain_state_long_double_t* const result)
{
    bbtc_status_e status;
    long double   half_length_m;
    long double   maximum_regression_m;
    long double   radius_m;
    long double   length_m;
    long double   remaining_volume_m3;
    long double   burning_surface_area_m2;
    long double   remaining_regression_m;
    long double   radial_fraction;
    long double   axial_fraction;
    long double   radial_remaining_fraction;
    long double   consumed_fraction;

    if (result == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    *result = (bbtc_ib_propellant_grain_state_long_double_t){0};

    status = bbtc_ib_solid_cylindrical_grain_geometry_validate_long_double(geometry);
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    if (isnan(regression_depth_m))
        return BBTC_STATUS_NAN_INPUT;

    if (isinf(regression_depth_m))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (regression_depth_m < 0.0L)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    half_length_m        = 0.5L * geometry->initial_length_m;
    maximum_regression_m = fminl(geometry->initial_radius_m, half_length_m);

    if (half_length_m <= 0.0L || maximum_regression_m <= 0.0L)
        return BBTC_STATUS_NUMERICAL_FAILURE;

    if (regression_depth_m > maximum_regression_m)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    if (regression_depth_m == maximum_regression_m)
    {
        result->consumed_volume_fraction = 1.0L;
        return BBTC_STATUS_SUCCESS;
    }

    radius_m = geometry->initial_radius_m - regression_depth_m;
    length_m = 2.0L * (half_length_m - regression_depth_m);

    remaining_volume_m3 = BBTC_PI_ld
                        * radius_m
                        * radius_m
                        * length_m;

    burning_surface_area_m2 = 2.0L
                            * BBTC_PI_ld
                            * radius_m
                            * (length_m + radius_m);

    remaining_regression_m  = maximum_regression_m - regression_depth_m;

    radial_fraction           = regression_depth_m / geometry->initial_radius_m;
    axial_fraction            = regression_depth_m / half_length_m;
    radial_remaining_fraction = 1.0L - radial_fraction;

    consumed_fraction = radial_fraction * (2.0L - radial_fraction)
                      + radial_remaining_fraction * radial_remaining_fraction * axial_fraction;

    return grain_state_commit_long_double(remaining_volume_m3,
                                          burning_surface_area_m2,
                                          remaining_regression_m,
                                          consumed_fraction,
                                          result);
}


bbtc_status_e bbtc_ib_rectangular_prismatic_grain_evaluate_float(
    const bbtc_ib_rectangular_prismatic_grain_geometry_float_t* const geometry,
    const float regression_depth_m,
    bbtc_ib_propellant_grain_state_float_t* const result)
{
    bbtc_status_e status;
    float         half_length_m;
    float         half_width_m;
    float         half_thickness_m;
    float         maximum_regression_m;
    float         length_m;
    float         width_m;
    float         thickness_m;
    float         remaining_volume_m3;
    float         burning_surface_area_m2;
    float         remaining_regression_m;
    float         length_fraction;
    float         width_fraction;
    float         thickness_fraction;
    float         length_remaining_fraction;
    float         width_remaining_fraction;
    float         consumed_fraction;

    if (result == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    *result = (bbtc_ib_propellant_grain_state_float_t){0};

    status = bbtc_ib_rectangular_prismatic_grain_geometry_validate_float(geometry);
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    if (isnan(regression_depth_m))
        return BBTC_STATUS_NAN_INPUT;

    if (isinf(regression_depth_m))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (regression_depth_m < 0.0f)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    half_length_m = 0.5f * geometry->initial_length_m;
    half_width_m = 0.5f * geometry->initial_width_m;
    half_thickness_m = 0.5f * geometry->initial_thickness_m;
    maximum_regression_m = fminf(half_length_m, fminf(half_width_m, half_thickness_m));

    if (half_length_m <= 0.0f || half_width_m <= 0.0f ||
        half_thickness_m <= 0.0f || maximum_regression_m <= 0.0f)
    {
        return BBTC_STATUS_NUMERICAL_FAILURE;
    }

    if (regression_depth_m > maximum_regression_m)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    if (regression_depth_m == maximum_regression_m)
    {
        result->consumed_volume_fraction = 1.0f;
        return BBTC_STATUS_SUCCESS;
    }

    length_m = 2.0f * (half_length_m - regression_depth_m);
    width_m  = 2.0f * (half_width_m  - regression_depth_m);

    thickness_m = 2.0f * (half_thickness_m - regression_depth_m);

    remaining_volume_m3 = length_m * width_m * thickness_m;

    burning_surface_area_m2 = 2.0f
                            * (length_m * width_m     +
                               length_m * thickness_m +
                               width_m  * thickness_m);

    remaining_regression_m = maximum_regression_m - regression_depth_m;

    length_fraction        = regression_depth_m / half_length_m;
    width_fraction         = regression_depth_m / half_width_m;
    thickness_fraction     = regression_depth_m / half_thickness_m;

    length_remaining_fraction = 1.0f - length_fraction;
    width_remaining_fraction  = 1.0f - width_fraction;

    consumed_fraction = length_fraction
                      + length_remaining_fraction * width_fraction
                      + length_remaining_fraction * width_remaining_fraction * thickness_fraction;

    return grain_state_commit_float(remaining_volume_m3,
                                    burning_surface_area_m2,
                                    remaining_regression_m,
                                    consumed_fraction,
                                    result);
}


bbtc_status_e bbtc_ib_rectangular_prismatic_grain_evaluate_double(
    const bbtc_ib_rectangular_prismatic_grain_geometry_double_t* const geometry,
    const double regression_depth_m,
    bbtc_ib_propellant_grain_state_double_t* const result)
{
    bbtc_status_e status;
    double        half_length_m;
    double        half_width_m;
    double        half_thickness_m;
    double        maximum_regression_m;
    double        length_m;
    double        width_m;
    double        thickness_m;
    double        remaining_volume_m3;
    double        burning_surface_area_m2;
    double        remaining_regression_m;
    double        length_fraction;
    double        width_fraction;
    double        thickness_fraction;
    double        length_remaining_fraction;
    double        width_remaining_fraction;
    double        consumed_fraction;

    if (result == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    *result = (bbtc_ib_propellant_grain_state_double_t){0};

    status = bbtc_ib_rectangular_prismatic_grain_geometry_validate_double(geometry);
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    if (isnan(regression_depth_m))
        return BBTC_STATUS_NAN_INPUT;

    if (isinf(regression_depth_m))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (regression_depth_m < 0.0)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    half_length_m        = 0.5 * geometry->initial_length_m;
    half_width_m         = 0.5 * geometry->initial_width_m;
    half_thickness_m     = 0.5 * geometry->initial_thickness_m;
    maximum_regression_m = fmin(half_length_m, fmin(half_width_m, half_thickness_m));

    if (half_length_m <= 0.0 || half_width_m <= 0.0 ||
        half_thickness_m <= 0.0 || maximum_regression_m <= 0.0)
    {
        return BBTC_STATUS_NUMERICAL_FAILURE;
    }

    if (regression_depth_m > maximum_regression_m)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    if (regression_depth_m == maximum_regression_m)
    {
        result->consumed_volume_fraction = 1.0;
        return BBTC_STATUS_SUCCESS;
    }

    length_m = 2.0 * (half_length_m - regression_depth_m);
    width_m  = 2.0 * (half_width_m  - regression_depth_m);

    thickness_m = 2.0 * (half_thickness_m - regression_depth_m);

    remaining_volume_m3 = length_m * width_m * thickness_m;

    burning_surface_area_m2 = 2.0
                            * (length_m * width_m     +
                               length_m * thickness_m +
                               width_m  * thickness_m);

    remaining_regression_m = maximum_regression_m - regression_depth_m;

    length_fraction        = regression_depth_m / half_length_m;
    width_fraction         = regression_depth_m / half_width_m;
    thickness_fraction     = regression_depth_m / half_thickness_m;

    length_remaining_fraction = 1.0 - length_fraction;
    width_remaining_fraction  = 1.0 - width_fraction;

    consumed_fraction = length_fraction
                      + length_remaining_fraction * width_fraction
                      + length_remaining_fraction * width_remaining_fraction * thickness_fraction;

    return grain_state_commit_double(remaining_volume_m3,
                                     burning_surface_area_m2,
                                     remaining_regression_m,
                                     consumed_fraction,
                                     result);
}


bbtc_status_e bbtc_ib_rectangular_prismatic_grain_evaluate_long_double(
    const bbtc_ib_rectangular_prismatic_grain_geometry_long_double_t* const geometry,
    const long double regression_depth_m,
    bbtc_ib_propellant_grain_state_long_double_t* const result)
{
    bbtc_status_e status;
    long double   half_length_m;
    long double   half_width_m;
    long double   half_thickness_m;
    long double   maximum_regression_m;
    long double   length_m;
    long double   width_m;
    long double   thickness_m;
    long double   remaining_volume_m3;
    long double   burning_surface_area_m2;
    long double   remaining_regression_m;
    long double   length_fraction;
    long double   width_fraction;
    long double   thickness_fraction;
    long double   length_remaining_fraction;
    long double   width_remaining_fraction;
    long double   consumed_fraction;

    if (result == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    *result = (bbtc_ib_propellant_grain_state_long_double_t){0};

    status = bbtc_ib_rectangular_prismatic_grain_geometry_validate_long_double(geometry);
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    if (isnan(regression_depth_m))
        return BBTC_STATUS_NAN_INPUT;

    if (isinf(regression_depth_m))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (regression_depth_m < 0.0L)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    half_length_m        = 0.5L * geometry->initial_length_m;
    half_width_m         = 0.5L * geometry->initial_width_m;
    half_thickness_m     = 0.5L * geometry->initial_thickness_m;
    maximum_regression_m = fminl(half_length_m, fminl(half_width_m, half_thickness_m));

    if (half_length_m <= 0.0L || half_width_m <= 0.0L ||
        half_thickness_m <= 0.0L || maximum_regression_m <= 0.0L)
    {
        return BBTC_STATUS_NUMERICAL_FAILURE;
    }

    if (regression_depth_m > maximum_regression_m)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    if (regression_depth_m == maximum_regression_m)
    {
        result->consumed_volume_fraction = 1.0L;
        return BBTC_STATUS_SUCCESS;
    }

    length_m = 2.0L * (half_length_m - regression_depth_m);
    width_m  = 2.0L * (half_width_m - regression_depth_m);

    thickness_m = 2.0L * (half_thickness_m - regression_depth_m);

    remaining_volume_m3 = length_m * width_m * thickness_m;

    burning_surface_area_m2 = 2.0L
                            * (length_m * width_m     +
                               length_m * thickness_m +
                               width_m  * thickness_m);

    remaining_regression_m = maximum_regression_m - regression_depth_m;

    length_fraction           = regression_depth_m / half_length_m;
    width_fraction            = regression_depth_m / half_width_m;
    thickness_fraction        = regression_depth_m / half_thickness_m;

    length_remaining_fraction = 1.0L - length_fraction;
    width_remaining_fraction  = 1.0L - width_fraction;

    consumed_fraction = length_fraction
                      + length_remaining_fraction * width_fraction
                      + length_remaining_fraction * width_remaining_fraction * thickness_fraction;

    return grain_state_commit_long_double(remaining_volume_m3,
                                          burning_surface_area_m2,
                                          remaining_regression_m,
                                          consumed_fraction,
                                          result);
}


bbtc_status_e bbtc_ib_single_perforated_cylindrical_grain_evaluate_float(
    const bbtc_ib_single_perforated_cylindrical_grain_geometry_float_t* const geometry,
    const float regression_depth_m,
    bbtc_ib_propellant_grain_state_float_t* const result)
{
    bbtc_status_e status;
    float         half_radial_web_m;
    float         half_length_m;
    float         maximum_regression_m;
    float         outer_radius_m;
    float         inner_radius_m;
    float         radial_gap_m;
    float         length_m;
    float         radius_ratio;
    float         annular_cross_section_m2;
    float         outer_lateral_area_m2;
    float         inner_lateral_area_m2;
    float         end_area_m2;
    float         remaining_volume_m3;
    float         burning_surface_area_m2;
    float         remaining_regression_m;
    float         radial_fraction;
    float         axial_fraction;
    float         radial_remaining_fraction;
    float         consumed_fraction;

    if (result == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    *result = (bbtc_ib_propellant_grain_state_float_t){0};

    status = bbtc_ib_single_perforated_cylindrical_grain_geometry_validate_float(geometry);
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    if (isnan(regression_depth_m))
        return BBTC_STATUS_NAN_INPUT;

    if (isinf(regression_depth_m))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (regression_depth_m < 0.0f)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    half_radial_web_m = 0.5f
                      * (geometry->initial_outer_radius_m - geometry->initial_inner_radius_m);

    half_length_m = 0.5f * geometry->initial_length_m;

    maximum_regression_m = fminf(half_radial_web_m, half_length_m);

    if (half_radial_web_m <= 0.0f || half_length_m <= 0.0f || maximum_regression_m <= 0.0f)
        return BBTC_STATUS_NUMERICAL_FAILURE;

    if (regression_depth_m > maximum_regression_m)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    if (regression_depth_m == maximum_regression_m)
    {
        result->consumed_volume_fraction = 1.0f;
        return BBTC_STATUS_SUCCESS;
    }

    outer_radius_m = geometry->initial_outer_radius_m - regression_depth_m;
    inner_radius_m = geometry->initial_inner_radius_m + regression_depth_m;

    radial_gap_m = outer_radius_m - inner_radius_m;

    length_m     = 2.0f * (half_length_m - regression_depth_m);

    radius_ratio = inner_radius_m / outer_radius_m;

    annular_cross_section_m2 = BBTC_PI_f
                             * radial_gap_m
                             * outer_radius_m
                             * (1.0f + radius_ratio);

    outer_lateral_area_m2 = 2.0f
                          * BBTC_PI_f
                          * (outer_radius_m * length_m);

    inner_lateral_area_m2 = 2.0f
                          * BBTC_PI_f
                          * (inner_radius_m * length_m);

    end_area_m2 = 2.0f * annular_cross_section_m2;

    remaining_volume_m3 = annular_cross_section_m2 * length_m;

    burning_surface_area_m2 = outer_lateral_area_m2
                            + inner_lateral_area_m2
                            + end_area_m2;

    remaining_regression_m  = maximum_regression_m - regression_depth_m;

    radial_fraction           = regression_depth_m / half_radial_web_m;
    axial_fraction            = regression_depth_m / half_length_m;
    radial_remaining_fraction = 1.0f - radial_fraction;

    consumed_fraction         = radial_fraction
                              + radial_remaining_fraction * axial_fraction;

    return grain_state_commit_float(remaining_volume_m3,
                                    burning_surface_area_m2,
                                    remaining_regression_m,
                                    consumed_fraction,
                                    result);
}


bbtc_status_e bbtc_ib_single_perforated_cylindrical_grain_evaluate_double(
    const bbtc_ib_single_perforated_cylindrical_grain_geometry_double_t* const geometry,
    const double regression_depth_m,
    bbtc_ib_propellant_grain_state_double_t* const result)
{
    bbtc_status_e status;
    double        half_radial_web_m;
    double        half_length_m;
    double        maximum_regression_m;
    double        outer_radius_m;
    double        inner_radius_m;
    double        radial_gap_m;
    double        length_m;
    double        radius_ratio;
    double        annular_cross_section_m2;
    double        outer_lateral_area_m2;
    double        inner_lateral_area_m2;
    double        end_area_m2;
    double        remaining_volume_m3;
    double        burning_surface_area_m2;
    double        remaining_regression_m;
    double        radial_fraction;
    double        axial_fraction;
    double        radial_remaining_fraction;
    double        consumed_fraction;

    if (result == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    *result = (bbtc_ib_propellant_grain_state_double_t){0};

    status = bbtc_ib_single_perforated_cylindrical_grain_geometry_validate_double(geometry);
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    if (isnan(regression_depth_m))
        return BBTC_STATUS_NAN_INPUT;

    if (isinf(regression_depth_m))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (regression_depth_m < 0.0)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    half_radial_web_m = 0.5 * (geometry->initial_outer_radius_m - geometry->initial_inner_radius_m);

    half_length_m = 0.5 * geometry->initial_length_m;

    maximum_regression_m = fmin(half_radial_web_m, half_length_m);

    if (half_radial_web_m <= 0.0 || half_length_m <= 0.0 || maximum_regression_m <= 0.0)
        return BBTC_STATUS_NUMERICAL_FAILURE;

    if (regression_depth_m > maximum_regression_m)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    if (regression_depth_m == maximum_regression_m)
    {
        result->consumed_volume_fraction = 1.0;
        return BBTC_STATUS_SUCCESS;
    }

    outer_radius_m = geometry->initial_outer_radius_m - regression_depth_m;
    inner_radius_m = geometry->initial_inner_radius_m + regression_depth_m;

    radial_gap_m   = outer_radius_m - inner_radius_m;

    length_m       = 2.0 * (half_length_m - regression_depth_m);

    radius_ratio   = inner_radius_m / outer_radius_m;

    annular_cross_section_m2 = BBTC_PI
                             * radial_gap_m
                             * outer_radius_m
                             * (1.0 + radius_ratio);

    outer_lateral_area_m2 = 2.0
                          * BBTC_PI
                          * (outer_radius_m * length_m);

    inner_lateral_area_m2 = 2.0
                          * BBTC_PI
                          * (inner_radius_m * length_m);

    end_area_m2 = 2.0 * annular_cross_section_m2;

    remaining_volume_m3 = annular_cross_section_m2 * length_m;

    burning_surface_area_m2 = outer_lateral_area_m2 + inner_lateral_area_m2 + end_area_m2;

    remaining_regression_m    = maximum_regression_m - regression_depth_m;
    radial_fraction           = regression_depth_m / half_radial_web_m;
    axial_fraction            = regression_depth_m / half_length_m;
    radial_remaining_fraction = 1.0 - radial_fraction;

    consumed_fraction         = radial_fraction
                              + radial_remaining_fraction * axial_fraction;

    return grain_state_commit_double(remaining_volume_m3,
                                     burning_surface_area_m2,
                                     remaining_regression_m,
                                     consumed_fraction,
                                     result);
}


bbtc_status_e bbtc_ib_single_perforated_cylindrical_grain_evaluate_long_double(
    const bbtc_ib_single_perforated_cylindrical_grain_geometry_long_double_t* const geometry,
    const long double regression_depth_m,
    bbtc_ib_propellant_grain_state_long_double_t* const result)
{
    bbtc_status_e status;
    long double   half_radial_web_m;
    long double   half_length_m;
    long double   maximum_regression_m;
    long double   outer_radius_m;
    long double   inner_radius_m;
    long double   radial_gap_m;
    long double   length_m;
    long double   radius_ratio;
    long double   annular_cross_section_m2;
    long double   outer_lateral_area_m2;
    long double   inner_lateral_area_m2;
    long double   end_area_m2;
    long double   remaining_volume_m3;
    long double   burning_surface_area_m2;
    long double   remaining_regression_m;
    long double   radial_fraction;
    long double   axial_fraction;
    long double   radial_remaining_fraction;
    long double   consumed_fraction;

    if (result == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    *result = (bbtc_ib_propellant_grain_state_long_double_t){0};

    status = bbtc_ib_single_perforated_cylindrical_grain_geometry_validate_long_double(geometry);
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    if (isnan(regression_depth_m))
        return BBTC_STATUS_NAN_INPUT;

    if (isinf(regression_depth_m))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (regression_depth_m < 0.0L)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    half_radial_web_m = 0.5L
                      * (geometry->initial_outer_radius_m - geometry->initial_inner_radius_m);

    half_length_m = 0.5L * geometry->initial_length_m;

    maximum_regression_m = fminl(half_radial_web_m, half_length_m);

    if (half_radial_web_m <= 0.0L || half_length_m <= 0.0L || maximum_regression_m <= 0.0L)
        return BBTC_STATUS_NUMERICAL_FAILURE;

    if (regression_depth_m > maximum_regression_m)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    if (regression_depth_m == maximum_regression_m)
    {
        result->consumed_volume_fraction = 1.0L;
        return BBTC_STATUS_SUCCESS;
    }

    outer_radius_m = geometry->initial_outer_radius_m - regression_depth_m;
    inner_radius_m = geometry->initial_inner_radius_m + regression_depth_m;

    radial_gap_m   = outer_radius_m - inner_radius_m;

    length_m       = 2.0L * (half_length_m - regression_depth_m);

    radius_ratio   = inner_radius_m / outer_radius_m;

    annular_cross_section_m2 = BBTC_PI_ld
                             * radial_gap_m
                             * outer_radius_m
                             * (1.0L + radius_ratio);

    outer_lateral_area_m2 = 2.0L * BBTC_PI_ld * (outer_radius_m * length_m);
    inner_lateral_area_m2 = 2.0L * BBTC_PI_ld * (inner_radius_m * length_m);
    end_area_m2           = 2.0L * annular_cross_section_m2;

    remaining_volume_m3   = annular_cross_section_m2 * length_m;

    burning_surface_area_m2 = outer_lateral_area_m2 + inner_lateral_area_m2 + end_area_m2;

    remaining_regression_m  = maximum_regression_m - regression_depth_m;

    radial_fraction           = regression_depth_m / half_radial_web_m;
    axial_fraction            = regression_depth_m / half_length_m;
    radial_remaining_fraction = 1.0L - radial_fraction;

    consumed_fraction         = radial_fraction
                              + radial_remaining_fraction * axial_fraction;

    return grain_state_commit_long_double(remaining_volume_m3,
                                          burning_surface_area_m2,
                                          remaining_regression_m,
                                          consumed_fraction,
                                          result);
}
