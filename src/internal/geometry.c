#include "bbtc/internal_ballistics/geometry.h"

#include <stddef.h>
#include <math.h>

bbtc_status_e
bbtc_ib_geometry_validate_float(const bbtc_ib_geometry_float_t* const geometry)
{
    if (geometry == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (isnan(geometry->initial_behind_projectile_volume_m3) ||
        isnan(geometry->bore_cross_sectional_area_m2)        ||
        isnan(geometry->projectile_effective_base_area_m2)   ||
        isnan(geometry->projectile_travel_to_muzzle_m))
    {
        return BBTC_STATUS_NAN_INPUT;
    }

    if (isinf(geometry->initial_behind_projectile_volume_m3) ||
        isinf(geometry->bore_cross_sectional_area_m2)        ||
        isinf(geometry->projectile_effective_base_area_m2)   ||
        isinf(geometry->projectile_travel_to_muzzle_m))
    {
        return BBTC_STATUS_NONFINITE_INPUT;
    }

    if (geometry->initial_behind_projectile_volume_m3 <= 0.0f  ||
        geometry->bore_cross_sectional_area_m2        <= 0.0f  ||
        geometry->projectile_effective_base_area_m2   <= 0.0f  ||
        geometry->projectile_travel_to_muzzle_m       <= 0.0f)
    {
        return BBTC_STATUS_OUTSIDE_DOMAIN;
    }

    return BBTC_STATUS_SUCCESS;
}

bbtc_status_e
bbtc_ib_geometry_validate_double(const bbtc_ib_geometry_double_t* const geometry)
{
    if (geometry == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (isnan(geometry->initial_behind_projectile_volume_m3) ||
        isnan(geometry->bore_cross_sectional_area_m2)        ||
        isnan(geometry->projectile_effective_base_area_m2)   ||
        isnan(geometry->projectile_travel_to_muzzle_m))
    {
        return BBTC_STATUS_NAN_INPUT;
    }

    if (isinf(geometry->initial_behind_projectile_volume_m3) ||
        isinf(geometry->bore_cross_sectional_area_m2)        ||
        isinf(geometry->projectile_effective_base_area_m2)   ||
        isinf(geometry->projectile_travel_to_muzzle_m))
    {
        return BBTC_STATUS_NONFINITE_INPUT;
    }

    if (geometry->initial_behind_projectile_volume_m3 <= 0.0  ||
        geometry->bore_cross_sectional_area_m2        <= 0.0  ||
        geometry->projectile_effective_base_area_m2   <= 0.0  ||
        geometry->projectile_travel_to_muzzle_m       <= 0.0)
    {
        return BBTC_STATUS_OUTSIDE_DOMAIN;
    }

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e
bbtc_ib_geometry_validate_long_double(const bbtc_ib_geometry_long_double_t* const geometry)
{
    if (geometry == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (isnan(geometry->initial_behind_projectile_volume_m3) ||
        isnan(geometry->bore_cross_sectional_area_m2)        ||
        isnan(geometry->projectile_effective_base_area_m2)   ||
        isnan(geometry->projectile_travel_to_muzzle_m))
    {
        return BBTC_STATUS_NAN_INPUT;
    }

    if (isinf(geometry->initial_behind_projectile_volume_m3) ||
        isinf(geometry->bore_cross_sectional_area_m2)        ||
        isinf(geometry->projectile_effective_base_area_m2)   ||
        isinf(geometry->projectile_travel_to_muzzle_m))
    {
        return BBTC_STATUS_NONFINITE_INPUT;
    }

    if (geometry->initial_behind_projectile_volume_m3 <= 0.0L ||
        geometry->bore_cross_sectional_area_m2        <= 0.0L ||
        geometry->projectile_effective_base_area_m2   <= 0.0L ||
        geometry->projectile_travel_to_muzzle_m       <= 0.0L)
    {
        return BBTC_STATUS_OUTSIDE_DOMAIN;
    }

    return BBTC_STATUS_SUCCESS;
}
