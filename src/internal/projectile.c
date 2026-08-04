/**
 * @file
 * @brief Validation for precision-qualified projectile records.
 */

#include "bbtc/internal_ballistics/projectile.h"

#include <stddef.h>
#include <math.h>

bbtc_status_e
bbtc_ib_projectile_validate_float(const bbtc_ib_projectile_float_t* const projectile)
{
    if (projectile == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (!isfinite(projectile->mass_kg))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (projectile->mass_kg <= 0.0f)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e
bbtc_ib_projectile_validate_double(const bbtc_ib_projectile_double_t* const projectile)
{
    if (projectile == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (!isfinite(projectile->mass_kg))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (projectile->mass_kg <= 0.0)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e
bbtc_ib_projectile_validate_long_double(const bbtc_ib_projectile_long_double_t* const projectile)
{
    if (projectile == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (!isfinite(projectile->mass_kg))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (projectile->mass_kg <= 0.0L)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    return BBTC_STATUS_SUCCESS;
}
