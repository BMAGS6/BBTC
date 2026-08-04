/**
 * @file
 * @brief Precision-qualified internal-ballistics projectile records.
 */

#ifndef BBTC_INTERNAL_BALLISTICS_PROJECTILE_H
#define BBTC_INTERNAL_BALLISTICS_PROJECTILE_H

#include <bbtc/status.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Internal-ballistics projectile data expressed with native `float`.
 *
 * `mass_kg` is the total translational inertial mass of the modeled projectile
 * assembly accelerated through the bore. It includes components that remain
 * mechanically coupled during the modeled bore travel and excludes propellant,
 * cartridge-case, gas, and firearm recoiling mass. A zero-initialized record is
 * deliberately invalid.
 */
typedef struct bbtc_ib_projectile_float_t
{
    /** Total modeled projectile-assembly mass, in kilograms. */
    float mass_kg;
}
bbtc_ib_projectile_float_t;


/**
 * @brief Internal-ballistics projectile data expressed with native `double`.
 *
 * The record has the same physical meaning and validation contract as
 * `bbtc_ib_projectile_float_t`, without converting through another scalar
 * family.
 */
typedef struct bbtc_ib_projectile_double_t
{
    /** Total modeled projectile-assembly mass, in kilograms. */
    double mass_kg;
}
bbtc_ib_projectile_double_t;


/**
 * @brief Internal-ballistics projectile data expressed with native `long double`.
 *
 * The record has the same physical meaning and validation contract as the other
 * scalar families. It does not imply that `long double` is wider than `double`
 * on every supported platform.
 */
typedef struct bbtc_ib_projectile_long_double_t
{
    /** Total modeled projectile-assembly mass, in kilograms. */
    long double mass_kg;
}
bbtc_ib_projectile_long_double_t;


/**
 * @brief Validates one native-float internal-ballistics projectile record.
 *
 * A null pointer returns `BBTC_STATUS_INVALID_ARGUMENT`. A NaN or infinity
 * returns `BBTC_STATUS_NONFINITE_INPUT`. Mass must otherwise be greater than
 * zero; zero or a negative value returns `BBTC_STATUS_OUTSIDE_DOMAIN`. The
 * function does not modify the caller-owned record.
 *
 * @param projectile Projectile record to validate.
 *
 * @return `BBTC_STATUS_SUCCESS` when mass is finite and positive; otherwise the
 *         validation status described above.
 */
bbtc_status_e
bbtc_ib_projectile_validate_float(const bbtc_ib_projectile_float_t* projectile);


/**
 * @brief Validates one native-double internal-ballistics projectile record.
 *
 * Validation and ownership semantics match
 * `bbtc_ib_projectile_validate_float()`.
 *
 * @param projectile Projectile record to validate.
 *
 * @return `BBTC_STATUS_SUCCESS` when mass is finite and positive; otherwise a
 *         nonzero validation status.
 */
bbtc_status_e
bbtc_ib_projectile_validate_double(const bbtc_ib_projectile_double_t* projectile);


/**
 * @brief Validates one native-long-double internal-ballistics projectile record.
 *
 * Validation and ownership semantics match
 * `bbtc_ib_projectile_validate_float()`.
 *
 * @param projectile Projectile record to validate.
 *
 * @return `BBTC_STATUS_SUCCESS` when mass is finite and positive; otherwise a
 *         nonzero validation status.
 */
bbtc_status_e
bbtc_ib_projectile_validate_long_double(const bbtc_ib_projectile_long_double_t* projectile);

#ifdef __cplusplus
}   /* extern "C" */
#endif
#endif /* BBTC_INTERNAL_BALLISTICS_PROJECTILE_H */
