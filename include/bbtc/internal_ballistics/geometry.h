/**
 * @file
 * @brief Precision-qualified internal-ballistics geometry records.
 */

#ifndef BBTC_INTERNAL_BALLISTICS_GEOMETRY_H
#define BBTC_INTERNAL_BALLISTICS_GEOMETRY_H

#include <bbtc/status.h>

#ifdef __cplusplus
    extern "C" {
#endif

/**
 * @brief Internal-ballistics geometry expressed with native `float` scalars.
 *
 * The record is caller-owned and contains no hidden pointers, ownership, or
 * allocation. A zero-initialized record is deliberately invalid until every
 * required field is assigned a finite positive value.
 */
typedef struct bbtc_ib_geometry_float_t
{
    /**
     * Enclosed geometric volume behind the projectile at its initial modeled
     * position, before subtracting condensed propellant volume, in cubic meters.
     */
    float initial_behind_projectile_volume_m3;

    /**
     * Effective bore area used to increase volume as the projectile advances,
     * in square meters.
     */
    float bore_cross_sectional_area_m2;

    /**
     * Effective projectile-base area used to convert modeled gas pressure into
     * axial projectile force, in square meters.
     */
    float projectile_effective_base_area_m2;

    /**
     * Axial travel from the projectile's initial modeled reference position to
     * the muzzle-exit event, in meters.
     */
    float projectile_travel_to_muzzle_m;
}
bbtc_ib_geometry_float_t;


/**
 * @brief Internal-ballistics geometry expressed with native `double` scalars.
 *
 * The record has the same physical meaning and validation contract as
 * `bbtc_ib_geometry_float_t`, without converting through another scalar family.
 */
typedef struct bbtc_ib_geometry_double_t
{
    /** Initial behind-projectile geometric volume, in cubic meters. */
    double initial_behind_projectile_volume_m3;

    /** Effective bore area used for volume growth, in square meters. */
    double bore_cross_sectional_area_m2;

    /** Effective projectile-base pressure area, in square meters. */
    double projectile_effective_base_area_m2;

    /** Projectile travel from the initial reference to muzzle exit, in meters. */
    double projectile_travel_to_muzzle_m;
}
bbtc_ib_geometry_double_t;


/**
 * @brief Internal-ballistics geometry expressed with native `long double` scalars.
 *
 * The record has the same physical meaning and validation contract as the other
 * scalar families. It does not imply that `long double` is wider than `double`
 * on every supported platform.
 */
typedef struct bbtc_ib_geometry_long_double_t
{
    /** Initial behind-projectile geometric volume, in cubic meters. */
    long double initial_behind_projectile_volume_m3;

    /** Effective bore area used for volume growth, in square meters. */
    long double bore_cross_sectional_area_m2;

    /** Effective projectile-base pressure area, in square meters. */
    long double projectile_effective_base_area_m2;

    /** Projectile travel from the initial reference to muzzle exit, in meters. */
    long double projectile_travel_to_muzzle_m;
}
bbtc_ib_geometry_long_double_t;


/**
 * @brief Validates one native-float internal-ballistics geometry record.
 *
 * A null pointer returns `BBTC_STATUS_INVALID_ARGUMENT`. A NaN in any field
 * returns `BBTC_STATUS_NAN_INPUT`. Otherwise, positive or negative infinity in
 * any field returns `BBTC_STATUS_NONFINITE_INPUT`. Every field must otherwise
 * be greater than zero; zero or a negative value returns
 * `BBTC_STATUS_OUTSIDE_DOMAIN`. When multiple fields are nonfinite at once, NaN
 * takes precedence over infinity within this record. The function does not
 * modify the caller-owned record and does not require the two effective areas
 * to be equal.
 *
 * @param geometry Geometry record to validate.
 *
 * @return `BBTC_STATUS_SUCCESS` when every field is finite and positive;
 *         otherwise the status described above.
 */
bbtc_status_e
bbtc_ib_geometry_validate_float(const bbtc_ib_geometry_float_t* geometry);

/**
 * @brief Validates one native-double internal-ballistics geometry record.
 *
 * Validation and ownership semantics match
 * `bbtc_ib_geometry_validate_float()`.
 *
 * @param geometry Geometry record to validate.
 *
 * @return `BBTC_STATUS_SUCCESS` when every field is finite and positive;
 *         otherwise a nonzero validation status.
 */
bbtc_status_e
bbtc_ib_geometry_validate_double(const bbtc_ib_geometry_double_t* geometry);

/**
 * @brief Validates one native-long-double internal-ballistics geometry record.
 *
 * Validation and ownership semantics match
 * `bbtc_ib_geometry_validate_float()`.
 *
 * @param geometry Geometry record to validate.
 *
 * @return `BBTC_STATUS_SUCCESS` when every field is finite and positive;
 *         otherwise a nonzero validation status.
 */
bbtc_status_e
bbtc_ib_geometry_validate_long_double(const bbtc_ib_geometry_long_double_t* geometry);

#ifdef __cplusplus
}   /* extern "C" */
#endif
#endif /* BBTC_INTERNAL_BALLISTICS_GEOMETRY_H */
