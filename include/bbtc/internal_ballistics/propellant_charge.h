/**
 * @file
 * @brief Precision-qualified internal-ballistics propellant-charge records.
 */
#ifndef BBTC_INTERNAL_BALLISTICS_PROPELLANT_CHARGE_H
#define BBTC_INTERNAL_BALLISTICS_PROPELLANT_CHARGE_H

#include <bbtc/status.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Internal-ballistics propellant-charge data expressed with native `float`.
 *
 * `charge_mass_kg` is the total initial mass of the modeled propellant charge.
 * `condensed_phase_density_kg_per_m3` is the material density of the condensed
 * propellant phase and excludes intergranular void space. It is not bulk loading
 * density, gas density, or a burn-rate parameter. A zero-initialized record is
 * deliberately invalid.
 */
typedef struct bbtc_ib_propellant_charge_float_t
{
    /** Total initial modeled propellant-charge mass, in kilograms. */
    float charge_mass_kg;

    /**
     * Condensed propellant material density, excluding intergranular voids, in
     * kilograms per cubic meter.
     */
    float condensed_phase_density_kg_per_m3;
}
bbtc_ib_propellant_charge_float_t;


/**
 * @brief Internal-ballistics propellant-charge data expressed with native `double`.
 *
 * The record has the same physical meaning and validation contract as
 * `bbtc_ib_propellant_charge_float_t`, without converting through another
 * scalar family.
 */
typedef struct bbtc_ib_propellant_charge_double_t
{
    /** Total initial modeled propellant-charge mass, in kilograms. */
    double charge_mass_kg;

    /**
     * Condensed propellant material density, excluding intergranular voids, in
     * kilograms per cubic meter.
     */
    double condensed_phase_density_kg_per_m3;
}
bbtc_ib_propellant_charge_double_t;


/**
 * @brief Propellant-charge data expressed with native `long double`.
 *
 * The record has the same physical meaning and validation contract as the other
 * scalar families. It does not imply that `long double` is wider than `double`
 * on every supported platform.
 */
typedef struct bbtc_ib_propellant_charge_long_double_t
{
    /** Total initial modeled propellant-charge mass, in kilograms. */
    long double charge_mass_kg;

    /**
     * Condensed propellant material density, excluding intergranular voids, in
     * kilograms per cubic meter.
     */
    long double condensed_phase_density_kg_per_m3;
}
bbtc_ib_propellant_charge_long_double_t;


/**
 * @brief Validates one native-float propellant-charge record.
 *
 * A null pointer returns `BBTC_STATUS_INVALID_ARGUMENT`. A NaN in either field
 * returns `BBTC_STATUS_NAN_INPUT`. Otherwise, positive or negative infinity in
 * either field returns `BBTC_STATUS_NONFINITE_INPUT`. Both fields must otherwise
 * be greater than zero; zero or a negative value returns
 * `BBTC_STATUS_OUTSIDE_DOMAIN`. When multiple fields are nonfinite at once, NaN
 * takes precedence over infinity within this record. The function does not
 * modify the caller-owned record.
 *
 * @param charge Propellant-charge record to validate.
 *
 * @return `BBTC_STATUS_SUCCESS` when both fields are finite and positive;
 *         otherwise the validation status described above.
 */
bbtc_status_e
bbtc_ib_propellant_charge_validate_float(const bbtc_ib_propellant_charge_float_t* charge);


/**
 * @brief Validates one native-double propellant-charge record.
 *
 * Validation and ownership semantics match
 * `bbtc_ib_propellant_charge_validate_float()`.
 *
 * @param charge Propellant-charge record to validate.
 *
 * @return `BBTC_STATUS_SUCCESS` when both fields are finite and positive;
 *         otherwise a nonzero validation status.
 */
bbtc_status_e
bbtc_ib_propellant_charge_validate_double(const bbtc_ib_propellant_charge_double_t* charge);


/**
 * @brief Validates one native-long-double propellant-charge record.
 *
 * Validation and ownership semantics match
 * `bbtc_ib_propellant_charge_validate_float()`.
 *
 * @param charge Propellant-charge record to validate.
 *
 * @return `BBTC_STATUS_SUCCESS` when both fields are finite and positive;
 *         otherwise a nonzero validation status.
 */
bbtc_status_e
bbtc_ib_propellant_charge_validate_long_double(const bbtc_ib_propellant_charge_long_double_t* charge);

#ifdef __cplusplus
}   /* extern "C" */
#endif
#endif /* BBTC_INTERNAL_BALLISTICS_PROPELLANT_CHARGE_H */
