/**
 * @file
 * @brief Explicit caloric reference records for reduced internal-ballistics gas models.
 *
 * @details
 * Specific internal energy is defined only up to an additive datum unless a
 * thermochemical convention supplies an absolute reference. BBTC therefore
 * keeps the reduced-model caloric datum explicit rather than silently choosing
 * a reference temperature or zero of energy.
 *
 * These records describe the hypothetical dilute-gas caloric branch:
 *
 *     e_0(T_ref) = e_ref
 *
 * They do not imply a chemical standard state, standard pressure, equilibrium
 * composition, heat of formation, or species-resolved thermochemical datum.
 * A caller may explicitly choose a conventional reference temperature such as
 * 298.15 K when appropriate to its data, but BBTC supplies no hidden physical
 * default.
 */
#ifndef BBTC_INTERNAL_BALLISTICS_CALORIC_REFERENCE_H
#define BBTC_INTERNAL_BALLISTICS_CALORIC_REFERENCE_H

#include <bbtc/status.h>

#ifdef __cplusplus
    extern "C" {
#endif

/**
 * @brief Reduced-gas dilute-branch caloric datum using native `float`.
 */
typedef struct bbtc_ib_caloric_reference_float_t
{
    /**
     * Absolute temperature at which the dilute-gas energy datum applies, in
     * kelvins. The value must be finite and strictly positive.
     */
    float reference_temperature_k;

    /**
     * Dilute-gas specific internal energy at `reference_temperature_k`, in
     * joules per kilogram.
     *
     * The value may be negative, zero, or positive because its zero is a
     * caller-selected datum.
     */
    float reference_specific_internal_energy_j_per_kg;
}
bbtc_ib_caloric_reference_float_t;


/**
 * @brief Native-`double` reduced-gas dilute-branch caloric datum.
 */
typedef struct bbtc_ib_caloric_reference_double_t
{
    /** Absolute datum temperature, in kelvins. */
    double reference_temperature_k;

    /** Dilute-gas specific internal-energy datum, in joules per kilogram. */
    double reference_specific_internal_energy_j_per_kg;
}
bbtc_ib_caloric_reference_double_t;


/**
 * @brief Native-`long double` reduced-gas dilute-branch caloric datum.
 *
 * @details
 * This record has the same physical meaning and validation contract as the
 * other scalar families. It does not imply that `long double` is wider than
 * `double` on every supported platform.
 */
typedef struct bbtc_ib_caloric_reference_long_double_t
{
    /** Absolute datum temperature, in kelvins. */
    long double reference_temperature_k;

    /** Dilute-gas specific internal-energy datum, in joules per kilogram. */
    long double reference_specific_internal_energy_j_per_kg;
}
bbtc_ib_caloric_reference_long_double_t;


/**
 * @brief Validates one native-`float` reduced-gas caloric reference.
 *
 * @details
 * A null pointer returns `BBTC_STATUS_INVALID_ARGUMENT`. A NaN in either field
 * returns `BBTC_STATUS_NAN_INPUT`. Otherwise, positive or negative infinity in
 * either field returns `BBTC_STATUS_NONFINITE_INPUT`. The reference temperature
 * must be strictly positive. The reference specific internal energy has no sign
 * restriction because it defines an arbitrary additive datum. When both fields
 * are nonfinite, NaN takes precedence over infinity within this record.
 *
 * Validation does not require the datum temperature to lie inside any
 * particular gas-model calibration interval. Model-specific state evaluators
 * separately validate the actual evaluation temperature.
 *
 * @param reference Caller-owned caloric reference to validate.
 *
 * @return `BBTC_STATUS_SUCCESS` when the record satisfies the documented
 *         domain; otherwise the validation status described above.
 */
bbtc_status_e
bbtc_ib_caloric_reference_validate_float(const bbtc_ib_caloric_reference_float_t* reference);


/**
 * @brief Validates one native-`double` reduced-gas caloric reference.
 *
 * @param reference Caller-owned caloric reference to validate.
 *
 * @return The same status contract as
 *         `bbtc_ib_caloric_reference_validate_float()`.
 */
bbtc_status_e
bbtc_ib_caloric_reference_validate_double(const bbtc_ib_caloric_reference_double_t* reference);


/**
 * @brief Validates one native-`long double` reduced-gas caloric reference.
 *
 * @param reference Caller-owned caloric reference to validate.
 *
 * @return The same status contract as
 *         `bbtc_ib_caloric_reference_validate_float()`.
 */
bbtc_status_e
bbtc_ib_caloric_reference_validate_long_double(const bbtc_ib_caloric_reference_long_double_t* reference);

#ifdef __cplusplus
}   /* extern "C" */
#endif
#endif /* BBTC_INTERNAL_BALLISTICS_CALORIC_REFERENCE_H */
