/**
 * @file
 * @brief Precision-qualified calorically perfect Noble-Abel gas-model records.
 */
#ifndef BBTC_INTERNAL_BALLISTICS_NOBLE_ABEL_GAS_MODEL_H
#define BBTC_INTERNAL_BALLISTICS_NOBLE_ABEL_GAS_MODEL_H

#include <bbtc/status.h>

#ifdef __cplusplus
    extern "C" {
#endif

/**
 * @brief Calorically perfect Noble-Abel gas model using native `float`.
 *
 * The future mechanical equation of state is
 * `p * (V - m * b) = m * R * T`, where `R` is
 * `specific_gas_constant_j_per_kg_k` and `b` is
 * `covolume_m3_per_kg`. The caloric closure uses the constant-volume specific
 * heat stored in `constant_volume_specific_heat_j_per_kg_k`.
 *
 * A zero covolume is valid and explicitly selects the ideal-gas limit of this
 * model. The record does not identify a gas composition, propellant product,
 * or calibration source. A zero-initialized record is deliberately invalid.
 */
typedef struct bbtc_ib_noble_abel_gas_model_float_t
{
    /** Specific gas constant, in joules per kilogram-kelvin. */
    float specific_gas_constant_j_per_kg_k;

    /** Constant-volume specific heat, in joules per kilogram-kelvin. */
    float constant_volume_specific_heat_j_per_kg_k;

    /** Noble-Abel covolume, in cubic meters per kilogram. */
    float covolume_m3_per_kg;
}
bbtc_ib_noble_abel_gas_model_float_t;


/**
 * @brief Calorically perfect Noble-Abel gas model using native `double`.
 *
 * The record has the same physical meaning and validation contract as
 * `bbtc_ib_noble_abel_gas_model_float_t`, without conversion through another
 * scalar family.
 */
typedef struct bbtc_ib_noble_abel_gas_model_double_t
{
    /** Specific gas constant, in joules per kilogram-kelvin. */
    double specific_gas_constant_j_per_kg_k;

    /** Constant-volume specific heat, in joules per kilogram-kelvin. */
    double constant_volume_specific_heat_j_per_kg_k;

    /** Noble-Abel covolume, in cubic meters per kilogram. */
    double covolume_m3_per_kg;
}
bbtc_ib_noble_abel_gas_model_double_t;


/**
 * @brief Calorically perfect Noble-Abel gas model using native `long double`.
 *
 * The record has the same physical meaning and validation contract as the other
 * scalar families. It does not imply that `long double` is wider than `double`
 * on every supported platform.
 */
typedef struct bbtc_ib_noble_abel_gas_model_long_double_t
{
    /** Specific gas constant, in joules per kilogram-kelvin. */
    long double specific_gas_constant_j_per_kg_k;

    /** Constant-volume specific heat, in joules per kilogram-kelvin. */
    long double constant_volume_specific_heat_j_per_kg_k;

    /** Noble-Abel covolume, in cubic meters per kilogram. */
    long double covolume_m3_per_kg;
}
bbtc_ib_noble_abel_gas_model_long_double_t;


/**
 * @brief Validates one native-float Noble-Abel gas-model record.
 *
 * A null pointer returns `BBTC_STATUS_INVALID_ARGUMENT`. A NaN in any field
 * returns `BBTC_STATUS_NAN_INPUT`. Otherwise, positive or negative infinity in
 * any field returns `BBTC_STATUS_NONFINITE_INPUT`. The specific gas constant and
 * constant-volume specific heat must be greater than zero. Covolume must be
 * nonnegative; zero is the explicit ideal-gas limit. When multiple fields are
 * nonfinite at once, NaN takes precedence over infinity within this record. The
 * function does not modify the caller-owned record.
 *
 * @param model Noble-Abel gas-model record to validate.
 *
 * @return `BBTC_STATUS_SUCCESS` when every field satisfies the documented
 *         domain; otherwise the validation status described above.
 */
bbtc_status_e
bbtc_ib_noble_abel_gas_model_validate_float(const bbtc_ib_noble_abel_gas_model_float_t* model);


/**
 * @brief Validates one native-double Noble-Abel gas-model record.
 *
 * Validation and ownership semantics match
 * `bbtc_ib_noble_abel_gas_model_validate_float()`.
 *
 * @param model Noble-Abel gas-model record to validate.
 *
 * @return `BBTC_STATUS_SUCCESS` when every field satisfies the documented
 *         domain; otherwise a nonzero validation status.
 */
bbtc_status_e
bbtc_ib_noble_abel_gas_model_validate_double(const bbtc_ib_noble_abel_gas_model_double_t* model);


/**
 * @brief Validates one native-long-double Noble-Abel gas-model record.
 *
 * Validation and ownership semantics match
 * `bbtc_ib_noble_abel_gas_model_validate_float()`.
 *
 * @param model Noble-Abel gas-model record to validate.
 *
 * @return `BBTC_STATUS_SUCCESS` when every field satisfies the documented
 *         domain; otherwise a nonzero validation status.
 */
bbtc_status_e
bbtc_ib_noble_abel_gas_model_validate_long_double(const bbtc_ib_noble_abel_gas_model_long_double_t* model);

#ifdef __cplusplus
}   /* extern "C" */
#endif
#endif /* BBTC_INTERNAL_BALLISTICS_NOBLE_ABEL_GAS_MODEL_H */
