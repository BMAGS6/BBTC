/**
 * @file
 * @brief Initial condensed-propellant thermal-condition records.
 *
 * @details
 * This header defines the primitive initial absolute temperature of the
 * condensed propellant charge. The condition is intentionally distinct from
 * ambient-air, cartridge-case, chamber, initial free-gas, and later
 * combustion-product temperatures.
 *
 * A caller may supply numerically equal temperatures when its modeled setup
 * establishes thermal equilibrium, but BBTC does not silently alias or copy
 * temperatures between those physical populations.
 */
#ifndef BBTC_INTERNAL_BALLISTICS_INITIAL_PROPELLANT_CONDITION_H
#define BBTC_INTERNAL_BALLISTICS_INITIAL_PROPELLANT_CONDITION_H

#include <bbtc/status.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @struct bbtc_ib_initial_propellant_condition_float_t
 * @brief Initial condensed-propellant condition using native `float`.
 *
 * @details
 * This record is an initial boundary condition, not an evolving thermal state.
 * Its existence does not imply constant propellant temperature after ignition
 * and does not define burn kinetics, a temperature correction, heat transfer,
 * or material degradation.
 */
typedef struct bbtc_ib_initial_propellant_condition_float_t
{
    /** Initial absolute condensed-propellant temperature, in kelvins. */
    float temperature_k;
}
bbtc_ib_initial_propellant_condition_float_t;

/** @brief Native-`double` initial condensed-propellant condition. */
typedef struct bbtc_ib_initial_propellant_condition_double_t
{
    /** Initial absolute condensed-propellant temperature, in kelvins. */
    double temperature_k;
}
bbtc_ib_initial_propellant_condition_double_t;

/** @brief Native-`long double` initial condensed-propellant condition. */
typedef struct bbtc_ib_initial_propellant_condition_long_double_t
{
    /** Initial absolute condensed-propellant temperature, in kelvins. */
    long double temperature_k;
}
bbtc_ib_initial_propellant_condition_long_double_t;

/**
 * @brief Validates one native-`float` initial propellant condition.
 *
 * @details
 * Validation establishes only a well-formed absolute Kelvin boundary:
 *
 * - null -> `BBTC_STATUS_INVALID_ARGUMENT`;
 * - NaN -> `BBTC_STATUS_NAN_INPUT`;
 * - positive or negative infinity -> `BBTC_STATUS_NONFINITE_INPUT`;
 * - finite `temperature_k <= 0` -> `BBTC_STATUS_OUTSIDE_DOMAIN`; and
 * - every finite `temperature_k > 0` -> `BBTC_STATUS_SUCCESS`.
 *
 * Zero initialization is deliberately invalid. BBTC supplies no physical
 * default, ambient alias, narrow "normal temperature" interval, or universal
 * finite upper-temperature cap. Positive finite representable values near the
 * scalar family's lower range remain structurally valid.
 *
 * Passing validation does not establish propellant stability, cook-off margin,
 * chemical lifetime, temperature-sensitive burn-law applicability, ammunition
 * compatibility, firearm strength, or firing safety. The caller-owned record
 * is immutable to this function and no allocation occurs.
 *
 * @param condition Caller-owned initial propellant condition.
 * @return The validation status described above.
 */
bbtc_status_e
bbtc_ib_initial_propellant_condition_validate_float(
    const bbtc_ib_initial_propellant_condition_float_t* condition
);

/**
 * @brief Validates one native-`double` initial propellant condition.
 * @details Semantics match the native-`float` validator exactly.
 * @param condition Caller-owned initial propellant condition.
 * @return The same status contract as the native-`float` validator.
 */
bbtc_status_e
bbtc_ib_initial_propellant_condition_validate_double(
    const bbtc_ib_initial_propellant_condition_double_t* condition
);

/**
 * @brief Validates one native-`long double` initial propellant condition.
 * @details Semantics match the native-`float` validator exactly.
 * @param condition Caller-owned initial propellant condition.
 * @return The same status contract as the native-`float` validator.
 */
bbtc_status_e
bbtc_ib_initial_propellant_condition_validate_long_double(
    const bbtc_ib_initial_propellant_condition_long_double_t* condition
);

#ifdef __cplusplus
}   /* extern "C" */
#endif

#endif /* BBTC_INTERNAL_BALLISTICS_INITIAL_PROPELLANT_CONDITION_H */
