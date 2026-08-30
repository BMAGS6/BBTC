/**
 * @file
 * @brief Precision-qualified initial internal-ballistics gas-state records.
 */
#ifndef BBTC_INTERNAL_BALLISTICS_INITIAL_GAS_STATE_H
#define BBTC_INTERNAL_BALLISTICS_INITIAL_GAS_STATE_H

#include <bbtc/status.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initial gas conditions expressed with native `float`.
 *
 * `absolute_pressure_pa` is the initial absolute pressure of the free gas behind
 * the projectile. It is not gauge pressure and is not silently copied from the
 * current ambient atmosphere. `temperature_k` is the initial absolute gas
 * temperature in kelvin.
 *
 * Pressure and temperature are independent caller-supplied boundary conditions
 * in this checkpoint. Neither can be derived from the other without additional
 * information such as free-gas volume, gas amount, composition, and an equation
 * of state. A zero-initialized record is deliberately invalid.
 */
typedef struct bbtc_ib_initial_gas_state_float_t
{
    /** Initial free-gas absolute pressure, in pascals. */
    float absolute_pressure_pa;

    /** Initial free-gas absolute temperature, in kelvin. */
    float temperature_k;
}
bbtc_ib_initial_gas_state_float_t;


/**
 * @brief Initial gas conditions expressed with native `double`.
 *
 * The record has the same physical meaning and validation contract as
 * `bbtc_ib_initial_gas_state_float_t`, without conversion through another
 * scalar family.
 */
typedef struct bbtc_ib_initial_gas_state_double_t
{
    /** Initial free-gas absolute pressure, in pascals. */
    double absolute_pressure_pa;

    /** Initial free-gas absolute temperature, in kelvin. */
    double temperature_k;
}
bbtc_ib_initial_gas_state_double_t;


/**
 * @brief Initial gas conditions expressed with native `long double`.
 *
 * The record has the same physical meaning and validation contract as the other
 * scalar families. It does not imply that `long double` is wider than `double`
 * on every supported platform.
 */
typedef struct bbtc_ib_initial_gas_state_long_double_t
{
    /** Initial free-gas absolute pressure, in pascals. */
    long double absolute_pressure_pa;

    /** Initial free-gas absolute temperature, in kelvin. */
    long double temperature_k;
}
bbtc_ib_initial_gas_state_long_double_t;


/**
 * @brief Validates one native-float initial gas-state record.
 *
 * A null pointer returns `BBTC_STATUS_INVALID_ARGUMENT`. A NaN in either field
 * returns `BBTC_STATUS_NAN_INPUT`. Otherwise, positive or negative infinity in
 * either field returns `BBTC_STATUS_NONFINITE_INPUT`. Both fields must otherwise
 * be greater than zero; zero or a negative value returns
 * `BBTC_STATUS_OUTSIDE_DOMAIN`. When multiple fields are nonfinite at once, NaN
 * takes precedence over infinity within this record. The function does not
 * modify the caller-owned record.
 *
 * @param state Initial gas-state record to validate.
 *
 * @return `BBTC_STATUS_SUCCESS` when both fields are finite and positive;
 *         otherwise the validation status described above.
 */
bbtc_status_e
bbtc_ib_initial_gas_state_validate_float(const bbtc_ib_initial_gas_state_float_t* state);


/**
 * @brief Validates one native-double initial gas-state record.
 *
 * Validation and ownership semantics match
 * `bbtc_ib_initial_gas_state_validate_float()`.
 *
 * @param state Initial gas-state record to validate.
 *
 * @return `BBTC_STATUS_SUCCESS` when both fields are finite and positive;
 *         otherwise a nonzero validation status.
 */
bbtc_status_e
bbtc_ib_initial_gas_state_validate_double(const bbtc_ib_initial_gas_state_double_t* state);


/**
 * @brief Validates one native-long-double initial gas-state record.
 *
 * Validation and ownership semantics match
 * `bbtc_ib_initial_gas_state_validate_float()`.
 *
 * @param state Initial gas-state record to validate.
 *
 * @return `BBTC_STATUS_SUCCESS` when both fields are finite and positive;
 *         otherwise a nonzero validation status.
 */
bbtc_status_e
bbtc_ib_initial_gas_state_validate_long_double(const bbtc_ib_initial_gas_state_long_double_t* state);

#ifdef __cplusplus
}   /* extern "C" */
#endif
#endif /* BBTC_INTERNAL_BALLISTICS_INITIAL_GAS_STATE_H */
