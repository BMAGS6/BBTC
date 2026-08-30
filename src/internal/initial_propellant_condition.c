/**
 * @file
 * @brief Initial condensed-propellant thermal-condition validation.
 */
#include <bbtc/internal_ballistics/initial_propellant_condition.h>

#include <math.h>
#include <stddef.h>

/** @brief Implements native-`float` initial propellant-condition validation. */
bbtc_status_e
bbtc_ib_initial_propellant_condition_validate_float(
    const bbtc_ib_initial_propellant_condition_float_t* const condition
)
{
    if (condition == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (isnan(condition->temperature_k))
        return BBTC_STATUS_NAN_INPUT;

    if (isinf(condition->temperature_k))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (condition->temperature_k <= 0.0f)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    return BBTC_STATUS_SUCCESS;
}

/** @brief Implements native-`double` initial propellant-condition validation. */
bbtc_status_e
bbtc_ib_initial_propellant_condition_validate_double(
    const bbtc_ib_initial_propellant_condition_double_t* const condition
)
{
    if (condition == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (isnan(condition->temperature_k))
        return BBTC_STATUS_NAN_INPUT;

    if (isinf(condition->temperature_k))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (condition->temperature_k <= 0.0)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    return BBTC_STATUS_SUCCESS;
}

/**
 * @brief Implements native-`long double` initial propellant-condition
 *        validation.
 */
bbtc_status_e
bbtc_ib_initial_propellant_condition_validate_long_double(
    const bbtc_ib_initial_propellant_condition_long_double_t* const condition
)
{
    if (condition == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (isnan(condition->temperature_k))
        return BBTC_STATUS_NAN_INPUT;

    if (isinf(condition->temperature_k))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (condition->temperature_k <= 0.0L)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    return BBTC_STATUS_SUCCESS;
}
