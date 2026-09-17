/**
 * @file
 * @brief Normalized pressure-power propellant burn-kinetics implementation.
 *
 * @details
 * This translation unit implements the pressure-only empirical relation
 *
 * `r(P) = r_ref * (P / P_ref)^n`
 *
 * independently in BBTC's native `float`, `double`, and `long double` scalar
 * families. It deliberately contains no ignition, temperature correction,
 * grain-geometry, reacted-mass, thermochemistry, gas-state, projectile-motion,
 * or firing-safety logic.
 */
#include <stddef.h>
#include <math.h>

#include "bbtc/internal_ballistics/propellant_burn_kinetics.h"


bbtc_status_e
bbtc_ib_pressure_power_burn_kinetics_validate_float(
    const bbtc_ib_pressure_power_burn_kinetics_float_t* const model
)
{
    if (model == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    /*
     * NaN has precedence over infinity within this complete five-scalar model
     * layer, independent of field order. Finite-domain relationships are not
     * inspected until all caller nonfinite values have been classified.
     */
    if (isnan(model->reference_burn_rate_m_per_s)    ||
        isnan(model->reference_pressure_pa)          ||
        isnan(model->pressure_exponent)              ||
        isnan(model->minimum_calibrated_pressure_pa) ||
        isnan(model->maximum_calibrated_pressure_pa))
    {
        return BBTC_STATUS_NAN_INPUT;
    }

    if (isinf(model->reference_burn_rate_m_per_s)    ||
        isinf(model->reference_pressure_pa)          ||
        isinf(model->pressure_exponent)              ||
        isinf(model->minimum_calibrated_pressure_pa) ||
        isinf(model->maximum_calibrated_pressure_pa))
    {
        return BBTC_STATUS_NONFINITE_INPUT;
    }

    if (model->reference_burn_rate_m_per_s    <= 0.0f ||
        model->reference_pressure_pa          <= 0.0f ||
        model->pressure_exponent              <= 0.0f ||
        model->minimum_calibrated_pressure_pa <= 0.0f ||
        model->maximum_calibrated_pressure_pa <= model->minimum_calibrated_pressure_pa ||
        model->reference_pressure_pa          < model->minimum_calibrated_pressure_pa  ||
        model->reference_pressure_pa          > model->maximum_calibrated_pressure_pa)
    {
        return BBTC_STATUS_OUTSIDE_DOMAIN;
    }

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e
bbtc_ib_pressure_power_burn_kinetics_validate_double(
    const bbtc_ib_pressure_power_burn_kinetics_double_t* const model
)
{
    if (model == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (isnan(model->reference_burn_rate_m_per_s)    ||
        isnan(model->reference_pressure_pa)          ||
        isnan(model->pressure_exponent)              ||
        isnan(model->minimum_calibrated_pressure_pa) ||
        isnan(model->maximum_calibrated_pressure_pa))
    {
        return BBTC_STATUS_NAN_INPUT;
    }

    if (isinf(model->reference_burn_rate_m_per_s)    ||
        isinf(model->reference_pressure_pa)          ||
        isinf(model->pressure_exponent)              ||
        isinf(model->minimum_calibrated_pressure_pa) ||
        isinf(model->maximum_calibrated_pressure_pa))
    {
        return BBTC_STATUS_NONFINITE_INPUT;
    }

    if (model->reference_burn_rate_m_per_s    <= 0.0 ||
        model->reference_pressure_pa          <= 0.0 ||
        model->pressure_exponent              <= 0.0 ||
        model->minimum_calibrated_pressure_pa <= 0.0 ||
        model->maximum_calibrated_pressure_pa <= model->minimum_calibrated_pressure_pa ||
        model->reference_pressure_pa          < model->minimum_calibrated_pressure_pa  ||
        model->reference_pressure_pa          > model->maximum_calibrated_pressure_pa)
    {
        return BBTC_STATUS_OUTSIDE_DOMAIN;
    }

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e
bbtc_ib_pressure_power_burn_kinetics_validate_long_double(
    const bbtc_ib_pressure_power_burn_kinetics_long_double_t* const model
)
{
    if (model == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (isnan(model->reference_burn_rate_m_per_s)    ||
        isnan(model->reference_pressure_pa)          ||
        isnan(model->pressure_exponent)              ||
        isnan(model->minimum_calibrated_pressure_pa) ||
        isnan(model->maximum_calibrated_pressure_pa))
    {
        return BBTC_STATUS_NAN_INPUT;
    }

    if (isinf(model->reference_burn_rate_m_per_s)    ||
        isinf(model->reference_pressure_pa)          ||
        isinf(model->pressure_exponent)              ||
        isinf(model->minimum_calibrated_pressure_pa) ||
        isinf(model->maximum_calibrated_pressure_pa))
    {
        return BBTC_STATUS_NONFINITE_INPUT;
    }

    if (model->reference_burn_rate_m_per_s    <= 0.0L ||
        model->reference_pressure_pa          <= 0.0L ||
        model->pressure_exponent              <= 0.0L ||
        model->minimum_calibrated_pressure_pa <= 0.0L ||
        model->maximum_calibrated_pressure_pa <= model->minimum_calibrated_pressure_pa ||
        model->reference_pressure_pa          < model->minimum_calibrated_pressure_pa  ||
        model->reference_pressure_pa          > model->maximum_calibrated_pressure_pa)
    {
        return BBTC_STATUS_OUTSIDE_DOMAIN;
    }

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e
bbtc_ib_pressure_power_burn_kinetics_evaluate_float(
    const bbtc_ib_pressure_power_burn_kinetics_float_t* const model,
    const float absolute_pressure_pa,
    bbtc_ib_propellant_burn_kinetics_result_float_t* const result
)
{
    bbtc_status_e status;
    float         pressure_difference_pa;
    float         log_pressure_ratio;
    float         log_burn_rate;
    float         burn_rate_m_per_s;

    if (result == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    /* Every failure after result-pointer acceptance leaves a zero record. */
    *result = (bbtc_ib_propellant_burn_kinetics_result_float_t){0};

    status = bbtc_ib_pressure_power_burn_kinetics_validate_float(model);

    if (status != BBTC_STATUS_SUCCESS)
        return status;

    if (isnan(absolute_pressure_pa))
        return BBTC_STATUS_NAN_INPUT;

    if (isinf(absolute_pressure_pa))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (absolute_pressure_pa < 0.0f)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    /*
     * Zero pressure is an exact boundary of this mathematical backend. It is
     * necessarily outside every valid calibration interval because calibrated
     * pressures are strictly positive.
     */
    if (absolute_pressure_pa == 0.0f)
    {
        result->applicability_flags = BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN;
        return BBTC_STATUS_SUCCESS;
    }

    /* Preserve the defining reference state exactly. */
    if (absolute_pressure_pa == model->reference_pressure_pa)
    {
        result->burn_rate_m_per_s = model->reference_burn_rate_m_per_s;
        return BBTC_STATUS_SUCCESS;
    }

    pressure_difference_pa =
        absolute_pressure_pa - model->reference_pressure_pa;

    /*
     * Near the reference state, log1p avoids subtracting two nearly equal
     * logarithms. Away from the reference state, subtracting logarithms avoids
     * explicitly forming P/P_ref, which can overflow or underflow even when
     * both caller inputs are finite and positive.
     */
    if (fabsf(pressure_difference_pa) <= 0.5f * model->reference_pressure_pa)
    {
        log_pressure_ratio = log1pf(pressure_difference_pa / model->reference_pressure_pa);
    }
    else
        log_pressure_ratio = logf(absolute_pressure_pa) - logf(model->reference_pressure_pa);


    log_burn_rate = logf(model->reference_burn_rate_m_per_s)
                  + model->pressure_exponent * log_pressure_ratio;


    if (!isfinite(log_burn_rate))
        return BBTC_STATUS_NUMERICAL_FAILURE;

    burn_rate_m_per_s = expf(log_burn_rate);

    if (!isfinite(burn_rate_m_per_s) || burn_rate_m_per_s <= 0.0f)
        return BBTC_STATUS_NUMERICAL_FAILURE;

    if (absolute_pressure_pa < model->minimum_calibrated_pressure_pa ||
        absolute_pressure_pa > model->maximum_calibrated_pressure_pa)
    {
        result->applicability_flags = BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN;
    }

    result->burn_rate_m_per_s = burn_rate_m_per_s;

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e
bbtc_ib_pressure_power_burn_kinetics_evaluate_double(
    const bbtc_ib_pressure_power_burn_kinetics_double_t* const model,
    const double absolute_pressure_pa,
    bbtc_ib_propellant_burn_kinetics_result_double_t* const result
)
{
    bbtc_status_e status;
    double        pressure_difference_pa;
    double        log_pressure_ratio;
    double        log_burn_rate;
    double        burn_rate_m_per_s;

    if (result == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    *result = (bbtc_ib_propellant_burn_kinetics_result_double_t){0};

    status = bbtc_ib_pressure_power_burn_kinetics_validate_double(model);

    if (status != BBTC_STATUS_SUCCESS)
        return status;

    if (isnan(absolute_pressure_pa))
        return BBTC_STATUS_NAN_INPUT;

    if (isinf(absolute_pressure_pa))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (absolute_pressure_pa < 0.0)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    if (absolute_pressure_pa == 0.0)
    {
        result->applicability_flags =
            BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN;
        return BBTC_STATUS_SUCCESS;
    }

    if (absolute_pressure_pa == model->reference_pressure_pa)
    {
        result->burn_rate_m_per_s = model->reference_burn_rate_m_per_s;
        return BBTC_STATUS_SUCCESS;
    }

    pressure_difference_pa =
        absolute_pressure_pa - model->reference_pressure_pa;

    if (fabs(pressure_difference_pa) <= 0.5 * model->reference_pressure_pa)
    {
        log_pressure_ratio = log1p(pressure_difference_pa / model->reference_pressure_pa);
    }
    else
        log_pressure_ratio = log(absolute_pressure_pa) - log(model->reference_pressure_pa);


    log_burn_rate = log(model->reference_burn_rate_m_per_s)
                  + model->pressure_exponent * log_pressure_ratio;

    if (!isfinite(log_burn_rate))
        return BBTC_STATUS_NUMERICAL_FAILURE;

    burn_rate_m_per_s = exp(log_burn_rate);

    if (!isfinite(burn_rate_m_per_s) || burn_rate_m_per_s <= 0.0)
        return BBTC_STATUS_NUMERICAL_FAILURE;

    if (absolute_pressure_pa < model->minimum_calibrated_pressure_pa ||
        absolute_pressure_pa > model->maximum_calibrated_pressure_pa)
    {
        result->applicability_flags = BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN;
    }

    result->burn_rate_m_per_s = burn_rate_m_per_s;

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e
bbtc_ib_pressure_power_burn_kinetics_evaluate_long_double(
    const bbtc_ib_pressure_power_burn_kinetics_long_double_t* const model,
    const long double absolute_pressure_pa,
    bbtc_ib_propellant_burn_kinetics_result_long_double_t* const result
)
{
    bbtc_status_e status;
    long double   pressure_difference_pa;
    long double   log_pressure_ratio;
    long double   log_burn_rate;
    long double   burn_rate_m_per_s;

    if (result == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    *result = (bbtc_ib_propellant_burn_kinetics_result_long_double_t){0};

    status = bbtc_ib_pressure_power_burn_kinetics_validate_long_double(model);

    if (status != BBTC_STATUS_SUCCESS)
        return status;

    if (isnan(absolute_pressure_pa))
        return BBTC_STATUS_NAN_INPUT;

    if (isinf(absolute_pressure_pa))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (absolute_pressure_pa < 0.0L)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    if (absolute_pressure_pa == 0.0L)
    {
        result->applicability_flags =
            BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN;
        return BBTC_STATUS_SUCCESS;
    }

    if (absolute_pressure_pa == model->reference_pressure_pa)
    {
        result->burn_rate_m_per_s = model->reference_burn_rate_m_per_s;
        return BBTC_STATUS_SUCCESS;
    }

    pressure_difference_pa = absolute_pressure_pa - model->reference_pressure_pa;

    if (fabsl(pressure_difference_pa) <= 0.5L * model->reference_pressure_pa)
    {
        log_pressure_ratio = log1pl(pressure_difference_pa / model->reference_pressure_pa);
    }
    else
        log_pressure_ratio = logl(absolute_pressure_pa) - logl(model->reference_pressure_pa);


    log_burn_rate = logl(model->reference_burn_rate_m_per_s)
                  + model->pressure_exponent * log_pressure_ratio;

    if (!isfinite(log_burn_rate))
        return BBTC_STATUS_NUMERICAL_FAILURE;

    burn_rate_m_per_s = expl(log_burn_rate);

    if (!isfinite(burn_rate_m_per_s) || burn_rate_m_per_s <= 0.0L)
        return BBTC_STATUS_NUMERICAL_FAILURE;

    if (absolute_pressure_pa < model->minimum_calibrated_pressure_pa ||
        absolute_pressure_pa > model->maximum_calibrated_pressure_pa)
    {
        result->applicability_flags = BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN;
    }

    result->burn_rate_m_per_s = burn_rate_m_per_s;

    return BBTC_STATUS_SUCCESS;
}
