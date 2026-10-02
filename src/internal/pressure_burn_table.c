/**
 * @file
 * @brief Tabulated absolute-pressure propellant burn-kinetics implementation.
 *
 * @details
 * This translation unit evaluates caller-owned empirical pressure/burn-rate
 * tables. Between adjacent pressure knots, interpolation is piecewise linear in
 * log-pressure/log-burn-rate space:
 *
 * `x = ln(P / P0) / ln(P1 / P0)`
 *
 * `r = r0 * exp(x * ln(r1 / r0))`
 *
 * The implementation preserves exact stored knot values, performs no pressure
 * extrapolation, allocates no memory, and keeps all arithmetic in the selected
 * native scalar family.
 */
#include <stddef.h>
#include <math.h>

#include "bbtc/internal_ballistics/propellant_burn_kinetics.h"


/**
 * @brief Computes `ln(numerator / denominator)` robustly in native `float`.
 *
 * @details
 * Both arguments are required by the validated caller to be finite and
 * strictly positive. Near unity, `log1p` avoids cancellation from subtracting
 * nearly equal logarithms. Away from unity, subtracting logarithms avoids
 * explicitly forming a ratio that could overflow or underflow even though both
 * original values are representable.
 */
static float
log_positive_ratio_float(
    const float numerator,
    const float denominator
)
{
    const float difference = numerator - denominator;

    if (fabsf(difference) <= 0.5f * denominator)
        return log1pf(difference / denominator);

    return logf(numerator) - logf(denominator);
}


/**
 * @brief Computes `ln(numerator / denominator)` robustly in native `double`.
 *
 * @details
 * Semantics match `log_positive_ratio_float()` while retaining native
 * `double` arithmetic.
 */
static double
log_positive_ratio_double(
    const double numerator,
    const double denominator
)
{
    const double difference = numerator - denominator;

    if (fabs(difference) <= 0.5 * denominator)
        return log1p(difference / denominator);

    return log(numerator) - log(denominator);
}


/**
 * @brief Computes `ln(numerator / denominator)` in native `long double`.
 *
 * @details
 * Semantics match `log_positive_ratio_float()` without routing the calculation
 * through `double`.
 */
static long double
log_positive_ratio_long_double(
    const long double numerator,
    const long double denominator
)
{
    const long double difference = numerator - denominator;

    if (fabsl(difference) <= 0.5L * denominator)
        return log1pl(difference / denominator);

    return logl(numerator) - logl(denominator);
}


bbtc_status_e
bbtc_ib_pressure_burn_table_validate_float(
    const bbtc_ib_pressure_burn_table_float_t* const model
)
{
    size_t i;

    if (model == NULL || model->points == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (model->point_count < 2U)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    /*
     * NaN precedence is defined over the complete scalar-data layer, not by
     * point order. Scan the entire table before classifying infinities or finite
     * domain errors.
     */
    for (i = 0U; i < model->point_count; ++i)
    {
        if (isnan(model->points[i].pressure_pa)
            || isnan(model->points[i].burn_rate_m_per_s))
        {
            return BBTC_STATUS_NAN_INPUT;
        }
    }

    /*
     * Infinity is the next scalar-data classification. Delaying finite-domain
     * checks until this pass completes keeps status precedence independent of
     * where malformed data happens to appear in the caller-owned array.
     */
    for (i = 0U; i < model->point_count; ++i)
    {
        if (isinf(model->points[i].pressure_pa)
            || isinf(model->points[i].burn_rate_m_per_s))
        {
            return BBTC_STATUS_NONFINITE_INPUT;
        }
    }

    for (i = 0U; i < model->point_count; ++i)
    {
        if (model->points[i].pressure_pa <= 0.0f
            || model->points[i].burn_rate_m_per_s <= 0.0f)
        {
            return BBTC_STATUS_OUTSIDE_DOMAIN;
        }

        if (i > 0U
            && model->points[i].pressure_pa
                <= model->points[i - 1U].pressure_pa)
        {
            return BBTC_STATUS_OUTSIDE_DOMAIN;
        }
    }

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e
bbtc_ib_pressure_burn_table_validate_double(
    const bbtc_ib_pressure_burn_table_double_t* const model
)
{
    size_t i;

    if (model == NULL || model->points == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (model->point_count < 2U)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    for (i = 0U; i < model->point_count; ++i)
    {
        if (isnan(model->points[i].pressure_pa)
            || isnan(model->points[i].burn_rate_m_per_s))
        {
            return BBTC_STATUS_NAN_INPUT;
        }
    }

    for (i = 0U; i < model->point_count; ++i)
    {
        if (isinf(model->points[i].pressure_pa)
            || isinf(model->points[i].burn_rate_m_per_s))
        {
            return BBTC_STATUS_NONFINITE_INPUT;
        }
    }

    for (i = 0U; i < model->point_count; ++i)
    {
        if (model->points[i].pressure_pa <= 0.0
            || model->points[i].burn_rate_m_per_s <= 0.0)
        {
            return BBTC_STATUS_OUTSIDE_DOMAIN;
        }

        if (i > 0U
            && model->points[i].pressure_pa
                <= model->points[i - 1U].pressure_pa)
        {
            return BBTC_STATUS_OUTSIDE_DOMAIN;
        }
    }

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e
bbtc_ib_pressure_burn_table_validate_long_double(
    const bbtc_ib_pressure_burn_table_long_double_t* const model
)
{
    size_t i;

    if (model == NULL || model->points == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (model->point_count < 2U)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    for (i = 0U; i < model->point_count; ++i)
    {
        if (isnan(model->points[i].pressure_pa)
            || isnan(model->points[i].burn_rate_m_per_s))
        {
            return BBTC_STATUS_NAN_INPUT;
        }
    }

    for (i = 0U; i < model->point_count; ++i)
    {
        if (isinf(model->points[i].pressure_pa)
            || isinf(model->points[i].burn_rate_m_per_s))
        {
            return BBTC_STATUS_NONFINITE_INPUT;
        }
    }

    for (i = 0U; i < model->point_count; ++i)
    {
        if (model->points[i].pressure_pa <= 0.0L
            || model->points[i].burn_rate_m_per_s <= 0.0L)
        {
            return BBTC_STATUS_OUTSIDE_DOMAIN;
        }

        if (i > 0U
            && model->points[i].pressure_pa
                <= model->points[i - 1U].pressure_pa)
        {
            return BBTC_STATUS_OUTSIDE_DOMAIN;
        }
    }

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e
bbtc_ib_pressure_burn_table_evaluate_float(
    const bbtc_ib_pressure_burn_table_float_t* const model,
    const float absolute_pressure_pa,
    bbtc_ib_propellant_burn_kinetics_result_float_t* const result
)
{
    bbtc_status_e status;
    size_t lower;
    size_t upper;
    size_t middle;
    float log_pressure_offset;
    float log_pressure_span;
    float interpolation_fraction;
    float log_rate_span;
    float log_burn_rate;
    float burn_rate_m_per_s;

    if (result == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    *result = (bbtc_ib_propellant_burn_kinetics_result_float_t){0};

    status = bbtc_ib_pressure_burn_table_validate_float(model);

    if (status != BBTC_STATUS_SUCCESS)
        return status;

    if (isnan(absolute_pressure_pa))
        return BBTC_STATUS_NAN_INPUT;

    if (isinf(absolute_pressure_pa))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (absolute_pressure_pa < model->points[0U].pressure_pa
        || absolute_pressure_pa
            > model->points[model->point_count - 1U].pressure_pa)
    {
        return BBTC_STATUS_OUTSIDE_DOMAIN;
    }

    /*
     * Keep exact endpoints outside the search loop. In addition to making the
     * closed-domain behavior explicit, this guarantees bit-for-bit recovery of
     * the caller's stored endpoint rates without transcendental reconstruction.
     */
    if (absolute_pressure_pa == model->points[0U].pressure_pa)
    {
        result->burn_rate_m_per_s = model->points[0U].burn_rate_m_per_s;
        return BBTC_STATUS_SUCCESS;
    }

    if (absolute_pressure_pa
        == model->points[model->point_count - 1U].pressure_pa)
    {
        result->burn_rate_m_per_s =
            model->points[model->point_count - 1U].burn_rate_m_per_s;
        return BBTC_STATUS_SUCCESS;
    }

    /*
     * Model validation guarantees strict pressure ordering. Because the direct
     * pressure is already known to lie strictly between the endpoint knots, the
     * loop finishes with adjacent indices satisfying
     *
     *     points[lower].pressure_pa < P < points[upper].pressure_pa
     *
     * unless an exact interior knot is found first.
     */
    lower = 0U;
    upper = model->point_count - 1U;

    while (upper - lower > 1U)
    {
        middle = lower + (upper - lower) / 2U;

        if (absolute_pressure_pa == model->points[middle].pressure_pa)
        {
            result->burn_rate_m_per_s =
                model->points[middle].burn_rate_m_per_s;
            return BBTC_STATUS_SUCCESS;
        }

        if (absolute_pressure_pa < model->points[middle].pressure_pa)
            upper = middle;
        else
            lower = middle;
    }

    /*
     * Equal endpoint rates define an exactly flat log/log segment. Preserve the
     * caller's stored value directly instead of reconstructing it through
     * log/exp, which is both more accurate and especially helpful near native
     * representability limits.
     */
    if (model->points[lower].burn_rate_m_per_s
        == model->points[upper].burn_rate_m_per_s)
    {
        result->burn_rate_m_per_s =
            model->points[lower].burn_rate_m_per_s;
        return BBTC_STATUS_SUCCESS;
    }

    log_pressure_offset = log_positive_ratio_float(
        absolute_pressure_pa,
        model->points[lower].pressure_pa
    );
    log_pressure_span = log_positive_ratio_float(
        model->points[upper].pressure_pa,
        model->points[lower].pressure_pa
    );

    if (!isfinite(log_pressure_offset)
        || !isfinite(log_pressure_span)
        || log_pressure_span <= 0.0f)
    {
        return BBTC_STATUS_NUMERICAL_FAILURE;
    }

    interpolation_fraction = log_pressure_offset / log_pressure_span;

    if (!isfinite(interpolation_fraction))
        return BBTC_STATUS_NUMERICAL_FAILURE;

    log_rate_span = log_positive_ratio_float(
        model->points[upper].burn_rate_m_per_s,
        model->points[lower].burn_rate_m_per_s
    );

    if (!isfinite(log_rate_span))
        return BBTC_STATUS_NUMERICAL_FAILURE;

    log_burn_rate = logf(model->points[lower].burn_rate_m_per_s)
                  + interpolation_fraction * log_rate_span;

    if (!isfinite(log_burn_rate))
        return BBTC_STATUS_NUMERICAL_FAILURE;

    burn_rate_m_per_s = expf(log_burn_rate);

    if (!isfinite(burn_rate_m_per_s) || burn_rate_m_per_s <= 0.0f)
        return BBTC_STATUS_NUMERICAL_FAILURE;

    result->burn_rate_m_per_s = burn_rate_m_per_s;

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e
bbtc_ib_pressure_burn_table_evaluate_double(
    const bbtc_ib_pressure_burn_table_double_t* const model,
    const double absolute_pressure_pa,
    bbtc_ib_propellant_burn_kinetics_result_double_t* const result
)
{
    bbtc_status_e status;
    size_t lower;
    size_t upper;
    size_t middle;
    double log_pressure_offset;
    double log_pressure_span;
    double interpolation_fraction;
    double log_rate_span;
    double log_burn_rate;
    double burn_rate_m_per_s;

    if (result == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    *result = (bbtc_ib_propellant_burn_kinetics_result_double_t){0};

    status = bbtc_ib_pressure_burn_table_validate_double(model);

    if (status != BBTC_STATUS_SUCCESS)
        return status;

    if (isnan(absolute_pressure_pa))
        return BBTC_STATUS_NAN_INPUT;

    if (isinf(absolute_pressure_pa))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (absolute_pressure_pa < model->points[0U].pressure_pa
        || absolute_pressure_pa
            > model->points[model->point_count - 1U].pressure_pa)
    {
        return BBTC_STATUS_OUTSIDE_DOMAIN;
    }

    if (absolute_pressure_pa == model->points[0U].pressure_pa)
    {
        result->burn_rate_m_per_s = model->points[0U].burn_rate_m_per_s;
        return BBTC_STATUS_SUCCESS;
    }

    if (absolute_pressure_pa
        == model->points[model->point_count - 1U].pressure_pa)
    {
        result->burn_rate_m_per_s =
            model->points[model->point_count - 1U].burn_rate_m_per_s;
        return BBTC_STATUS_SUCCESS;
    }

    lower = 0U;
    upper = model->point_count - 1U;

    while (upper - lower > 1U)
    {
        middle = lower + (upper - lower) / 2U;

        if (absolute_pressure_pa == model->points[middle].pressure_pa)
        {
            result->burn_rate_m_per_s =
                model->points[middle].burn_rate_m_per_s;
            return BBTC_STATUS_SUCCESS;
        }

        if (absolute_pressure_pa < model->points[middle].pressure_pa)
            upper = middle;
        else
            lower = middle;
    }

    if (model->points[lower].burn_rate_m_per_s
        == model->points[upper].burn_rate_m_per_s)
    {
        result->burn_rate_m_per_s =
            model->points[lower].burn_rate_m_per_s;
        return BBTC_STATUS_SUCCESS;
    }

    log_pressure_offset = log_positive_ratio_double(
        absolute_pressure_pa,
        model->points[lower].pressure_pa
    );
    log_pressure_span = log_positive_ratio_double(
        model->points[upper].pressure_pa,
        model->points[lower].pressure_pa
    );

    if (!isfinite(log_pressure_offset)
        || !isfinite(log_pressure_span)
        || log_pressure_span <= 0.0)
    {
        return BBTC_STATUS_NUMERICAL_FAILURE;
    }

    interpolation_fraction = log_pressure_offset / log_pressure_span;

    if (!isfinite(interpolation_fraction))
        return BBTC_STATUS_NUMERICAL_FAILURE;

    log_rate_span = log_positive_ratio_double(
        model->points[upper].burn_rate_m_per_s,
        model->points[lower].burn_rate_m_per_s
    );

    if (!isfinite(log_rate_span))
        return BBTC_STATUS_NUMERICAL_FAILURE;

    log_burn_rate = log(model->points[lower].burn_rate_m_per_s)
                  + interpolation_fraction * log_rate_span;

    if (!isfinite(log_burn_rate))
        return BBTC_STATUS_NUMERICAL_FAILURE;

    burn_rate_m_per_s = exp(log_burn_rate);

    if (!isfinite(burn_rate_m_per_s) || burn_rate_m_per_s <= 0.0)
        return BBTC_STATUS_NUMERICAL_FAILURE;

    result->burn_rate_m_per_s = burn_rate_m_per_s;

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e
bbtc_ib_pressure_burn_table_evaluate_long_double(
    const bbtc_ib_pressure_burn_table_long_double_t* const model,
    const long double absolute_pressure_pa,
    bbtc_ib_propellant_burn_kinetics_result_long_double_t* const result
)
{
    bbtc_status_e status;
    size_t lower;
    size_t upper;
    size_t middle;
    long double log_pressure_offset;
    long double log_pressure_span;
    long double interpolation_fraction;
    long double log_rate_span;
    long double log_burn_rate;
    long double burn_rate_m_per_s;

    if (result == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    *result = (bbtc_ib_propellant_burn_kinetics_result_long_double_t){0};

    status = bbtc_ib_pressure_burn_table_validate_long_double(model);

    if (status != BBTC_STATUS_SUCCESS)
        return status;

    if (isnan(absolute_pressure_pa))
        return BBTC_STATUS_NAN_INPUT;

    if (isinf(absolute_pressure_pa))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (absolute_pressure_pa < model->points[0U].pressure_pa
        || absolute_pressure_pa
            > model->points[model->point_count - 1U].pressure_pa)
    {
        return BBTC_STATUS_OUTSIDE_DOMAIN;
    }

    if (absolute_pressure_pa == model->points[0U].pressure_pa)
    {
        result->burn_rate_m_per_s = model->points[0U].burn_rate_m_per_s;
        return BBTC_STATUS_SUCCESS;
    }

    if (absolute_pressure_pa
        == model->points[model->point_count - 1U].pressure_pa)
    {
        result->burn_rate_m_per_s =
            model->points[model->point_count - 1U].burn_rate_m_per_s;
        return BBTC_STATUS_SUCCESS;
    }

    lower = 0U;
    upper = model->point_count - 1U;

    while (upper - lower > 1U)
    {
        middle = lower + (upper - lower) / 2U;

        if (absolute_pressure_pa == model->points[middle].pressure_pa)
        {
            result->burn_rate_m_per_s =
                model->points[middle].burn_rate_m_per_s;
            return BBTC_STATUS_SUCCESS;
        }

        if (absolute_pressure_pa < model->points[middle].pressure_pa)
            upper = middle;
        else
            lower = middle;
    }

    if (model->points[lower].burn_rate_m_per_s
        == model->points[upper].burn_rate_m_per_s)
    {
        result->burn_rate_m_per_s =
            model->points[lower].burn_rate_m_per_s;
        return BBTC_STATUS_SUCCESS;
    }

    log_pressure_offset = log_positive_ratio_long_double(
        absolute_pressure_pa,
        model->points[lower].pressure_pa
    );
    log_pressure_span = log_positive_ratio_long_double(
        model->points[upper].pressure_pa,
        model->points[lower].pressure_pa
    );

    if (!isfinite(log_pressure_offset)
        || !isfinite(log_pressure_span)
        || log_pressure_span <= 0.0L)
    {
        return BBTC_STATUS_NUMERICAL_FAILURE;
    }

    interpolation_fraction = log_pressure_offset / log_pressure_span;

    if (!isfinite(interpolation_fraction))
        return BBTC_STATUS_NUMERICAL_FAILURE;

    log_rate_span = log_positive_ratio_long_double(
        model->points[upper].burn_rate_m_per_s,
        model->points[lower].burn_rate_m_per_s
    );

    if (!isfinite(log_rate_span))
        return BBTC_STATUS_NUMERICAL_FAILURE;

    log_burn_rate = logl(model->points[lower].burn_rate_m_per_s)
                  + interpolation_fraction * log_rate_span;

    if (!isfinite(log_burn_rate))
        return BBTC_STATUS_NUMERICAL_FAILURE;

    burn_rate_m_per_s = expl(log_burn_rate);

    if (!isfinite(burn_rate_m_per_s) || burn_rate_m_per_s <= 0.0L)
        return BBTC_STATUS_NUMERICAL_FAILURE;

    result->burn_rate_m_per_s = burn_rate_m_per_s;

    return BBTC_STATUS_SUCCESS;
}
