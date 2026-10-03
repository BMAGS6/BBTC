/**
 * @file
 * @brief Whole-charge propellant mass coupling from canonical grain state.
 *
 * @details
 * The implementation in this translation unit is intentionally explicit rather
 * than generic. Each scalar family keeps its own arithmetic so `float`,
 * `double`, and `long double` remain independently testable native execution
 * paths. The only shared behavior is contractual: validation precedence,
 * endpoint semantics, applicability propagation, and representability rules are
 * mirrored across all three families.
 */
#include <math.h>
#include <stddef.h>

#include "bbtc/internal_ballistics/propellant_mass.h"


/**
 * @brief Computes `a / (b * c)` without forming `b * c` directly in `float`.
 *
 * @details
 * All arguments are already known to be finite and strictly positive. Splitting
 * every factor into a binary mantissa and exponent keeps intermediate
 * mantissas close to unity, preventing an otherwise avoidable denominator
 * overflow or underflow. The final `ldexpf()` is allowed to report the actual
 * representability limit of the requested native-`float` result; the caller
 * classifies zero or nonfinite output as a numerical failure when positivity is
 * required.
 */
static float positive_div_product_float(
    const float a,
    const float b,
    const float c)
{
    int exponent_a;
    int exponent_b;
    int exponent_c;
    int exponent_mantissa;

    const float mantissa_a = frexpf(a, &exponent_a);
    const float mantissa_b = frexpf(b, &exponent_b);
    const float mantissa_c = frexpf(c, &exponent_c);

    float mantissa = mantissa_a / (mantissa_b * mantissa_c);

    mantissa = frexpf(mantissa, &exponent_mantissa);

    return ldexpf(
        mantissa,
        exponent_a - exponent_b - exponent_c + exponent_mantissa
    );
}


/**
 * @brief Computes `(a * b) / c` without avoidable intermediate range loss.
 */
static float positive_product_ratio_float(
    const float a,
    const float b,
    const float c)
{
    int exponent_a;
    int exponent_b;
    int exponent_c;
    int exponent_mantissa;

    const float mantissa_a = frexpf(a, &exponent_a);
    const float mantissa_b = frexpf(b, &exponent_b);
    const float mantissa_c = frexpf(c, &exponent_c);

    float mantissa = (mantissa_a * mantissa_b) / mantissa_c;

    mantissa = frexpf(mantissa, &exponent_mantissa);

    return ldexpf(
        mantissa,
        exponent_a + exponent_b - exponent_c + exponent_mantissa
    );
}


/**
 * @brief Computes `(a * b * c) / d` with exponent-separated `float` factors.
 */
static float positive_triple_product_ratio_float(
    const float a,
    const float b,
    const float c,
    const float d)
{
    int exponent_a;
    int exponent_b;
    int exponent_c;
    int exponent_d;
    int exponent_mantissa;

    const float mantissa_a = frexpf(a, &exponent_a);
    const float mantissa_b = frexpf(b, &exponent_b);
    const float mantissa_c = frexpf(c, &exponent_c);
    const float mantissa_d = frexpf(d, &exponent_d);

    float mantissa = (mantissa_a * mantissa_b * mantissa_c) / mantissa_d;

    mantissa = frexpf(mantissa, &exponent_mantissa);

    return ldexpf(
        mantissa,
        exponent_a + exponent_b + exponent_c - exponent_d + exponent_mantissa
    );
}


/**
 * @brief Native-`double` form of `positive_div_product_float()`.
 */
static double positive_div_product_double(
    const double a,
    const double b,
    const double c)
{
    int exponent_a;
    int exponent_b;
    int exponent_c;
    int exponent_mantissa;

    const double mantissa_a = frexp(a, &exponent_a);
    const double mantissa_b = frexp(b, &exponent_b);
    const double mantissa_c = frexp(c, &exponent_c);

    double mantissa = mantissa_a / (mantissa_b * mantissa_c);

    mantissa = frexp(mantissa, &exponent_mantissa);

    return ldexp(
        mantissa,
        exponent_a - exponent_b - exponent_c + exponent_mantissa
    );
}


/**
 * @brief Native-`double` form of `positive_product_ratio_float()`.
 */
static double positive_product_ratio_double(
    const double a,
    const double b,
    const double c)
{
    int exponent_a;
    int exponent_b;
    int exponent_c;
    int exponent_mantissa;

    const double mantissa_a = frexp(a, &exponent_a);
    const double mantissa_b = frexp(b, &exponent_b);
    const double mantissa_c = frexp(c, &exponent_c);

    double mantissa = (mantissa_a * mantissa_b) / mantissa_c;

    mantissa = frexp(mantissa, &exponent_mantissa);

    return ldexp(
        mantissa,
        exponent_a + exponent_b - exponent_c + exponent_mantissa
    );
}


/**
 * @brief Native-`double` form of `positive_triple_product_ratio_float()`.
 */
static double positive_triple_product_ratio_double(
    const double a,
    const double b,
    const double c,
    const double d)
{
    int exponent_a;
    int exponent_b;
    int exponent_c;
    int exponent_d;
    int exponent_mantissa;

    const double mantissa_a = frexp(a, &exponent_a);
    const double mantissa_b = frexp(b, &exponent_b);
    const double mantissa_c = frexp(c, &exponent_c);
    const double mantissa_d = frexp(d, &exponent_d);

    double mantissa = (mantissa_a * mantissa_b * mantissa_c) / mantissa_d;

    mantissa = frexp(mantissa, &exponent_mantissa);

    return ldexp(
        mantissa,
        exponent_a + exponent_b + exponent_c - exponent_d + exponent_mantissa
    );
}


/**
 * @brief Native-`long double` form of `positive_div_product_float()`.
 */
static long double positive_div_product_long_double(
    const long double a,
    const long double b,
    const long double c)
{
    int exponent_a;
    int exponent_b;
    int exponent_c;
    int exponent_mantissa;

    const long double mantissa_a = frexpl(a, &exponent_a);
    const long double mantissa_b = frexpl(b, &exponent_b);
    const long double mantissa_c = frexpl(c, &exponent_c);

    long double mantissa = mantissa_a / (mantissa_b * mantissa_c);

    mantissa = frexpl(mantissa, &exponent_mantissa);

    return ldexpl(
        mantissa,
        exponent_a - exponent_b - exponent_c + exponent_mantissa
    );
}


/**
 * @brief Native-`long double` form of `positive_product_ratio_float()`.
 */
static long double positive_product_ratio_long_double(
    const long double a,
    const long double b,
    const long double c)
{
    int exponent_a;
    int exponent_b;
    int exponent_c;
    int exponent_mantissa;

    const long double mantissa_a = frexpl(a, &exponent_a);
    const long double mantissa_b = frexpl(b, &exponent_b);
    const long double mantissa_c = frexpl(c, &exponent_c);

    long double mantissa = (mantissa_a * mantissa_b) / mantissa_c;

    mantissa = frexpl(mantissa, &exponent_mantissa);

    return ldexpl(
        mantissa,
        exponent_a + exponent_b - exponent_c + exponent_mantissa
    );
}


/**
 * @brief Native-`long double` form of `positive_triple_product_ratio_float()`.
 */
static long double positive_triple_product_ratio_long_double(
    const long double a,
    const long double b,
    const long double c,
    const long double d)
{
    int exponent_a;
    int exponent_b;
    int exponent_c;
    int exponent_d;
    int exponent_mantissa;

    const long double mantissa_a = frexpl(a, &exponent_a);
    const long double mantissa_b = frexpl(b, &exponent_b);
    const long double mantissa_c = frexpl(c, &exponent_c);
    const long double mantissa_d = frexpl(d, &exponent_d);

    long double mantissa =
        (mantissa_a * mantissa_b * mantissa_c) / mantissa_d;

    mantissa = frexpl(mantissa, &exponent_mantissa);

    return ldexpl(
        mantissa,
        exponent_a + exponent_b + exponent_c - exponent_d + exponent_mantissa
    );
}


/**
 * @brief Validates one common native-`float` grain state for mass coupling.
 *
 * @details
 * This is deliberately a coupling-layer validator, not a replacement for the
 * geometry backend. It checks only scalar validity and the broad endpoint /
 * interior invariants that the coupling layer can defend without duplicating a
 * geometry-specific relation between remaining volume and consumed fraction.
 */
static bbtc_status_e validate_grain_state_float(
    const bbtc_ib_propellant_grain_state_float_t* const grain_state,
    const float initial_grain_volume_m3)
{
    if (grain_state == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (isnan(grain_state->remaining_volume_m3)
        || isnan(grain_state->burning_surface_area_m2)
        || isnan(grain_state->remaining_regression_to_burnout_m)
        || isnan(grain_state->consumed_volume_fraction))
    {
        return BBTC_STATUS_NAN_INPUT;
    }

    if (isinf(grain_state->remaining_volume_m3)
        || isinf(grain_state->burning_surface_area_m2)
        || isinf(grain_state->remaining_regression_to_burnout_m)
        || isinf(grain_state->consumed_volume_fraction))
    {
        return BBTC_STATUS_NONFINITE_INPUT;
    }

    if (grain_state->remaining_volume_m3 < 0.0f
        || grain_state->burning_surface_area_m2 < 0.0f
        || grain_state->remaining_regression_to_burnout_m < 0.0f
        || grain_state->consumed_volume_fraction < 0.0f
        || grain_state->consumed_volume_fraction > 1.0f)
    {
        return BBTC_STATUS_OUTSIDE_DOMAIN;
    }

    if (grain_state->remaining_volume_m3 > initial_grain_volume_m3)
        return BBTC_STATUS_INCONSISTENT_CONFIGURATION;

    if (grain_state->consumed_volume_fraction == 0.0f)
    {
        if (grain_state->remaining_volume_m3 != initial_grain_volume_m3
            || grain_state->burning_surface_area_m2 <= 0.0f
            || grain_state->remaining_regression_to_burnout_m <= 0.0f)
        {
            return BBTC_STATUS_INCONSISTENT_CONFIGURATION;
        }

        return BBTC_STATUS_SUCCESS;
    }

    if (grain_state->consumed_volume_fraction == 1.0f)
    {
        if (grain_state->remaining_volume_m3 != 0.0f
            || grain_state->burning_surface_area_m2 != 0.0f
            || grain_state->remaining_regression_to_burnout_m != 0.0f)
        {
            return BBTC_STATUS_INCONSISTENT_CONFIGURATION;
        }

        return BBTC_STATUS_SUCCESS;
    }

    if (grain_state->remaining_volume_m3 <= 0.0f
        || grain_state->remaining_volume_m3 >= initial_grain_volume_m3
        || grain_state->burning_surface_area_m2 <= 0.0f
        || grain_state->remaining_regression_to_burnout_m <= 0.0f)
    {
        return BBTC_STATUS_INCONSISTENT_CONFIGURATION;
    }

    return BBTC_STATUS_SUCCESS;
}


/** @brief Native-`double` grain-state validation for mass coupling. */
static bbtc_status_e validate_grain_state_double(
    const bbtc_ib_propellant_grain_state_double_t* const grain_state,
    const double initial_grain_volume_m3)
{
    if (grain_state == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (isnan(grain_state->remaining_volume_m3)
        || isnan(grain_state->burning_surface_area_m2)
        || isnan(grain_state->remaining_regression_to_burnout_m)
        || isnan(grain_state->consumed_volume_fraction))
    {
        return BBTC_STATUS_NAN_INPUT;
    }

    if (isinf(grain_state->remaining_volume_m3)
        || isinf(grain_state->burning_surface_area_m2)
        || isinf(grain_state->remaining_regression_to_burnout_m)
        || isinf(grain_state->consumed_volume_fraction))
    {
        return BBTC_STATUS_NONFINITE_INPUT;
    }

    if (grain_state->remaining_volume_m3 < 0.0
        || grain_state->burning_surface_area_m2 < 0.0
        || grain_state->remaining_regression_to_burnout_m < 0.0
        || grain_state->consumed_volume_fraction < 0.0
        || grain_state->consumed_volume_fraction > 1.0)
    {
        return BBTC_STATUS_OUTSIDE_DOMAIN;
    }

    if (grain_state->remaining_volume_m3 > initial_grain_volume_m3)
        return BBTC_STATUS_INCONSISTENT_CONFIGURATION;

    if (grain_state->consumed_volume_fraction == 0.0)
    {
        if (grain_state->remaining_volume_m3 != initial_grain_volume_m3
            || grain_state->burning_surface_area_m2 <= 0.0
            || grain_state->remaining_regression_to_burnout_m <= 0.0)
        {
            return BBTC_STATUS_INCONSISTENT_CONFIGURATION;
        }

        return BBTC_STATUS_SUCCESS;
    }

    if (grain_state->consumed_volume_fraction == 1.0)
    {
        if (grain_state->remaining_volume_m3 != 0.0
            || grain_state->burning_surface_area_m2 != 0.0
            || grain_state->remaining_regression_to_burnout_m != 0.0)
        {
            return BBTC_STATUS_INCONSISTENT_CONFIGURATION;
        }

        return BBTC_STATUS_SUCCESS;
    }

    if (grain_state->remaining_volume_m3 <= 0.0
        || grain_state->remaining_volume_m3 >= initial_grain_volume_m3
        || grain_state->burning_surface_area_m2 <= 0.0
        || grain_state->remaining_regression_to_burnout_m <= 0.0)
    {
        return BBTC_STATUS_INCONSISTENT_CONFIGURATION;
    }

    return BBTC_STATUS_SUCCESS;
}


/** @brief Native-`long double` grain-state validation for mass coupling. */
static bbtc_status_e validate_grain_state_long_double(
    const bbtc_ib_propellant_grain_state_long_double_t* const grain_state,
    const long double initial_grain_volume_m3)
{
    if (grain_state == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (isnan(grain_state->remaining_volume_m3)
        || isnan(grain_state->burning_surface_area_m2)
        || isnan(grain_state->remaining_regression_to_burnout_m)
        || isnan(grain_state->consumed_volume_fraction))
    {
        return BBTC_STATUS_NAN_INPUT;
    }

    if (isinf(grain_state->remaining_volume_m3)
        || isinf(grain_state->burning_surface_area_m2)
        || isinf(grain_state->remaining_regression_to_burnout_m)
        || isinf(grain_state->consumed_volume_fraction))
    {
        return BBTC_STATUS_NONFINITE_INPUT;
    }

    if (grain_state->remaining_volume_m3 < 0.0L
        || grain_state->burning_surface_area_m2 < 0.0L
        || grain_state->remaining_regression_to_burnout_m < 0.0L
        || grain_state->consumed_volume_fraction < 0.0L
        || grain_state->consumed_volume_fraction > 1.0L)
    {
        return BBTC_STATUS_OUTSIDE_DOMAIN;
    }

    if (grain_state->remaining_volume_m3 > initial_grain_volume_m3)
        return BBTC_STATUS_INCONSISTENT_CONFIGURATION;

    if (grain_state->consumed_volume_fraction == 0.0L)
    {
        if (grain_state->remaining_volume_m3 != initial_grain_volume_m3
            || grain_state->burning_surface_area_m2 <= 0.0L
            || grain_state->remaining_regression_to_burnout_m <= 0.0L)
        {
            return BBTC_STATUS_INCONSISTENT_CONFIGURATION;
        }

        return BBTC_STATUS_SUCCESS;
    }

    if (grain_state->consumed_volume_fraction == 1.0L)
    {
        if (grain_state->remaining_volume_m3 != 0.0L
            || grain_state->burning_surface_area_m2 != 0.0L
            || grain_state->remaining_regression_to_burnout_m != 0.0L)
        {
            return BBTC_STATUS_INCONSISTENT_CONFIGURATION;
        }

        return BBTC_STATUS_SUCCESS;
    }

    if (grain_state->remaining_volume_m3 <= 0.0L
        || grain_state->remaining_volume_m3 >= initial_grain_volume_m3
        || grain_state->burning_surface_area_m2 <= 0.0L
        || grain_state->remaining_regression_to_burnout_m <= 0.0L)
    {
        return BBTC_STATUS_INCONSISTENT_CONFIGURATION;
    }

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e bbtc_ib_propellant_mass_evaluate_float(
    const bbtc_ib_propellant_charge_float_t* const charge,
    const float initial_grain_volume_m3,
    const bbtc_ib_propellant_grain_state_float_t* const grain_state,
    const bbtc_ib_propellant_burn_kinetics_result_float_t* const kinetics,
    bbtc_ib_propellant_mass_result_float_t* const result)
{
    bbtc_status_e status;
    float equivalent_population_scale;
    float remaining_volume_m3;
    float burning_surface_area_m2;
    float remaining_mass_kg;
    float reacted_mass_kg;
    float reacted_mass_rate_kg_per_s;

    if (result == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    *result = (bbtc_ib_propellant_mass_result_float_t){0};

    status = bbtc_ib_propellant_charge_validate_float(charge);
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    if (isnan(initial_grain_volume_m3))
        return BBTC_STATUS_NAN_INPUT;

    if (isinf(initial_grain_volume_m3))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (initial_grain_volume_m3 <= 0.0f)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    status = validate_grain_state_float(grain_state, initial_grain_volume_m3);
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    if (kinetics == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (isnan(kinetics->burn_rate_m_per_s))
        return BBTC_STATUS_NAN_INPUT;

    if (isinf(kinetics->burn_rate_m_per_s))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (kinetics->burn_rate_m_per_s < 0.0f)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    equivalent_population_scale = positive_div_product_float(
        charge->charge_mass_kg,
        charge->condensed_phase_density_kg_per_m3,
        initial_grain_volume_m3
    );

    if (!isfinite(equivalent_population_scale)
        || equivalent_population_scale <= 0.0f)
    {
        return BBTC_STATUS_NUMERICAL_FAILURE;
    }

    if (grain_state->consumed_volume_fraction == 1.0f)
    {
        result->applicability_flags = kinetics->applicability_flags;
        result->equivalent_population_scale = equivalent_population_scale;
        result->reacted_mass_kg = charge->charge_mass_kg;
        return BBTC_STATUS_SUCCESS;
    }

    burning_surface_area_m2 =
        equivalent_population_scale * grain_state->burning_surface_area_m2;

    if (!isfinite(burning_surface_area_m2) || burning_surface_area_m2 <= 0.0f)
        return BBTC_STATUS_NUMERICAL_FAILURE;

    if (grain_state->consumed_volume_fraction == 0.0f)
    {
        remaining_volume_m3 =
            charge->charge_mass_kg / charge->condensed_phase_density_kg_per_m3;
        remaining_mass_kg = charge->charge_mass_kg;
        reacted_mass_kg = 0.0f;
    }
    else
    {
        remaining_volume_m3 =
            equivalent_population_scale * grain_state->remaining_volume_m3;

        remaining_mass_kg = positive_product_ratio_float(
            charge->charge_mass_kg,
            grain_state->remaining_volume_m3,
            initial_grain_volume_m3
        );

        reacted_mass_kg =
            charge->charge_mass_kg * grain_state->consumed_volume_fraction;
    }

    if (!isfinite(remaining_volume_m3) || remaining_volume_m3 <= 0.0f
        || !isfinite(remaining_mass_kg) || remaining_mass_kg <= 0.0f)
    {
        return BBTC_STATUS_NUMERICAL_FAILURE;
    }

    if (grain_state->consumed_volume_fraction != 0.0f
        && (!isfinite(reacted_mass_kg) || reacted_mass_kg <= 0.0f))
    {
        return BBTC_STATUS_NUMERICAL_FAILURE;
    }

    if (kinetics->burn_rate_m_per_s == 0.0f)
    {
        reacted_mass_rate_kg_per_s = 0.0f;
    }
    else
    {
        reacted_mass_rate_kg_per_s = positive_triple_product_ratio_float(
            charge->charge_mass_kg,
            grain_state->burning_surface_area_m2,
            kinetics->burn_rate_m_per_s,
            initial_grain_volume_m3
        );

        if (!isfinite(reacted_mass_rate_kg_per_s)
            || reacted_mass_rate_kg_per_s <= 0.0f)
        {
            return BBTC_STATUS_NUMERICAL_FAILURE;
        }
    }

    result->applicability_flags = kinetics->applicability_flags;
    result->equivalent_population_scale = equivalent_population_scale;
    result->remaining_volume_m3 = remaining_volume_m3;
    result->burning_surface_area_m2 = burning_surface_area_m2;
    result->remaining_mass_kg = remaining_mass_kg;
    result->reacted_mass_kg = reacted_mass_kg;
    result->reacted_mass_rate_kg_per_s = reacted_mass_rate_kg_per_s;

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e bbtc_ib_propellant_mass_evaluate_double(
    const bbtc_ib_propellant_charge_double_t* const charge,
    const double initial_grain_volume_m3,
    const bbtc_ib_propellant_grain_state_double_t* const grain_state,
    const bbtc_ib_propellant_burn_kinetics_result_double_t* const kinetics,
    bbtc_ib_propellant_mass_result_double_t* const result)
{
    bbtc_status_e status;
    double equivalent_population_scale;
    double remaining_volume_m3;
    double burning_surface_area_m2;
    double remaining_mass_kg;
    double reacted_mass_kg;
    double reacted_mass_rate_kg_per_s;

    if (result == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    *result = (bbtc_ib_propellant_mass_result_double_t){0};

    status = bbtc_ib_propellant_charge_validate_double(charge);
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    if (isnan(initial_grain_volume_m3))
        return BBTC_STATUS_NAN_INPUT;

    if (isinf(initial_grain_volume_m3))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (initial_grain_volume_m3 <= 0.0)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    status = validate_grain_state_double(grain_state, initial_grain_volume_m3);
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    if (kinetics == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (isnan(kinetics->burn_rate_m_per_s))
        return BBTC_STATUS_NAN_INPUT;

    if (isinf(kinetics->burn_rate_m_per_s))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (kinetics->burn_rate_m_per_s < 0.0)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    equivalent_population_scale = positive_div_product_double(
        charge->charge_mass_kg,
        charge->condensed_phase_density_kg_per_m3,
        initial_grain_volume_m3
    );

    if (!isfinite(equivalent_population_scale)
        || equivalent_population_scale <= 0.0)
    {
        return BBTC_STATUS_NUMERICAL_FAILURE;
    }

    if (grain_state->consumed_volume_fraction == 1.0)
    {
        result->applicability_flags = kinetics->applicability_flags;
        result->equivalent_population_scale = equivalent_population_scale;
        result->reacted_mass_kg = charge->charge_mass_kg;
        return BBTC_STATUS_SUCCESS;
    }

    burning_surface_area_m2 =
        equivalent_population_scale * grain_state->burning_surface_area_m2;

    if (!isfinite(burning_surface_area_m2) || burning_surface_area_m2 <= 0.0)
        return BBTC_STATUS_NUMERICAL_FAILURE;

    if (grain_state->consumed_volume_fraction == 0.0)
    {
        remaining_volume_m3 =
            charge->charge_mass_kg / charge->condensed_phase_density_kg_per_m3;
        remaining_mass_kg = charge->charge_mass_kg;
        reacted_mass_kg = 0.0;
    }
    else
    {
        remaining_volume_m3 =
            equivalent_population_scale * grain_state->remaining_volume_m3;

        remaining_mass_kg = positive_product_ratio_double(
            charge->charge_mass_kg,
            grain_state->remaining_volume_m3,
            initial_grain_volume_m3
        );

        reacted_mass_kg =
            charge->charge_mass_kg * grain_state->consumed_volume_fraction;
    }

    if (!isfinite(remaining_volume_m3) || remaining_volume_m3 <= 0.0
        || !isfinite(remaining_mass_kg) || remaining_mass_kg <= 0.0)
    {
        return BBTC_STATUS_NUMERICAL_FAILURE;
    }

    if (grain_state->consumed_volume_fraction != 0.0
        && (!isfinite(reacted_mass_kg) || reacted_mass_kg <= 0.0))
    {
        return BBTC_STATUS_NUMERICAL_FAILURE;
    }

    if (kinetics->burn_rate_m_per_s == 0.0)
    {
        reacted_mass_rate_kg_per_s = 0.0;
    }
    else
    {
        reacted_mass_rate_kg_per_s = positive_triple_product_ratio_double(
            charge->charge_mass_kg,
            grain_state->burning_surface_area_m2,
            kinetics->burn_rate_m_per_s,
            initial_grain_volume_m3
        );

        if (!isfinite(reacted_mass_rate_kg_per_s)
            || reacted_mass_rate_kg_per_s <= 0.0)
        {
            return BBTC_STATUS_NUMERICAL_FAILURE;
        }
    }

    result->applicability_flags = kinetics->applicability_flags;
    result->equivalent_population_scale = equivalent_population_scale;
    result->remaining_volume_m3 = remaining_volume_m3;
    result->burning_surface_area_m2 = burning_surface_area_m2;
    result->remaining_mass_kg = remaining_mass_kg;
    result->reacted_mass_kg = reacted_mass_kg;
    result->reacted_mass_rate_kg_per_s = reacted_mass_rate_kg_per_s;

    return BBTC_STATUS_SUCCESS;
}


bbtc_status_e bbtc_ib_propellant_mass_evaluate_long_double(
    const bbtc_ib_propellant_charge_long_double_t* const charge,
    const long double initial_grain_volume_m3,
    const bbtc_ib_propellant_grain_state_long_double_t* const grain_state,
    const bbtc_ib_propellant_burn_kinetics_result_long_double_t* const kinetics,
    bbtc_ib_propellant_mass_result_long_double_t* const result)
{
    bbtc_status_e status;
    long double equivalent_population_scale;
    long double remaining_volume_m3;
    long double burning_surface_area_m2;
    long double remaining_mass_kg;
    long double reacted_mass_kg;
    long double reacted_mass_rate_kg_per_s;

    if (result == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    *result = (bbtc_ib_propellant_mass_result_long_double_t){0};

    status = bbtc_ib_propellant_charge_validate_long_double(charge);
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    if (isnan(initial_grain_volume_m3))
        return BBTC_STATUS_NAN_INPUT;

    if (isinf(initial_grain_volume_m3))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (initial_grain_volume_m3 <= 0.0L)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    status = validate_grain_state_long_double(
        grain_state,
        initial_grain_volume_m3
    );
    if (status != BBTC_STATUS_SUCCESS)
        return status;

    if (kinetics == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    if (isnan(kinetics->burn_rate_m_per_s))
        return BBTC_STATUS_NAN_INPUT;

    if (isinf(kinetics->burn_rate_m_per_s))
        return BBTC_STATUS_NONFINITE_INPUT;

    if (kinetics->burn_rate_m_per_s < 0.0L)
        return BBTC_STATUS_OUTSIDE_DOMAIN;

    equivalent_population_scale = positive_div_product_long_double(
        charge->charge_mass_kg,
        charge->condensed_phase_density_kg_per_m3,
        initial_grain_volume_m3
    );

    if (!isfinite(equivalent_population_scale)
        || equivalent_population_scale <= 0.0L)
    {
        return BBTC_STATUS_NUMERICAL_FAILURE;
    }

    if (grain_state->consumed_volume_fraction == 1.0L)
    {
        result->applicability_flags = kinetics->applicability_flags;
        result->equivalent_population_scale = equivalent_population_scale;
        result->reacted_mass_kg = charge->charge_mass_kg;
        return BBTC_STATUS_SUCCESS;
    }

    burning_surface_area_m2 =
        equivalent_population_scale * grain_state->burning_surface_area_m2;

    if (!isfinite(burning_surface_area_m2)
        || burning_surface_area_m2 <= 0.0L)
    {
        return BBTC_STATUS_NUMERICAL_FAILURE;
    }

    if (grain_state->consumed_volume_fraction == 0.0L)
    {
        remaining_volume_m3 =
            charge->charge_mass_kg / charge->condensed_phase_density_kg_per_m3;
        remaining_mass_kg = charge->charge_mass_kg;
        reacted_mass_kg = 0.0L;
    }
    else
    {
        remaining_volume_m3 =
            equivalent_population_scale * grain_state->remaining_volume_m3;

        remaining_mass_kg = positive_product_ratio_long_double(
            charge->charge_mass_kg,
            grain_state->remaining_volume_m3,
            initial_grain_volume_m3
        );

        reacted_mass_kg =
            charge->charge_mass_kg * grain_state->consumed_volume_fraction;
    }

    if (!isfinite(remaining_volume_m3) || remaining_volume_m3 <= 0.0L
        || !isfinite(remaining_mass_kg) || remaining_mass_kg <= 0.0L)
    {
        return BBTC_STATUS_NUMERICAL_FAILURE;
    }

    if (grain_state->consumed_volume_fraction != 0.0L
        && (!isfinite(reacted_mass_kg) || reacted_mass_kg <= 0.0L))
    {
        return BBTC_STATUS_NUMERICAL_FAILURE;
    }

    if (kinetics->burn_rate_m_per_s == 0.0L)
    {
        reacted_mass_rate_kg_per_s = 0.0L;
    }
    else
    {
        reacted_mass_rate_kg_per_s = positive_triple_product_ratio_long_double(
            charge->charge_mass_kg,
            grain_state->burning_surface_area_m2,
            kinetics->burn_rate_m_per_s,
            initial_grain_volume_m3
        );

        if (!isfinite(reacted_mass_rate_kg_per_s)
            || reacted_mass_rate_kg_per_s <= 0.0L)
        {
            return BBTC_STATUS_NUMERICAL_FAILURE;
        }
    }

    result->applicability_flags = kinetics->applicability_flags;
    result->equivalent_population_scale = equivalent_population_scale;
    result->remaining_volume_m3 = remaining_volume_m3;
    result->burning_surface_area_m2 = burning_surface_area_m2;
    result->remaining_mass_kg = remaining_mass_kg;
    result->reacted_mass_kg = reacted_mass_kg;
    result->reacted_mass_rate_kg_per_s = reacted_mass_rate_kg_per_s;

    return BBTC_STATUS_SUCCESS;
}
