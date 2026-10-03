/**
 * @file propellant_mass.h
 * @brief Whole-charge propellant mass coupling from grain regression state.
 *
 * @details
 * This module maps one canonical grain's already-evaluated regression state and
 * one already-evaluated linear surface-regression rate into whole-charge
 * condensed-propellant volume, burning area, remaining mass, reacted mass, and
 * reacted-mass rate. The complete charge is represented as a real-valued
 * equivalent population of identical canonical grains.
 *
 * The coupling layer deliberately does not evaluate a grain geometry or a burn
 * law itself. It consumes the common outputs of those subsystems and therefore
 * remains independent of the selected geometry and kinetics backends. It also
 * does not model ignition, thermochemical source generation, gas-state
 * evolution, projectile motion, time integration, or firearm safety.
 */
#ifndef BBTC_INTERNAL_BALLISTICS_PROPELLANT_MASS_H
#define BBTC_INTERNAL_BALLISTICS_PROPELLANT_MASS_H

#include <bbtc/diagnostics.h>
#include <bbtc/status.h>
#include <bbtc/internal_ballistics/propellant_burn_kinetics.h>
#include <bbtc/internal_ballistics/propellant_charge.h>
#include <bbtc/internal_ballistics/propellant_grain_geometry.h>

#ifdef __cplusplus
    extern "C" {
#endif

/**
 * @struct bbtc_ib_propellant_mass_result_float_t
 * @brief Whole-charge propellant mass-coupling result using native `float`.
 *
 * @details
 * The result describes the equivalent whole-charge state corresponding to one
 * canonical grain state. `equivalent_population_scale` is a real-valued scaling
 * factor; it is not a literal integer grain count and is never rounded to one.
 *
 * `remaining_volume_m3` and `burning_surface_area_m2` are whole-charge
 * quantities, not the one-grain values supplied to the evaluator. The mass-rate
 * field is the instantaneous reacted-propellant source rate implied by the
 * supplied grain surface and linear regression rate.
 *
 * `applicability_flags` preserves the complete incoming burn-kinetics
 * applicability mask on success. This layer introduces no additional
 * applicability bit in IB0.4e-B1.
 */
typedef struct bbtc_ib_propellant_mass_result_float_t
{
    /** Nonfatal scientific applicability metadata propagated from kinetics. */
    bbtc_applicability_flags_t applicability_flags;

    /** Real-valued equivalent canonical-grain population scale, dimensionless. */
    float equivalent_population_scale;

    /** Whole-charge remaining condensed-propellant volume, in cubic meters. */
    float remaining_volume_m3;

    /** Whole-charge geometrically exposed burning surface area, in square meters. */
    float burning_surface_area_m2;

    /** Whole-charge remaining condensed-propellant mass, in kilograms. */
    float remaining_mass_kg;

    /** Whole-charge reacted propellant mass, in kilograms. */
    float reacted_mass_kg;

    /** Instantaneous whole-charge reacted-propellant mass rate, in kg/s. */
    float reacted_mass_rate_kg_per_s;
}
bbtc_ib_propellant_mass_result_float_t;


/**
 * @struct bbtc_ib_propellant_mass_result_double_t
 * @brief Whole-charge propellant mass-coupling result using native `double`.
 *
 * @details
 * Physical meaning, units, endpoint semantics, and applicability propagation
 * match `bbtc_ib_propellant_mass_result_float_t`. Calculations in the matching
 * evaluator remain in native `double`.
 */
typedef struct bbtc_ib_propellant_mass_result_double_t
{
    /** Nonfatal scientific applicability metadata propagated from kinetics. */
    bbtc_applicability_flags_t applicability_flags;

    /** Real-valued equivalent canonical-grain population scale, dimensionless. */
    double equivalent_population_scale;

    /** Whole-charge remaining condensed-propellant volume, in cubic meters. */
    double remaining_volume_m3;

    /** Whole-charge geometrically exposed burning surface area, in square meters. */
    double burning_surface_area_m2;

    /** Whole-charge remaining condensed-propellant mass, in kilograms. */
    double remaining_mass_kg;

    /** Whole-charge reacted propellant mass, in kilograms. */
    double reacted_mass_kg;

    /** Instantaneous whole-charge reacted-propellant mass rate, in kg/s. */
    double reacted_mass_rate_kg_per_s;
}
bbtc_ib_propellant_mass_result_double_t;


/**
 * @struct bbtc_ib_propellant_mass_result_long_double_t
 * @brief Whole-charge propellant mass-coupling result using native `long double`.
 *
 * @details
 * Physical meaning and validation semantics match the other scalar families.
 * The corresponding evaluator does not route arithmetic through `double` and
 * does not assume that `long double` is wider on every supported platform.
 */
typedef struct bbtc_ib_propellant_mass_result_long_double_t
{
    /** Nonfatal scientific applicability metadata propagated from kinetics. */
    bbtc_applicability_flags_t applicability_flags;

    /** Real-valued equivalent canonical-grain population scale, dimensionless. */
    long double equivalent_population_scale;

    /** Whole-charge remaining condensed-propellant volume, in cubic meters. */
    long double remaining_volume_m3;

    /** Whole-charge geometrically exposed burning surface area, in square meters. */
    long double burning_surface_area_m2;

    /** Whole-charge remaining condensed-propellant mass, in kilograms. */
    long double remaining_mass_kg;

    /** Whole-charge reacted propellant mass, in kilograms. */
    long double reacted_mass_kg;

    /** Instantaneous whole-charge reacted-propellant mass rate, in kg/s. */
    long double reacted_mass_rate_kg_per_s;
}
bbtc_ib_propellant_mass_result_long_double_t;


/**
 * @brief Evaluates whole-charge propellant mass coupling using native `float`.
 *
 * @details
 * The evaluator consumes:
 *
 * - one valid propellant charge with initial mass `m0` and condensed material
 *   density `rho_p`;
 * - one explicit initial canonical-grain volume `V_g0`;
 * - one current canonical-grain state containing `V_g`, `A_g`, and consumed
 *   volume fraction `f`; and
 * - one already-evaluated burn-kinetics result containing `r = ds/dt`.
 *
 * The equivalent population scale is mathematically
 *
 * `N_eq = m0 / (rho_p * V_g0)`.
 *
 * Whole-charge quantities then satisfy
 *
 * `V_remaining = N_eq * V_g`,
 *
 * `A_total = N_eq * A_g`,
 *
 * `m_remaining = m0 * V_g / V_g0`,
 *
 * `m_reacted = m0 * f`, and
 *
 * `dm_reacted/dt = m0 * A_g * r / V_g0`.
 *
 * These equations define the physical relation rather than a mandatory
 * floating-point operation sequence. The implementation uses algebraically
 * equivalent native-precision forms where useful to avoid unnecessary
 * intermediate range loss.
 *
 * Validation order is deliberate. A null `result` returns
 * `BBTC_STATUS_INVALID_ARGUMENT`. Once `result` is known to be nonnull, the
 * complete record is cleared before any later validation. Charge validation
 * occurs first, followed by `initial_grain_volume_m3`, the complete grain-state
 * scalar layer, cross-record grain-state consistency, and finally the kinetics
 * burn-rate scalar. Within the grain state, any NaN takes precedence over any
 * infinity before finite-domain and cross-record checks.
 *
 * `initial_grain_volume_m3` must be finite and strictly positive. Grain-state
 * volumes, area, and remaining regression must be finite and nonnegative, while
 * consumed fraction must be in `[0, 1]`. Exact initial state requires fraction
 * zero, remaining volume exactly `V_g0`, and positive area and remaining
 * regression. Exact burnout requires fraction one and exact zero remaining
 * volume, area, and remaining regression. Interior fraction requires positive
 * remaining volume below `V_g0`, positive area, and positive remaining
 * regression. Violations of those relationships return
 * `BBTC_STATUS_INCONSISTENT_CONFIGURATION`.
 *
 * Burn rate may be exactly zero, producing exact zero reacted-mass rate. A
 * negative finite rate is outside the domain. At exact burnout, burning area
 * and reacted-mass rate remain exact zero even if the supplied kinetics rate is
 * positive. On every successful evaluation the complete incoming applicability
 * mask is copied verbatim, including bits unknown to this library version.
 *
 * Every mathematically required positive output must remain finite and strictly
 * positive in native `float`. Overflow or underflow to zero of such an output
 * returns `BBTC_STATUS_NUMERICAL_FAILURE`; the evaluator never clamps or
 * fabricates a replacement value.
 *
 * @param charge Validated physical charge data. The evaluator performs the
 *        existing charge validation itself and does not modify this record.
 * @param initial_grain_volume_m3 Initial volume of one canonical grain, in m^3.
 * @param grain_state Current one-grain regression state.
 * @param kinetics Already-evaluated linear surface-regression result.
 * @param result Caller-owned whole-charge mass result.
 *
 * @return `BBTC_STATUS_SUCCESS` on successful coupling; otherwise the
 *         documented argument, nonfinite, finite-domain, inconsistency, or
 *         numerical-failure status.
 */
bbtc_status_e
bbtc_ib_propellant_mass_evaluate_float(
    const bbtc_ib_propellant_charge_float_t* charge,
    float initial_grain_volume_m3,
    const bbtc_ib_propellant_grain_state_float_t* grain_state,
    const bbtc_ib_propellant_burn_kinetics_result_float_t* kinetics,
    bbtc_ib_propellant_mass_result_float_t* result
);


/**
 * @brief Evaluates whole-charge propellant mass coupling using native `double`.
 *
 * @details
 * Equations, validation precedence, endpoint semantics, applicability
 * propagation, output clearing, and numerical-failure behavior match
 * `bbtc_ib_propellant_mass_evaluate_float()` exactly while retaining native
 * `double` arithmetic.
 *
 * @param charge Validated physical charge data.
 * @param initial_grain_volume_m3 Initial volume of one canonical grain, in m^3.
 * @param grain_state Current one-grain regression state.
 * @param kinetics Already-evaluated linear surface-regression result.
 * @param result Caller-owned whole-charge mass result.
 *
 * @return The same status contract as the native-`float` evaluator.
 */
bbtc_status_e
bbtc_ib_propellant_mass_evaluate_double(
    const bbtc_ib_propellant_charge_double_t* charge,
    double initial_grain_volume_m3,
    const bbtc_ib_propellant_grain_state_double_t* grain_state,
    const bbtc_ib_propellant_burn_kinetics_result_double_t* kinetics,
    bbtc_ib_propellant_mass_result_double_t* result
);


/**
 * @brief Evaluates whole-charge propellant mass coupling using native
 *        `long double`.
 *
 * @details
 * Equations, validation precedence, endpoint semantics, applicability
 * propagation, output clearing, and numerical-failure behavior match
 * `bbtc_ib_propellant_mass_evaluate_float()` exactly while retaining native
 * `long double` arithmetic.
 *
 * @param charge Validated physical charge data.
 * @param initial_grain_volume_m3 Initial volume of one canonical grain, in m^3.
 * @param grain_state Current one-grain regression state.
 * @param kinetics Already-evaluated linear surface-regression result.
 * @param result Caller-owned whole-charge mass result.
 *
 * @return The same status contract as the other native evaluators.
 */
bbtc_status_e
bbtc_ib_propellant_mass_evaluate_long_double(
    const bbtc_ib_propellant_charge_long_double_t* charge,
    long double initial_grain_volume_m3,
    const bbtc_ib_propellant_grain_state_long_double_t* grain_state,
    const bbtc_ib_propellant_burn_kinetics_result_long_double_t* kinetics,
    bbtc_ib_propellant_mass_result_long_double_t* result
);

#ifdef __cplusplus
}   /* extern "C" */
#endif
#endif /* BBTC_INTERNAL_BALLISTICS_PROPELLANT_MASS_H */
