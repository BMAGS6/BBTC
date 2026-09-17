/**
 * @file propellant_burn_kinetics.h
 * @brief Empirical propellant surface-regression kinetics.
 *
 * @details
 * This module defines empirical models that map an explicit physical state to
 * the linear normal regression rate of an already-burning propellant surface.
 * Burn rate is `ds/dt`, where `s` is the same normal-regression coordinate
 * consumed by BBTC's propellant-grain geometry evaluators.
 *
 * The initial concrete backend is a normalized pressure-power relation in the
 * Saint-Robert/Vieille family. A later IB0.4d increment will add the separately
 * specified tabulated pressure backend while reusing the common result records
 * declared here.
 *
 * Burn kinetics are deliberately orthogonal to ignition, grain geometry,
 * whole-charge grain count, reacted-propellant mass rate, thermochemical source
 * terms, gas-state evolution, projectile motion, and ammunition/firearm safety.
 * A pressure-only kinetics backend also does not consume initial propellant
 * temperature or imply that temperature sensitivity has been modeled.
 */
#ifndef BBTC_INTERNAL_BALLISTICS_PROPELLANT_BURN_KINETICS_H
#define BBTC_INTERNAL_BALLISTICS_PROPELLANT_BURN_KINETICS_H

#include <bbtc/diagnostics.h>
#include <bbtc/status.h>

#ifdef __cplusplus
    extern "C" {
#endif

/**
 * @struct bbtc_ib_pressure_power_burn_kinetics_float_t
 * @brief Normalized pressure-power burn kinetics using native `float`.
 *
 * @details
 * The represented empirical relation is
 *
 * `r(P) = r_ref * (P / P_ref)^n`
 *
 * where:
 *
 * - `r(P)` is linear normal surface-regression rate, in meters per second;
 * - `r_ref` is `reference_burn_rate_m_per_s`;
 * - `P` is caller-supplied absolute pressure, in pascals;
 * - `P_ref` is `reference_pressure_pa`; and
 * - `n` is the positive dimensionless `pressure_exponent`.
 *
 * Normalizing pressure by `P_ref` keeps the exponentiation dimensionless and
 * gives `reference_burn_rate_m_per_s` the invariant physical meaning of burn
 * rate at the explicit reference pressure. The coefficient therefore does not
 * acquire unit-dependent dimensions when `pressure_exponent` changes.
 *
 * `reference_pressure_pa` must lie inside the inclusive calibrated-pressure
 * interval. Pressures outside that interval can remain mathematically
 * evaluable, but a successful evaluation reports
 * `BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN` rather than silently clipping
 * the pressure or pretending that extrapolation is calibrated.
 *
 * This primitive record does not identify a propellant product, lot, test
 * fixture, conditioning temperature, ignition model, grain geometry,
 * thermochemistry model, or projectile model. Passing validation establishes a
 * well-formed mathematical backend, not physical accuracy or firing safety.
 */
typedef struct bbtc_ib_pressure_power_burn_kinetics_float_t
{
    /** Linear regression rate at `reference_pressure_pa`, in meters/second. */
    float reference_burn_rate_m_per_s;

    /** Absolute pressure defining the reference burn rate, in pascals. */
    float reference_pressure_pa;

    /** Positive dimensionless pressure exponent `n`. */
    float pressure_exponent;

    /** Inclusive lower calibrated absolute pressure, in pascals. */
    float minimum_calibrated_pressure_pa;

    /** Inclusive upper calibrated absolute pressure, in pascals. */
    float maximum_calibrated_pressure_pa;
}
bbtc_ib_pressure_power_burn_kinetics_float_t;


/**
 * @struct bbtc_ib_pressure_power_burn_kinetics_double_t
 * @brief Normalized pressure-power burn kinetics using native `double`.
 *
 * @details
 * Physical meaning, SI units, calibration semantics, ownership, and validation
 * rules match `bbtc_ib_pressure_power_burn_kinetics_float_t`. Calculations in
 * the corresponding evaluator remain in native `double`.
 */
typedef struct bbtc_ib_pressure_power_burn_kinetics_double_t
{
    /** Linear regression rate at `reference_pressure_pa`, in meters/second. */
    double reference_burn_rate_m_per_s;

    /** Absolute pressure defining the reference burn rate, in pascals. */
    double reference_pressure_pa;

    /** Positive dimensionless pressure exponent `n`. */
    double pressure_exponent;

    /** Inclusive lower calibrated absolute pressure, in pascals. */
    double minimum_calibrated_pressure_pa;

    /** Inclusive upper calibrated absolute pressure, in pascals. */
    double maximum_calibrated_pressure_pa;
}
bbtc_ib_pressure_power_burn_kinetics_double_t;


/**
 * @struct bbtc_ib_pressure_power_burn_kinetics_long_double_t
 * @brief Normalized pressure-power burn kinetics using native `long double`.
 *
 * @details
 * Physical meaning, SI units, calibration semantics, ownership, and validation
 * rules match the other scalar families. This type does not assume that
 * `long double` is wider than `double` on every platform, and the corresponding
 * evaluator does not route its arithmetic through `double`.
 */
typedef struct bbtc_ib_pressure_power_burn_kinetics_long_double_t
{
    /** Linear regression rate at `reference_pressure_pa`, in meters/second. */
    long double reference_burn_rate_m_per_s;

    /** Absolute pressure defining the reference burn rate, in pascals. */
    long double reference_pressure_pa;

    /** Positive dimensionless pressure exponent `n`. */
    long double pressure_exponent;

    /** Inclusive lower calibrated absolute pressure, in pascals. */
    long double minimum_calibrated_pressure_pa;

    /** Inclusive upper calibrated absolute pressure, in pascals. */
    long double maximum_calibrated_pressure_pa;
}
bbtc_ib_pressure_power_burn_kinetics_long_double_t;


/**
 * @struct bbtc_ib_propellant_burn_kinetics_result_float_t
 * @brief Common empirical burn-kinetics result using native `float`.
 *
 * @details
 * `burn_rate_m_per_s` is the evaluated linear normal surface-regression rate of
 * an already-burning propellant surface. It is not reacted-propellant mass
 * rate, burn fraction, burning area, ignition progress, gas-generation rate, or
 * projectile velocity.
 *
 * `applicability_flags` reports nonfatal limitations on scientific
 * interpretation independently from `bbtc_status_e`. For the pressure-power
 * backend, successful evaluation outside the documented calibrated-pressure
 * interval sets `BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN`.
 *
 * A successful pressure-power evaluation at exactly zero absolute pressure is
 * the exact mathematical zero-rate boundary and therefore returns a zero burn
 * rate together with outside-calibration applicability. That boundary is not
 * an ignition model and does not assert that real propellant burns in vacuum.
 */
typedef struct bbtc_ib_propellant_burn_kinetics_result_float_t
{
    /** Nonfatal scientific model-applicability metadata. */
    bbtc_applicability_flags_t applicability_flags;

    /** Linear normal surface-regression rate, in meters per second. */
    float burn_rate_m_per_s;
}
bbtc_ib_propellant_burn_kinetics_result_float_t;


/**
 * @struct bbtc_ib_propellant_burn_kinetics_result_double_t
 * @brief Common empirical burn-kinetics result using native `double`.
 *
 * @details
 * Physical and diagnostic semantics match
 * `bbtc_ib_propellant_burn_kinetics_result_float_t`.
 */
typedef struct bbtc_ib_propellant_burn_kinetics_result_double_t
{
    /** Nonfatal scientific model-applicability metadata. */
    bbtc_applicability_flags_t applicability_flags;

    /** Linear normal surface-regression rate, in meters per second. */
    double burn_rate_m_per_s;
}
bbtc_ib_propellant_burn_kinetics_result_double_t;


/**
 * @struct bbtc_ib_propellant_burn_kinetics_result_long_double_t
 * @brief Common empirical burn-kinetics result using native `long double`.
 *
 * @details
 * Physical and diagnostic semantics match the other scalar families.
 */
typedef struct bbtc_ib_propellant_burn_kinetics_result_long_double_t
{
    /** Nonfatal scientific model-applicability metadata. */
    bbtc_applicability_flags_t applicability_flags;

    /** Linear normal surface-regression rate, in meters per second. */
    long double burn_rate_m_per_s;
}
bbtc_ib_propellant_burn_kinetics_result_long_double_t;


/**
 * @brief Validates one native-`float` pressure-power burn-kinetics record.
 *
 * @details
 * Validation uses the following precedence within this model layer:
 *
 * 1. a null model pointer returns `BBTC_STATUS_INVALID_ARGUMENT`;
 * 2. NaN in any of the five scalar fields returns `BBTC_STATUS_NAN_INPUT`;
 * 3. otherwise, either infinity in any field returns
 *    `BBTC_STATUS_NONFINITE_INPUT`;
 * 4. finite violations of the mathematical model domain return
 *    `BBTC_STATUS_OUTSIDE_DOMAIN`; and
 * 5. an otherwise well-formed record returns `BBTC_STATUS_SUCCESS`.
 *
 * The reference burn rate, reference pressure, pressure exponent, and both
 * calibration pressures must be strictly positive. Maximum calibrated pressure
 * must be strictly greater than minimum calibrated pressure, and the reference
 * pressure must lie inside that inclusive interval. Zero initialization is
 * deliberately invalid.
 *
 * The validator imposes no arbitrary finite upper bound on the parameters. It
 * also does not establish calibration quality, data provenance, propellant
 * identity, temperature applicability, ammunition compatibility, firearm
 * strength, or firing safety. The caller-owned record is not modified.
 *
 * @param model Caller-owned pressure-power burn-kinetics record.
 *
 * @return `BBTC_STATUS_SUCCESS` when the record satisfies the documented
 *         mathematical contract; otherwise the validation status above.
 */
bbtc_status_e
bbtc_ib_pressure_power_burn_kinetics_validate_float(
    const bbtc_ib_pressure_power_burn_kinetics_float_t* model
);


/**
 * @brief Validates one native-`double` pressure-power burn-kinetics record.
 *
 * @details
 * Validation precedence, finite-domain rules, ownership, and scientific
 * limitations match
 * `bbtc_ib_pressure_power_burn_kinetics_validate_float()` exactly.
 *
 * @param model Caller-owned pressure-power burn-kinetics record.
 *
 * @return The same validation-status contract as the native-`float` validator.
 */
bbtc_status_e
bbtc_ib_pressure_power_burn_kinetics_validate_double(
    const bbtc_ib_pressure_power_burn_kinetics_double_t* model
);


/**
 * @brief Validates one native-`long double` pressure-power kinetics record.
 *
 * @details
 * Validation precedence, finite-domain rules, ownership, and scientific
 * limitations match
 * `bbtc_ib_pressure_power_burn_kinetics_validate_float()` exactly.
 *
 * @param model Caller-owned pressure-power burn-kinetics record.
 *
 * @return The same validation-status contract as the native-`float` validator.
 */
bbtc_status_e
bbtc_ib_pressure_power_burn_kinetics_validate_long_double(
    const bbtc_ib_pressure_power_burn_kinetics_long_double_t* model
);


/**
 * @brief Evaluates native-`float` normalized pressure-power burn kinetics.
 *
 * @details
 * The evaluator consumes caller-supplied absolute pressure in pascals and uses
 *
 * `r(P) = r_ref * (P / P_ref)^n`.
 *
 * Direct-pressure semantics are:
 *
 * - NaN -> `BBTC_STATUS_NAN_INPUT`;
 * - positive or negative infinity -> `BBTC_STATUS_NONFINITE_INPUT`;
 * - finite `P < 0` -> `BBTC_STATUS_OUTSIDE_DOMAIN`;
 * - `P == 0` -> successful exact zero-rate mathematical boundary; and
 * - `P > 0` -> positive pressure-power evaluation.
 *
 * A null result pointer returns `BBTC_STATUS_INVALID_ARGUMENT`. Once a nonnull
 * result pointer is accepted, the complete result is cleared before model or
 * direct-pressure validation, so every later failure leaves a deterministic
 * zero record. Model validation precedes direct-pressure classification.
 *
 * At `P == P_ref`, the evaluator returns the stored reference burn rate exactly
 * rather than reconstructing it through transcendental arithmetic. Positive
 * pressure below `minimum_calibrated_pressure_pa` or above
 * `maximum_calibrated_pressure_pa` remains mathematically evaluable and sets
 * `BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN`; the inclusive calibration
 * endpoints do not set the flag.
 *
 * For other positive pressures the implementation may use an algebraically
 * equivalent log-domain form to avoid unnecessary range loss in the explicit
 * pressure ratio. Accepted finite inputs that cannot produce the required
 * finite strictly positive native result return
 * `BBTC_STATUS_NUMERICAL_FAILURE`. Such failure is never converted into a
 * physical zero-rate result or silently clamped into range.
 *
 * This pressure-only evaluator does not consume initial propellant temperature
 * or model ignition, burning-surface geometry, reacted-mass rate,
 * thermochemical sources, gas evolution, projectile motion, or firing safety.
 *
 * @param model Pressure-power burn-kinetics model.
 * @param absolute_pressure_pa Absolute pressure driving this primitive backend,
 *        in pascals. The caller defines which modeled pressure quantity is
 *        supplied when composing this primitive into a later solver.
 * @param result Caller-owned common burn-kinetics result.
 *
 * @return `BBTC_STATUS_SUCCESS` on successful evaluation; otherwise a
 *         documented argument, NaN-input, nonfinite-input, finite-domain, or
 *         numerical-failure status.
 */
bbtc_status_e
bbtc_ib_pressure_power_burn_kinetics_evaluate_float(
    const bbtc_ib_pressure_power_burn_kinetics_float_t* model,
    float absolute_pressure_pa,
    bbtc_ib_propellant_burn_kinetics_result_float_t* result
);


/**
 * @brief Evaluates native-`double` normalized pressure-power burn kinetics.
 *
 * @details
 * Equation, exact boundaries, validation order, output clearing, calibration
 * metadata, and numerical-failure semantics match the native-`float` evaluator.
 *
 * @param model Pressure-power burn-kinetics model.
 * @param absolute_pressure_pa Absolute pressure driving this primitive backend,
 *        in pascals.
 * @param result Caller-owned common burn-kinetics result.
 *
 * @return The same status contract as the native-`float` evaluator.
 */
bbtc_status_e
bbtc_ib_pressure_power_burn_kinetics_evaluate_double(
    const bbtc_ib_pressure_power_burn_kinetics_double_t* model,
    double absolute_pressure_pa,
    bbtc_ib_propellant_burn_kinetics_result_double_t* result
);


/**
 * @brief Evaluates native-`long double` normalized pressure-power kinetics.
 *
 * @details
 * Equation, exact boundaries, validation order, output clearing, calibration
 * metadata, and numerical-failure semantics match the native-`float` evaluator.
 * Arithmetic remains in native `long double`.
 *
 * @param model Pressure-power burn-kinetics model.
 * @param absolute_pressure_pa Absolute pressure driving this primitive backend,
 *        in pascals.
 * @param result Caller-owned common burn-kinetics result.
 *
 * @return The same status contract as the native-`float` evaluator.
 */
bbtc_status_e
bbtc_ib_pressure_power_burn_kinetics_evaluate_long_double(
    const bbtc_ib_pressure_power_burn_kinetics_long_double_t* model,
    long double absolute_pressure_pa,
    bbtc_ib_propellant_burn_kinetics_result_long_double_t* result
);


#ifdef __cplusplus
}   /* extern "C" */
#endif
#endif /* BBTC_INTERNAL_BALLISTICS_PROPELLANT_BURN_KINETICS_H */
