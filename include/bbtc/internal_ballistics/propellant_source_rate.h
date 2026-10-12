/**
 * @file propellant_source_rate.h
 * @brief Reduced thermochemical source rates driven by propellant mass consumption.
 *
 * @details
 * This module is the explicit IB0.4f coupling layer between the whole-charge
 * reacted-propellant mass rate produced by `propellant_mass.h` and the reduced
 * thermochemical coefficients defined by `propellant_thermochemistry.h`.
 *
 * For an already-evaluated reacted-propellant mass rate `mdot_r`, gaseous-product
 * mass fraction `y_g`, and positive specific reaction internal-energy release
 * `q_r`, the layer evaluates:
 *
 *     mdot_g = y_g * mdot_r
 *     mdot_c = (1 - y_g) * mdot_r
 *     Qdot_r = q_r * mdot_r
 *
 * The two mass-source rates are expressed in kilograms per second. The
 * reaction-energy release rate is expressed in watts (joules per second).
 *
 * This module deliberately does not evaluate burn kinetics, grain geometry,
 * reacted-propellant mass, gas mixing, an equation of state, chamber
 * temperature, chamber pressure, projectile motion, or time integration. It
 * converts one already-resolved upstream mass-consumption rate into the source
 * rates needed by a later chamber-state and energy-balance layer.
 *
 * The complete incoming applicability mask from the propellant-mass result is
 * preserved verbatim on success. The rate layer does not interpret unknown
 * applicability bits and introduces no new applicability flag.
 */
#ifndef BBTC_INTERNAL_BALLISTICS_PROPELLANT_SOURCE_RATE_H
#define BBTC_INTERNAL_BALLISTICS_PROPELLANT_SOURCE_RATE_H

#include <bbtc/diagnostics.h>
#include <bbtc/status.h>
#include <bbtc/internal_ballistics/propellant_mass.h>
#include <bbtc/internal_ballistics/propellant_thermochemistry.h>

#ifdef __cplusplus
    extern "C" {
#endif

/**
 * @struct bbtc_ib_propellant_source_rate_float_t
 * @brief Instantaneous reduced propellant source rates using native `float`.
 *
 * @details
 * The record contains only source rates and propagated scientific-applicability
 * metadata. It is not a chamber state and does not own or imply accumulated
 * product masses, gas temperature, pressure, or integrated reaction energy.
 *
 * `reaction_energy_release_rate_w` is a positive reaction internal-energy
 * release rate made available to a later energy balance. It is not directly
 * the time derivative of chamber-gas internal energy, because a later balance
 * must also account for work, heat transfer, population calorics, and reference
 * states.
 */
typedef struct bbtc_ib_propellant_source_rate_float_t
{
    /** Nonfatal applicability metadata propagated from the upstream mass result. */
    bbtc_applicability_flags_t applicability_flags;

    /** Generated gaseous-product mass rate, in kilograms per second. */
    float gas_product_mass_rate_kg_per_s;

    /** Generated condensed-product mass rate, in kilograms per second. */
    float condensed_product_mass_rate_kg_per_s;

    /** Positive reaction internal-energy release rate, in watts. */
    float reaction_energy_release_rate_w;
}
bbtc_ib_propellant_source_rate_float_t;


/**
 * @struct bbtc_ib_propellant_source_rate_double_t
 * @brief Instantaneous reduced propellant source rates using native `double`.
 *
 * Physical meaning, ownership, validation, and applicability semantics match
 * `bbtc_ib_propellant_source_rate_float_t`.
 */
typedef struct bbtc_ib_propellant_source_rate_double_t
{
    /** Nonfatal applicability metadata propagated from the upstream mass result. */
    bbtc_applicability_flags_t applicability_flags;

    /** Generated gaseous-product mass rate, in kilograms per second. */
    double gas_product_mass_rate_kg_per_s;

    /** Generated condensed-product mass rate, in kilograms per second. */
    double condensed_product_mass_rate_kg_per_s;

    /** Positive reaction internal-energy release rate, in watts. */
    double reaction_energy_release_rate_w;
}
bbtc_ib_propellant_source_rate_double_t;


/**
 * @struct bbtc_ib_propellant_source_rate_long_double_t
 * @brief Instantaneous reduced propellant source rates using native `long double`.
 *
 * Physical meaning, ownership, validation, and applicability semantics match
 * the other scalar families. This type does not assume that `long double` has
 * greater precision or range than `double` on every supported platform.
 */
typedef struct bbtc_ib_propellant_source_rate_long_double_t
{
    /** Nonfatal applicability metadata propagated from the upstream mass result. */
    bbtc_applicability_flags_t applicability_flags;

    /** Generated gaseous-product mass rate, in kilograms per second. */
    long double gas_product_mass_rate_kg_per_s;

    /** Generated condensed-product mass rate, in kilograms per second. */
    long double condensed_product_mass_rate_kg_per_s;

    /** Positive reaction internal-energy release rate, in watts. */
    long double reaction_energy_release_rate_w;
}
bbtc_ib_propellant_source_rate_long_double_t;


/**
 * @brief Evaluates reduced thermochemical source rates using native `float`.
 *
 * @details
 * The evaluator consumes an existing reduced thermochemistry record and a
 * complete IB0.4e propellant-mass result. Only
 * `mass->reacted_mass_rate_kg_per_s` and `mass->applicability_flags` participate
 * in this coupling layer. Other mass-result fields are intentionally not
 * revalidated here because the charge, grain geometry, and initial-grain-volume
 * inputs required to repeat the IB0.4e consistency checks are not available to
 * this layer.
 *
 * Validation order is deterministic:
 *
 * 1. a null `result` returns `BBTC_STATUS_INVALID_ARGUMENT`;
 * 2. a nonnull output is cleared;
 * 3. `thermochemistry` is validated with the existing IB0.4a validator;
 * 4. a null `mass` returns `BBTC_STATUS_INVALID_ARGUMENT`;
 * 5. the reacted-mass-rate scalar is classified for NaN and infinity;
 * 6. a finite negative rate returns `BBTC_STATUS_OUTSIDE_DOMAIN`;
 * 7. exact zero succeeds with exact zero source rates and propagated flags;
 * 8. positive source rates are evaluated and checked for representability; and
 * 9. the complete successful result is committed.
 *
 * For a positive reacted-mass rate, required positive source terms that
 * overflow, underflow to zero, or otherwise become nonfinite return
 * `BBTC_STATUS_NUMERICAL_FAILURE`. Exact
 * `thermochemistry->gas_product_mass_fraction == 1` is the all-gas limit and
 * produces exactly zero condensed-product mass rate.
 *
 * The input records are borrowed and immutable for the duration of the call.
 * The function performs no allocation and retains no input pointers.
 *
 * @param thermochemistry Reduced constant-coefficient propellant thermochemistry.
 * @param mass Upstream whole-charge propellant-mass result.
 * @param result Caller-owned source-rate destination.
 *
 * @return `BBTC_STATUS_SUCCESS` on success; otherwise the validation or
 *         numerical-failure status described above.
 */
bbtc_status_e
bbtc_ib_propellant_source_rate_evaluate_float(
    const bbtc_ib_propellant_thermochemistry_float_t* thermochemistry,
    const bbtc_ib_propellant_mass_result_float_t*     mass,
    bbtc_ib_propellant_source_rate_float_t*           result
);


/**
 * @brief Evaluates reduced thermochemical source rates using native `double`.
 *
 * Equations, validation precedence, output clearing, applicability propagation,
 * ownership, and failure semantics match
 * `bbtc_ib_propellant_source_rate_evaluate_float()`.
 *
 * @param thermochemistry Reduced constant-coefficient propellant thermochemistry.
 * @param mass Upstream whole-charge propellant-mass result.
 * @param result Caller-owned source-rate destination.
 *
 * @return `BBTC_STATUS_SUCCESS` on success; otherwise a documented nonzero
 *         status.
 */
bbtc_status_e
bbtc_ib_propellant_source_rate_evaluate_double(
    const bbtc_ib_propellant_thermochemistry_double_t* thermochemistry,
    const bbtc_ib_propellant_mass_result_double_t*     mass,
    bbtc_ib_propellant_source_rate_double_t*           result
);


/**
 * @brief Evaluates reduced thermochemical source rates using native `long double`.
 *
 * Equations, validation precedence, output clearing, applicability propagation,
 * ownership, and failure semantics match
 * `bbtc_ib_propellant_source_rate_evaluate_float()`.
 *
 * @param thermochemistry Reduced constant-coefficient propellant thermochemistry.
 * @param mass Upstream whole-charge propellant-mass result.
 * @param result Caller-owned source-rate destination.
 *
 * @return `BBTC_STATUS_SUCCESS` on success; otherwise a documented nonzero
 *         status.
 */
bbtc_status_e
bbtc_ib_propellant_source_rate_evaluate_long_double(
    const bbtc_ib_propellant_thermochemistry_long_double_t* thermochemistry,
    const bbtc_ib_propellant_mass_result_long_double_t*     mass,
    bbtc_ib_propellant_source_rate_long_double_t*           result
);

#ifdef __cplusplus
}   /* extern "C" */
#endif
#endif /* BBTC_INTERNAL_BALLISTICS_PROPELLANT_SOURCE_RATE_H */
