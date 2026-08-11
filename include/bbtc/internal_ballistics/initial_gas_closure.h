/**
 * @file
 * @brief Initial free-gas mass/density closure for BBTC reduced gas backends.
 *
 * @details
 * This layer closes an explicitly specified initial free-gas state from
 * absolute pressure, absolute temperature, initial free-gas volume, and one
 * concrete reduced mechanical equation of state.
 *
 * It returns the initial gas density and mass only. It deliberately does not
 * evaluate caloric state, require an internal-energy datum, represent
 * combustion, infer gas composition, integrate projectile motion, or make any
 * firearm/ammunition safety judgment.
 */
#ifndef BBTC_INTERNAL_BALLISTICS_INITIAL_GAS_CLOSURE_H
#define BBTC_INTERNAL_BALLISTICS_INITIAL_GAS_CLOSURE_H

#include <bbtc/diagnostics.h>
#include <bbtc/internal_ballistics/first_order_virial_gas_model.h>
#include <bbtc/internal_ballistics/initial_gas_state.h>
#include <bbtc/internal_ballistics/noble_abel_gas_model.h>
#include <bbtc/status.h>

#ifdef __cplusplus
    extern "C" {
#endif

/**
 * @brief Initial free-gas closure solution using native `float`.
 *
 * @details
 * `density_kg_per_m3` and `gas_mass_kg` describe only the initial free-gas
 * population occupying the supplied initial free-gas volume. They do not
 * include condensed propellant, future combustion-product gas, projectile
 * mass, cartridge-case mass, or firearm mass.
 *
 * `applicability_flags` reports nonfatal scientific model limitations. A zero
 * mask does not establish predictive accuracy or firearm/ammunition safety.
 */
typedef struct bbtc_ib_initial_gas_solution_float_t
{
    /** Nonfatal scientific model-applicability metadata. */
    bbtc_applicability_flags_t applicability_flags;

    /** Closed initial gas density, in kilograms per cubic meter. */
    float density_kg_per_m3;

    /** Closed initial free-gas mass, in kilograms. */
    float gas_mass_kg;
}
bbtc_ib_initial_gas_solution_float_t;

/** @brief Native-`double` initial free-gas closure solution. */
typedef struct bbtc_ib_initial_gas_solution_double_t
{
    /** Nonfatal scientific model-applicability metadata. */
    bbtc_applicability_flags_t applicability_flags;

    /** Closed initial gas density, in kilograms per cubic meter. */
    double density_kg_per_m3;

    /** Closed initial free-gas mass, in kilograms. */
    double gas_mass_kg;
}
bbtc_ib_initial_gas_solution_double_t;

/** @brief Native-`long double` initial free-gas closure solution. */
typedef struct bbtc_ib_initial_gas_solution_long_double_t
{
    /** Nonfatal scientific model-applicability metadata. */
    bbtc_applicability_flags_t applicability_flags;

    /** Closed initial gas density, in kilograms per cubic meter. */
    long double density_kg_per_m3;

    /** Closed initial free-gas mass, in kilograms. */
    long double gas_mass_kg;
}
bbtc_ib_initial_gas_solution_long_double_t;

/**
 * @brief Solves one native-`float` Noble-Abel initial free-gas state.
 *
 * @details
 * With `q = p / (R*T)`, the mechanical closure is
 *
 *     rho = q / (1 + b*q)
 *     m   = rho * V
 *
 * `b == 0` is the exact ideal-gas limit. The implementation uses targeted
 * exponent-safe arithmetic for `p/(R*T)` and algebraically equivalent forms
 * where useful to avoid unnecessary intermediate range failure. It does not
 * promise arbitrary-range arithmetic beyond the selected native scalar type.
 *
 * The returned density must remain strictly inside the representable
 * Noble-Abel excluded-volume domain `1 - b*rho > 0`. A mathematically valid
 * state that cannot preserve that strict interior after native-precision
 * rounding returns `BBTC_STATUS_NUMERICAL_FAILURE`.
 *
 * The output is cleared before any validation that can fail after a nonnull
 * solution pointer is accepted.
 */
bbtc_status_e
bbtc_ib_noble_abel_initial_gas_solve_float(
    const bbtc_ib_noble_abel_gas_model_float_t* model,
    const bbtc_ib_initial_gas_state_float_t* initial_gas_state,
    float initial_free_gas_volume_m3,
    bbtc_ib_initial_gas_solution_float_t* solution
);

/** @brief Native-`double` form of the Noble-Abel initial-gas solve. */
bbtc_status_e
bbtc_ib_noble_abel_initial_gas_solve_double(
    const bbtc_ib_noble_abel_gas_model_double_t* model,
    const bbtc_ib_initial_gas_state_double_t* initial_gas_state,
    double initial_free_gas_volume_m3,
    bbtc_ib_initial_gas_solution_double_t* solution
);

/** @brief Native-`long double` form of the Noble-Abel initial-gas solve. */
bbtc_status_e
bbtc_ib_noble_abel_initial_gas_solve_long_double(
    const bbtc_ib_noble_abel_gas_model_long_double_t* model,
    const bbtc_ib_initial_gas_state_long_double_t* initial_gas_state,
    long double initial_free_gas_volume_m3,
    bbtc_ib_initial_gas_solution_long_double_t* solution
);

/**
 * @brief Solves one native-`float` first-order-virial initial free-gas state.
 *
 * @details
 * The closure evaluates `B(T)` from the model's bounded Chebyshev law and
 * solves `p = rho*R*T*(1 + B*rho)`.
 *
 * Defining `q = p/(R*T)`, BBTC selects the unique density root continuous with
 * the ideal-gas limit and having positive local isothermal mechanical
 * stiffness. For `B == 0`, `rho = q`. For `B < 0`, the stable branch exists
 * only for `B*q > -1/4`; equality is the zero-stiffness boundary. For `B > 0`,
 * the implementation avoids directly forming a potentially overflowing `B*q`
 * by evaluating `sqrt(B*q)` through exponent-scaled square-root-product
 * arithmetic and the stable positive-root form.
 *
 * A derived density outside the calibrated density interval succeeds with
 * `BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN`. Temperature outside the
 * represented coefficient-law interval is a hard domain failure.
 *
 * The output is cleared before any validation that can fail after a nonnull
 * solution pointer is accepted.
 */
bbtc_status_e
bbtc_ib_first_order_virial_initial_gas_solve_float(
    const bbtc_ib_first_order_virial_gas_model_float_t* model,
    const bbtc_ib_initial_gas_state_float_t* initial_gas_state,
    float initial_free_gas_volume_m3,
    bbtc_ib_initial_gas_solution_float_t* solution
);

/** @brief Native-`double` form of the first-order-virial initial-gas solve. */
bbtc_status_e
bbtc_ib_first_order_virial_initial_gas_solve_double(
    const bbtc_ib_first_order_virial_gas_model_double_t* model,
    const bbtc_ib_initial_gas_state_double_t* initial_gas_state,
    double initial_free_gas_volume_m3,
    bbtc_ib_initial_gas_solution_double_t* solution
);

/** @brief Native-`long double` form of the first-order-virial initial-gas solve. */
bbtc_status_e
bbtc_ib_first_order_virial_initial_gas_solve_long_double(
    const bbtc_ib_first_order_virial_gas_model_long_double_t* model,
    const bbtc_ib_initial_gas_state_long_double_t* initial_gas_state,
    long double initial_free_gas_volume_m3,
    bbtc_ib_initial_gas_solution_long_double_t* solution
);

#ifdef __cplusplus
}   /* extern "C" */
#endif
#endif /* BBTC_INTERNAL_BALLISTICS_INITIAL_GAS_CLOSURE_H */
