/**
 * @file
 * @brief Thermodynamic state evaluation for BBTC reduced gas-model backends.
 *
 * @details
 * This header converts an explicit gas density and temperature, a concrete
 * reduced equation-of-state backend, and an explicit caloric datum into a
 * common set of thermodynamic quantities.
 *
 * It deliberately does not perform gas-mass closure, combustion, ignition,
 * propellant burning, projectile integration, sound-speed evaluation, or any
 * firearm/ammunition safety judgment.
 */
#ifndef BBTC_INTERNAL_BALLISTICS_REDUCED_GAS_THERMODYNAMICS_H
#define BBTC_INTERNAL_BALLISTICS_REDUCED_GAS_THERMODYNAMICS_H

#include <bbtc/diagnostics.h>
#include <bbtc/status.h>
#include <bbtc/internal_ballistics/caloric_reference.h>
#include <bbtc/internal_ballistics/first_order_virial_gas_model.h>
#include <bbtc/internal_ballistics/noble_abel_gas_model.h>

#ifdef __cplusplus
    extern "C" {
#endif

/**
 * @brief Common reduced-gas thermodynamic evaluation result using native
 *        `float`.
 *
 * @details
 * The two pressure derivatives are
 *
 *     (partial p / partial rho)_T
 *
 * and
 *
 *     (partial p / partial T)_rho.
 *
 * `applicability_flags` describes nonfatal limitations on interpretation of a
 * successful result. In particular, a first-order virial evaluation may set
 * `BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN` when density lies outside the
 * model's documented calibration interval while remaining mathematically
 * evaluable.
 *
 * A successful status or zero applicability mask does not establish predictive
 * accuracy or firearm/ammunition safety.
 */
typedef struct bbtc_ib_reduced_gas_thermodynamic_result_float_t
{
    /** Nonfatal scientific model-applicability metadata. */
    bbtc_applicability_flags_t applicability_flags;

    /** Absolute thermodynamic pressure, in pascals. */
    float pressure_pa;

    /** Specific internal energy relative to the supplied datum, in J/kg. */
    float specific_internal_energy_j_per_kg;

    /** Constant-volume specific heat at the evaluated state, in J/(kg K). */
    float constant_volume_specific_heat_j_per_kg_k;

    /**
     * `(partial p / partial rho)_T`, in pascal cubic meters per kilogram.
     */
    float pressure_density_derivative_at_constant_temperature_pa_m3_per_kg;

    /** `(partial p / partial T)_rho`, in pascals per kelvin. */
    float pressure_temperature_derivative_at_constant_density_pa_per_k;
}
bbtc_ib_reduced_gas_thermodynamic_result_float_t;


/**
 * @brief Native-`double` common reduced-gas thermodynamic evaluation result.
 */
typedef struct bbtc_ib_reduced_gas_thermodynamic_result_double_t
{
    /** Nonfatal scientific model-applicability metadata. */
    bbtc_applicability_flags_t applicability_flags;

    /** Absolute thermodynamic pressure, in pascals. */
    double pressure_pa;

    /** Specific internal energy relative to the supplied datum, in J/kg. */
    double specific_internal_energy_j_per_kg;

    /** Constant-volume specific heat at the evaluated state, in J/(kg K). */
    double constant_volume_specific_heat_j_per_kg_k;

    /**
     * `(partial p / partial rho)_T`, in pascal cubic meters per kilogram.
     */
    double pressure_density_derivative_at_constant_temperature_pa_m3_per_kg;

    /** `(partial p / partial T)_rho`, in pascals per kelvin. */
    double pressure_temperature_derivative_at_constant_density_pa_per_k;
}
bbtc_ib_reduced_gas_thermodynamic_result_double_t;


/**
 * @brief Native-`long double` common reduced-gas thermodynamic evaluation
 *        result.
 */
typedef struct bbtc_ib_reduced_gas_thermodynamic_result_long_double_t
{
    /** Nonfatal scientific model-applicability metadata. */
    bbtc_applicability_flags_t applicability_flags;

    /** Absolute thermodynamic pressure, in pascals. */
    long double pressure_pa;

    /** Specific internal energy relative to the supplied datum, in J/kg. */
    long double specific_internal_energy_j_per_kg;

    /** Constant-volume specific heat at the evaluated state, in J/(kg K). */
    long double constant_volume_specific_heat_j_per_kg_k;

    /**
     * `(partial p / partial rho)_T`, in pascal cubic meters per kilogram.
     */
    long double pressure_density_derivative_at_constant_temperature_pa_m3_per_kg;

    /** `(partial p / partial T)_rho`, in pascals per kelvin. */
    long double pressure_temperature_derivative_at_constant_density_pa_per_k;
}
bbtc_ib_reduced_gas_thermodynamic_result_long_double_t;


/**
 * @brief Evaluates one native-`float` calorically perfect Noble-Abel state.
 *
 * @details
 * The evaluator uses
 *
 *     p = rho * R * T / (1 - b * rho)
 *
 *     e = e_ref + cv * (T - T_ref)
 *
 *     cv_state = cv
 *
 *     (partial p / partial rho)_T =
 *         R * T / (1 - b * rho)^2
 *
 *     (partial p / partial T)_rho =
 *         rho * R / (1 - b * rho)
 *
 * Density may be zero, which is the exact vacuum/dilute boundary of the
 * represented mathematics. Negative density is rejected. Temperature must be
 * strictly positive. The Noble-Abel mechanical domain requires
 * `1 - b * rho > 0`.
 *
 * Within the direct density/temperature validation layer, a NaN in either
 * scalar returns `BBTC_STATUS_NAN_INPUT`; otherwise positive or negative
 * infinity in either scalar returns `BBTC_STATUS_NONFINITE_INPUT`. NaN takes
 * precedence over infinity within this two-scalar layer. Earlier model and
 * caloric-reference validation retains its established precedence.
 *
 * The output is cleared before any validation that can fail. Arithmetic that
 * becomes nonfinite from otherwise finite inputs returns
 * `BBTC_STATUS_NUMERICAL_FAILURE`.
 *
 * @param model Concrete Noble-Abel constitutive model.
 * @param density_kg_per_m3 Gas mass density, in kilograms per cubic meter.
 * @param temperature_k Absolute gas temperature, in kelvins.
 * @param caloric_reference Explicit dilute-branch energy datum.
 * @param result Output thermodynamic state and applicability metadata.
 *
 * @return `BBTC_STATUS_SUCCESS` on success; otherwise a documented argument,
 *         NaN-input, nonfinite-input, domain, or numerical-failure status.
 */
bbtc_status_e
bbtc_ib_noble_abel_thermodynamics_evaluate_float(
    const bbtc_ib_noble_abel_gas_model_float_t* model,
    float density_kg_per_m3,
    float temperature_k,
    const bbtc_ib_caloric_reference_float_t* caloric_reference,
    bbtc_ib_reduced_gas_thermodynamic_result_float_t* result
);


/** @brief Native-`double` form of the Noble-Abel thermodynamic evaluator. */
bbtc_status_e
bbtc_ib_noble_abel_thermodynamics_evaluate_double(
    const bbtc_ib_noble_abel_gas_model_double_t* model,
    double density_kg_per_m3,
    double temperature_k,
    const bbtc_ib_caloric_reference_double_t* caloric_reference,
    bbtc_ib_reduced_gas_thermodynamic_result_double_t* result
);


/** @brief Native-`long double` form of the Noble-Abel thermodynamic evaluator. */
bbtc_status_e
bbtc_ib_noble_abel_thermodynamics_evaluate_long_double(
    const bbtc_ib_noble_abel_gas_model_long_double_t* model,
    long double density_kg_per_m3,
    long double temperature_k,
    const bbtc_ib_caloric_reference_long_double_t* caloric_reference,
    bbtc_ib_reduced_gas_thermodynamic_result_long_double_t* result
);


/**
 * @brief Evaluates one native-`float` temperature-dependent first-order virial
 *        state.
 *
 * @details
 * With `B = B(T)`, `B1 = dB/dT`, and `B2 = d^2B/dT^2`, the evaluator uses
 *
 *     p = rho * R * T * (1 + B * rho)
 *
 *     e = e_ref
 *       + cv0 * (T - T_ref)
 *       - rho * R * T^2 * B1
 *
 *     cv_state = cv0
 *              - rho * R * (2*T*B1 + T^2*B2)
 *
 *     (partial p / partial rho)_T =
 *         R * T * (1 + 2*B*rho)
 *
 *     (partial p / partial T)_rho =
 *         rho * R * (1 + B*rho + rho*T*B1)
 *
 * The existing first-order-virial temperature-law evaluator supplies `B`,
 * `B1`, and `B2`; this function does not duplicate the Chebyshev recurrence.
 *
 * State admissibility requires:
 *
 * - `rho >= 0`;
 * - `T > 0` and inside the represented Chebyshev temperature interval;
 * - `1 + B*rho > 0`;
 * - `(partial p / partial rho)_T > 0`; and
 * - `cv_state > 0`.
 *
 * The positive isothermal pressure-density derivative is the local mechanical
 * stability condition. A state may therefore have positive pressure while
 * still being rejected if its isothermal compressibility would be unstable.
 *
 * Density outside the model's calibrated density interval is not, by itself,
 * a mathematical failure. A mathematically admissible evaluation succeeds and
 * sets `BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN`.
 *
 * Within the direct density/temperature validation layer, a NaN in either
 * scalar returns `BBTC_STATUS_NAN_INPUT`; otherwise positive or negative
 * infinity in either scalar returns `BBTC_STATUS_NONFINITE_INPUT`. NaN takes
 * precedence over infinity within this two-scalar layer. Earlier model and
 * caloric-reference validation retains its established precedence.
 *
 * The output is cleared before any validation that can fail. Arithmetic that
 * becomes nonfinite from otherwise finite inputs returns
 * `BBTC_STATUS_NUMERICAL_FAILURE`.
 *
 * @param model Concrete first-order virial constitutive model.
 * @param density_kg_per_m3 Gas mass density, in kilograms per cubic meter.
 * @param temperature_k Absolute gas temperature, in kelvins.
 * @param caloric_reference Explicit dilute-branch energy datum.
 * @param result Output thermodynamic state and applicability metadata.
 *
 * @return `BBTC_STATUS_SUCCESS` on success; otherwise a documented argument,
 *         NaN-input, nonfinite-input, domain, or numerical-failure status.
 */
bbtc_status_e
bbtc_ib_first_order_virial_thermodynamics_evaluate_float(
    const bbtc_ib_first_order_virial_gas_model_float_t* model,
    float density_kg_per_m3,
    float temperature_k,
    const bbtc_ib_caloric_reference_float_t* caloric_reference,
    bbtc_ib_reduced_gas_thermodynamic_result_float_t* result
);


/** @brief Native-`double` form of the first-order virial evaluator. */
bbtc_status_e
bbtc_ib_first_order_virial_thermodynamics_evaluate_double(
    const bbtc_ib_first_order_virial_gas_model_double_t* model,
    double density_kg_per_m3,
    double temperature_k,
    const bbtc_ib_caloric_reference_double_t* caloric_reference,
    bbtc_ib_reduced_gas_thermodynamic_result_double_t* result
);


/** @brief Native-`long double` form of the first-order virial evaluator. */
bbtc_status_e
bbtc_ib_first_order_virial_thermodynamics_evaluate_long_double(
    const bbtc_ib_first_order_virial_gas_model_long_double_t* model,
    long double density_kg_per_m3,
    long double temperature_k,
    const bbtc_ib_caloric_reference_long_double_t* caloric_reference,
    bbtc_ib_reduced_gas_thermodynamic_result_long_double_t* result
);

#ifdef __cplusplus
}   /* extern "C" */
#endif
#endif /* BBTC_INTERNAL_BALLISTICS_REDUCED_GAS_THERMODYNAMICS_H */
