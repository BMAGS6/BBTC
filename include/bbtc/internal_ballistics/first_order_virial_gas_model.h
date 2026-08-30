/**
 * @file
 * @brief Temperature-dependent first-order density-virial gas-model records.
 *
 * @details
 * This header defines the parameter and coefficient-law boundary for BBTC's
 * temperature-dependent first-order virial equation-of-state backend. It does
 * not evaluate pressure, internal energy, sound speed, combustion, projectile
 * motion, or ammunition safety.
 */
#ifndef BBTC_INTERNAL_BALLISTICS_FIRST_ORDER_VIRIAL_GAS_MODEL_H
#define BBTC_INTERNAL_BALLISTICS_FIRST_ORDER_VIRIAL_GAS_MODEL_H

#include <stddef.h>
#include <bbtc/status.h>

#ifdef __cplusplus
    extern "C" {
#endif

/**
 * @brief Chebyshev temperature law for the second density virial coefficient
 *        using native `float`.
 *
 * @details
 * The caller-owned coefficient array represents
 *
 * `B(T) = sum(c[k] * T_k(x), k = 0 .. coefficient_count - 1)`,
 *
 * where `T_k` is the Chebyshev polynomial of the first kind and
 *
 * `x = 2 * (T - minimum_temperature_k)
 *          / (maximum_temperature_k - minimum_temperature_k) - 1`.
 *
 * Consequently, the closed temperature interval maps to `x` in `[-1, 1]`.
 * Every coefficient has units of cubic meters per kilogram. Coefficients may be
 * positive, zero, or negative. A one-element zero array is the explicit
 * ideal-gas limit.
 *
 * The coefficient pointer is borrowed. BBTC does not allocate, copy, modify, or
 * free the array. The caller must keep it valid and immutable while this record
 * is validated or evaluated.
 */
typedef struct bbtc_ib_first_order_virial_temperature_law_float_t
{
    /** Lowest absolute temperature represented by the series, in kelvins. */
    float minimum_temperature_k;

    /** Highest absolute temperature represented by the series, in kelvins. */
    float maximum_temperature_k;

    /**
     * Borrowed Chebyshev coefficients for `B(T)`, each in cubic meters per
     * kilogram.
     */
    const float* second_density_virial_chebyshev_coefficients_m3_per_kg;

    /** Number of readable elements in the borrowed coefficient array. */
    size_t coefficient_count;
}
bbtc_ib_first_order_virial_temperature_law_float_t;


/**
 * @brief Native-`double` form of the first-order virial temperature law.
 *
 * @details
 * This record has the same physical meaning, coefficient convention,
 * ownership contract, and validation rules as
 * `bbtc_ib_first_order_virial_temperature_law_float_t`.
 */
typedef struct bbtc_ib_first_order_virial_temperature_law_double_t
{
    /** Lowest absolute temperature represented by the series, in kelvins. */
    double minimum_temperature_k;

    /** Highest absolute temperature represented by the series, in kelvins. */
    double maximum_temperature_k;

    /**
     * Borrowed Chebyshev coefficients for `B(T)`, each in cubic meters per
     * kilogram.
     */
    const double* second_density_virial_chebyshev_coefficients_m3_per_kg;

    /** Number of readable elements in the borrowed coefficient array. */
    size_t coefficient_count;
}
bbtc_ib_first_order_virial_temperature_law_double_t;


/**
 * @brief Native-`long double` form of the first-order virial temperature law.
 *
 * @details
 * This record has the same physical meaning, coefficient convention,
 * ownership contract, and validation rules as the other scalar families. It
 * does not imply that `long double` is wider than `double` on every platform.
 */
typedef struct bbtc_ib_first_order_virial_temperature_law_long_double_t
{
    /** Lowest absolute temperature represented by the series, in kelvins. */
    long double minimum_temperature_k;

    /** Highest absolute temperature represented by the series, in kelvins. */
    long double maximum_temperature_k;

    /**
     * Borrowed Chebyshev coefficients for `B(T)`, each in cubic meters per
     * kilogram.
     */
    const long double* second_density_virial_chebyshev_coefficients_m3_per_kg;

    /** Number of readable elements in the borrowed coefficient array. */
    size_t coefficient_count;
}
bbtc_ib_first_order_virial_temperature_law_long_double_t;


/**
 * @brief Evaluated virial coefficient and temperature derivatives using
 *        native `float`.
 *
 * @details
 * The first derivative is `dB/dT`; the second derivative is `d^2B/dT^2`.
 * Derivatives are analytic derivatives of the represented Chebyshev series,
 * not finite-difference approximations.
 */
typedef struct bbtc_ib_first_order_virial_temperature_terms_float_t
{
    /** Evaluated `B(T)`, in cubic meters per kilogram. */
    float second_density_virial_coefficient_m3_per_kg;

    /** Evaluated `dB/dT`, in cubic meters per kilogram-kelvin. */
    float first_temperature_derivative_m3_per_kg_k;

    /** Evaluated `d^2B/dT^2`, in cubic meters per kilogram-kelvin squared. */
    float second_temperature_derivative_m3_per_kg_k2;
}
bbtc_ib_first_order_virial_temperature_terms_float_t;


/**
 * @brief Native-`double` virial coefficient and temperature derivatives.
 */
typedef struct bbtc_ib_first_order_virial_temperature_terms_double_t
{
    /** Evaluated `B(T)`, in cubic meters per kilogram. */
    double second_density_virial_coefficient_m3_per_kg;

    /** Evaluated `dB/dT`, in cubic meters per kilogram-kelvin. */
    double first_temperature_derivative_m3_per_kg_k;

    /** Evaluated `d^2B/dT^2`, in cubic meters per kilogram-kelvin squared. */
    double second_temperature_derivative_m3_per_kg_k2;
}
bbtc_ib_first_order_virial_temperature_terms_double_t;


/**
 * @brief Native-`long double` virial coefficient and temperature derivatives.
 */
typedef struct bbtc_ib_first_order_virial_temperature_terms_long_double_t
{
    /** Evaluated `B(T)`, in cubic meters per kilogram. */
    long double second_density_virial_coefficient_m3_per_kg;

    /** Evaluated `dB/dT`, in cubic meters per kilogram-kelvin. */
    long double first_temperature_derivative_m3_per_kg_k;

    /** Evaluated `d^2B/dT^2`, in cubic meters per kilogram-kelvin squared. */
    long double second_temperature_derivative_m3_per_kg_k2;
}
bbtc_ib_first_order_virial_temperature_terms_long_double_t;


/**
 * @brief Temperature-dependent first-order virial gas model using native
 *        `float`.
 *
 * @details
 * A future constitutive evaluator will use the mechanical equation
 *
 * `p = rho * R * T * (1 + B(T) * rho)`,
 *
 * where `R` is `specific_gas_constant_j_per_kg_k` and `B(T)` comes from
 * `second_density_virial_coefficient_law`.
 *
 * `ideal_gas_constant_volume_specific_heat_j_per_kg_k` is the dilute-gas
 * reference heat capacity. It is not, by itself, the complete finite-density
 * heat capacity when `B(T)` varies with temperature.
 *
 * The calibrated density interval records model applicability metadata. It
 * does not cause validation to prove every state in the interval mechanically
 * admissible; a future state evaluator must separately require
 * `1 + B(T) * rho > 0`.
 */
typedef struct bbtc_ib_first_order_virial_gas_model_float_t
{
    /** Mixture-specific gas constant, in joules per kilogram-kelvin. */
    float specific_gas_constant_j_per_kg_k;

    /**
     * Dilute-gas constant-volume specific heat, in joules per
     * kilogram-kelvin.
     */
    float ideal_gas_constant_volume_specific_heat_j_per_kg_k;

    /** Lowest calibrated gas density, in kilograms per cubic meter. */
    float minimum_calibrated_density_kg_per_m3;

    /** Highest calibrated gas density, in kilograms per cubic meter. */
    float maximum_calibrated_density_kg_per_m3;

    /** Temperature-dependent law for the second density virial coefficient. */
    bbtc_ib_first_order_virial_temperature_law_float_t
        second_density_virial_coefficient_law;
}
bbtc_ib_first_order_virial_gas_model_float_t;


/**
 * @brief Native-`double` temperature-dependent first-order virial gas model.
 *
 * @details
 * This record has the same physical meaning and validation contract as
 * `bbtc_ib_first_order_virial_gas_model_float_t`.
 */
typedef struct bbtc_ib_first_order_virial_gas_model_double_t
{
    /** Mixture-specific gas constant, in joules per kilogram-kelvin. */
    double specific_gas_constant_j_per_kg_k;

    /**
     * Dilute-gas constant-volume specific heat, in joules per
     * kilogram-kelvin.
     */
    double ideal_gas_constant_volume_specific_heat_j_per_kg_k;

    /** Lowest calibrated gas density, in kilograms per cubic meter. */
    double minimum_calibrated_density_kg_per_m3;

    /** Highest calibrated gas density, in kilograms per cubic meter. */
    double maximum_calibrated_density_kg_per_m3;

    /** Temperature-dependent law for the second density virial coefficient. */
    bbtc_ib_first_order_virial_temperature_law_double_t
        second_density_virial_coefficient_law;
}
bbtc_ib_first_order_virial_gas_model_double_t;


/**
 * @brief Native-`long double` temperature-dependent first-order virial gas
 *        model.
 *
 * @details
 * This record has the same physical meaning and validation contract as the
 * other scalar families.
 */
typedef struct bbtc_ib_first_order_virial_gas_model_long_double_t
{
    /** Mixture-specific gas constant, in joules per kilogram-kelvin. */
    long double specific_gas_constant_j_per_kg_k;

    /**
     * Dilute-gas constant-volume specific heat, in joules per
     * kilogram-kelvin.
     */
    long double ideal_gas_constant_volume_specific_heat_j_per_kg_k;

    /** Lowest calibrated gas density, in kilograms per cubic meter. */
    long double minimum_calibrated_density_kg_per_m3;

    /** Highest calibrated gas density, in kilograms per cubic meter. */
    long double maximum_calibrated_density_kg_per_m3;

    /** Temperature-dependent law for the second density virial coefficient. */
    bbtc_ib_first_order_virial_temperature_law_long_double_t
        second_density_virial_coefficient_law;
}
bbtc_ib_first_order_virial_gas_model_long_double_t;


/**
 * @brief Validates one native-`float` virial-coefficient temperature law.
 *
 * @details
 * A null law or null coefficient pointer returns
 * `BBTC_STATUS_INVALID_ARGUMENT`. A NaN bound or coefficient returns
 * `BBTC_STATUS_NAN_INPUT`. Otherwise, positive or negative infinity in a bound
 * or coefficient returns `BBTC_STATUS_NONFINITE_INPUT`. NaN takes precedence
 * over infinity across the complete law, including the borrowed coefficient
 * array. The minimum temperature must be positive, the maximum must exceed the
 * minimum, and `coefficient_count` must be nonzero. Coefficient signs are
 * unrestricted.
 *
 * @param law Caller-owned temperature law to validate.
 *
 * @return `BBTC_STATUS_SUCCESS` when the law satisfies the documented domain;
 *         otherwise the validation status described above.
 */
bbtc_status_e
bbtc_ib_first_order_virial_temperature_law_validate_float(
    const bbtc_ib_first_order_virial_temperature_law_float_t* law
);


/**
 * @brief Validates one native-`double` virial-coefficient temperature law.
 *
 * @param law Caller-owned temperature law to validate.
 *
 * @return The same status contract as
 *         `bbtc_ib_first_order_virial_temperature_law_validate_float()`.
 */
bbtc_status_e
bbtc_ib_first_order_virial_temperature_law_validate_double(
    const bbtc_ib_first_order_virial_temperature_law_double_t* law
);


/**
 * @brief Validates one native-`long double` virial-coefficient temperature law.
 *
 * @param law Caller-owned temperature law to validate.
 *
 * @return The same status contract as
 *         `bbtc_ib_first_order_virial_temperature_law_validate_float()`.
 */
bbtc_status_e
bbtc_ib_first_order_virial_temperature_law_validate_long_double(
    const bbtc_ib_first_order_virial_temperature_law_long_double_t* law
);


/**
 * @brief Evaluates native-`float` `B(T)`, `dB/dT`, and `d^2B/dT^2`.
 *
 * @details
 * The complete law is validated before evaluation. Temperature must be finite
 * and lie in the closed law interval. The output is cleared before any
 * validation that can fail. Evaluation uses the native scalar family, performs
 * no allocation, and returns `BBTC_STATUS_NUMERICAL_FAILURE` if a recurrence or
 * unit conversion produces a nonfinite result.
 *
 * @param law Caller-owned temperature law.
 * @param temperature_k Absolute evaluation temperature, in kelvins.
 * @param terms Output coefficient and derivative terms.
 *
 * @return `BBTC_STATUS_SUCCESS` on success; otherwise a documented argument,
 *         domain, NaN-input, nonfinite-input, or numerical-failure status.
 */
bbtc_status_e
bbtc_ib_first_order_virial_temperature_law_evaluate_float(
    const bbtc_ib_first_order_virial_temperature_law_float_t* law,
    float temperature_k,
    bbtc_ib_first_order_virial_temperature_terms_float_t* terms
);


/**
 * @brief Evaluates native-`double` `B(T)`, `dB/dT`, and `d^2B/dT^2`.
 *
 * @param law Caller-owned temperature law.
 * @param temperature_k Absolute evaluation temperature, in kelvins.
 * @param terms Output coefficient and derivative terms.
 *
 * @return The same status contract as
 *         `bbtc_ib_first_order_virial_temperature_law_evaluate_float()`.
 */
bbtc_status_e
bbtc_ib_first_order_virial_temperature_law_evaluate_double(
    const bbtc_ib_first_order_virial_temperature_law_double_t* law,
    double temperature_k,
    bbtc_ib_first_order_virial_temperature_terms_double_t* terms
);


/**
 * @brief Evaluates native-`long double` `B(T)`, `dB/dT`, and `d^2B/dT^2`.
 *
 * @param law Caller-owned temperature law.
 * @param temperature_k Absolute evaluation temperature, in kelvins.
 * @param terms Output coefficient and derivative terms.
 *
 * @return The same status contract as
 *         `bbtc_ib_first_order_virial_temperature_law_evaluate_float()`.
 */
bbtc_status_e
bbtc_ib_first_order_virial_temperature_law_evaluate_long_double(
    const bbtc_ib_first_order_virial_temperature_law_long_double_t* law,
    long double temperature_k,
    bbtc_ib_first_order_virial_temperature_terms_long_double_t* terms
);


/**
 * @brief Validates one native-`float` first-order virial gas-model record.
 *
 * @details
 * A null model returns `BBTC_STATUS_INVALID_ARGUMENT`. A NaN top-level scalar
 * field returns `BBTC_STATUS_NAN_INPUT`; otherwise, positive or negative
 * infinity in a top-level scalar field returns `BBTC_STATUS_NONFINITE_INPUT`.
 * The gas constant and dilute-gas heat capacity must be positive. The minimum
 * calibrated density must be nonnegative, and the maximum must exceed the
 * minimum.
 *
 * Top-level scalar and domain validation is completed before the nested
 * temperature law is validated. Once that layer succeeds, the nested law's
 * validation status is propagated unchanged.
 *
 * Passing validation establishes parameter-domain consistency only. It does not
 * establish parameter provenance, calibration quality, predictive accuracy, or
 * ammunition safety.
 *
 * @param model Caller-owned gas-model record to validate.
 *
 * @return `BBTC_STATUS_SUCCESS` when every field satisfies the documented
 *         domain; otherwise a validation status.
 */
bbtc_status_e
bbtc_ib_first_order_virial_gas_model_validate_float(
    const bbtc_ib_first_order_virial_gas_model_float_t* model
);


/**
 * @brief Validates one native-`double` first-order virial gas-model record.
 *
 * @param model Caller-owned gas-model record to validate.
 *
 * @return The same status contract as
 *         `bbtc_ib_first_order_virial_gas_model_validate_float()`.
 */
bbtc_status_e
bbtc_ib_first_order_virial_gas_model_validate_double(
    const bbtc_ib_first_order_virial_gas_model_double_t* model
);


/**
 * @brief Validates one native-`long double` first-order virial gas-model record.
 *
 * @param model Caller-owned gas-model record to validate.
 *
 * @return The same status contract as
 *         `bbtc_ib_first_order_virial_gas_model_validate_float()`.
 */
bbtc_status_e
bbtc_ib_first_order_virial_gas_model_validate_long_double(
    const bbtc_ib_first_order_virial_gas_model_long_double_t * model
);


#ifdef __cplusplus
}   /* extern "C" */
#endif
#endif /* BBTC_INTERNAL_BALLISTICS_FIRST_ORDER_VIRIAL_GAS_MODEL_H */
