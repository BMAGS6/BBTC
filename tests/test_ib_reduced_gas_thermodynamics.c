/**
 * @file
 * @brief Core contract tests for reduced-gas thermodynamic evaluation.
 */
#include <stddef.h>
#include <math.h>
#include <float.h>

#include <bbtc/bbtc.h>

static int
nearly_equal_float(float actual, float expected, float tolerance)
{
    return fabsf(actual - expected) <= tolerance;
}


static int
nearly_equal_double(double actual, double expected, double tolerance)
{
    return fabs(actual - expected) <= tolerance;
}


static int
nearly_equal_long_double(
    long double actual,
    long double expected,
    long double tolerance
)
{
    return fabsl(actual - expected) <= tolerance;
}


static int
test_caloric_reference_validation(void)
{
    const bbtc_ib_caloric_reference_double_t valid_reference =
    {
        .reference_temperature_k = 298.15,
        .reference_specific_internal_energy_j_per_kg = -1250.0
    };

    bbtc_ib_caloric_reference_double_t invalid_reference = valid_reference;

    if (bbtc_ib_caloric_reference_validate_double(NULL)
        != BBTC_STATUS_INVALID_ARGUMENT)
    {
        return 1;
    }

    if (bbtc_ib_caloric_reference_validate_double(&valid_reference)
        != BBTC_STATUS_SUCCESS)
    {
        return 1;
    }

    invalid_reference.reference_temperature_k = 0.0;
    if (bbtc_ib_caloric_reference_validate_double(&invalid_reference)
        != BBTC_STATUS_OUTSIDE_DOMAIN)
    {
        return 1;
    }

    invalid_reference = valid_reference;
    invalid_reference.reference_specific_internal_energy_j_per_kg = NAN;
    if (bbtc_ib_caloric_reference_validate_double(&invalid_reference)
        != BBTC_STATUS_NONFINITE_INPUT)
    {
        return 1;
    }

    return 0;
}


static int
test_noble_abel_double(void)
{
    const bbtc_ib_noble_abel_gas_model_double_t model =
    {
        .specific_gas_constant_j_per_kg_k = 287.0,
        .constant_volume_specific_heat_j_per_kg_k = 718.0,
        .covolume_m3_per_kg = 0.0
    };

    const bbtc_ib_caloric_reference_double_t reference =
    {
        .reference_temperature_k = 298.15,
        .reference_specific_internal_energy_j_per_kg = -100.0
    };

    bbtc_ib_reduced_gas_thermodynamic_result_double_t result = {0};

    if (bbtc_ib_noble_abel_thermodynamics_evaluate_double(
            &model,
            1.2,
            300.0,
            &reference,
            &result
        ) != BBTC_STATUS_SUCCESS)
    {
        return 1;
    }

    if (result.applicability_flags != BBTC_APPLICABILITY_NONE_REPORTED ||
        !nearly_equal_double(result.pressure_pa, 103320.0, 1.0e-9) ||
        !nearly_equal_double(
            result.specific_internal_energy_j_per_kg,
            1228.3,
            1.0e-9
        ) ||
        !nearly_equal_double(
            result.constant_volume_specific_heat_j_per_kg_k,
            718.0,
            1.0e-12
        ) ||
        !nearly_equal_double(
            result.pressure_density_derivative_at_constant_temperature_pa_m3_per_kg,
            86100.0,
            1.0e-9
        ) ||
        !nearly_equal_double(
            result.pressure_temperature_derivative_at_constant_density_pa_per_k,
            344.4,
            1.0e-12
        ))
    {
        return 1;
    }

    return 0;
}


static int
test_noble_abel_zero_density_all_precisions(void)
{
    const bbtc_ib_noble_abel_gas_model_float_t model_float =
    {
        .specific_gas_constant_j_per_kg_k = 287.0f,
        .constant_volume_specific_heat_j_per_kg_k = 718.0f,
        .covolume_m3_per_kg = 0.001f
    };
    const bbtc_ib_caloric_reference_float_t reference_float =
    {
        .reference_temperature_k = 300.0f,
        .reference_specific_internal_energy_j_per_kg = 10.0f
    };
    bbtc_ib_reduced_gas_thermodynamic_result_float_t result_float = {0};

    const bbtc_ib_noble_abel_gas_model_double_t model_double =
    {
        .specific_gas_constant_j_per_kg_k = 287.0,
        .constant_volume_specific_heat_j_per_kg_k = 718.0,
        .covolume_m3_per_kg = 0.001
    };
    const bbtc_ib_caloric_reference_double_t reference_double =
    {
        .reference_temperature_k = 300.0,
        .reference_specific_internal_energy_j_per_kg = 10.0
    };
    bbtc_ib_reduced_gas_thermodynamic_result_double_t result_double = {0};

    const bbtc_ib_noble_abel_gas_model_long_double_t model_long_double =
    {
        .specific_gas_constant_j_per_kg_k = 287.0L,
        .constant_volume_specific_heat_j_per_kg_k = 718.0L,
        .covolume_m3_per_kg = 0.001L
    };
    const bbtc_ib_caloric_reference_long_double_t reference_long_double =
    {
        .reference_temperature_k = 300.0L,
        .reference_specific_internal_energy_j_per_kg = 10.0L
    };
    bbtc_ib_reduced_gas_thermodynamic_result_long_double_t result_long_double = {0};

    if (bbtc_ib_noble_abel_thermodynamics_evaluate_float(
            &model_float, 0.0f, 300.0f, &reference_float, &result_float
        ) != BBTC_STATUS_SUCCESS ||
        bbtc_ib_noble_abel_thermodynamics_evaluate_double(
            &model_double, 0.0, 300.0, &reference_double, &result_double
        ) != BBTC_STATUS_SUCCESS ||
        bbtc_ib_noble_abel_thermodynamics_evaluate_long_double(
            &model_long_double,
            0.0L,
            300.0L,
            &reference_long_double,
            &result_long_double
        ) != BBTC_STATUS_SUCCESS)
    {
        return 1;
    }

    if (result_float.pressure_pa != 0.0f ||
        result_float.specific_internal_energy_j_per_kg != 10.0f ||
        result_double.pressure_pa != 0.0 ||
        result_double.specific_internal_energy_j_per_kg != 10.0 ||
        result_long_double.pressure_pa != 0.0L ||
        result_long_double.specific_internal_energy_j_per_kg != 10.0L)
    {
        return 1;
    }

    if (!nearly_equal_float(
            result_float.pressure_density_derivative_at_constant_temperature_pa_m3_per_kg,
            86100.0f,
            0.1f
        ) ||
        !nearly_equal_double(
            result_double.pressure_density_derivative_at_constant_temperature_pa_m3_per_kg,
            86100.0,
            1.0e-9
        ) ||
        !nearly_equal_long_double(
            result_long_double.pressure_density_derivative_at_constant_temperature_pa_m3_per_kg,
            86100.0L,
            1.0e-12L
        ))
    {
        return 1;
    }

    return 0;
}


static int
test_noble_abel_domain_and_output_clearing(void)
{
    const bbtc_ib_noble_abel_gas_model_double_t model =
    {
        .specific_gas_constant_j_per_kg_k = 287.0,
        .constant_volume_specific_heat_j_per_kg_k = 718.0,
        .covolume_m3_per_kg = 0.001
    };
    const bbtc_ib_caloric_reference_double_t reference =
    {
        .reference_temperature_k = 300.0,
        .reference_specific_internal_energy_j_per_kg = 0.0
    };
    bbtc_ib_reduced_gas_thermodynamic_result_double_t result =
    {
        .applicability_flags = BBTC_APPLICABILITY_DATA_PROVENANCE_UNKNOWN,
        .pressure_pa = 1.0,
        .specific_internal_energy_j_per_kg = 1.0,
        .constant_volume_specific_heat_j_per_kg_k = 1.0,
        .pressure_density_derivative_at_constant_temperature_pa_m3_per_kg = 1.0,
        .pressure_temperature_derivative_at_constant_density_pa_per_k = 1.0
    };

    if (bbtc_ib_noble_abel_thermodynamics_evaluate_double(
            &model, 1000.0, 300.0, &reference, &result
        ) != BBTC_STATUS_OUTSIDE_DOMAIN)
    {
        return 1;
    }

    if (result.applicability_flags != 0 ||
        result.pressure_pa != 0.0 ||
        result.specific_internal_energy_j_per_kg != 0.0 ||
        result.constant_volume_specific_heat_j_per_kg_k != 0.0 ||
        result.pressure_density_derivative_at_constant_temperature_pa_m3_per_kg != 0.0 ||
        result.pressure_temperature_derivative_at_constant_density_pa_per_k != 0.0)
    {
        return 1;
    }

    return 0;
}


static int
test_virial_thermodynamic_consistency_double(void)
{
    const double coefficients[] =
    {
        0.001,
        0.002
    };
    const bbtc_ib_first_order_virial_gas_model_double_t model =
    {
        .specific_gas_constant_j_per_kg_k = 100.0,
        .ideal_gas_constant_volume_specific_heat_j_per_kg_k = 500.0,
        .minimum_calibrated_density_kg_per_m3 = 0.0,
        .maximum_calibrated_density_kg_per_m3 = 100.0,
        .second_density_virial_coefficient_law =
        {
            .minimum_temperature_k = 200.0,
            .maximum_temperature_k = 400.0,
            .second_density_virial_chebyshev_coefficients_m3_per_kg =
                coefficients,
            .coefficient_count = 2u
        }
    };
    const bbtc_ib_caloric_reference_double_t reference =
    {
        .reference_temperature_k = 300.0,
        .reference_specific_internal_energy_j_per_kg = 1000.0
    };
    bbtc_ib_reduced_gas_thermodynamic_result_double_t result = {0};

    if (bbtc_ib_first_order_virial_thermodynamics_evaluate_double(
            &model, 10.0, 300.0, &reference, &result
        ) != BBTC_STATUS_SUCCESS)
    {
        return 1;
    }

    if (result.applicability_flags != BBTC_APPLICABILITY_NONE_REPORTED ||
        !nearly_equal_double(result.pressure_pa, 303000.0, 1.0e-8) ||
        !nearly_equal_double(
            result.specific_internal_energy_j_per_kg, -800.0, 1.0e-8
        ) ||
        !nearly_equal_double(
            result.constant_volume_specific_heat_j_per_kg_k, 488.0, 1.0e-10
        ) ||
        !nearly_equal_double(
            result.pressure_density_derivative_at_constant_temperature_pa_m3_per_kg,
            30600.0,
            1.0e-8
        ) ||
        !nearly_equal_double(
            result.pressure_temperature_derivative_at_constant_density_pa_per_k,
            1070.0,
            1.0e-10
        ))
    {
        return 1;
    }

    return 0;
}


static int
test_virial_calibration_flag(void)
{
    const double coefficients[] = {0.001};
    const bbtc_ib_first_order_virial_gas_model_double_t model =
    {
        .specific_gas_constant_j_per_kg_k = 100.0,
        .ideal_gas_constant_volume_specific_heat_j_per_kg_k = 500.0,
        .minimum_calibrated_density_kg_per_m3 = 0.0,
        .maximum_calibrated_density_kg_per_m3 = 5.0,
        .second_density_virial_coefficient_law =
        {
            .minimum_temperature_k = 200.0,
            .maximum_temperature_k = 400.0,
            .second_density_virial_chebyshev_coefficients_m3_per_kg =
                coefficients,
            .coefficient_count = 1u
        }
    };
    const bbtc_ib_caloric_reference_double_t reference =
    {
        .reference_temperature_k = 300.0,
        .reference_specific_internal_energy_j_per_kg = 0.0
    };
    bbtc_ib_reduced_gas_thermodynamic_result_double_t result = {0};

    if (bbtc_ib_first_order_virial_thermodynamics_evaluate_double(
            &model, 10.0, 300.0, &reference, &result
        ) != BBTC_STATUS_SUCCESS)
    {
        return 1;
    }

    if ((result.applicability_flags
         & BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN) == 0)
    {
        return 1;
    }

    return 0;
}


static int
test_virial_stability_rejection(void)
{
    const double coefficients[] = {-0.075};
    const bbtc_ib_first_order_virial_gas_model_double_t model =
    {
        .specific_gas_constant_j_per_kg_k = 100.0,
        .ideal_gas_constant_volume_specific_heat_j_per_kg_k = 500.0,
        .minimum_calibrated_density_kg_per_m3 = 0.0,
        .maximum_calibrated_density_kg_per_m3 = 20.0,
        .second_density_virial_coefficient_law =
        {
            .minimum_temperature_k = 200.0,
            .maximum_temperature_k = 400.0,
            .second_density_virial_chebyshev_coefficients_m3_per_kg =
                coefficients,
            .coefficient_count = 1u
        }
    };
    const bbtc_ib_caloric_reference_double_t reference =
    {
        .reference_temperature_k = 300.0,
        .reference_specific_internal_energy_j_per_kg = 0.0
    };
    bbtc_ib_reduced_gas_thermodynamic_result_double_t result = {0};

    /*
     * At rho = 10, 1 + B*rho = 0.25 remains positive, but
     * 1 + 2*B*rho = -0.5. The state has positive pressure but negative
     * isothermal pressure-density derivative and must be rejected.
     */
    if (bbtc_ib_first_order_virial_thermodynamics_evaluate_double(
            &model, 10.0, 300.0, &reference, &result
        ) != BBTC_STATUS_OUTSIDE_DOMAIN)
    {
        return 1;
    }

    return 0;
}


static int
test_virial_heat_capacity_rejection(void)
{
    const double coefficients[] =
    {
        0.0,
        1.0
    };
    const bbtc_ib_first_order_virial_gas_model_double_t model =
    {
        .specific_gas_constant_j_per_kg_k = 100.0,
        .ideal_gas_constant_volume_specific_heat_j_per_kg_k = 500.0,
        .minimum_calibrated_density_kg_per_m3 = 0.0,
        .maximum_calibrated_density_kg_per_m3 = 20.0,
        .second_density_virial_coefficient_law =
        {
            .minimum_temperature_k = 200.0,
            .maximum_temperature_k = 400.0,
            .second_density_virial_chebyshev_coefficients_m3_per_kg =
                coefficients,
            .coefficient_count = 2u
        }
    };
    const bbtc_ib_caloric_reference_double_t reference =
    {
        .reference_temperature_k = 300.0,
        .reference_specific_internal_energy_j_per_kg = 0.0
    };
    bbtc_ib_reduced_gas_thermodynamic_result_double_t result = {0};

    if (bbtc_ib_first_order_virial_thermodynamics_evaluate_double(
            &model, 10.0, 300.0, &reference, &result
        ) != BBTC_STATUS_OUTSIDE_DOMAIN)
    {
        return 1;
    }

    return 0;
}

static int
test_noble_abel_nonzero_covolume_double(void)
{
    const bbtc_ib_noble_abel_gas_model_double_t model =
    {
        .specific_gas_constant_j_per_kg_k = 100.0,
        .constant_volume_specific_heat_j_per_kg_k = 500.0,
        .covolume_m3_per_kg = 0.01
    };
    const bbtc_ib_caloric_reference_double_t reference =
    {
        .reference_temperature_k = 250.0,
        .reference_specific_internal_energy_j_per_kg = 1000.0
    };
    bbtc_ib_reduced_gas_thermodynamic_result_double_t result = {0};

    if (bbtc_ib_noble_abel_thermodynamics_evaluate_double(
            &model, 10.0, 300.0, &reference, &result
        ) != BBTC_STATUS_SUCCESS)
    {
        return 1;
    }

    if (!nearly_equal_double(result.pressure_pa, 333333.3333333333, 1.0e-8) ||
        !nearly_equal_double(
            result.specific_internal_energy_j_per_kg, 26000.0, 1.0e-12
        ) ||
        !nearly_equal_double(
            result.constant_volume_specific_heat_j_per_kg_k, 500.0, 1.0e-12
        ) ||
        !nearly_equal_double(
            result.pressure_density_derivative_at_constant_temperature_pa_m3_per_kg,
            37037.03703703704,
            1.0e-8
        ) ||
        !nearly_equal_double(
            result.pressure_temperature_derivative_at_constant_density_pa_per_k,
            1111.111111111111,
            1.0e-9
        ))
    {
        return 1;
    }

    return 0;
}


static int
test_reduced_gas_ideal_limit_all_precisions(void)
{
    const float coefficients_float[] = {0.0f};
    const double coefficients_double[] = {0.0};
    const long double coefficients_long_double[] = {0.0L};

    const bbtc_ib_noble_abel_gas_model_float_t noble_float =
    {
        .specific_gas_constant_j_per_kg_k = 2.0f,
        .constant_volume_specific_heat_j_per_kg_k = 3.0f,
        .covolume_m3_per_kg = 0.0f
    };
    const bbtc_ib_first_order_virial_gas_model_float_t virial_float =
    {
        .specific_gas_constant_j_per_kg_k = 2.0f,
        .ideal_gas_constant_volume_specific_heat_j_per_kg_k = 3.0f,
        .minimum_calibrated_density_kg_per_m3 = 0.0f,
        .maximum_calibrated_density_kg_per_m3 = 10.0f,
        .second_density_virial_coefficient_law =
        {
            .minimum_temperature_k = 200.0f,
            .maximum_temperature_k = 400.0f,
            .second_density_virial_chebyshev_coefficients_m3_per_kg =
                coefficients_float,
            .coefficient_count = 1u
        }
    };
    const bbtc_ib_caloric_reference_float_t reference_float =
    {
        .reference_temperature_k = 250.0f,
        .reference_specific_internal_energy_j_per_kg = 4.0f
    };
    bbtc_ib_reduced_gas_thermodynamic_result_float_t noble_result_float = {0};
    bbtc_ib_reduced_gas_thermodynamic_result_float_t virial_result_float = {0};

    const bbtc_ib_noble_abel_gas_model_double_t noble_double =
    {
        .specific_gas_constant_j_per_kg_k = 2.0,
        .constant_volume_specific_heat_j_per_kg_k = 3.0,
        .covolume_m3_per_kg = 0.0
    };
    const bbtc_ib_first_order_virial_gas_model_double_t virial_double =
    {
        .specific_gas_constant_j_per_kg_k = 2.0,
        .ideal_gas_constant_volume_specific_heat_j_per_kg_k = 3.0,
        .minimum_calibrated_density_kg_per_m3 = 0.0,
        .maximum_calibrated_density_kg_per_m3 = 10.0,
        .second_density_virial_coefficient_law =
        {
            .minimum_temperature_k = 200.0,
            .maximum_temperature_k = 400.0,
            .second_density_virial_chebyshev_coefficients_m3_per_kg =
                coefficients_double,
            .coefficient_count = 1u
        }
    };
    const bbtc_ib_caloric_reference_double_t reference_double =
    {
        .reference_temperature_k = 250.0,
        .reference_specific_internal_energy_j_per_kg = 4.0
    };
    bbtc_ib_reduced_gas_thermodynamic_result_double_t noble_result_double = {0};
    bbtc_ib_reduced_gas_thermodynamic_result_double_t virial_result_double = {0};

    const bbtc_ib_noble_abel_gas_model_long_double_t noble_long_double =
    {
        .specific_gas_constant_j_per_kg_k = 2.0L,
        .constant_volume_specific_heat_j_per_kg_k = 3.0L,
        .covolume_m3_per_kg = 0.0L
    };
    const bbtc_ib_first_order_virial_gas_model_long_double_t virial_long_double =
    {
        .specific_gas_constant_j_per_kg_k = 2.0L,
        .ideal_gas_constant_volume_specific_heat_j_per_kg_k = 3.0L,
        .minimum_calibrated_density_kg_per_m3 = 0.0L,
        .maximum_calibrated_density_kg_per_m3 = 10.0L,
        .second_density_virial_coefficient_law =
        {
            .minimum_temperature_k = 200.0L,
            .maximum_temperature_k = 400.0L,
            .second_density_virial_chebyshev_coefficients_m3_per_kg =
                coefficients_long_double,
            .coefficient_count = 1u
        }
    };
    const bbtc_ib_caloric_reference_long_double_t reference_long_double =
    {
        .reference_temperature_k = 250.0L,
        .reference_specific_internal_energy_j_per_kg = 4.0L
    };
    bbtc_ib_reduced_gas_thermodynamic_result_long_double_t noble_result_long_double = {0};
    bbtc_ib_reduced_gas_thermodynamic_result_long_double_t virial_result_long_double = {0};

    if (bbtc_ib_noble_abel_thermodynamics_evaluate_float(
            &noble_float, 1.0f, 300.0f, &reference_float, &noble_result_float
        ) != BBTC_STATUS_SUCCESS ||
        bbtc_ib_first_order_virial_thermodynamics_evaluate_float(
            &virial_float, 1.0f, 300.0f, &reference_float, &virial_result_float
        ) != BBTC_STATUS_SUCCESS ||
        bbtc_ib_noble_abel_thermodynamics_evaluate_double(
            &noble_double, 1.0, 300.0, &reference_double, &noble_result_double
        ) != BBTC_STATUS_SUCCESS ||
        bbtc_ib_first_order_virial_thermodynamics_evaluate_double(
            &virial_double, 1.0, 300.0, &reference_double, &virial_result_double
        ) != BBTC_STATUS_SUCCESS ||
        bbtc_ib_noble_abel_thermodynamics_evaluate_long_double(
            &noble_long_double,
            1.0L,
            300.0L,
            &reference_long_double,
            &noble_result_long_double
        ) != BBTC_STATUS_SUCCESS ||
        bbtc_ib_first_order_virial_thermodynamics_evaluate_long_double(
            &virial_long_double,
            1.0L,
            300.0L,
            &reference_long_double,
            &virial_result_long_double
        ) != BBTC_STATUS_SUCCESS)
    {
        return 1;
    }

    if (noble_result_float.pressure_pa != 600.0f ||
        noble_result_float.specific_internal_energy_j_per_kg != 154.0f ||
        noble_result_float.constant_volume_specific_heat_j_per_kg_k != 3.0f ||
        noble_result_float.pressure_density_derivative_at_constant_temperature_pa_m3_per_kg != 600.0f ||
        noble_result_float.pressure_temperature_derivative_at_constant_density_pa_per_k != 2.0f ||
        virial_result_float.pressure_pa != noble_result_float.pressure_pa ||
        virial_result_float.specific_internal_energy_j_per_kg
            != noble_result_float.specific_internal_energy_j_per_kg ||
        virial_result_float.constant_volume_specific_heat_j_per_kg_k
            != noble_result_float.constant_volume_specific_heat_j_per_kg_k ||
        virial_result_float.pressure_density_derivative_at_constant_temperature_pa_m3_per_kg
            != noble_result_float.pressure_density_derivative_at_constant_temperature_pa_m3_per_kg ||
        virial_result_float.pressure_temperature_derivative_at_constant_density_pa_per_k
            != noble_result_float.pressure_temperature_derivative_at_constant_density_pa_per_k)
    {
        return 1;
    }

    if (noble_result_double.pressure_pa != 600.0 ||
        noble_result_double.specific_internal_energy_j_per_kg != 154.0 ||
        noble_result_double.constant_volume_specific_heat_j_per_kg_k != 3.0 ||
        noble_result_double.pressure_density_derivative_at_constant_temperature_pa_m3_per_kg != 600.0 ||
        noble_result_double.pressure_temperature_derivative_at_constant_density_pa_per_k != 2.0 ||
        virial_result_double.pressure_pa != noble_result_double.pressure_pa ||
        virial_result_double.specific_internal_energy_j_per_kg
            != noble_result_double.specific_internal_energy_j_per_kg ||
        virial_result_double.constant_volume_specific_heat_j_per_kg_k
            != noble_result_double.constant_volume_specific_heat_j_per_kg_k ||
        virial_result_double.pressure_density_derivative_at_constant_temperature_pa_m3_per_kg
            != noble_result_double.pressure_density_derivative_at_constant_temperature_pa_m3_per_kg ||
        virial_result_double.pressure_temperature_derivative_at_constant_density_pa_per_k
            != noble_result_double.pressure_temperature_derivative_at_constant_density_pa_per_k)
    {
        return 1;
    }

    if (noble_result_long_double.pressure_pa != 600.0L ||
        noble_result_long_double.specific_internal_energy_j_per_kg != 154.0L ||
        noble_result_long_double.constant_volume_specific_heat_j_per_kg_k != 3.0L ||
        noble_result_long_double.pressure_density_derivative_at_constant_temperature_pa_m3_per_kg != 600.0L ||
        noble_result_long_double.pressure_temperature_derivative_at_constant_density_pa_per_k != 2.0L ||
        virial_result_long_double.pressure_pa != noble_result_long_double.pressure_pa ||
        virial_result_long_double.specific_internal_energy_j_per_kg
            != noble_result_long_double.specific_internal_energy_j_per_kg ||
        virial_result_long_double.constant_volume_specific_heat_j_per_kg_k
            != noble_result_long_double.constant_volume_specific_heat_j_per_kg_k ||
        virial_result_long_double.pressure_density_derivative_at_constant_temperature_pa_m3_per_kg
            != noble_result_long_double.pressure_density_derivative_at_constant_temperature_pa_m3_per_kg ||
        virial_result_long_double.pressure_temperature_derivative_at_constant_density_pa_per_k
            != noble_result_long_double.pressure_temperature_derivative_at_constant_density_pa_per_k)
    {
        return 1;
    }

    return 0;
}


static int
test_numerical_failure_output_clearing(void)
{
    const bbtc_ib_caloric_reference_double_t reference =
    {
        .reference_temperature_k = 300.0,
        .reference_specific_internal_energy_j_per_kg = 0.0
    };
    const bbtc_ib_noble_abel_gas_model_double_t noble_model =
    {
        .specific_gas_constant_j_per_kg_k = DBL_MAX,
        .constant_volume_specific_heat_j_per_kg_k = 1.0,
        .covolume_m3_per_kg = 0.0
    };
    const double virial_coefficients[] = {0.0};
    const bbtc_ib_first_order_virial_gas_model_double_t virial_model =
    {
        .specific_gas_constant_j_per_kg_k = DBL_MAX,
        .ideal_gas_constant_volume_specific_heat_j_per_kg_k = 1.0,
        .minimum_calibrated_density_kg_per_m3 = 0.0,
        .maximum_calibrated_density_kg_per_m3 = 10.0,
        .second_density_virial_coefficient_law =
        {
            .minimum_temperature_k = 200.0,
            .maximum_temperature_k = 400.0,
            .second_density_virial_chebyshev_coefficients_m3_per_kg =
                virial_coefficients,
            .coefficient_count = 1u
        }
    };
    bbtc_ib_reduced_gas_thermodynamic_result_double_t result =
    {
        .applicability_flags = BBTC_APPLICABILITY_DATA_PROVENANCE_UNKNOWN,
        .pressure_pa = 1.0,
        .specific_internal_energy_j_per_kg = 1.0,
        .constant_volume_specific_heat_j_per_kg_k = 1.0,
        .pressure_density_derivative_at_constant_temperature_pa_m3_per_kg = 1.0,
        .pressure_temperature_derivative_at_constant_density_pa_per_k = 1.0
    };

    if (bbtc_ib_noble_abel_thermodynamics_evaluate_double(
            &noble_model, 2.0, 2.0, &reference, &result
        ) != BBTC_STATUS_NUMERICAL_FAILURE)
    {
        return 1;
    }

    if (result.applicability_flags != 0 ||
        result.pressure_pa != 0.0 ||
        result.specific_internal_energy_j_per_kg != 0.0 ||
        result.constant_volume_specific_heat_j_per_kg_k != 0.0 ||
        result.pressure_density_derivative_at_constant_temperature_pa_m3_per_kg != 0.0 ||
        result.pressure_temperature_derivative_at_constant_density_pa_per_k != 0.0)
    {
        return 1;
    }

    result.applicability_flags = BBTC_APPLICABILITY_DATA_PROVENANCE_UNKNOWN;
    result.pressure_pa = 1.0;
    result.specific_internal_energy_j_per_kg = 1.0;
    result.constant_volume_specific_heat_j_per_kg_k = 1.0;
    result.pressure_density_derivative_at_constant_temperature_pa_m3_per_kg = 1.0;
    result.pressure_temperature_derivative_at_constant_density_pa_per_k = 1.0;

    if (bbtc_ib_first_order_virial_thermodynamics_evaluate_double(
            &virial_model, 2.0, 300.0, &reference, &result
        ) != BBTC_STATUS_NUMERICAL_FAILURE)
    {
        return 1;
    }

    if (result.applicability_flags != 0 ||
        result.pressure_pa != 0.0 ||
        result.specific_internal_energy_j_per_kg != 0.0 ||
        result.constant_volume_specific_heat_j_per_kg_k != 0.0 ||
        result.pressure_density_derivative_at_constant_temperature_pa_m3_per_kg != 0.0 ||
        result.pressure_temperature_derivative_at_constant_density_pa_per_k != 0.0)
    {
        return 1;
    }

    return 0;
}


static int
test_virial_temperature_domain_and_reference_semantics(void)
{
    const double coefficients[] = {0.0};
    const bbtc_ib_first_order_virial_gas_model_double_t model =
    {
        .specific_gas_constant_j_per_kg_k = 2.0,
        .ideal_gas_constant_volume_specific_heat_j_per_kg_k = 3.0,
        .minimum_calibrated_density_kg_per_m3 = 0.0,
        .maximum_calibrated_density_kg_per_m3 = 10.0,
        .second_density_virial_coefficient_law =
        {
            .minimum_temperature_k = 200.0,
            .maximum_temperature_k = 400.0,
            .second_density_virial_chebyshev_coefficients_m3_per_kg =
                coefficients,
            .coefficient_count = 1u
        }
    };
    const bbtc_ib_caloric_reference_double_t reference =
    {
        .reference_temperature_k = 100.0,
        .reference_specific_internal_energy_j_per_kg = 7.0
    };
    bbtc_ib_reduced_gas_thermodynamic_result_double_t result = {0};

    /*
     * The caloric datum belongs to the hypothetical dilute branch and need not
     * lie inside the virial coefficient law's represented temperature range.
     */
    if (bbtc_ib_first_order_virial_thermodynamics_evaluate_double(
            &model, 1.0, 300.0, &reference, &result
        ) != BBTC_STATUS_SUCCESS ||
        result.specific_internal_energy_j_per_kg != 607.0)
    {
        return 1;
    }

    result.pressure_pa = 1.0;
    result.specific_internal_energy_j_per_kg = 1.0;
    result.constant_volume_specific_heat_j_per_kg_k = 1.0;

    if (bbtc_ib_first_order_virial_thermodynamics_evaluate_double(
            &model, 1.0, 199.0, &reference, &result
        ) != BBTC_STATUS_OUTSIDE_DOMAIN)
    {
        return 1;
    }

    if (result.applicability_flags != 0 ||
        result.pressure_pa != 0.0 ||
        result.specific_internal_energy_j_per_kg != 0.0 ||
        result.constant_volume_specific_heat_j_per_kg_k != 0.0 ||
        result.pressure_density_derivative_at_constant_temperature_pa_m3_per_kg != 0.0 ||
        result.pressure_temperature_derivative_at_constant_density_pa_per_k != 0.0)
    {
        return 1;
    }

    return 0;
}


static int
test_virial_calibration_interval_boundaries(void)
{
    const double coefficients[] = {0.0};
    const bbtc_ib_first_order_virial_gas_model_double_t model =
    {
        .specific_gas_constant_j_per_kg_k = 2.0,
        .ideal_gas_constant_volume_specific_heat_j_per_kg_k = 3.0,
        .minimum_calibrated_density_kg_per_m3 = 2.0,
        .maximum_calibrated_density_kg_per_m3 = 5.0,
        .second_density_virial_coefficient_law =
        {
            .minimum_temperature_k = 200.0,
            .maximum_temperature_k = 400.0,
            .second_density_virial_chebyshev_coefficients_m3_per_kg =
                coefficients,
            .coefficient_count = 1u
        }
    };
    const bbtc_ib_caloric_reference_double_t reference =
    {
        .reference_temperature_k = 300.0,
        .reference_specific_internal_energy_j_per_kg = 0.0
    };
    bbtc_ib_reduced_gas_thermodynamic_result_double_t result = {0};

    if (bbtc_ib_first_order_virial_thermodynamics_evaluate_double(
            &model, 2.0, 300.0, &reference, &result
        ) != BBTC_STATUS_SUCCESS ||
        result.applicability_flags != BBTC_APPLICABILITY_NONE_REPORTED)
    {
        return 1;
    }

    if (bbtc_ib_first_order_virial_thermodynamics_evaluate_double(
            &model, 5.0, 300.0, &reference, &result
        ) != BBTC_STATUS_SUCCESS ||
        result.applicability_flags != BBTC_APPLICABILITY_NONE_REPORTED)
    {
        return 1;
    }

    if (bbtc_ib_first_order_virial_thermodynamics_evaluate_double(
            &model, 1.0, 300.0, &reference, &result
        ) != BBTC_STATUS_SUCCESS ||
        (result.applicability_flags
         & BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN) == 0)
    {
        return 1;
    }

    if (bbtc_ib_first_order_virial_thermodynamics_evaluate_double(
            &model, 6.0, 300.0, &reference, &result
        ) != BBTC_STATUS_SUCCESS ||
        (result.applicability_flags
         & BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN) == 0)
    {
        return 1;
    }

    return 0;
}


int
main(void)
{
    if (test_caloric_reference_validation() != 0)
        return 1;

    if (test_noble_abel_double() != 0)
        return 1;

    if (test_noble_abel_zero_density_all_precisions() != 0)
        return 1;

    if (test_noble_abel_domain_and_output_clearing() != 0)
        return 1;

    if (test_noble_abel_nonzero_covolume_double() != 0)
        return 1;

    if (test_reduced_gas_ideal_limit_all_precisions() != 0)
        return 1;

    if (test_numerical_failure_output_clearing() != 0)
        return 1;

    if (test_virial_thermodynamic_consistency_double() != 0)
        return 1;

    if (test_virial_calibration_flag() != 0)
        return 1;

    if (test_virial_calibration_interval_boundaries() != 0)
        return 1;

    if (test_virial_temperature_domain_and_reference_semantics() != 0)
        return 1;

    if (test_virial_stability_rejection() != 0)
        return 1;

    if (test_virial_heat_capacity_rejection() != 0)
        return 1;

    return 0;
}
