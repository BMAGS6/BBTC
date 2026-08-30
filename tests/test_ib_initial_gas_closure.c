/**
 * @file
 * @brief Core contract tests for initial reduced-gas mass/density closure.
 */

#include <stddef.h>
#include <math.h>

#include <bbtc/bbtc.h>

static int
nearly_equal_double(double actual, double expected, double relative_tolerance)
{
    const double scale = fmax(fabs(actual), fabs(expected));

    if (scale == 0.0)
        return actual == expected;

    return fabs(actual - expected) <= relative_tolerance * scale;
}

static int
solution_is_clear_double(const bbtc_ib_initial_gas_solution_double_t* solution)
{
    return solution->applicability_flags == BBTC_APPLICABILITY_NONE_REPORTED
        && solution->density_kg_per_m3 == 0.0
        && solution->gas_mass_kg == 0.0;
}

static int
test_ideal_limit_all_precisions(void)
{
    static const float bf[] = {0.0f};
    static const double bd[] = {0.0};
    static const long double bl[] = {0.0L};
    const bbtc_ib_initial_gas_state_float_t sf =
    {
        .absolute_pressure_pa = 600.0f,
        .temperature_k = 3.0f
    };
    const bbtc_ib_initial_gas_state_double_t sd =
    {
        .absolute_pressure_pa = 600.0,
        .temperature_k = 3.0
    };
    const bbtc_ib_initial_gas_state_long_double_t sl =
    {
        .absolute_pressure_pa = 600.0L,
        .temperature_k = 3.0L
    };
    const bbtc_ib_noble_abel_gas_model_float_t nf =
    {
        .specific_gas_constant_j_per_kg_k = 100.0f,
        .constant_volume_specific_heat_j_per_kg_k = 500.0f,
        .covolume_m3_per_kg = 0.0f
    };
    const bbtc_ib_noble_abel_gas_model_double_t nd =
    {
        .specific_gas_constant_j_per_kg_k = 100.0,
        .constant_volume_specific_heat_j_per_kg_k = 500.0,
        .covolume_m3_per_kg = 0.0
    };
    const bbtc_ib_noble_abel_gas_model_long_double_t nl =
    {
        .specific_gas_constant_j_per_kg_k = 100.0L,
        .constant_volume_specific_heat_j_per_kg_k = 500.0L,
        .covolume_m3_per_kg = 0.0L
    };
    const bbtc_ib_first_order_virial_gas_model_float_t vf =
    {
        .specific_gas_constant_j_per_kg_k = 100.0f,
        .ideal_gas_constant_volume_specific_heat_j_per_kg_k = 500.0f,
        .minimum_calibrated_density_kg_per_m3 = 0.0f,
        .maximum_calibrated_density_kg_per_m3 = 5.0f,
        .second_density_virial_coefficient_law =
        {
            .minimum_temperature_k = 1.0f,
            .maximum_temperature_k = 5.0f,
            .second_density_virial_chebyshev_coefficients_m3_per_kg = bf,
            .coefficient_count = 1U
        }
    };
    const bbtc_ib_first_order_virial_gas_model_double_t vd =
    {
        .specific_gas_constant_j_per_kg_k = 100.0,
        .ideal_gas_constant_volume_specific_heat_j_per_kg_k = 500.0,
        .minimum_calibrated_density_kg_per_m3 = 0.0,
        .maximum_calibrated_density_kg_per_m3 = 5.0,
        .second_density_virial_coefficient_law =
        {
            .minimum_temperature_k = 1.0,
            .maximum_temperature_k = 5.0,
            .second_density_virial_chebyshev_coefficients_m3_per_kg = bd,
            .coefficient_count = 1U
        }
    };
    const bbtc_ib_first_order_virial_gas_model_long_double_t vl =
    {
        .specific_gas_constant_j_per_kg_k = 100.0L,
        .ideal_gas_constant_volume_specific_heat_j_per_kg_k = 500.0L,
        .minimum_calibrated_density_kg_per_m3 = 0.0L,
        .maximum_calibrated_density_kg_per_m3 = 5.0L,
        .second_density_virial_coefficient_law =
        {
            .minimum_temperature_k = 1.0L,
            .maximum_temperature_k = 5.0L,
            .second_density_virial_chebyshev_coefficients_m3_per_kg = bl,
            .coefficient_count = 1U
        }
    };
    bbtc_ib_initial_gas_solution_float_t anf = {0};
    bbtc_ib_initial_gas_solution_float_t avf = {0};
    bbtc_ib_initial_gas_solution_double_t and = {0};
    bbtc_ib_initial_gas_solution_double_t avd = {0};
    bbtc_ib_initial_gas_solution_long_double_t anl = {0};
    bbtc_ib_initial_gas_solution_long_double_t avl = {0};

    if (bbtc_ib_noble_abel_initial_gas_solve_float(&nf, &sf, 0.25f, &anf) != BBTC_STATUS_SUCCESS ||
        bbtc_ib_first_order_virial_initial_gas_solve_float(&vf, &sf, 0.25f, &avf) != BBTC_STATUS_SUCCESS ||
        bbtc_ib_noble_abel_initial_gas_solve_double(&nd, &sd, 0.25, &and) != BBTC_STATUS_SUCCESS ||
        bbtc_ib_first_order_virial_initial_gas_solve_double(&vd, &sd, 0.25, &avd) != BBTC_STATUS_SUCCESS ||
        bbtc_ib_noble_abel_initial_gas_solve_long_double(&nl, &sl, 0.25L, &anl) != BBTC_STATUS_SUCCESS ||
        bbtc_ib_first_order_virial_initial_gas_solve_long_double(&vl, &sl, 0.25L, &avl) != BBTC_STATUS_SUCCESS)
        return 1;

    if (anf.density_kg_per_m3 != 2.0f || anf.gas_mass_kg != 0.5f ||
        avf.density_kg_per_m3 != 2.0f || avf.gas_mass_kg != 0.5f ||
        and.density_kg_per_m3 != 2.0 || and.gas_mass_kg != 0.5 ||
        avd.density_kg_per_m3 != 2.0 || avd.gas_mass_kg != 0.5 ||
        anl.density_kg_per_m3 != 2.0L || anl.gas_mass_kg != 0.5L ||
        avl.density_kg_per_m3 != 2.0L || avl.gas_mass_kg != 0.5L)
        return 1;

    return 0;
}

static int
test_noble_covolume_and_scaled_ratio(void)
{
    const bbtc_ib_noble_abel_gas_model_double_t ordinary =
    {
        .specific_gas_constant_j_per_kg_k = 100.0,
        .constant_volume_specific_heat_j_per_kg_k = 500.0,
        .covolume_m3_per_kg = 0.1
    };
    const bbtc_ib_initial_gas_state_double_t state =
    {
        .absolute_pressure_pa = 600.0,
        .temperature_k = 3.0
    };
    const bbtc_ib_noble_abel_gas_model_double_t scaled =
    {
        .specific_gas_constant_j_per_kg_k = 1.0e200,
        .constant_volume_specific_heat_j_per_kg_k = 1.0,
        .covolume_m3_per_kg = 0.0
    };
    const bbtc_ib_initial_gas_state_double_t scaled_state =
    {
        .absolute_pressure_pa = 1.0e300,
        .temperature_k = 1.0e200
    };
    bbtc_ib_initial_gas_solution_double_t solution = {0};

    if (bbtc_ib_noble_abel_initial_gas_solve_double(&ordinary, &state, 0.3, &solution) != BBTC_STATUS_SUCCESS ||
        !nearly_equal_double(solution.density_kg_per_m3, 5.0 / 3.0, 1.0e-13) ||
        !nearly_equal_double(solution.gas_mass_kg, 0.5, 1.0e-13))
        return 1;

    if (bbtc_ib_noble_abel_initial_gas_solve_double(&scaled, &scaled_state, 1.0e100, &solution) != BBTC_STATUS_SUCCESS ||
        !nearly_equal_double(solution.density_kg_per_m3, 1.0e-100, 1.0e-12) ||
        !nearly_equal_double(solution.gas_mass_kg, 1.0, 1.0e-12))
        return 1;

    return 0;
}

static int
test_virial_branches(void)
{
    static const double bp[] = {0.1};
    static const double bn[] = {-0.1};
    bbtc_ib_first_order_virial_gas_model_double_t model =
    {
        .specific_gas_constant_j_per_kg_k = 100.0,
        .ideal_gas_constant_volume_specific_heat_j_per_kg_k = 500.0,
        .minimum_calibrated_density_kg_per_m3 = 0.0,
        .maximum_calibrated_density_kg_per_m3 = 5.0,
        .second_density_virial_coefficient_law =
        {
            .minimum_temperature_k = 1.0,
            .maximum_temperature_k = 5.0,
            .second_density_virial_chebyshev_coefficients_m3_per_kg = bp,
            .coefficient_count = 1U
        }
    };
    const bbtc_ib_initial_gas_state_double_t state =
    {
        .absolute_pressure_pa = 600.0,
        .temperature_k = 3.0
    };
    bbtc_ib_initial_gas_solution_double_t solution = {0};
    double expected;

    expected = 2.0 / (0.5 + sqrt(0.25 + 0.2));
    if (bbtc_ib_first_order_virial_initial_gas_solve_double(&model, &state, 0.5, &solution) != BBTC_STATUS_SUCCESS ||
        !nearly_equal_double(solution.density_kg_per_m3, expected, 1.0e-13))
        return 1;

    model.second_density_virial_coefficient_law.second_density_virial_chebyshev_coefficients_m3_per_kg = bn;
    expected = 2.0 / ((1.0 + sqrt(0.2)) * 0.5);
    if (bbtc_ib_first_order_virial_initial_gas_solve_double(&model, &state, 0.25, &solution) != BBTC_STATUS_SUCCESS ||
        !nearly_equal_double(solution.density_kg_per_m3, expected, 1.0e-13) ||
        !(1.0 + 2.0 * (-0.1) * solution.density_kg_per_m3 > 0.0))
        return 1;

    return 0;
}

static int
test_virial_extreme_positive_product(void)
{
    static const double b[] = {1.0e200};
    const bbtc_ib_first_order_virial_gas_model_double_t model =
    {
        .specific_gas_constant_j_per_kg_k = 1.0,
        .ideal_gas_constant_volume_specific_heat_j_per_kg_k = 1.0,
        .minimum_calibrated_density_kg_per_m3 = 0.0,
        .maximum_calibrated_density_kg_per_m3 = 2.0,
        .second_density_virial_coefficient_law =
        {
            .minimum_temperature_k = 0.5,
            .maximum_temperature_k = 2.0,
            .second_density_virial_chebyshev_coefficients_m3_per_kg = b,
            .coefficient_count = 1U
        }
    };
    const bbtc_ib_initial_gas_state_double_t state =
    {
        .absolute_pressure_pa = 1.0e200,
        .temperature_k = 1.0
    };
    bbtc_ib_initial_gas_solution_double_t solution = {0};

    if (bbtc_ib_first_order_virial_initial_gas_solve_double(&model, &state, 2.0, &solution) != BBTC_STATUS_SUCCESS ||
        !nearly_equal_double(solution.density_kg_per_m3, 1.0, 1.0e-12) ||
        !nearly_equal_double(solution.gas_mass_kg, 2.0, 1.0e-12))
        return 1;

    return 0;
}

static int
test_domains_flags_and_clearing(void)
{
    static const double boundary[] = {-0.25};
    static const double zero[] = {0.0};
    bbtc_ib_first_order_virial_gas_model_double_t model =
    {
        .specific_gas_constant_j_per_kg_k = 100.0,
        .ideal_gas_constant_volume_specific_heat_j_per_kg_k = 500.0,
        .minimum_calibrated_density_kg_per_m3 = 0.0,
        .maximum_calibrated_density_kg_per_m3 = 1.0,
        .second_density_virial_coefficient_law =
        {
            .minimum_temperature_k = 0.5,
            .maximum_temperature_k = 2.0,
            .second_density_virial_chebyshev_coefficients_m3_per_kg = boundary,
            .coefficient_count = 1U
        }
    };
    const bbtc_ib_initial_gas_state_double_t boundary_state =
    {
        .absolute_pressure_pa = 100.0,
        .temperature_k = 1.0
    };
    const bbtc_ib_initial_gas_state_double_t flag_state =
    {
        .absolute_pressure_pa = 300.0,
        .temperature_k = 1.5
    };
    bbtc_ib_initial_gas_solution_double_t solution =
    {
        .applicability_flags = BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN,
        .density_kg_per_m3 = 123.0,
        .gas_mass_kg = 456.0
    };

    if (bbtc_ib_first_order_virial_initial_gas_solve_double(&model, &boundary_state, 1.0, &solution) != BBTC_STATUS_OUTSIDE_DOMAIN ||
        !solution_is_clear_double(&solution))
        return 1;

    model.second_density_virial_coefficient_law.second_density_virial_chebyshev_coefficients_m3_per_kg = zero;
    if (bbtc_ib_first_order_virial_initial_gas_solve_double(&model, &flag_state, 0.25, &solution) != BBTC_STATUS_SUCCESS ||
        solution.applicability_flags != BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN ||
        !nearly_equal_double(solution.density_kg_per_m3, 2.0, 1.0e-13))
        return 1;

    solution.density_kg_per_m3 = 123.0;
    solution.gas_mass_kg = 456.0;
    if (bbtc_ib_first_order_virial_initial_gas_solve_double(&model, &flag_state, NAN, &solution) != BBTC_STATUS_NAN_INPUT ||
        !solution_is_clear_double(&solution))
        return 1;

    solution.density_kg_per_m3 = 123.0;
    solution.gas_mass_kg = 456.0;
    if (bbtc_ib_first_order_virial_initial_gas_solve_double(&model, &flag_state, INFINITY, &solution) != BBTC_STATUS_NONFINITE_INPUT ||
        !solution_is_clear_double(&solution))
        return 1;

    return 0;
}

int main(void)
{
    if (test_ideal_limit_all_precisions() != 0)
        return 1;
    if (test_noble_covolume_and_scaled_ratio() != 0)
        return 1;
    if (test_virial_branches() != 0)
        return 1;
    if (test_virial_extreme_positive_product() != 0)
        return 1;
    if (test_domains_flags_and_clearing() != 0)
        return 1;
    return 0;
}
