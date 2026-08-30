/**
 * @file
 * @brief Tests canonical propellant-grain regression geometry.
 */
#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>

#include <bbtc/internal_ballistics/propellant_grain_geometry.h>

static const float test_pi_f = 3.14159265358979323846f;
static const double test_pi_d = 3.141592653589793238462643383279502884;
static const long double test_pi_ld = 3.141592653589793238462643383279502884L;

static int nearly_equal_float(const float actual,
                              const float expected,
                              const float relative_tolerance)
{
    const float scale = fmaxf(1.0f, fabsf(expected));
    return fabsf(actual - expected) <= relative_tolerance * scale;
}

static int nearly_equal_double(const double actual,
                               const double expected,
                               const double relative_tolerance)
{
    const double scale = fmax(1.0, fabs(expected));
    return fabs(actual - expected) <= relative_tolerance * scale;
}

static int nearly_equal_long_double(const long double actual,
                                    const long double expected,
                                    const long double relative_tolerance)
{
    const long double scale = fmaxl(1.0L, fabsl(expected));
    return fabsl(actual - expected) <= relative_tolerance * scale;
}

static int state_is_zero_double(const bbtc_ib_propellant_grain_state_double_t* const state)
{
    return state->remaining_volume_m3 == 0.0
        && state->burning_surface_area_m2 == 0.0
        && state->remaining_regression_to_burnout_m == 0.0
        && state->consumed_volume_fraction == 0.0;
}

static int state_is_burnout_double(const bbtc_ib_propellant_grain_state_double_t* const state)
{
    return state->remaining_volume_m3 == 0.0
        && state->burning_surface_area_m2 == 0.0
        && state->remaining_regression_to_burnout_m == 0.0
        && state->consumed_volume_fraction == 1.0;
}

static int state_is_zero_float(const bbtc_ib_propellant_grain_state_float_t* const state)
{
    return state->remaining_volume_m3 == 0.0f
        && state->burning_surface_area_m2 == 0.0f
        && state->remaining_regression_to_burnout_m == 0.0f
        && state->consumed_volume_fraction == 0.0f;
}

static int test_spherical_double(void)
{
    const bbtc_ib_spherical_grain_geometry_double_t geometry =
    {
        .initial_radius_m = 2.0
    };
    bbtc_ib_propellant_grain_state_double_t state = {0};

    if (bbtc_ib_spherical_grain_geometry_validate_double(&geometry)
        != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (bbtc_ib_spherical_grain_evaluate_double(&geometry, 0.5, &state)
        != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (!nearly_equal_double(state.remaining_volume_m3, 4.5 * test_pi_d, 1.0e-14)
        || !nearly_equal_double(state.burning_surface_area_m2, 9.0 * test_pi_d, 1.0e-14)
        || state.remaining_regression_to_burnout_m != 1.5
        || state.consumed_volume_fraction != 0.578125)
    {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

static int test_solid_cylinder_double(void)
{
    const bbtc_ib_solid_cylindrical_grain_geometry_double_t geometry =
    {
        .initial_radius_m = 2.0,
        .initial_length_m = 6.0
    };
    bbtc_ib_propellant_grain_state_double_t state = {0};

    if (bbtc_ib_solid_cylindrical_grain_evaluate_double(&geometry, 1.0, &state)
        != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (!nearly_equal_double(state.remaining_volume_m3, 4.0 * test_pi_d, 1.0e-14)
        || !nearly_equal_double(state.burning_surface_area_m2, 10.0 * test_pi_d, 1.0e-14)
        || state.remaining_regression_to_burnout_m != 1.0
        || !nearly_equal_double(state.consumed_volume_fraction, 5.0 / 6.0, 1.0e-14))
    {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

static int test_rectangular_prism_double(void)
{
    const bbtc_ib_rectangular_prismatic_grain_geometry_double_t geometry =
    {
        .initial_length_m = 6.0,
        .initial_width_m = 4.0,
        .initial_thickness_m = 2.0
    };
    bbtc_ib_propellant_grain_state_double_t state = {0};

    if (bbtc_ib_rectangular_prismatic_grain_evaluate_double(&geometry, 0.5, &state)
        != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (state.remaining_volume_m3 != 15.0
        || state.burning_surface_area_m2 != 46.0
        || state.remaining_regression_to_burnout_m != 0.5
        || state.consumed_volume_fraction != 0.6875)
    {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

static int test_single_perforated_double(void)
{
    const bbtc_ib_single_perforated_cylindrical_grain_geometry_double_t geometry =
    {
        .initial_outer_radius_m = 3.0,
        .initial_inner_radius_m = 1.0,
        .initial_length_m = 8.0
    };
    bbtc_ib_propellant_grain_state_double_t state = {0};

    if (bbtc_ib_single_perforated_cylindrical_grain_evaluate_double(&geometry, 0.5, &state)
        != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (!nearly_equal_double(state.remaining_volume_m3, 28.0 * test_pi_d, 1.0e-14)
        || !nearly_equal_double(state.burning_surface_area_m2, 64.0 * test_pi_d, 1.0e-14)
        || state.remaining_regression_to_burnout_m != 0.5
        || state.consumed_volume_fraction != 0.5625)
    {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

static int test_initial_states_double(void)
{
    const bbtc_ib_spherical_grain_geometry_double_t sphere = { .initial_radius_m = 2.0 };
    const bbtc_ib_solid_cylindrical_grain_geometry_double_t cylinder =
    {
        .initial_radius_m = 2.0,
        .initial_length_m = 6.0
    };
    const bbtc_ib_rectangular_prismatic_grain_geometry_double_t prism =
    {
        .initial_length_m = 6.0,
        .initial_width_m = 4.0,
        .initial_thickness_m = 2.0
    };
    const bbtc_ib_single_perforated_cylindrical_grain_geometry_double_t tube =
    {
        .initial_outer_radius_m = 3.0,
        .initial_inner_radius_m = 1.0,
        .initial_length_m = 8.0
    };
    bbtc_ib_propellant_grain_state_double_t state = {0};

    if (bbtc_ib_spherical_grain_evaluate_double(&sphere, 0.0, &state)
            != BBTC_STATUS_SUCCESS
        || !nearly_equal_double(state.remaining_volume_m3,
                                (32.0 / 3.0) * test_pi_d,
                                1.0e-14)
        || !nearly_equal_double(state.burning_surface_area_m2,
                                16.0 * test_pi_d,
                                1.0e-14)
        || state.remaining_regression_to_burnout_m != 2.0
        || state.consumed_volume_fraction != 0.0)
    {
        return EXIT_FAILURE;
    }

    if (bbtc_ib_solid_cylindrical_grain_evaluate_double(&cylinder, 0.0, &state)
            != BBTC_STATUS_SUCCESS
        || !nearly_equal_double(state.remaining_volume_m3, 24.0 * test_pi_d, 1.0e-14)
        || !nearly_equal_double(state.burning_surface_area_m2, 32.0 * test_pi_d, 1.0e-14)
        || state.remaining_regression_to_burnout_m != 2.0
        || state.consumed_volume_fraction != 0.0)
    {
        return EXIT_FAILURE;
    }

    if (bbtc_ib_rectangular_prismatic_grain_evaluate_double(&prism, 0.0, &state)
            != BBTC_STATUS_SUCCESS
        || state.remaining_volume_m3 != 48.0
        || state.burning_surface_area_m2 != 88.0
        || state.remaining_regression_to_burnout_m != 1.0
        || state.consumed_volume_fraction != 0.0)
    {
        return EXIT_FAILURE;
    }

    if (bbtc_ib_single_perforated_cylindrical_grain_evaluate_double(&tube, 0.0, &state)
            != BBTC_STATUS_SUCCESS
        || !nearly_equal_double(state.remaining_volume_m3, 64.0 * test_pi_d, 1.0e-14)
        || !nearly_equal_double(state.burning_surface_area_m2, 80.0 * test_pi_d, 1.0e-14)
        || state.remaining_regression_to_burnout_m != 1.0
        || state.consumed_volume_fraction != 0.0)
    {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

static int test_exact_burnout_and_overshoot_double(void)
{
    const bbtc_ib_spherical_grain_geometry_double_t sphere = { .initial_radius_m = 2.0 };
    const bbtc_ib_solid_cylindrical_grain_geometry_double_t cylinder =
    {
        .initial_radius_m = 2.0,
        .initial_length_m = 6.0
    };
    const bbtc_ib_rectangular_prismatic_grain_geometry_double_t prism =
    {
        .initial_length_m = 6.0,
        .initial_width_m = 4.0,
        .initial_thickness_m = 2.0
    };
    const bbtc_ib_single_perforated_cylindrical_grain_geometry_double_t tube =
    {
        .initial_outer_radius_m = 3.0,
        .initial_inner_radius_m = 1.0,
        .initial_length_m = 8.0
    };
    bbtc_ib_propellant_grain_state_double_t state = {0};

    if (bbtc_ib_spherical_grain_evaluate_double(&sphere, 2.0, &state)
            != BBTC_STATUS_SUCCESS
        || !state_is_burnout_double(&state)
        || bbtc_ib_solid_cylindrical_grain_evaluate_double(&cylinder, 2.0, &state)
            != BBTC_STATUS_SUCCESS
        || !state_is_burnout_double(&state)
        || bbtc_ib_rectangular_prismatic_grain_evaluate_double(&prism, 1.0, &state)
            != BBTC_STATUS_SUCCESS
        || !state_is_burnout_double(&state)
        || bbtc_ib_single_perforated_cylindrical_grain_evaluate_double(&tube, 1.0, &state)
            != BBTC_STATUS_SUCCESS
        || !state_is_burnout_double(&state))
    {
        return EXIT_FAILURE;
    }

    state = (bbtc_ib_propellant_grain_state_double_t)
    {
        .remaining_volume_m3 = 1.0,
        .burning_surface_area_m2 = 1.0,
        .remaining_regression_to_burnout_m = 1.0,
        .consumed_volume_fraction = 1.0
    };

    if (bbtc_ib_spherical_grain_evaluate_double(&sphere, 2.1, &state)
            != BBTC_STATUS_OUTSIDE_DOMAIN
        || !state_is_zero_double(&state))
    {
        return EXIT_FAILURE;
    }

    state.remaining_volume_m3 = 1.0;
    if (bbtc_ib_solid_cylindrical_grain_evaluate_double(&cylinder, 2.1, &state)
            != BBTC_STATUS_OUTSIDE_DOMAIN
        || !state_is_zero_double(&state))
    {
        return EXIT_FAILURE;
    }

    state.remaining_volume_m3 = 1.0;
    if (bbtc_ib_rectangular_prismatic_grain_evaluate_double(&prism, 1.1, &state)
            != BBTC_STATUS_OUTSIDE_DOMAIN
        || !state_is_zero_double(&state))
    {
        return EXIT_FAILURE;
    }

    state.remaining_volume_m3 = 1.0;
    if (bbtc_ib_single_perforated_cylindrical_grain_evaluate_double(&tube, 1.1, &state)
            != BBTC_STATUS_OUTSIDE_DOMAIN
        || !state_is_zero_double(&state))
    {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

static int test_limiting_burnout_modes_double(void)
{
    const bbtc_ib_solid_cylindrical_grain_geometry_double_t radial_cylinder =
    {
        .initial_radius_m = 1.0,
        .initial_length_m = 10.0
    };
    const bbtc_ib_solid_cylindrical_grain_geometry_double_t axial_cylinder =
    {
        .initial_radius_m = 5.0,
        .initial_length_m = 2.0
    };
    const bbtc_ib_single_perforated_cylindrical_grain_geometry_double_t radial_tube =
    {
        .initial_outer_radius_m = 3.0,
        .initial_inner_radius_m = 1.0,
        .initial_length_m = 10.0
    };
    const bbtc_ib_single_perforated_cylindrical_grain_geometry_double_t axial_tube =
    {
        .initial_outer_radius_m = 5.0,
        .initial_inner_radius_m = 1.0,
        .initial_length_m = 2.0
    };
    bbtc_ib_propellant_grain_state_double_t state = {0};

    if (bbtc_ib_solid_cylindrical_grain_evaluate_double(&radial_cylinder, 1.0, &state)
            != BBTC_STATUS_SUCCESS
        || !state_is_burnout_double(&state)
        || bbtc_ib_solid_cylindrical_grain_evaluate_double(&axial_cylinder, 1.0, &state)
            != BBTC_STATUS_SUCCESS
        || !state_is_burnout_double(&state)
        || bbtc_ib_single_perforated_cylindrical_grain_evaluate_double(&radial_tube, 1.0, &state)
            != BBTC_STATUS_SUCCESS
        || !state_is_burnout_double(&state)
        || bbtc_ib_single_perforated_cylindrical_grain_evaluate_double(&axial_tube, 1.0, &state)
            != BBTC_STATUS_SUCCESS
        || !state_is_burnout_double(&state))
    {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

static int test_validation_and_failure_order(void)
{
    const bbtc_ib_spherical_grain_geometry_double_t zero_radius =
    {
        .initial_radius_m = 0.0
    };
    const bbtc_ib_spherical_grain_geometry_double_t nan_radius =
    {
        .initial_radius_m = NAN
    };
    const bbtc_ib_single_perforated_cylindrical_grain_geometry_double_t invalid_tube =
    {
        .initial_outer_radius_m = 1.0,
        .initial_inner_radius_m = 1.0,
        .initial_length_m = 2.0
    };
    const bbtc_ib_solid_cylindrical_grain_geometry_double_t cylinder =
    {
        .initial_radius_m = 1.0,
        .initial_length_m = 4.0
    };
    const bbtc_ib_rectangular_prismatic_grain_geometry_double_t prism =
    {
        .initial_length_m = 4.0,
        .initial_width_m = 3.0,
        .initial_thickness_m = 2.0
    };
    const bbtc_ib_single_perforated_cylindrical_grain_geometry_double_t tube =
    {
        .initial_outer_radius_m = 2.0,
        .initial_inner_radius_m = 1.0,
        .initial_length_m = 4.0
    };
    bbtc_ib_propellant_grain_state_double_t state = {0};

    if (bbtc_ib_spherical_grain_geometry_validate_double(NULL)
            != BBTC_STATUS_INVALID_ARGUMENT
        || bbtc_ib_spherical_grain_geometry_validate_double(&zero_radius)
            != BBTC_STATUS_OUTSIDE_DOMAIN
        || bbtc_ib_spherical_grain_geometry_validate_double(&nan_radius)
            != BBTC_STATUS_NAN_INPUT
        || bbtc_ib_single_perforated_cylindrical_grain_geometry_validate_double(&invalid_tube)
            != BBTC_STATUS_INCONSISTENT_CONFIGURATION)
    {
        return EXIT_FAILURE;
    }

    if (bbtc_ib_spherical_grain_evaluate_double(&zero_radius, NAN, &state)
            != BBTC_STATUS_OUTSIDE_DOMAIN
        || !state_is_zero_double(&state)
        || bbtc_ib_spherical_grain_evaluate_double(&nan_radius, 0.0, &state)
            != BBTC_STATUS_NAN_INPUT
        || !state_is_zero_double(&state))
    {
        return EXIT_FAILURE;
    }

    if (bbtc_ib_spherical_grain_evaluate_double(NULL, 0.0, &state)
            != BBTC_STATUS_INVALID_ARGUMENT
        || !state_is_zero_double(&state)
        || bbtc_ib_spherical_grain_evaluate_double(&zero_radius, 0.0, NULL)
            != BBTC_STATUS_INVALID_ARGUMENT)
    {
        return EXIT_FAILURE;
    }

    if (bbtc_ib_spherical_grain_evaluate_double(&(bbtc_ib_spherical_grain_geometry_double_t){1.0},
                                                 -0.1,
                                                 &state)
            != BBTC_STATUS_OUTSIDE_DOMAIN
        || !state_is_zero_double(&state)
        || bbtc_ib_solid_cylindrical_grain_evaluate_double(&cylinder, NAN, &state)
            != BBTC_STATUS_NAN_INPUT
        || !state_is_zero_double(&state)
        || bbtc_ib_rectangular_prismatic_grain_evaluate_double(&prism, INFINITY, &state)
            != BBTC_STATUS_NONFINITE_INPUT
        || !state_is_zero_double(&state)
        || bbtc_ib_single_perforated_cylindrical_grain_evaluate_double(&tube, -0.1, &state)
            != BBTC_STATUS_OUTSIDE_DOMAIN
        || !state_is_zero_double(&state))
    {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

static int test_validator_nonfinite_precedence(void)
{
    const bbtc_ib_spherical_grain_geometry_float_t sphere_nan_f =
    {
        .initial_radius_m = NAN
    };

    const bbtc_ib_spherical_grain_geometry_long_double_t sphere_nan_ld =
    {
        .initial_radius_m = NAN
    };

    const bbtc_ib_spherical_grain_geometry_long_double_t sphere_infinity_ld =
    {
        .initial_radius_m = INFINITY
    };

    bbtc_ib_solid_cylindrical_grain_geometry_double_t cylinder =
    {
        .initial_radius_m = INFINITY,
        .initial_length_m = NAN
    };

    bbtc_ib_rectangular_prismatic_grain_geometry_double_t prism =
    {
        .initial_length_m = INFINITY,
        .initial_width_m = 1.0,
        .initial_thickness_m = NAN
    };

    bbtc_ib_single_perforated_cylindrical_grain_geometry_double_t tube =
    {
        .initial_outer_radius_m = INFINITY,
        .initial_inner_radius_m = 1.0,
        .initial_length_m = NAN
    };

    if (bbtc_ib_spherical_grain_geometry_validate_float(&sphere_nan_f)
            != BBTC_STATUS_NAN_INPUT
        || bbtc_ib_spherical_grain_geometry_validate_long_double(&sphere_nan_ld)
            != BBTC_STATUS_NAN_INPUT
        || bbtc_ib_spherical_grain_geometry_validate_long_double(
               &sphere_infinity_ld
           ) != BBTC_STATUS_NONFINITE_INPUT
        || bbtc_ib_solid_cylindrical_grain_geometry_validate_double(&cylinder)
            != BBTC_STATUS_NAN_INPUT
        || bbtc_ib_rectangular_prismatic_grain_geometry_validate_double(&prism)
            != BBTC_STATUS_NAN_INPUT
        || bbtc_ib_single_perforated_cylindrical_grain_geometry_validate_double(
               &tube
           ) != BBTC_STATUS_NAN_INPUT)
    {
        return EXIT_FAILURE;
    }

    cylinder.initial_radius_m = NAN;
    cylinder.initial_length_m = INFINITY;

    prism.initial_length_m = NAN;
    prism.initial_thickness_m = INFINITY;

    tube.initial_outer_radius_m = NAN;
    tube.initial_length_m = INFINITY;

    if (bbtc_ib_solid_cylindrical_grain_geometry_validate_double(&cylinder)
            != BBTC_STATUS_NAN_INPUT
        || bbtc_ib_rectangular_prismatic_grain_geometry_validate_double(&prism)
            != BBTC_STATUS_NAN_INPUT
        || bbtc_ib_single_perforated_cylindrical_grain_geometry_validate_double(
               &tube
           ) != BBTC_STATUS_NAN_INPUT)
    {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}


static int test_native_precision_analytical_results(void)
{
    const bbtc_ib_spherical_grain_geometry_float_t sphere_f = { .initial_radius_m = 2.0f };
    const bbtc_ib_spherical_grain_geometry_long_double_t sphere_ld = { .initial_radius_m = 2.0L };
    const bbtc_ib_solid_cylindrical_grain_geometry_float_t cylinder_f =
    {
        .initial_radius_m = 2.0f,
        .initial_length_m = 6.0f
    };
    const bbtc_ib_solid_cylindrical_grain_geometry_long_double_t cylinder_ld =
    {
        .initial_radius_m = 2.0L,
        .initial_length_m = 6.0L
    };
    const bbtc_ib_rectangular_prismatic_grain_geometry_float_t prism_f =
    {
        .initial_length_m = 6.0f,
        .initial_width_m = 4.0f,
        .initial_thickness_m = 2.0f
    };
    const bbtc_ib_rectangular_prismatic_grain_geometry_long_double_t prism_ld =
    {
        .initial_length_m = 6.0L,
        .initial_width_m = 4.0L,
        .initial_thickness_m = 2.0L
    };
    const bbtc_ib_single_perforated_cylindrical_grain_geometry_float_t tube_f =
    {
        .initial_outer_radius_m = 3.0f,
        .initial_inner_radius_m = 1.0f,
        .initial_length_m = 8.0f
    };
    const bbtc_ib_single_perforated_cylindrical_grain_geometry_long_double_t tube_ld =
    {
        .initial_outer_radius_m = 3.0L,
        .initial_inner_radius_m = 1.0L,
        .initial_length_m = 8.0L
    };
    bbtc_ib_propellant_grain_state_float_t state_f = {0};
    bbtc_ib_propellant_grain_state_long_double_t state_ld = {0};
    const float tol_f = 64.0f * FLT_EPSILON;
    const long double tol_ld = 64.0L * LDBL_EPSILON;

    if (bbtc_ib_spherical_grain_evaluate_float(&sphere_f, 0.5f, &state_f)
            != BBTC_STATUS_SUCCESS
        || !nearly_equal_float(state_f.remaining_volume_m3, 4.5f * test_pi_f, tol_f)
        || !nearly_equal_float(state_f.burning_surface_area_m2, 9.0f * test_pi_f, tol_f)
        || !nearly_equal_float(state_f.consumed_volume_fraction, 0.578125f, tol_f)
        || bbtc_ib_solid_cylindrical_grain_evaluate_float(&cylinder_f, 1.0f, &state_f)
            != BBTC_STATUS_SUCCESS
        || !nearly_equal_float(state_f.remaining_volume_m3, 4.0f * test_pi_f, tol_f)
        || !nearly_equal_float(state_f.burning_surface_area_m2, 10.0f * test_pi_f, tol_f)
        || bbtc_ib_rectangular_prismatic_grain_evaluate_float(&prism_f, 0.5f, &state_f)
            != BBTC_STATUS_SUCCESS
        || state_f.remaining_volume_m3 != 15.0f
        || state_f.burning_surface_area_m2 != 46.0f
        || bbtc_ib_single_perforated_cylindrical_grain_evaluate_float(&tube_f, 0.5f, &state_f)
            != BBTC_STATUS_SUCCESS
        || !nearly_equal_float(state_f.remaining_volume_m3, 28.0f * test_pi_f, tol_f)
        || !nearly_equal_float(state_f.burning_surface_area_m2, 64.0f * test_pi_f, tol_f))
    {
        return EXIT_FAILURE;
    }

    if (bbtc_ib_spherical_grain_evaluate_long_double(&sphere_ld, 0.5L, &state_ld)
            != BBTC_STATUS_SUCCESS
        || !nearly_equal_long_double(state_ld.remaining_volume_m3, 4.5L * test_pi_ld, tol_ld)
        || !nearly_equal_long_double(state_ld.burning_surface_area_m2, 9.0L * test_pi_ld, tol_ld)
        || !nearly_equal_long_double(state_ld.consumed_volume_fraction, 0.578125L, tol_ld)
        || bbtc_ib_solid_cylindrical_grain_evaluate_long_double(&cylinder_ld, 1.0L, &state_ld)
            != BBTC_STATUS_SUCCESS
        || !nearly_equal_long_double(state_ld.remaining_volume_m3, 4.0L * test_pi_ld, tol_ld)
        || !nearly_equal_long_double(state_ld.burning_surface_area_m2, 10.0L * test_pi_ld, tol_ld)
        || bbtc_ib_rectangular_prismatic_grain_evaluate_long_double(&prism_ld, 0.5L, &state_ld)
            != BBTC_STATUS_SUCCESS
        || state_ld.remaining_volume_m3 != 15.0L
        || state_ld.burning_surface_area_m2 != 46.0L
        || bbtc_ib_single_perforated_cylindrical_grain_evaluate_long_double(&tube_ld, 0.5L, &state_ld)
            != BBTC_STATUS_SUCCESS
        || !nearly_equal_long_double(state_ld.remaining_volume_m3, 28.0L * test_pi_ld, tol_ld)
        || !nearly_equal_long_double(state_ld.burning_surface_area_m2, 64.0L * test_pi_ld, tol_ld))
    {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

static int test_monotonic_progression_double(void)
{
    const bbtc_ib_spherical_grain_geometry_double_t geometry = { .initial_radius_m = 2.0 };
    bbtc_ib_propellant_grain_state_double_t previous = {0};
    bbtc_ib_propellant_grain_state_double_t current = {0};
    const double samples[] = {0.25, 0.5, 1.0, 1.5};
    size_t index;

    if (bbtc_ib_spherical_grain_evaluate_double(&geometry, 0.0, &previous)
        != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    for (index = 0U; index < sizeof(samples) / sizeof(samples[0]); ++index)
    {
        if (bbtc_ib_spherical_grain_evaluate_double(&geometry, samples[index], &current)
                != BBTC_STATUS_SUCCESS
            || current.remaining_volume_m3 >= previous.remaining_volume_m3
            || current.remaining_regression_to_burnout_m
                >= previous.remaining_regression_to_burnout_m
            || current.consumed_volume_fraction <= previous.consumed_volume_fraction)
        {
            return EXIT_FAILURE;
        }

        previous = current;
    }

    return EXIT_SUCCESS;
}

static int test_derived_regression_underflow(void)
{
    const bbtc_ib_solid_cylindrical_grain_geometry_float_t tiny_length_cylinder =
    {
        .initial_radius_m = 1.0f,
        .initial_length_m = FLT_TRUE_MIN
    };
    const bbtc_ib_rectangular_prismatic_grain_geometry_float_t tiny_thickness_prism =
    {
        .initial_length_m = 1.0f,
        .initial_width_m = 1.0f,
        .initial_thickness_m = FLT_TRUE_MIN
    };
    const bbtc_ib_single_perforated_cylindrical_grain_geometry_float_t tiny_radial_web_tube =
    {
        .initial_outer_radius_m = 2.0f * FLT_TRUE_MIN,
        .initial_inner_radius_m = FLT_TRUE_MIN,
        .initial_length_m = 1.0f
    };
    bbtc_ib_propellant_grain_state_float_t state = {0};

    if (bbtc_ib_solid_cylindrical_grain_geometry_validate_float(&tiny_length_cylinder)
            != BBTC_STATUS_SUCCESS
        || bbtc_ib_solid_cylindrical_grain_evaluate_float(&tiny_length_cylinder,
                                                           0.0f,
                                                           &state)
            != BBTC_STATUS_NUMERICAL_FAILURE
        || !state_is_zero_float(&state)
        || bbtc_ib_rectangular_prismatic_grain_geometry_validate_float(&tiny_thickness_prism)
            != BBTC_STATUS_SUCCESS
        || bbtc_ib_rectangular_prismatic_grain_evaluate_float(&tiny_thickness_prism,
                                                               0.0f,
                                                               &state)
            != BBTC_STATUS_NUMERICAL_FAILURE
        || !state_is_zero_float(&state)
        || bbtc_ib_single_perforated_cylindrical_grain_geometry_validate_float(&tiny_radial_web_tube)
            != BBTC_STATUS_SUCCESS
        || bbtc_ib_single_perforated_cylindrical_grain_evaluate_float(&tiny_radial_web_tube,
                                                                       0.0f,
                                                                       &state)
            != BBTC_STATUS_NUMERICAL_FAILURE
        || !state_is_zero_float(&state))
    {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

static int test_numerical_failure(void)
{
    const bbtc_ib_spherical_grain_geometry_double_t huge =
    {
        .initial_radius_m = DBL_MAX / 4.0
    };
    bbtc_ib_propellant_grain_state_double_t state = {0};

    if (bbtc_ib_spherical_grain_evaluate_double(&huge, 0.0, &state)
        != BBTC_STATUS_NUMERICAL_FAILURE)
    {
        return EXIT_FAILURE;
    }

    return state_is_zero_double(&state) ? EXIT_SUCCESS : EXIT_FAILURE;
}

/**
 * @brief Runs the IB0.4b propellant-grain regression geometry tests.
 *
 * @return `EXIT_SUCCESS` when every test passes; otherwise `EXIT_FAILURE`.
 */
int main(void)
{
    if (test_spherical_double() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_solid_cylinder_double() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_rectangular_prism_double() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_single_perforated_double() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_initial_states_double() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_exact_burnout_and_overshoot_double() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_limiting_burnout_modes_double() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_validation_and_failure_order() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_validator_nonfinite_precedence() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_native_precision_analytical_results() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_monotonic_progression_double() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_derived_regression_underflow() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_numerical_failure() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    return EXIT_SUCCESS;
}
