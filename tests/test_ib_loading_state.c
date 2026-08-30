/**
 * @file
 * @brief Tests composed loading-state validation and derived initial volumes.
 */

#include <bbtc/bbtc.h>

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static_assert(_Generic(((bbtc_ib_loading_state_float_t*)0)->geometry,
                       bbtc_ib_geometry_float_t: 1,
                       default: 0));

static_assert(_Generic(((bbtc_ib_loading_state_double_t*)0)->projectile,
                       bbtc_ib_projectile_double_t: 1,
                       default: 0));

static_assert(_Generic(((bbtc_ib_loading_state_long_double_t*)0)->propellant_charge,
                       bbtc_ib_propellant_charge_long_double_t: 1,
                       default: 0));


static_assert(_Generic(((bbtc_ib_loading_state_volumes_float_t*)0)
                           ->initial_free_gas_volume_m3,
                       float:   1,
                       default: 0));

static_assert(_Generic(((bbtc_ib_loading_state_volumes_double_t*)0)
                           ->condensed_propellant_volume_m3,
                       double:  1,
                       default: 0));

static_assert(_Generic(((bbtc_ib_loading_state_volumes_long_double_t*)0)
                           ->initial_free_gas_volume_m3,
                       long double: 1,
                       default:     0));


static int
require_status(const char*   label,
               bbtc_status_e actual,
               bbtc_status_e expected)
{
    if (actual == expected)
        return EXIT_SUCCESS;

    fprintf(stderr,
            "%s returned %u; expected %u\n",
            label,
            (unsigned int)actual,
            (unsigned int)expected);

    return EXIT_FAILURE;
}

static int
test_float_loading_state(void)
{
    const bbtc_ib_loading_state_float_t valid =
    {
        .geometry =
        {
            .initial_behind_projectile_volume_m3 = 8.0f,
            .bore_cross_sectional_area_m2        = 1.0f,
            .projectile_effective_base_area_m2   = 1.0f,
            .projectile_travel_to_muzzle_m       = 1.0f
        },

        .projectile =
        {
            .mass_kg = 1.0f
        },

        .propellant_charge =
        {
            .charge_mass_kg                      = 6.0f,
            .condensed_phase_density_kg_per_m3   = 2.0f
        }
    };

    bbtc_ib_loading_state_float_t candidate = valid;

    bbtc_ib_loading_state_volumes_float_t volumes =
    {
        .condensed_propellant_volume_m3 = 99.0f,
        .initial_free_gas_volume_m3     = 99.0f
    };

    if (require_status("float NULL output",
                       bbtc_ib_loading_state_evaluate_float(&candidate, NULL),
                       BBTC_STATUS_INVALID_ARGUMENT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (require_status("float null input",
                       bbtc_ib_loading_state_evaluate_float(NULL, &volumes),
                       BBTC_STATUS_INVALID_ARGUMENT)
        != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (volumes.condensed_propellant_volume_m3 != 0.0f ||
        volumes.initial_free_gas_volume_m3     != 0.0f)
    {
        fprintf(stderr, "float NULL-input failure did not clear output\n");
        return EXIT_FAILURE;
    }

    if (require_status("float valid",
                       bbtc_ib_loading_state_evaluate_float(&candidate, &volumes),
                       BBTC_STATUS_SUCCESS)
        != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (volumes.condensed_propellant_volume_m3 != 3.0f ||
        volumes.initial_free_gas_volume_m3     != 5.0f)
    {
        fprintf(stderr, "float derived volumes are incorrect\n");
        return EXIT_FAILURE;
    }

    if (candidate.geometry.initial_behind_projectile_volume_m3 !=
            valid.geometry.initial_behind_projectile_volume_m3
        ||
        candidate.projectile.mass_kg != valid.projectile.mass_kg
        ||
        candidate.propellant_charge.charge_mass_kg !=
            valid.propellant_charge.charge_mass_kg
        ||
        candidate.propellant_charge.condensed_phase_density_kg_per_m3 !=
            valid.propellant_charge.condensed_phase_density_kg_per_m3)
    {
        fprintf(stderr, "float evaluation modified its input\n");
        return EXIT_FAILURE;
    }


    candidate = valid;
    candidate.geometry.initial_behind_projectile_volume_m3 = 0.0f;

    volumes = (bbtc_ib_loading_state_volumes_float_t){99.0f, 99.0f};

    if (require_status("float invalid geometry",
                       bbtc_ib_loading_state_evaluate_float(&candidate, &volumes),
                       BBTC_STATUS_OUTSIDE_DOMAIN)
        != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (volumes.condensed_propellant_volume_m3 != 0.0f ||
        volumes.initial_free_gas_volume_m3 != 0.0f)
    {
        fprintf(stderr, "float geometry failure did not clear output\n");
        return EXIT_FAILURE;
    }


    candidate = valid;
    candidate.projectile.mass_kg = 0.0f;

    if (require_status("float invalid projectile",
                       bbtc_ib_loading_state_evaluate_float(&candidate, &volumes),
                       BBTC_STATUS_OUTSIDE_DOMAIN)
        != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }


    candidate = valid;
    candidate.propellant_charge.condensed_phase_density_kg_per_m3 = 0.0f;

    if (require_status("float invalid propellant charge",
                       bbtc_ib_loading_state_evaluate_float(&candidate, &volumes),
                       BBTC_STATUS_OUTSIDE_DOMAIN)
        != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }


    candidate = valid;
    candidate.geometry.bore_cross_sectional_area_m2 = NAN;

    if (require_status("float nonfinite component",
                       bbtc_ib_loading_state_evaluate_float(&candidate, &volumes),
                       BBTC_STATUS_NAN_INPUT)
        != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }


    candidate = valid;
    candidate.propellant_charge.charge_mass_kg = 16.0f;

    if (require_status("float zero free-gas volume",
                       bbtc_ib_loading_state_evaluate_float(&candidate, &volumes),
                       BBTC_STATUS_OUTSIDE_DOMAIN)
        != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }


    candidate = valid;
    candidate.propellant_charge.charge_mass_kg = 18.0f;

    if (require_status("float condensed volume exceeds geometry",
                       bbtc_ib_loading_state_evaluate_float(&candidate, &volumes),
                       BBTC_STATUS_OUTSIDE_DOMAIN)
        != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }


    candidate = valid;
    candidate.propellant_charge.charge_mass_kg = FLT_TRUE_MIN;
    candidate.propellant_charge.condensed_phase_density_kg_per_m3 = FLT_MAX;

    if (require_status("float derived-volume underflow",
                       bbtc_ib_loading_state_evaluate_float(&candidate, &volumes),
                       BBTC_STATUS_OUTSIDE_DOMAIN)
        != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (volumes.condensed_propellant_volume_m3 != 0.0f ||
        volumes.initial_free_gas_volume_m3 != 0.0f)
    {
        fprintf(stderr, "float cross-record failure did not clear output\n");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}


static int
test_double_loading_state(void)
{
    const bbtc_ib_loading_state_double_t valid =
    {
        .geometry =
        {
            .initial_behind_projectile_volume_m3 = 8.0,
            .bore_cross_sectional_area_m2        = 1.0,
            .projectile_effective_base_area_m2   = 1.0,
            .projectile_travel_to_muzzle_m       = 1.0
        },

        .projectile =
        {
            .mass_kg = 1.0
        },

        .propellant_charge =
        {
            .charge_mass_kg                      = 6.0,
            .condensed_phase_density_kg_per_m3   = 2.0
        }
    };

    bbtc_ib_loading_state_double_t candidate = valid;

    bbtc_ib_loading_state_volumes_double_t volumes =
    {
        .condensed_propellant_volume_m3 = 99.0,
        .initial_free_gas_volume_m3     = 99.0
    };

    if (require_status("double null output",
                       bbtc_ib_loading_state_evaluate_double(&candidate, NULL),
                       BBTC_STATUS_INVALID_ARGUMENT)
        != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (require_status("double null input",
                       bbtc_ib_loading_state_evaluate_double(NULL, &volumes),
                       BBTC_STATUS_INVALID_ARGUMENT)
        != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (volumes.condensed_propellant_volume_m3 != 0.0 ||
        volumes.initial_free_gas_volume_m3 != 0.0)
    {
        fprintf(stderr, "double NULL-input failure did not clear output\n");
        return EXIT_FAILURE;
    }

    if (require_status("double valid",
                       bbtc_ib_loading_state_evaluate_double(&candidate, &volumes),
                       BBTC_STATUS_SUCCESS)
        != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (volumes.condensed_propellant_volume_m3 != 3.0 ||
        volumes.initial_free_gas_volume_m3 != 5.0)
    {
        fprintf(stderr, "double derived volumes are incorrect\n");
        return EXIT_FAILURE;
    }

    if (candidate.geometry.initial_behind_projectile_volume_m3 !=
            valid.geometry.initial_behind_projectile_volume_m3
        ||
        candidate.projectile.mass_kg != valid.projectile.mass_kg
        ||
        candidate.propellant_charge.charge_mass_kg !=
            valid.propellant_charge.charge_mass_kg
        ||
        candidate.propellant_charge.condensed_phase_density_kg_per_m3 !=
            valid.propellant_charge.condensed_phase_density_kg_per_m3)
    {
        fprintf(stderr, "double evaluation modified its input\n");
        return EXIT_FAILURE;
    }


    candidate = valid;
    candidate.geometry.initial_behind_projectile_volume_m3 = 0.0;

    volumes = (bbtc_ib_loading_state_volumes_double_t){99.0, 99.0};

    if (require_status("double invalid geometry",
                       bbtc_ib_loading_state_evaluate_double(&candidate, &volumes),
                       BBTC_STATUS_OUTSIDE_DOMAIN)
        != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (volumes.condensed_propellant_volume_m3 != 0.0 ||
        volumes.initial_free_gas_volume_m3 != 0.0)
    {
        fprintf(stderr, "double geometry failure did not clear output\n");
        return EXIT_FAILURE;
    }


    candidate = valid;
    candidate.projectile.mass_kg = 0.0;

    if (require_status("double invalid projectile",
                       bbtc_ib_loading_state_evaluate_double(&candidate, &volumes),
                       BBTC_STATUS_OUTSIDE_DOMAIN)
        != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }


    candidate = valid;
    candidate.propellant_charge.condensed_phase_density_kg_per_m3 = 0.0;

    if (require_status("double invalid propellant charge",
                       bbtc_ib_loading_state_evaluate_double(&candidate, &volumes),
                       BBTC_STATUS_OUTSIDE_DOMAIN)
        != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }


    candidate = valid;
    candidate.geometry.bore_cross_sectional_area_m2 = NAN;

    if (require_status("double nonfinite component",
                       bbtc_ib_loading_state_evaluate_double(&candidate, &volumes),
                       BBTC_STATUS_NAN_INPUT)
        != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }


    candidate = valid;
    candidate.propellant_charge.charge_mass_kg = 16.0;

    if (require_status("double zero free-gas volume",
                       bbtc_ib_loading_state_evaluate_double(&candidate, &volumes),
                       BBTC_STATUS_OUTSIDE_DOMAIN)
        != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }


    candidate = valid;
    candidate.propellant_charge.charge_mass_kg = 18.0;

    if (require_status("double condensed volume exceeds geometry",
                       bbtc_ib_loading_state_evaluate_double(&candidate, &volumes),
                       BBTC_STATUS_OUTSIDE_DOMAIN)
        != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }


    candidate = valid;
    candidate.propellant_charge.charge_mass_kg = DBL_TRUE_MIN;
    candidate.propellant_charge.condensed_phase_density_kg_per_m3 = DBL_MAX;

    if (require_status("double derived-volume underflow",
                       bbtc_ib_loading_state_evaluate_double(&candidate, &volumes),
                       BBTC_STATUS_OUTSIDE_DOMAIN)
        != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (volumes.condensed_propellant_volume_m3 != 0.0 ||
        volumes.initial_free_gas_volume_m3 != 0.0)
    {
        fprintf(stderr, "double cross-record failure did not clear output\n");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}


static int
test_long_double_loading_state(void)
{
    const bbtc_ib_loading_state_long_double_t valid =
    {
        .geometry =
        {
            .initial_behind_projectile_volume_m3 = 8.0L,
            .bore_cross_sectional_area_m2        = 1.0L,
            .projectile_effective_base_area_m2   = 1.0L,
            .projectile_travel_to_muzzle_m       = 1.0L
        },

        .projectile =
        {
            .mass_kg = 1.0L
        },

        .propellant_charge =
        {
            .charge_mass_kg                      = 6.0L,
            .condensed_phase_density_kg_per_m3   = 2.0L
        }
    };

    bbtc_ib_loading_state_long_double_t candidate = valid;

    bbtc_ib_loading_state_volumes_long_double_t volumes =
    {
        .condensed_propellant_volume_m3 = 99.0L,
        .initial_free_gas_volume_m3     = 99.0L
    };

    if (require_status("long double NULL output",
                       bbtc_ib_loading_state_evaluate_long_double(&candidate, NULL),
                       BBTC_STATUS_INVALID_ARGUMENT)
        != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (require_status("long double null input",
                       bbtc_ib_loading_state_evaluate_long_double(NULL, &volumes),
                       BBTC_STATUS_INVALID_ARGUMENT)
        != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (volumes.condensed_propellant_volume_m3 != 0.0L ||
        volumes.initial_free_gas_volume_m3 != 0.0L)
    {
        fprintf(stderr, "long double NULL-input failure did not clear output\n");
        return EXIT_FAILURE;
    }

    if (require_status("long double valid",
                       bbtc_ib_loading_state_evaluate_long_double(&candidate, &volumes),
                       BBTC_STATUS_SUCCESS)
        != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (volumes.condensed_propellant_volume_m3 != 3.0L ||
        volumes.initial_free_gas_volume_m3 != 5.0L)
    {
        fprintf(stderr, "long double derived volumes are incorrect\n");
        return EXIT_FAILURE;
    }

    if (candidate.geometry.initial_behind_projectile_volume_m3 !=
            valid.geometry.initial_behind_projectile_volume_m3
        ||
        candidate.projectile.mass_kg != valid.projectile.mass_kg
        ||
        candidate.propellant_charge.charge_mass_kg !=
            valid.propellant_charge.charge_mass_kg
        ||
        candidate.propellant_charge.condensed_phase_density_kg_per_m3 !=
            valid.propellant_charge.condensed_phase_density_kg_per_m3)
    {
        fprintf(stderr, "long double evaluation modified its input\n");
        return EXIT_FAILURE;
    }


    candidate = valid;
    candidate.geometry.initial_behind_projectile_volume_m3 = 0.0L;
    volumes = (bbtc_ib_loading_state_volumes_long_double_t){99.0L, 99.0L};

    if (require_status("long double invalid geometry",
                       bbtc_ib_loading_state_evaluate_long_double(&candidate, &volumes),
                       BBTC_STATUS_OUTSIDE_DOMAIN)
        != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (volumes.condensed_propellant_volume_m3 != 0.0L
        || volumes.initial_free_gas_volume_m3 != 0.0L)
    {
        fprintf(stderr, "long double geometry failure did not clear output\n");
        return EXIT_FAILURE;
    }


    candidate = valid;
    candidate.projectile.mass_kg = 0.0L;

    if (require_status("long double invalid projectile",
                       bbtc_ib_loading_state_evaluate_long_double(&candidate, &volumes),
                       BBTC_STATUS_OUTSIDE_DOMAIN)
        != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }


    candidate = valid;
    candidate.propellant_charge.condensed_phase_density_kg_per_m3 = 0.0L;

    if (require_status("long double invalid propellant charge",
                       bbtc_ib_loading_state_evaluate_long_double(&candidate, &volumes),
                       BBTC_STATUS_OUTSIDE_DOMAIN)
        != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }


    candidate = valid;
    candidate.geometry.bore_cross_sectional_area_m2 = NAN;

    if (require_status("long double nonfinite component",
                       bbtc_ib_loading_state_evaluate_long_double(&candidate, &volumes),
                       BBTC_STATUS_NAN_INPUT)
        != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }


    candidate = valid;
    candidate.propellant_charge.charge_mass_kg = 16.0L;

    if (require_status("long double zero free-gas volume",
                       bbtc_ib_loading_state_evaluate_long_double(&candidate, &volumes),
                       BBTC_STATUS_OUTSIDE_DOMAIN)
        != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }


    candidate = valid;
    candidate.propellant_charge.charge_mass_kg = 18.0L;

    if (require_status("long double condensed volume exceeds geometry",
                       bbtc_ib_loading_state_evaluate_long_double(&candidate, &volumes),
                       BBTC_STATUS_OUTSIDE_DOMAIN)
        != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }


    candidate = valid;
    candidate.propellant_charge.charge_mass_kg = LDBL_TRUE_MIN;

    candidate.propellant_charge.condensed_phase_density_kg_per_m3 = LDBL_MAX;
    if (require_status("long double derived-volume underflow",
                       bbtc_ib_loading_state_evaluate_long_double(&candidate, &volumes),
                       BBTC_STATUS_OUTSIDE_DOMAIN)
        != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (volumes.condensed_propellant_volume_m3 != 0.0L ||
        volumes.initial_free_gas_volume_m3 != 0.0L)
    {
        fprintf(stderr, "long double cross-record failure did not clear output\n");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}


int main(void)
{
    if (test_float_loading_state() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_double_loading_state() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    return test_long_double_loading_state();
}
