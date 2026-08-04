/**
 * @file
 * @brief Tests precision-qualified internal-ballistics propellant-charge records.
 */

#include <bbtc/bbtc.h>

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <float.h>
#include <math.h>

static_assert(_Generic(((bbtc_ib_propellant_charge_float_t*)0)->charge_mass_kg,
                       float: 1,
                       default: 0));
static_assert(_Generic(((bbtc_ib_propellant_charge_float_t*)0)->condensed_phase_density_kg_per_m3,
                       float: 1,
                       default: 0));

static_assert(_Generic(((bbtc_ib_propellant_charge_double_t*)0)->charge_mass_kg,
                       double: 1,
                       default: 0));
static_assert(_Generic(((bbtc_ib_propellant_charge_double_t*)0)->condensed_phase_density_kg_per_m3,
                       double: 1,
                       default: 0));

static_assert(_Generic(((bbtc_ib_propellant_charge_long_double_t*)0)->charge_mass_kg,
                       long double: 1,
                       default: 0));
static_assert(_Generic(((bbtc_ib_propellant_charge_long_double_t*)0)->condensed_phase_density_kg_per_m3,
                       long double: 1,
                       default: 0));


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
test_float_propellant_charge(void)
{
    const bbtc_ib_propellant_charge_float_t valid =
    {
        .charge_mass_kg                       = 0.0030f,
        .condensed_phase_density_kg_per_m3    = 1600.0f
    };

    bbtc_ib_propellant_charge_float_t candidate = valid;

    if (require_status("float NULL",
                       bbtc_ib_propellant_charge_validate_float(NULL),
                       BBTC_STATUS_INVALID_ARGUMENT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (require_status("float valid",
                       bbtc_ib_propellant_charge_validate_float(&candidate),
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (candidate.charge_mass_kg                    != valid.charge_mass_kg ||
        candidate.condensed_phase_density_kg_per_m3 != valid.condensed_phase_density_kg_per_m3)
    {
        fprintf(stderr, "float validation modified its input\n");
        return EXIT_FAILURE;
    }

    candidate.condensed_phase_density_kg_per_m3 = 1600.0f;
    candidate.charge_mass_kg = 0.0f;

    if (require_status("float charge mass zero",
                       bbtc_ib_propellant_charge_validate_float(&candidate),
                       BBTC_STATUS_OUTSIDE_DOMAIN) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.condensed_phase_density_kg_per_m3 = 1600.0f;
    candidate.charge_mass_kg = -1.0f;

    if (require_status("float charge mass negative",
                       bbtc_ib_propellant_charge_validate_float(&candidate),
                       BBTC_STATUS_OUTSIDE_DOMAIN) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.condensed_phase_density_kg_per_m3 = 1600.0f;
    candidate.charge_mass_kg = NAN;

    if (require_status("float charge mass NaN",
                       bbtc_ib_propellant_charge_validate_float(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.condensed_phase_density_kg_per_m3 = 1600.0f;
    candidate.charge_mass_kg = INFINITY;

    if (require_status("float charge mass positive infinity",
                       bbtc_ib_propellant_charge_validate_float(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.condensed_phase_density_kg_per_m3 = 1600.0f;
    candidate.charge_mass_kg = -INFINITY;

    if (require_status("float charge mass negative infinity",
                       bbtc_ib_propellant_charge_validate_float(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.condensed_phase_density_kg_per_m3 = 1600.0f;
    candidate.charge_mass_kg = FLT_TRUE_MIN;

    if (require_status("float charge mass positive subnormal",
                       bbtc_ib_propellant_charge_validate_float(&candidate),
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.condensed_phase_density_kg_per_m3 = 1600.0f;
    candidate.charge_mass_kg = FLT_MAX;

    if (require_status("float charge mass finite maximum",
                       bbtc_ib_propellant_charge_validate_float(&candidate),
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.charge_mass_kg = 0.0030f;
    candidate.condensed_phase_density_kg_per_m3 = 0.0f;

    if (require_status("float density zero",
                       bbtc_ib_propellant_charge_validate_float(&candidate),
                       BBTC_STATUS_OUTSIDE_DOMAIN) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.charge_mass_kg = 0.0030f;
    candidate.condensed_phase_density_kg_per_m3 = -1.0f;

    if (require_status("float density negative",
                       bbtc_ib_propellant_charge_validate_float(&candidate),
                       BBTC_STATUS_OUTSIDE_DOMAIN) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.charge_mass_kg = 0.0030f;
    candidate.condensed_phase_density_kg_per_m3 = NAN;

    if (require_status("float density NaN",
                       bbtc_ib_propellant_charge_validate_float(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.charge_mass_kg = 0.0030f;
    candidate.condensed_phase_density_kg_per_m3 = INFINITY;

    if (require_status("float density positive infinity",
                       bbtc_ib_propellant_charge_validate_float(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.charge_mass_kg = 0.0030f;
    candidate.condensed_phase_density_kg_per_m3 = -INFINITY;

    if (require_status("float density negative infinity",
                       bbtc_ib_propellant_charge_validate_float(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.charge_mass_kg = 0.0030f;
    candidate.condensed_phase_density_kg_per_m3 = FLT_TRUE_MIN;

    if (require_status("float density positive subnormal",
                       bbtc_ib_propellant_charge_validate_float(&candidate),
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.charge_mass_kg = 0.0030f;
    candidate.condensed_phase_density_kg_per_m3 = FLT_MAX;

    return require_status("float density finite maximum",
                          bbtc_ib_propellant_charge_validate_float(&candidate),
                          BBTC_STATUS_SUCCESS);
}


static int
test_double_propellant_charge(void)
{
    const bbtc_ib_propellant_charge_double_t valid =
    {
        .charge_mass_kg                       = 0.0030,
        .condensed_phase_density_kg_per_m3    = 1600.0
    };

    bbtc_ib_propellant_charge_double_t candidate = valid;

    if (require_status("double NULL",
                       bbtc_ib_propellant_charge_validate_double(NULL),
                       BBTC_STATUS_INVALID_ARGUMENT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (require_status("double valid",
                       bbtc_ib_propellant_charge_validate_double(&candidate),
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (candidate.charge_mass_kg                    != valid.charge_mass_kg ||
        candidate.condensed_phase_density_kg_per_m3 != valid.condensed_phase_density_kg_per_m3)
    {
        fprintf(stderr, "double validation modified its input\n");
        return EXIT_FAILURE;
    }


    candidate.condensed_phase_density_kg_per_m3 = 1600.0;
    candidate.charge_mass_kg = 0.0;

    if (require_status("double charge mass zero",
                       bbtc_ib_propellant_charge_validate_double(&candidate),
                       BBTC_STATUS_OUTSIDE_DOMAIN) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }


    candidate.condensed_phase_density_kg_per_m3 = 1600.0;
    candidate.charge_mass_kg = -1.0;

    if (require_status("double charge mass negative",
                       bbtc_ib_propellant_charge_validate_double(&candidate),
                       BBTC_STATUS_OUTSIDE_DOMAIN) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }


    candidate.condensed_phase_density_kg_per_m3 = 1600.0;
    candidate.charge_mass_kg = NAN;

    if (require_status("double charge mass NaN",
                       bbtc_ib_propellant_charge_validate_double(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }


    candidate.condensed_phase_density_kg_per_m3 = 1600.0;
    candidate.charge_mass_kg = INFINITY;

    if (require_status("double charge mass positive infinity",
                       bbtc_ib_propellant_charge_validate_double(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }


    candidate.condensed_phase_density_kg_per_m3 = 1600.0;
    candidate.charge_mass_kg = -INFINITY;

    if (require_status("double charge mass negative infinity",
                       bbtc_ib_propellant_charge_validate_double(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }


    candidate.condensed_phase_density_kg_per_m3 = 1600.0;
    candidate.charge_mass_kg = DBL_TRUE_MIN;

    if (require_status("double charge mass positive subnormal",
                       bbtc_ib_propellant_charge_validate_double(&candidate),
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }


    candidate.condensed_phase_density_kg_per_m3 = 1600.0;
    candidate.charge_mass_kg = DBL_MAX;

    if (require_status("double charge mass finite maximum",
                       bbtc_ib_propellant_charge_validate_double(&candidate),
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }


    candidate.charge_mass_kg = 0.0030;
    candidate.condensed_phase_density_kg_per_m3 = 0.0;

    if (require_status("double density zero",
                       bbtc_ib_propellant_charge_validate_double(&candidate),
                       BBTC_STATUS_OUTSIDE_DOMAIN) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }


    candidate.charge_mass_kg = 0.0030;
    candidate.condensed_phase_density_kg_per_m3 = -1.0;

    if (require_status("double density negative",
                       bbtc_ib_propellant_charge_validate_double(&candidate),
                       BBTC_STATUS_OUTSIDE_DOMAIN) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }


    candidate.charge_mass_kg = 0.0030;
    candidate.condensed_phase_density_kg_per_m3 = NAN;

    if (require_status("double density NaN",
                       bbtc_ib_propellant_charge_validate_double(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }


    candidate.charge_mass_kg = 0.0030;
    candidate.condensed_phase_density_kg_per_m3 = INFINITY;

    if (require_status("double density positive infinity",
                       bbtc_ib_propellant_charge_validate_double(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }


    candidate.charge_mass_kg = 0.0030;
    candidate.condensed_phase_density_kg_per_m3 = -INFINITY;

    if (require_status("double density negative infinity",
                       bbtc_ib_propellant_charge_validate_double(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }


    candidate.charge_mass_kg = 0.0030;
    candidate.condensed_phase_density_kg_per_m3 = DBL_TRUE_MIN;

    if (require_status("double density positive subnormal",
                       bbtc_ib_propellant_charge_validate_double(&candidate),
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }


    candidate.charge_mass_kg = 0.0030;
    candidate.condensed_phase_density_kg_per_m3 = DBL_MAX;

    return require_status("double density finite maximum",
                          bbtc_ib_propellant_charge_validate_double(&candidate),
                          BBTC_STATUS_SUCCESS);
}


static int
test_long_double_propellant_charge(void)
{
    const bbtc_ib_propellant_charge_long_double_t valid =
    {
        .charge_mass_kg                       = 0.0030L,
        .condensed_phase_density_kg_per_m3    = 1600.0L
    };

    bbtc_ib_propellant_charge_long_double_t candidate = valid;

    if (require_status("long double NULL",
                       bbtc_ib_propellant_charge_validate_long_double(NULL),
                       BBTC_STATUS_INVALID_ARGUMENT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (require_status("long double valid",
                       bbtc_ib_propellant_charge_validate_long_double(&candidate),
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (candidate.charge_mass_kg                    != valid.charge_mass_kg ||
        candidate.condensed_phase_density_kg_per_m3 != valid.condensed_phase_density_kg_per_m3)
    {
        fprintf(stderr, "long double validation modified its input\n");
        return EXIT_FAILURE;
    }


    candidate.condensed_phase_density_kg_per_m3 = 1600.0L;
    candidate.charge_mass_kg = 0.0L;

    if (require_status("long double charge mass zero",
                       bbtc_ib_propellant_charge_validate_long_double(&candidate),
                       BBTC_STATUS_OUTSIDE_DOMAIN) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }


    candidate.condensed_phase_density_kg_per_m3 = 1600.0L;
    candidate.charge_mass_kg = -1.0L;

    if (require_status("long double charge mass negative",
                       bbtc_ib_propellant_charge_validate_long_double(&candidate),
                       BBTC_STATUS_OUTSIDE_DOMAIN) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }


    candidate.condensed_phase_density_kg_per_m3 = 1600.0L;
    candidate.charge_mass_kg = NAN;

    if (require_status("long double charge mass NaN",
                       bbtc_ib_propellant_charge_validate_long_double(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }


    candidate.condensed_phase_density_kg_per_m3 = 1600.0L;
    candidate.charge_mass_kg = INFINITY;

    if (require_status("long double charge mass positive infinity",
                       bbtc_ib_propellant_charge_validate_long_double(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }


    candidate.condensed_phase_density_kg_per_m3 = 1600.0L;
    candidate.charge_mass_kg = -INFINITY;

    if (require_status("long double charge mass negative infinity",
                       bbtc_ib_propellant_charge_validate_long_double(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }


    candidate.condensed_phase_density_kg_per_m3 = 1600.0L;
    candidate.charge_mass_kg = LDBL_TRUE_MIN;

    if (require_status("long double charge mass positive subnormal",
                       bbtc_ib_propellant_charge_validate_long_double(&candidate),
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }


    candidate.condensed_phase_density_kg_per_m3 = 1600.0L;
    candidate.charge_mass_kg = LDBL_MAX;

    if (require_status("long double charge mass finite maximum",
                       bbtc_ib_propellant_charge_validate_long_double(&candidate),
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }


    candidate.charge_mass_kg = 0.0030L;
    candidate.condensed_phase_density_kg_per_m3 = 0.0L;

    if (require_status("long double density zero",
                       bbtc_ib_propellant_charge_validate_long_double(&candidate),
                       BBTC_STATUS_OUTSIDE_DOMAIN) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }


    candidate.charge_mass_kg = 0.0030L;
    candidate.condensed_phase_density_kg_per_m3 = -1.0L;

    if (require_status("long double density negative",
                       bbtc_ib_propellant_charge_validate_long_double(&candidate),
                       BBTC_STATUS_OUTSIDE_DOMAIN) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }


    candidate.charge_mass_kg = 0.0030L;
    candidate.condensed_phase_density_kg_per_m3 = NAN;

    if (require_status("long double density NaN",
                       bbtc_ib_propellant_charge_validate_long_double(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }


    candidate.charge_mass_kg = 0.0030L;
    candidate.condensed_phase_density_kg_per_m3 = INFINITY;

    if (require_status("long double density positive infinity",
                       bbtc_ib_propellant_charge_validate_long_double(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }


    candidate.charge_mass_kg = 0.0030L;
    candidate.condensed_phase_density_kg_per_m3 = -INFINITY;

    if (require_status("long double density negative infinity",
                       bbtc_ib_propellant_charge_validate_long_double(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }


    candidate.charge_mass_kg = 0.0030L;
    candidate.condensed_phase_density_kg_per_m3 = LDBL_TRUE_MIN;

    if (require_status("long double density positive subnormal",
                       bbtc_ib_propellant_charge_validate_long_double(&candidate),
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }


    candidate.charge_mass_kg = 0.0030L;
    candidate.condensed_phase_density_kg_per_m3 = LDBL_MAX;

    return require_status("long double density finite maximum",
                          bbtc_ib_propellant_charge_validate_long_double(&candidate),
                          BBTC_STATUS_SUCCESS);
}


int main(void)
{
    if (test_float_propellant_charge() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_double_propellant_charge() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    return test_long_double_propellant_charge();
}
