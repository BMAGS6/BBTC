/**
 * @file
 * @brief Tests precision-qualified initial internal-ballistics gas states.
 */

#include <bbtc/bbtc.h>

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

static_assert(_Generic(((bbtc_ib_initial_gas_state_float_t*)0)->absolute_pressure_pa,
                       float: 1,
                       default: 0));
static_assert(_Generic(((bbtc_ib_initial_gas_state_float_t*)0)->temperature_k,
                       float: 1,
                       default: 0));

static_assert(_Generic(((bbtc_ib_initial_gas_state_double_t*)0)->absolute_pressure_pa,
                       double: 1,
                       default: 0));
static_assert(_Generic(((bbtc_ib_initial_gas_state_double_t*)0)->temperature_k,
                       double: 1,
                       default: 0));

static_assert(_Generic(((bbtc_ib_initial_gas_state_long_double_t*)0)->absolute_pressure_pa,
                       long double: 1,
                       default: 0));
static_assert(_Generic(((bbtc_ib_initial_gas_state_long_double_t*)0)->temperature_k,
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
test_float_initial_gas_state(void)
{
    const bbtc_ib_initial_gas_state_float_t valid =
    {
        .absolute_pressure_pa = 101325.0f,
        .temperature_k        = 293.15f
    };

    bbtc_ib_initial_gas_state_float_t candidate = valid;

    if (require_status("float NULL",
                       bbtc_ib_initial_gas_state_validate_float(NULL),
                       BBTC_STATUS_INVALID_ARGUMENT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (require_status("float valid",
                       bbtc_ib_initial_gas_state_validate_float(&candidate),
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (candidate.absolute_pressure_pa != valid.absolute_pressure_pa ||
        candidate.temperature_k        != valid.temperature_k)
    {
        fprintf(stderr, "float validation modified its input\n");
        return EXIT_FAILURE;
    }

    candidate.temperature_k = 293.15f;
    candidate.absolute_pressure_pa = 0.0f;

    if (require_status("float pressure zero",
                       bbtc_ib_initial_gas_state_validate_float(&candidate),
                       BBTC_STATUS_OUTSIDE_DOMAIN) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.temperature_k = 293.15f;
    candidate.absolute_pressure_pa = -1.0f;

    if (require_status("float pressure negative",
                       bbtc_ib_initial_gas_state_validate_float(&candidate),
                       BBTC_STATUS_OUTSIDE_DOMAIN) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.temperature_k = 293.15f;
    candidate.absolute_pressure_pa = NAN;

    if (require_status("float pressure NaN",
                       bbtc_ib_initial_gas_state_validate_float(&candidate),
                       BBTC_STATUS_NAN_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.temperature_k = 293.15f;
    candidate.absolute_pressure_pa = INFINITY;

    if (require_status("float pressure positive infinity",
                       bbtc_ib_initial_gas_state_validate_float(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.temperature_k = 293.15f;
    candidate.absolute_pressure_pa = -INFINITY;

    if (require_status("float pressure negative infinity",
                       bbtc_ib_initial_gas_state_validate_float(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.temperature_k = 293.15f;
    candidate.absolute_pressure_pa = FLT_TRUE_MIN;

    if (require_status("float pressure positive subnormal",
                       bbtc_ib_initial_gas_state_validate_float(&candidate),
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.temperature_k = 293.15f;
    candidate.absolute_pressure_pa = FLT_MAX;

    if (require_status("float pressure finite maximum",
                       bbtc_ib_initial_gas_state_validate_float(&candidate),
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.absolute_pressure_pa = 101325.0f;
    candidate.temperature_k = 0.0f;

    if (require_status("float temperature zero",
                       bbtc_ib_initial_gas_state_validate_float(&candidate),
                       BBTC_STATUS_OUTSIDE_DOMAIN) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.absolute_pressure_pa = 101325.0f;
    candidate.temperature_k = -1.0f;

    if (require_status("float temperature negative",
                       bbtc_ib_initial_gas_state_validate_float(&candidate),
                       BBTC_STATUS_OUTSIDE_DOMAIN) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.absolute_pressure_pa = 101325.0f;
    candidate.temperature_k = NAN;

    if (require_status("float temperature NaN",
                       bbtc_ib_initial_gas_state_validate_float(&candidate),
                       BBTC_STATUS_NAN_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.absolute_pressure_pa = 101325.0f;
    candidate.temperature_k = INFINITY;

    if (require_status("float temperature positive infinity",
                       bbtc_ib_initial_gas_state_validate_float(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.absolute_pressure_pa = 101325.0f;
    candidate.temperature_k = -INFINITY;

    if (require_status("float temperature negative infinity",
                       bbtc_ib_initial_gas_state_validate_float(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.absolute_pressure_pa = 101325.0f;
    candidate.temperature_k = FLT_TRUE_MIN;

    if (require_status("float temperature positive subnormal",
                       bbtc_ib_initial_gas_state_validate_float(&candidate),
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.absolute_pressure_pa = 101325.0f;
    candidate.temperature_k = FLT_MAX;

    return require_status("float temperature finite maximum",
                          bbtc_ib_initial_gas_state_validate_float(&candidate),
                          BBTC_STATUS_SUCCESS);
}


static int
test_double_initial_gas_state(void)
{
    const bbtc_ib_initial_gas_state_double_t valid =
    {
        .absolute_pressure_pa = 101325.0,
        .temperature_k        = 293.15
    };

    bbtc_ib_initial_gas_state_double_t candidate = valid;

    if (require_status("double NULL",
                       bbtc_ib_initial_gas_state_validate_double(NULL),
                       BBTC_STATUS_INVALID_ARGUMENT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (require_status("double valid",
                       bbtc_ib_initial_gas_state_validate_double(&candidate),
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (candidate.absolute_pressure_pa != valid.absolute_pressure_pa ||
        candidate.temperature_k        != valid.temperature_k)
    {
        fprintf(stderr, "double validation modified its input\n");
        return EXIT_FAILURE;
    }

    candidate.temperature_k = 293.15;
    candidate.absolute_pressure_pa = 0.0;

    if (require_status("double pressure zero",
                       bbtc_ib_initial_gas_state_validate_double(&candidate),
                       BBTC_STATUS_OUTSIDE_DOMAIN) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.temperature_k = 293.15;
    candidate.absolute_pressure_pa = -1.0;

    if (require_status("double pressure negative",
                       bbtc_ib_initial_gas_state_validate_double(&candidate),
                       BBTC_STATUS_OUTSIDE_DOMAIN) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.temperature_k = 293.15;
    candidate.absolute_pressure_pa = NAN;

    if (require_status("double pressure NaN",
                       bbtc_ib_initial_gas_state_validate_double(&candidate),
                       BBTC_STATUS_NAN_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.temperature_k = 293.15;
    candidate.absolute_pressure_pa = INFINITY;

    if (require_status("double pressure positive infinity",
                       bbtc_ib_initial_gas_state_validate_double(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.temperature_k = 293.15;
    candidate.absolute_pressure_pa = -INFINITY;

    if (require_status("double pressure negative infinity",
                       bbtc_ib_initial_gas_state_validate_double(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.temperature_k = 293.15;
    candidate.absolute_pressure_pa = DBL_TRUE_MIN;

    if (require_status("double pressure positive subnormal",
                       bbtc_ib_initial_gas_state_validate_double(&candidate),
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.temperature_k = 293.15;
    candidate.absolute_pressure_pa = DBL_MAX;

    if (require_status("double pressure finite maximum",
                       bbtc_ib_initial_gas_state_validate_double(&candidate),
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.absolute_pressure_pa = 101325.0;
    candidate.temperature_k = 0.0;

    if (require_status("double temperature zero",
                       bbtc_ib_initial_gas_state_validate_double(&candidate),
                       BBTC_STATUS_OUTSIDE_DOMAIN) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.absolute_pressure_pa = 101325.0;
    candidate.temperature_k = -1.0;

    if (require_status("double temperature negative",
                       bbtc_ib_initial_gas_state_validate_double(&candidate),
                       BBTC_STATUS_OUTSIDE_DOMAIN) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.absolute_pressure_pa = 101325.0;
    candidate.temperature_k = NAN;

    if (require_status("double temperature NaN",
                       bbtc_ib_initial_gas_state_validate_double(&candidate),
                       BBTC_STATUS_NAN_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.absolute_pressure_pa = 101325.0;
    candidate.temperature_k = INFINITY;

    if (require_status("double temperature positive infinity",
                       bbtc_ib_initial_gas_state_validate_double(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.absolute_pressure_pa = 101325.0;
    candidate.temperature_k = -INFINITY;

    if (require_status("double temperature negative infinity",
                       bbtc_ib_initial_gas_state_validate_double(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.absolute_pressure_pa = 101325.0;
    candidate.temperature_k = DBL_TRUE_MIN;

    if (require_status("double temperature positive subnormal",
                       bbtc_ib_initial_gas_state_validate_double(&candidate),
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.absolute_pressure_pa = 101325.0;
    candidate.temperature_k = DBL_MAX;

    return require_status("double temperature finite maximum",
                          bbtc_ib_initial_gas_state_validate_double(&candidate),
                          BBTC_STATUS_SUCCESS);
}


static int
test_long_double_initial_gas_state(void)
{
    const bbtc_ib_initial_gas_state_long_double_t valid =
    {
        .absolute_pressure_pa = 101325.0L,
        .temperature_k        = 293.15L
    };

    bbtc_ib_initial_gas_state_long_double_t candidate = valid;

    if (require_status("long double NULL",
                       bbtc_ib_initial_gas_state_validate_long_double(NULL),
                       BBTC_STATUS_INVALID_ARGUMENT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (require_status("long double valid",
                       bbtc_ib_initial_gas_state_validate_long_double(&candidate),
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (candidate.absolute_pressure_pa != valid.absolute_pressure_pa ||
        candidate.temperature_k        != valid.temperature_k)
    {
        fprintf(stderr, "long double validation modified its input\n");
        return EXIT_FAILURE;
    }

    candidate.temperature_k = 293.15L;
    candidate.absolute_pressure_pa = 0.0L;

    if (require_status("long double pressure zero",
                       bbtc_ib_initial_gas_state_validate_long_double(&candidate),
                       BBTC_STATUS_OUTSIDE_DOMAIN) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.temperature_k = 293.15L;
    candidate.absolute_pressure_pa = -1.0L;

    if (require_status("long double pressure negative",
                       bbtc_ib_initial_gas_state_validate_long_double(&candidate),
                       BBTC_STATUS_OUTSIDE_DOMAIN) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.temperature_k = 293.15L;
    candidate.absolute_pressure_pa = NAN;

    if (require_status("long double pressure NaN",
                       bbtc_ib_initial_gas_state_validate_long_double(&candidate),
                       BBTC_STATUS_NAN_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.temperature_k = 293.15L;
    candidate.absolute_pressure_pa = INFINITY;

    if (require_status("long double pressure positive infinity",
                       bbtc_ib_initial_gas_state_validate_long_double(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.temperature_k = 293.15L;
    candidate.absolute_pressure_pa = -INFINITY;

    if (require_status("long double pressure negative infinity",
                       bbtc_ib_initial_gas_state_validate_long_double(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.temperature_k = 293.15L;
    candidate.absolute_pressure_pa = LDBL_TRUE_MIN;

    if (require_status("long double pressure positive subnormal",
                       bbtc_ib_initial_gas_state_validate_long_double(&candidate),
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.temperature_k = 293.15L;
    candidate.absolute_pressure_pa = LDBL_MAX;

    if (require_status("long double pressure finite maximum",
                       bbtc_ib_initial_gas_state_validate_long_double(&candidate),
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.absolute_pressure_pa = 101325.0L;
    candidate.temperature_k = 0.0L;

    if (require_status("long double temperature zero",
                       bbtc_ib_initial_gas_state_validate_long_double(&candidate),
                       BBTC_STATUS_OUTSIDE_DOMAIN) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.absolute_pressure_pa = 101325.0L;
    candidate.temperature_k = -1.0L;

    if (require_status("long double temperature negative",
                       bbtc_ib_initial_gas_state_validate_long_double(&candidate),
                       BBTC_STATUS_OUTSIDE_DOMAIN) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.absolute_pressure_pa = 101325.0L;
    candidate.temperature_k = NAN;

    if (require_status("long double temperature NaN",
                       bbtc_ib_initial_gas_state_validate_long_double(&candidate),
                       BBTC_STATUS_NAN_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.absolute_pressure_pa = 101325.0L;
    candidate.temperature_k = INFINITY;

    if (require_status("long double temperature positive infinity",
                       bbtc_ib_initial_gas_state_validate_long_double(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.absolute_pressure_pa = 101325.0L;
    candidate.temperature_k = -INFINITY;

    if (require_status("long double temperature negative infinity",
                       bbtc_ib_initial_gas_state_validate_long_double(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.absolute_pressure_pa = 101325.0L;
    candidate.temperature_k = LDBL_TRUE_MIN;

    if (require_status("long double temperature positive subnormal",
                       bbtc_ib_initial_gas_state_validate_long_double(&candidate),
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.absolute_pressure_pa = 101325.0L;
    candidate.temperature_k = LDBL_MAX;

    return require_status("long double temperature finite maximum",
                          bbtc_ib_initial_gas_state_validate_long_double(&candidate),
                          BBTC_STATUS_SUCCESS);
}


static int
test_nonfinite_precedence(void)
{
    bbtc_ib_initial_gas_state_float_t float_state =
    {
        .absolute_pressure_pa = INFINITY,
        .temperature_k        = NAN
    };

    if (require_status(
            "float infinity pressure plus NaN temperature",
            bbtc_ib_initial_gas_state_validate_float(&float_state),
            BBTC_STATUS_NAN_INPUT
        ) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    float_state.absolute_pressure_pa = NAN;
    float_state.temperature_k        = INFINITY;

    if (require_status(
            "float NaN pressure plus infinity temperature",
            bbtc_ib_initial_gas_state_validate_float(&float_state),
            BBTC_STATUS_NAN_INPUT
        ) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }


    bbtc_ib_initial_gas_state_double_t double_state =
    {
        .absolute_pressure_pa = INFINITY,
        .temperature_k        = NAN
    };

    if (require_status(
            "double infinity pressure plus NaN temperature",
            bbtc_ib_initial_gas_state_validate_double(&double_state),
            BBTC_STATUS_NAN_INPUT
        ) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    double_state.absolute_pressure_pa = NAN;
    double_state.temperature_k        = INFINITY;

    if (require_status(
            "double NaN pressure plus infinity temperature",
            bbtc_ib_initial_gas_state_validate_double(&double_state),
            BBTC_STATUS_NAN_INPUT
        ) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }


    bbtc_ib_initial_gas_state_long_double_t long_double_state =
    {
        .absolute_pressure_pa = INFINITY,
        .temperature_k        = NAN
    };

    if (require_status(
            "long double infinity pressure plus NaN temperature",
            bbtc_ib_initial_gas_state_validate_long_double(&long_double_state),
            BBTC_STATUS_NAN_INPUT
        ) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    long_double_state.absolute_pressure_pa = NAN;
    long_double_state.temperature_k        = INFINITY;

    return require_status(
        "long double NaN pressure plus infinity temperature",
        bbtc_ib_initial_gas_state_validate_long_double(&long_double_state),
        BBTC_STATUS_NAN_INPUT
    );
}


int main(void)
{
    if (test_float_initial_gas_state() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_double_initial_gas_state() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_long_double_initial_gas_state() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    return test_nonfinite_precedence();
}
