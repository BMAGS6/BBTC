/**
 * @file
 * @brief Tests precision-qualified internal-ballistics projectile records.
 */

#include <bbtc/bbtc.h>

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

static_assert(_Generic(((bbtc_ib_projectile_float_t*)0)->mass_kg,
                       float: 1,
                       default: 0));
static_assert(_Generic(((bbtc_ib_projectile_double_t*)0)->mass_kg,
                       double: 1,
                       default: 0));
static_assert(_Generic(((bbtc_ib_projectile_long_double_t*)0)->mass_kg,
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
test_float_projectile(void)
{
    const bbtc_ib_projectile_float_t valid =
    {
        .mass_kg = 0.01134f
    };
    bbtc_ib_projectile_float_t candidate = valid;

    if (require_status("float null",
                       bbtc_ib_projectile_validate_float(NULL),
                       BBTC_STATUS_INVALID_ARGUMENT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (require_status("float valid",
                       bbtc_ib_projectile_validate_float(&candidate),
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (candidate.mass_kg != valid.mass_kg)
    {
        fprintf(stderr, "float validation modified its input\n");
        return EXIT_FAILURE;
    }

    candidate.mass_kg = 0.0f;
    if (require_status("float zero",
                       bbtc_ib_projectile_validate_float(&candidate),
                       BBTC_STATUS_OUTSIDE_DOMAIN) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.mass_kg = -1.0f;
    if (require_status("float negative",
                       bbtc_ib_projectile_validate_float(&candidate),
                       BBTC_STATUS_OUTSIDE_DOMAIN) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.mass_kg = NAN;
    if (require_status("float NaN",
                       bbtc_ib_projectile_validate_float(&candidate),
                       BBTC_STATUS_NAN_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.mass_kg = INFINITY;
    if (require_status("float positive infinity",
                       bbtc_ib_projectile_validate_float(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.mass_kg = -INFINITY;
    if (require_status("float negative infinity",
                       bbtc_ib_projectile_validate_float(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.mass_kg = FLT_TRUE_MIN;
    if (require_status("float positive subnormal",
                       bbtc_ib_projectile_validate_float(&candidate),
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.mass_kg = FLT_MAX;
    return require_status("float finite maximum",
                          bbtc_ib_projectile_validate_float(&candidate),
                          BBTC_STATUS_SUCCESS);
}


static int
test_double_projectile(void)
{
    const bbtc_ib_projectile_double_t valid =
    {
        .mass_kg = 0.01134
    };
    bbtc_ib_projectile_double_t candidate = valid;

    if (require_status("double null",
                       bbtc_ib_projectile_validate_double(NULL),
                       BBTC_STATUS_INVALID_ARGUMENT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (require_status("double valid",
                       bbtc_ib_projectile_validate_double(&candidate),
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (candidate.mass_kg != valid.mass_kg)
    {
        fprintf(stderr, "double validation modified its input\n");
        return EXIT_FAILURE;
    }

    candidate.mass_kg = 0.0;
    if (require_status("double zero",
                       bbtc_ib_projectile_validate_double(&candidate),
                       BBTC_STATUS_OUTSIDE_DOMAIN) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.mass_kg = -1.0;
    if (require_status("double negative",
                       bbtc_ib_projectile_validate_double(&candidate),
                       BBTC_STATUS_OUTSIDE_DOMAIN) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.mass_kg = NAN;
    if (require_status("double NaN",
                       bbtc_ib_projectile_validate_double(&candidate),
                       BBTC_STATUS_NAN_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.mass_kg = INFINITY;
    if (require_status("double positive infinity",
                       bbtc_ib_projectile_validate_double(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.mass_kg = -INFINITY;
    if (require_status("double negative infinity",
                       bbtc_ib_projectile_validate_double(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.mass_kg = DBL_TRUE_MIN;
    if (require_status("double positive subnormal",
                       bbtc_ib_projectile_validate_double(&candidate),
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.mass_kg = DBL_MAX;
    return require_status("double finite maximum",
                          bbtc_ib_projectile_validate_double(&candidate),
                          BBTC_STATUS_SUCCESS);
}


static int
test_long_double_projectile(void)
{
    const bbtc_ib_projectile_long_double_t valid =
    {
        .mass_kg = 0.01134L
    };
    bbtc_ib_projectile_long_double_t candidate = valid;

    if (require_status("long double null",
                       bbtc_ib_projectile_validate_long_double(NULL),
                       BBTC_STATUS_INVALID_ARGUMENT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (require_status("long double valid",
                       bbtc_ib_projectile_validate_long_double(&candidate),
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (candidate.mass_kg != valid.mass_kg)
    {
        fprintf(stderr, "long-double validation modified its input\n");
        return EXIT_FAILURE;
    }

    candidate.mass_kg = 0.0L;
    if (require_status("long double zero",
                       bbtc_ib_projectile_validate_long_double(&candidate),
                       BBTC_STATUS_OUTSIDE_DOMAIN) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.mass_kg = -1.0L;
    if (require_status("long double negative",
                       bbtc_ib_projectile_validate_long_double(&candidate),
                       BBTC_STATUS_OUTSIDE_DOMAIN) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.mass_kg = NAN;
    if (require_status("long double NaN",
                       bbtc_ib_projectile_validate_long_double(&candidate),
                       BBTC_STATUS_NAN_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.mass_kg = INFINITY;
    if (require_status("long double positive infinity",
                       bbtc_ib_projectile_validate_long_double(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.mass_kg = -INFINITY;
    if (require_status("long double negative infinity",
                       bbtc_ib_projectile_validate_long_double(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.mass_kg = LDBL_TRUE_MIN;
    if (require_status("long double positive subnormal",
                       bbtc_ib_projectile_validate_long_double(&candidate),
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate.mass_kg = LDBL_MAX;
    return require_status("long double finite maximum",
                          bbtc_ib_projectile_validate_long_double(&candidate),
                          BBTC_STATUS_SUCCESS);
}


int
main(void)
{
    if (test_float_projectile() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_double_projectile() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    return test_long_double_projectile();
}
