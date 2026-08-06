/**
 * @file
 * @brief Tests precision-qualified Noble-Abel gas-model records.
 */

#include <bbtc/bbtc.h>

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

static_assert(
    _Generic(
        ((bbtc_ib_noble_abel_gas_model_float_t*)0)
            ->specific_gas_constant_j_per_kg_k,
        float:   1,
        default: 0
    )
);

static_assert(
    _Generic(
        ((bbtc_ib_noble_abel_gas_model_float_t*)0)
            ->constant_volume_specific_heat_j_per_kg_k,
        float:   1,
        default: 0
    )
);

static_assert(
    _Generic(
        ((bbtc_ib_noble_abel_gas_model_float_t*)0)->covolume_m3_per_kg,
        float:   1,
        default: 0
    )
);

static_assert(
    _Generic(
        ((bbtc_ib_noble_abel_gas_model_double_t*)0)
            ->specific_gas_constant_j_per_kg_k,
        double:  1,
        default: 0
    )
);

static_assert(
    _Generic(
        ((bbtc_ib_noble_abel_gas_model_double_t*)0)
            ->constant_volume_specific_heat_j_per_kg_k,
        double:  1,
        default: 0
    )
);

static_assert(
    _Generic(
        ((bbtc_ib_noble_abel_gas_model_double_t*)0)->covolume_m3_per_kg,
        double:  1,
        default: 0
    )
);

static_assert(
    _Generic(
        ((bbtc_ib_noble_abel_gas_model_long_double_t*)0)
            ->specific_gas_constant_j_per_kg_k,
        long double: 1,
        default:     0
    )
);

static_assert(
    _Generic(
        ((bbtc_ib_noble_abel_gas_model_long_double_t*)0)
            ->constant_volume_specific_heat_j_per_kg_k,
        long double: 1,
        default:     0
    )
);

static_assert(
    _Generic(
        ((bbtc_ib_noble_abel_gas_model_long_double_t*)0)
            ->covolume_m3_per_kg,
        long double: 1,
        default:     0
    )
);


/**
 * @brief Compares one actual status with its expected value.
 *
 * @param label Human-readable test-case label.
 * @param actual Status returned by the API.
 * @param expected Status required by the contract.
 *
 * @return `EXIT_SUCCESS` when the values match; otherwise `EXIT_FAILURE`.
 */
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


/**
 * @brief Tests the native-float Noble-Abel gas-model family.
 *
 * @return `EXIT_SUCCESS` when every validation case passes; otherwise
 *         `EXIT_FAILURE`.
 */
static int
test_float_gas_model(void)
{
    const bbtc_ib_noble_abel_gas_model_float_t valid =
    {
        .specific_gas_constant_j_per_kg_k         = 287.0f,
        .constant_volume_specific_heat_j_per_kg_k = 718.0f,
        .covolume_m3_per_kg                        = 0.001f
    };

    bbtc_ib_noble_abel_gas_model_float_t candidate = valid;

    if (require_status("float NULL",
                       bbtc_ib_noble_abel_gas_model_validate_float(NULL),
                       BBTC_STATUS_INVALID_ARGUMENT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (require_status("float valid",
                       bbtc_ib_noble_abel_gas_model_validate_float(&candidate),
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (candidate.specific_gas_constant_j_per_kg_k
            != valid.specific_gas_constant_j_per_kg_k
        || candidate.constant_volume_specific_heat_j_per_kg_k
            != valid.constant_volume_specific_heat_j_per_kg_k
        || candidate.covolume_m3_per_kg != valid.covolume_m3_per_kg)
    {
        fprintf(stderr, "float validation modified its input\n");
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.specific_gas_constant_j_per_kg_k = 0.0f;

    if (require_status("float gas constant zero",
                       bbtc_ib_noble_abel_gas_model_validate_float(&candidate),
                       BBTC_STATUS_OUTSIDE_DOMAIN) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.specific_gas_constant_j_per_kg_k = -1.0f;

    if (require_status("float gas constant negative",
                       bbtc_ib_noble_abel_gas_model_validate_float(&candidate),
                       BBTC_STATUS_OUTSIDE_DOMAIN) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.specific_gas_constant_j_per_kg_k = NAN;

    if (require_status("float gas constant NaN",
                       bbtc_ib_noble_abel_gas_model_validate_float(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.specific_gas_constant_j_per_kg_k = INFINITY;

    if (require_status("float gas constant positive infinity",
                       bbtc_ib_noble_abel_gas_model_validate_float(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.specific_gas_constant_j_per_kg_k = -INFINITY;

    if (require_status("float gas constant negative infinity",
                       bbtc_ib_noble_abel_gas_model_validate_float(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.specific_gas_constant_j_per_kg_k = FLT_TRUE_MIN;

    if (require_status("float gas constant positive subnormal",
                       bbtc_ib_noble_abel_gas_model_validate_float(&candidate),
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.specific_gas_constant_j_per_kg_k = FLT_MAX;

    if (require_status("float gas constant finite maximum",
                       bbtc_ib_noble_abel_gas_model_validate_float(&candidate),
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.constant_volume_specific_heat_j_per_kg_k = 0.0f;

    if (require_status("float specific heat zero",
                       bbtc_ib_noble_abel_gas_model_validate_float(&candidate),
                       BBTC_STATUS_OUTSIDE_DOMAIN) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.constant_volume_specific_heat_j_per_kg_k = -1.0f;

    if (require_status("float specific heat negative",
                       bbtc_ib_noble_abel_gas_model_validate_float(&candidate),
                       BBTC_STATUS_OUTSIDE_DOMAIN) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.constant_volume_specific_heat_j_per_kg_k = NAN;

    if (require_status("float specific heat NaN",
                       bbtc_ib_noble_abel_gas_model_validate_float(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.constant_volume_specific_heat_j_per_kg_k = INFINITY;

    if (require_status("float specific heat positive infinity",
                       bbtc_ib_noble_abel_gas_model_validate_float(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.constant_volume_specific_heat_j_per_kg_k = -INFINITY;

    if (require_status("float specific heat negative infinity",
                       bbtc_ib_noble_abel_gas_model_validate_float(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.constant_volume_specific_heat_j_per_kg_k = FLT_TRUE_MIN;

    if (require_status("float specific heat positive subnormal",
                       bbtc_ib_noble_abel_gas_model_validate_float(&candidate),
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.constant_volume_specific_heat_j_per_kg_k = FLT_MAX;

    if (require_status("float specific heat finite maximum",
                       bbtc_ib_noble_abel_gas_model_validate_float(&candidate),
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.covolume_m3_per_kg = 0.0f;

    if (require_status("float zero covolume ideal-gas limit",
                       bbtc_ib_noble_abel_gas_model_validate_float(&candidate),
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.covolume_m3_per_kg = -1.0f;

    if (require_status("float covolume negative",
                       bbtc_ib_noble_abel_gas_model_validate_float(&candidate),
                       BBTC_STATUS_OUTSIDE_DOMAIN) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.covolume_m3_per_kg = NAN;

    if (require_status("float covolume NaN",
                       bbtc_ib_noble_abel_gas_model_validate_float(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.covolume_m3_per_kg = INFINITY;

    if (require_status("float covolume positive infinity",
                       bbtc_ib_noble_abel_gas_model_validate_float(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.covolume_m3_per_kg = -INFINITY;

    if (require_status("float covolume negative infinity",
                       bbtc_ib_noble_abel_gas_model_validate_float(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.covolume_m3_per_kg = FLT_TRUE_MIN;

    if (require_status("float covolume positive subnormal",
                       bbtc_ib_noble_abel_gas_model_validate_float(&candidate),
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.covolume_m3_per_kg = FLT_MAX;

    return require_status("float covolume finite maximum",
                          bbtc_ib_noble_abel_gas_model_validate_float(&candidate),
                          BBTC_STATUS_SUCCESS);
}


/**
 * @brief Tests the native-double Noble-Abel gas-model family.
 *
 * @return `EXIT_SUCCESS` when every validation case passes; otherwise
 *         `EXIT_FAILURE`.
 */
static int
test_double_gas_model(void)
{
    const bbtc_ib_noble_abel_gas_model_double_t valid =
    {
        .specific_gas_constant_j_per_kg_k          = 287.0,
        .constant_volume_specific_heat_j_per_kg_k  = 718.0,
        .covolume_m3_per_kg                        = 0.001
    };

    bbtc_ib_noble_abel_gas_model_double_t candidate = valid;

    if (require_status("double NULL",
                       bbtc_ib_noble_abel_gas_model_validate_double(NULL),
                       BBTC_STATUS_INVALID_ARGUMENT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (require_status("double valid",
                       bbtc_ib_noble_abel_gas_model_validate_double(&candidate),
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (candidate.specific_gas_constant_j_per_kg_k !=
            valid.specific_gas_constant_j_per_kg_k
        ||
        candidate.constant_volume_specific_heat_j_per_kg_k !=
            valid.constant_volume_specific_heat_j_per_kg_k
        ||
        candidate.covolume_m3_per_kg != valid.covolume_m3_per_kg)
    {
        fprintf(stderr, "double validation modified its input\n");
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.specific_gas_constant_j_per_kg_k = 0.0;

    if (require_status("double gas constant zero",
                       bbtc_ib_noble_abel_gas_model_validate_double(&candidate),
                       BBTC_STATUS_OUTSIDE_DOMAIN) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.specific_gas_constant_j_per_kg_k = -1.0;

    if (require_status("double gas constant negative",
                       bbtc_ib_noble_abel_gas_model_validate_double(&candidate),
                       BBTC_STATUS_OUTSIDE_DOMAIN) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.specific_gas_constant_j_per_kg_k = NAN;

    if (require_status("double gas constant NaN",
                       bbtc_ib_noble_abel_gas_model_validate_double(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.specific_gas_constant_j_per_kg_k = INFINITY;

    if (require_status("double gas constant positive infinity",
                       bbtc_ib_noble_abel_gas_model_validate_double(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.specific_gas_constant_j_per_kg_k = -INFINITY;

    if (require_status("double gas constant negative infinity",
                       bbtc_ib_noble_abel_gas_model_validate_double(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.specific_gas_constant_j_per_kg_k = DBL_TRUE_MIN;

    if (require_status("double gas constant positive subnormal",
                       bbtc_ib_noble_abel_gas_model_validate_double(&candidate),
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.specific_gas_constant_j_per_kg_k = DBL_MAX;

    if (require_status("double gas constant finite maximum",
                       bbtc_ib_noble_abel_gas_model_validate_double(&candidate),
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.constant_volume_specific_heat_j_per_kg_k = 0.0;

    if (require_status("double specific heat zero",
                       bbtc_ib_noble_abel_gas_model_validate_double(&candidate),
                       BBTC_STATUS_OUTSIDE_DOMAIN) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.constant_volume_specific_heat_j_per_kg_k = -1.0;

    if (require_status("double specific heat negative",
                       bbtc_ib_noble_abel_gas_model_validate_double(&candidate),
                       BBTC_STATUS_OUTSIDE_DOMAIN) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.constant_volume_specific_heat_j_per_kg_k = NAN;

    if (require_status("double specific heat NaN",
                       bbtc_ib_noble_abel_gas_model_validate_double(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.constant_volume_specific_heat_j_per_kg_k = INFINITY;

    if (require_status("double specific heat positive infinity",
                       bbtc_ib_noble_abel_gas_model_validate_double(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.constant_volume_specific_heat_j_per_kg_k = -INFINITY;

    if (require_status("double specific heat negative infinity",
                       bbtc_ib_noble_abel_gas_model_validate_double(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.constant_volume_specific_heat_j_per_kg_k = DBL_TRUE_MIN;

    if (require_status("double specific heat positive subnormal",
                       bbtc_ib_noble_abel_gas_model_validate_double(&candidate),
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.constant_volume_specific_heat_j_per_kg_k = DBL_MAX;

    if (require_status("double specific heat finite maximum",
                       bbtc_ib_noble_abel_gas_model_validate_double(&candidate),
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.covolume_m3_per_kg = 0.0;

    if (require_status("double zero covolume ideal-gas limit",
                       bbtc_ib_noble_abel_gas_model_validate_double(&candidate),
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.covolume_m3_per_kg = -1.0;

    if (require_status("double covolume negative",
                       bbtc_ib_noble_abel_gas_model_validate_double(&candidate),
                       BBTC_STATUS_OUTSIDE_DOMAIN) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.covolume_m3_per_kg = NAN;

    if (require_status("double covolume NaN",
                       bbtc_ib_noble_abel_gas_model_validate_double(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.covolume_m3_per_kg = INFINITY;

    if (require_status("double covolume positive infinity",
                       bbtc_ib_noble_abel_gas_model_validate_double(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.covolume_m3_per_kg = -INFINITY;

    if (require_status("double covolume negative infinity",
                       bbtc_ib_noble_abel_gas_model_validate_double(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.covolume_m3_per_kg = DBL_TRUE_MIN;

    if (require_status("double covolume positive subnormal",
                       bbtc_ib_noble_abel_gas_model_validate_double(&candidate),
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.covolume_m3_per_kg = DBL_MAX;

    return require_status("double covolume finite maximum",
                          bbtc_ib_noble_abel_gas_model_validate_double(&candidate),
                          BBTC_STATUS_SUCCESS);
}


/**
 * @brief Tests the native-long-double Noble-Abel gas-model family.
 *
 * @return `EXIT_SUCCESS` when every validation case passes; otherwise
 *         `EXIT_FAILURE`.
 */
static int
test_long_double_gas_model(void)
{
    const bbtc_ib_noble_abel_gas_model_long_double_t valid =
    {
        .specific_gas_constant_j_per_kg_k         = 287.0L,
        .constant_volume_specific_heat_j_per_kg_k = 718.0L,
        .covolume_m3_per_kg                        = 0.001L
    };

    bbtc_ib_noble_abel_gas_model_long_double_t candidate = valid;

    if (require_status("long double NULL",
                       bbtc_ib_noble_abel_gas_model_validate_long_double(NULL),
                       BBTC_STATUS_INVALID_ARGUMENT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (require_status("long double valid",
                       bbtc_ib_noble_abel_gas_model_validate_long_double(&candidate),
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (candidate.specific_gas_constant_j_per_kg_k !=
            valid.specific_gas_constant_j_per_kg_k
        ||
        candidate.constant_volume_specific_heat_j_per_kg_k !=
            valid.constant_volume_specific_heat_j_per_kg_k
        ||
        candidate.covolume_m3_per_kg != valid.covolume_m3_per_kg)
    {
        fprintf(stderr, "long double validation modified its input\n");
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.specific_gas_constant_j_per_kg_k = 0.0L;

    if (require_status("long double gas constant zero",
                       bbtc_ib_noble_abel_gas_model_validate_long_double(&candidate),
                       BBTC_STATUS_OUTSIDE_DOMAIN) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.specific_gas_constant_j_per_kg_k = -1.0L;

    if (require_status("long double gas constant negative",
                       bbtc_ib_noble_abel_gas_model_validate_long_double(&candidate),
                       BBTC_STATUS_OUTSIDE_DOMAIN) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.specific_gas_constant_j_per_kg_k = NAN;

    if (require_status("long double gas constant NaN",
                       bbtc_ib_noble_abel_gas_model_validate_long_double(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.specific_gas_constant_j_per_kg_k = INFINITY;

    if (require_status("long double gas constant positive infinity",
                       bbtc_ib_noble_abel_gas_model_validate_long_double(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.specific_gas_constant_j_per_kg_k = -INFINITY;

    if (require_status("long double gas constant negative infinity",
                       bbtc_ib_noble_abel_gas_model_validate_long_double(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.specific_gas_constant_j_per_kg_k = LDBL_TRUE_MIN;

    if (require_status("long double gas constant positive subnormal",
                       bbtc_ib_noble_abel_gas_model_validate_long_double(&candidate),
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.specific_gas_constant_j_per_kg_k = LDBL_MAX;

    if (require_status("long double gas constant finite maximum",
                       bbtc_ib_noble_abel_gas_model_validate_long_double(&candidate),
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.constant_volume_specific_heat_j_per_kg_k = 0.0L;

    if (require_status("long double specific heat zero",
                       bbtc_ib_noble_abel_gas_model_validate_long_double(&candidate),
                       BBTC_STATUS_OUTSIDE_DOMAIN) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.constant_volume_specific_heat_j_per_kg_k = -1.0L;

    if (require_status("long double specific heat negative",
                       bbtc_ib_noble_abel_gas_model_validate_long_double(&candidate),
                       BBTC_STATUS_OUTSIDE_DOMAIN) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.constant_volume_specific_heat_j_per_kg_k = NAN;

    if (require_status("long double specific heat NaN",
                       bbtc_ib_noble_abel_gas_model_validate_long_double(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.constant_volume_specific_heat_j_per_kg_k = INFINITY;

    if (require_status("long double specific heat positive infinity",
                       bbtc_ib_noble_abel_gas_model_validate_long_double(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.constant_volume_specific_heat_j_per_kg_k = -INFINITY;

    if (require_status("long double specific heat negative infinity",
                       bbtc_ib_noble_abel_gas_model_validate_long_double(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.constant_volume_specific_heat_j_per_kg_k = LDBL_TRUE_MIN;

    if (require_status("long double specific heat positive subnormal",
                       bbtc_ib_noble_abel_gas_model_validate_long_double(&candidate),
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.constant_volume_specific_heat_j_per_kg_k = LDBL_MAX;

    if (require_status("long double specific heat finite maximum",
                       bbtc_ib_noble_abel_gas_model_validate_long_double(&candidate),
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.covolume_m3_per_kg = 0.0L;

    if (require_status("long double zero covolume ideal-gas limit",
                       bbtc_ib_noble_abel_gas_model_validate_long_double(&candidate),
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.covolume_m3_per_kg = -1.0L;

    if (require_status("long double covolume negative",
                       bbtc_ib_noble_abel_gas_model_validate_long_double(&candidate),
                       BBTC_STATUS_OUTSIDE_DOMAIN) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.covolume_m3_per_kg = NAN;

    if (require_status("long double covolume NaN",
                       bbtc_ib_noble_abel_gas_model_validate_long_double(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.covolume_m3_per_kg = INFINITY;

    if (require_status("long double covolume positive infinity",
                       bbtc_ib_noble_abel_gas_model_validate_long_double(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.covolume_m3_per_kg = -INFINITY;

    if (require_status("long double covolume negative infinity",
                       bbtc_ib_noble_abel_gas_model_validate_long_double(&candidate),
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.covolume_m3_per_kg = LDBL_TRUE_MIN;

    if (require_status("long double covolume positive subnormal",
                       bbtc_ib_noble_abel_gas_model_validate_long_double(&candidate),
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    candidate = valid;
    candidate.covolume_m3_per_kg = LDBL_MAX;

    return require_status("long double covolume finite maximum",
                          bbtc_ib_noble_abel_gas_model_validate_long_double(&candidate),
                          BBTC_STATUS_SUCCESS);
}


/**
 * @brief Runs the Noble-Abel gas-model validation tests.
 *
 * @return `EXIT_SUCCESS` when every scalar family passes; otherwise
 *         `EXIT_FAILURE`.
 */
int main(void)
{
    if (test_float_gas_model() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_double_gas_model() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    return test_long_double_gas_model();
}
