/**
 * @file
 * @brief Tests the IB0.4c initial condensed-propellant condition contract.
 */
#include <bbtc/bbtc.h>

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

static_assert(_Generic(
    ((bbtc_ib_initial_propellant_condition_float_t*)0)->temperature_k,
    float: 1,
    default: 0
));
static_assert(_Generic(
    ((bbtc_ib_initial_propellant_condition_double_t*)0)->temperature_k,
    double: 1,
    default: 0
));
static_assert(_Generic(
    ((bbtc_ib_initial_propellant_condition_long_double_t*)0)->temperature_k,
    long double: 1,
    default: 0
));

/* The first public member begins at offset zero; exact total struct size is not
 * part of the contract and is deliberately not asserted. */
static_assert(offsetof(
    bbtc_ib_initial_propellant_condition_float_t,
    temperature_k
) == 0);
static_assert(offsetof(
    bbtc_ib_initial_propellant_condition_double_t,
    temperature_k
) == 0);
static_assert(offsetof(
    bbtc_ib_initial_propellant_condition_long_double_t,
    temperature_k
) == 0);

/** @brief Reports one status mismatch with a readable case label. */
static int
require_status(
    const char* const label,
    const bbtc_status_e actual,
    const bbtc_status_e expected
)
{
    if (actual == expected)
        return EXIT_SUCCESS;

    fprintf(
        stderr,
        "%s returned %u; expected %u\n",
        label,
        (unsigned int)actual,
        (unsigned int)expected
    );
    return EXIT_FAILURE;
}

/**
 * @brief Defines one complete native-scalar validation test.
 *
 * @details
 * Each generated test covers null, 293.15 K, immutability, another large but
 * finite positive value (proving no arbitrary upper cap), the implementation's
 * positive representable floor, both signed zeros, a negative finite value,
 * NaN, and both infinities.
 */
#define DEFINE_CONDITION_TEST(                                         \
    name, type, validator, valid_value, high_value, true_min_value     \
)                                                                      \
static int                                                             \
test_##name##_condition(void)                                          \
{                                                                      \
    const type valid = { .temperature_k = (valid_value) };             \
    type candidate = valid;                                            \
                                                                       \
    if (require_status(#name " null", validator(NULL),                \
                       BBTC_STATUS_INVALID_ARGUMENT) != EXIT_SUCCESS)   \
        return EXIT_FAILURE;                                           \
                                                                       \
    if (require_status(#name " 293.15 K", validator(&candidate),      \
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)            \
        return EXIT_FAILURE;                                           \
                                                                       \
    if (candidate.temperature_k != valid.temperature_k)                \
    {                                                                  \
        fputs(#name " validation modified its input\n", stderr);      \
        return EXIT_FAILURE;                                           \
    }                                                                  \
                                                                       \
    candidate.temperature_k = (high_value);                            \
    if (require_status(#name " arbitrary positive finite",            \
                       validator(&candidate),                           \
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)            \
        return EXIT_FAILURE;                                           \
                                                                       \
    candidate.temperature_k = (true_min_value);                        \
    if (require_status(#name " positive representable floor",         \
                       validator(&candidate),                           \
                       BBTC_STATUS_SUCCESS) != EXIT_SUCCESS)            \
        return EXIT_FAILURE;                                           \
                                                                       \
    candidate.temperature_k = 0.0;                                    \
    if (require_status(#name " +0", validator(&candidate),            \
                       BBTC_STATUS_OUTSIDE_DOMAIN) != EXIT_SUCCESS)     \
        return EXIT_FAILURE;                                           \
                                                                       \
    candidate.temperature_k = -0.0;                                   \
    if (require_status(#name " -0", validator(&candidate),            \
                       BBTC_STATUS_OUTSIDE_DOMAIN) != EXIT_SUCCESS)     \
        return EXIT_FAILURE;                                           \
                                                                       \
    candidate.temperature_k = -(valid_value);                          \
    if (require_status(#name " negative", validator(&candidate),      \
                       BBTC_STATUS_OUTSIDE_DOMAIN) != EXIT_SUCCESS)     \
        return EXIT_FAILURE;                                           \
                                                                       \
    candidate.temperature_k = NAN;                                    \
    if (require_status(#name " NaN", validator(&candidate),           \
                       BBTC_STATUS_NAN_INPUT) != EXIT_SUCCESS)          \
        return EXIT_FAILURE;                                           \
                                                                       \
    candidate.temperature_k = INFINITY;                               \
    if (require_status(#name " +infinity", validator(&candidate),     \
                       BBTC_STATUS_NONFINITE_INPUT) != EXIT_SUCCESS)    \
        return EXIT_FAILURE;                                           \
                                                                       \
    candidate.temperature_k = -INFINITY;                              \
    return require_status(#name " -infinity", validator(&candidate),  \
                          BBTC_STATUS_NONFINITE_INPUT);                 \
}

DEFINE_CONDITION_TEST(
    float,
    bbtc_ib_initial_propellant_condition_float_t,
    bbtc_ib_initial_propellant_condition_validate_float,
    293.15f,
    1000000.0f,
    FLT_TRUE_MIN
)

DEFINE_CONDITION_TEST(
    double,
    bbtc_ib_initial_propellant_condition_double_t,
    bbtc_ib_initial_propellant_condition_validate_double,
    293.15,
    1000000.0,
    DBL_TRUE_MIN
)

DEFINE_CONDITION_TEST(
    long_double,
    bbtc_ib_initial_propellant_condition_long_double_t,
    bbtc_ib_initial_propellant_condition_validate_long_double,
    293.15L,
    1000000.0L,
    LDBL_TRUE_MIN
)

#undef DEFINE_CONDITION_TEST

/** @brief Runs the complete IB0.4c primitive-condition contract test. */
int
main(void)
{
    if (test_float_condition() != EXIT_SUCCESS)
        return EXIT_FAILURE;
    if (test_double_condition() != EXIT_SUCCESS)
        return EXIT_FAILURE;
    if (test_long_double_condition() != EXIT_SUCCESS)
        return EXIT_FAILURE;
    return EXIT_SUCCESS;
}
