#include <bbtc/precision.h>

#include <float.h>
#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define ARRAY_COUNT(array) (sizeof(array) / sizeof((array)[0]))

typedef struct precision_case_t
{
    bbtc_precision_e precision;
    const char* expected_string;
    uint32_t expected_radix;
    uint32_t expected_mantissa_digits;
    int32_t expected_minimum_normal_exponent;
    int32_t expected_maximum_finite_exponent;
    uint32_t expected_decimal_digits;
    uint32_t expected_storage_bytes;
} precision_case_t;

static_assert(sizeof(bbtc_precision_e) == sizeof(uint8_t));
static_assert(BBTC_PRECISION_FLOAT == 1);
static_assert(BBTC_PRECISION_DOUBLE == 2);
static_assert(BBTC_PRECISION_LONG_DOUBLE == 3);

static_assert(
    _Generic(
        bbtc_precision_string(BBTC_PRECISION_DOUBLE),
        const char*: 1,
        default: 0
    )
);

static_assert(
    _Generic(
        bbtc_precision_info(
            BBTC_PRECISION_DOUBLE,
            (bbtc_precision_info_t*)0
        ),
        bbtc_status_e: 1,
        default: 0
    )
);

static int
check_precision_string(
    const bbtc_precision_e precision,
    const char* const expected
)
{
    const char* const actual = bbtc_precision_string(precision);

    if (actual == NULL)
    {
        fprintf(
            stderr,
            "bbtc_precision_string(%" PRIuMAX ") returned null\n",
            (uintmax_t)(uint8_t)precision
        );
        return EXIT_FAILURE;
    }

    if (strcmp(actual, expected) != 0)
    {
        fprintf(
            stderr,
            "bbtc_precision_string(%" PRIuMAX
            ") returned \"%s\"; expected \"%s\"\n",
            (uintmax_t)(uint8_t)precision,
            actual,
            expected
        );
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

static int
check_precision_info(const precision_case_t* const expected)
{
    bbtc_precision_info_t actual = {0};

    if (
        bbtc_precision_info(expected->precision, &actual)
        != BBTC_STATUS_SUCCESS
    )
    {
        fprintf(stderr, "bbtc_precision_info() returned failure\n");
        return EXIT_FAILURE;
    }

    if (
        actual.precision != expected->precision
        || actual.radix != expected->expected_radix
        || actual.mantissa_digits != expected->expected_mantissa_digits
        || actual.minimum_normal_exponent
            != expected->expected_minimum_normal_exponent
        || actual.maximum_finite_exponent
            != expected->expected_maximum_finite_exponent
        || actual.decimal_digits != expected->expected_decimal_digits
        || actual.storage_bytes != expected->expected_storage_bytes
    )
    {
        fprintf(
            stderr,
            "bbtc_precision_info(%" PRIuMAX ") returned incorrect metadata\n",
            (uintmax_t)(uint8_t)expected->precision
        );
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

static int
precision_info_is_zero(const bbtc_precision_info_t* const info)
{
    return (
        (uint8_t)info->precision == UINT8_C(0)
        && info->radix == UINT32_C(0)
        && info->mantissa_digits == UINT32_C(0)
        && info->minimum_normal_exponent == INT32_C(0)
        && info->maximum_finite_exponent == INT32_C(0)
        && info->decimal_digits == UINT32_C(0)
        && info->storage_bytes == UINT32_C(0)
    );
}

static int
check_invalid_precision(const bbtc_precision_e precision)
{
    bbtc_precision_info_t info =
    {
        .precision = BBTC_PRECISION_DOUBLE,
        .radix = UINT32_MAX,
        .mantissa_digits = UINT32_MAX,
        .minimum_normal_exponent = INT32_MIN,
        .maximum_finite_exponent = INT32_MAX,
        .decimal_digits = UINT32_MAX,
        .storage_bytes = UINT32_MAX
    };

    if (
        bbtc_precision_info(precision, &info)
        != BBTC_STATUS_INVALID_ARGUMENT
    )
    {
        fprintf(stderr, "invalid precision did not return invalid argument\n");
        return EXIT_FAILURE;
    }

    if (!precision_info_is_zero(&info))
    {
        fprintf(stderr, "invalid precision did not clear the output record\n");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

int
main(void)
{
    static const precision_case_t cases[] =
    {
        {
            BBTC_PRECISION_FLOAT,
            "float",
            (uint32_t)FLT_RADIX,
            (uint32_t)FLT_MANT_DIG,
            (int32_t)FLT_MIN_EXP,
            (int32_t)FLT_MAX_EXP,
            (uint32_t)FLT_DECIMAL_DIG,
            (uint32_t)sizeof(float)
        },
        {
            BBTC_PRECISION_DOUBLE,
            "double",
            (uint32_t)FLT_RADIX,
            (uint32_t)DBL_MANT_DIG,
            (int32_t)DBL_MIN_EXP,
            (int32_t)DBL_MAX_EXP,
            (uint32_t)DBL_DECIMAL_DIG,
            (uint32_t)sizeof(double)
        },
        {
            BBTC_PRECISION_LONG_DOUBLE,
            "long double",
            (uint32_t)FLT_RADIX,
            (uint32_t)LDBL_MANT_DIG,
            (int32_t)LDBL_MIN_EXP,
            (int32_t)LDBL_MAX_EXP,
            (uint32_t)LDBL_DECIMAL_DIG,
            (uint32_t)sizeof(long double)
        }
    };

    for (size_t i = 0; i < ARRAY_COUNT(cases); ++i)
    {
        if (
            check_precision_string(
                cases[i].precision,
                cases[i].expected_string
            ) != EXIT_SUCCESS
            || check_precision_info(&cases[i]) != EXIT_SUCCESS
        )
        {
            return EXIT_FAILURE;
        }
    }

    if (
        check_precision_string(
            (bbtc_precision_e)0,
            "unknown BBTC precision"
        ) != EXIT_SUCCESS
        || check_precision_string(
            (bbtc_precision_e)UINT8_MAX,
            "unknown BBTC precision"
        ) != EXIT_SUCCESS
        || check_invalid_precision((bbtc_precision_e)0) != EXIT_SUCCESS
        || check_invalid_precision((bbtc_precision_e)UINT8_MAX)
            != EXIT_SUCCESS
    )
    {
        return EXIT_FAILURE;
    }

    return (
        bbtc_precision_info(BBTC_PRECISION_DOUBLE, NULL)
        == BBTC_STATUS_INVALID_ARGUMENT
    ) ? EXIT_SUCCESS : EXIT_FAILURE;
}
