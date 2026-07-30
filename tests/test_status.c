#include <bbtc/bbtc.h>

#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define ARRAY_COUNT(array) (sizeof(array) / sizeof((array)[0]))

typedef struct status_case
{
    bbtc_status_e status;
    const char* expected;
} status_case_t;

static_assert(sizeof(bbtc_status_e) == sizeof(uint32_t));
static_assert(BBTC_STATUS_SUCCESS == 0);
static_assert(BBTC_STATUS_INVALID_ARGUMENT == 1);
static_assert(BBTC_STATUS_NONFINITE_INPUT == 2);
static_assert(BBTC_STATUS_OUTSIDE_DOMAIN == 3);
static_assert(BBTC_STATUS_INCONSISTENT_CONFIGURATION == 4);
static_assert(BBTC_STATUS_UNSUPPORTED_MODEL_OR_OPTION == 5);
static_assert(BBTC_STATUS_INSUFFICIENT_STORAGE == 6);
static_assert(BBTC_STATUS_NUMERICAL_FAILURE == 7);
static_assert(BBTC_STATUS_ITERATION_LIMIT == 8);
static_assert(BBTC_STATUS_INTERNAL_INVARIANT_FAILURE == 9);

static_assert(
    _Generic(
        bbtc_status_string(BBTC_STATUS_SUCCESS),
        const char*: 1,
        default: 0
    )
);

static int
check_status_string(const bbtc_status_e status, const char* const expected)
{
    const char* const actual = bbtc_status_string(status);

    if (actual == NULL)
    {
        fprintf(
            stderr,
            "bbtc_status_string(%" PRIu32 ") returned null\n",
            (uint32_t)status
        );
        return EXIT_FAILURE;
    }

    if (strcmp(actual, expected) != 0)
    {
        fprintf(
            stderr,
            "bbtc_status_string(%" PRIu32
            ") returned \"%s\"; expected \"%s\"\n",
            (uint32_t)status,
            actual,
            expected
        );
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

int
main(void)
{
    static const status_case_t known_cases[] =
    {
        {BBTC_STATUS_SUCCESS, "success"},
        {BBTC_STATUS_INVALID_ARGUMENT, "invalid argument"},
        {BBTC_STATUS_NONFINITE_INPUT, "non-finite input"},
        {
            BBTC_STATUS_OUTSIDE_DOMAIN,
            "value outside mathematical domain"
        },
        {
            BBTC_STATUS_INCONSISTENT_CONFIGURATION,
            "inconsistent geometry or configuration"
        },
        {
            BBTC_STATUS_UNSUPPORTED_MODEL_OR_OPTION,
            "unsupported model or option"
        },
        {
            BBTC_STATUS_INSUFFICIENT_STORAGE,
            "insufficient caller-provided storage"
        },
        {BBTC_STATUS_NUMERICAL_FAILURE, "numerical failure"},
        {
            BBTC_STATUS_ITERATION_LIMIT,
            "iteration or step limit reached"
        },
        {
            BBTC_STATUS_INTERNAL_INVARIANT_FAILURE,
            "internal invariant failure"
        }
    };

    for (size_t i = 0; i < ARRAY_COUNT(known_cases); ++i)
    {
        if (
            check_status_string(
                known_cases[i].status,
                known_cases[i].expected
            ) != EXIT_SUCCESS
        )
        {
            return EXIT_FAILURE;
        }
    }

    if (
        check_status_string(
            (bbtc_status_e)UINT32_C(10),
            "unknown BBTC status"
        ) != EXIT_SUCCESS
    )
    {
        return EXIT_FAILURE;
    }

    if (
        check_status_string(
            (bbtc_status_e)UINT32_MAX,
            "unknown BBTC status"
        ) != EXIT_SUCCESS
    )
    {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
