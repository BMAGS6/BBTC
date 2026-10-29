/**
 * @file exact_metric_oracle_driver.c
 * @brief Test-only exact hexadecimal dyadic transport for Python Fraction.
 *
 * @details
 * Reads one record per line: F|D|L A|R HEX_REFERENCE HEX_OBSERVATION.
 * Each HEX operand is representable exactly by its stated native binary
 * family; Python is the source of test expectations. This executable is a
 * transport only, NEVER an independent oracle by itself.
 *
 * Writes STATUS HEX_NUMERATOR NUMERATOR_EXP HEX_DENOMINATOR DENOMINATOR_EXP.
 * The entire 520-limb numerator is transmitted as a single lossless hex
 * integer with no C floating conversion and no architecture-dependent bytes.
 * An unsupported native family aborts rather than claiming success.
 */
#include "exact_metric.h"

#include <float.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#define TOKEN_SIZE 128

/** @brief Report one full exact rational without omitting high words. */
static void reply(bbtc_vv_operation_status_e status, const bbtc_vv_exact_metric_t* value)
{
    printf("%u ", (unsigned)status);
    if (value->used == 0)
    {
        putchar('0');
    }
    else
    {
        printf("%" PRIx64, value->word[value->used - 1]);
        for (size_t i = value->used - 1; i != 0; --i)
            printf("%016" PRIx64, value->word[i - 1]);
    }
    printf(" %" PRId32 " %" PRIx64 " %" PRId32 "\n",
        value->numerator_exponent, value->denominator,
        value->denominator_exponent);
}

/** @brief Report transport failures as process errors, not comparison statuses. */
static bool consumed(const char* token, const char* end)
{
    return end != token && *end == '\0';
}

int main(void)
{
    if (FLT_RADIX != 2 || FLT_MANT_DIG != 24 || DBL_MANT_DIG != 53 ||
        LDBL_MANT_DIG != 64 || LDBL_MAX_EXP != 16384)
    {
        fputs("metric oracle: initial x87 platform requirement not met\n", stderr);
        return EXIT_FAILURE;
    }

    char family = '\0';
    char mode = '\0';
    char r[TOKEN_SIZE] = {0};
    char o[TOKEN_SIZE] = {0};
    while (scanf(" %c %c %127s %127s", &family, &mode, r, o) == 4)
    {
        const bbtc_vv_comparison_method_e selected =
            mode == 'A' ? BBTC_VV_COMPARISON_ABSOLUTE :
            mode == 'R' ? BBTC_VV_COMPARISON_REFERENCE_RELATIVE :
            (bbtc_vv_comparison_method_e)255;
        bbtc_vv_exact_metric_t result = {0};
        bbtc_vv_operation_status_e status = BBTC_VV_STATUS_INVALID_ARGUMENT;
        char* end = NULL;
        if (family == 'F')
        {
            const float reference = strtof(r, &end);
            if (!consumed(r, end)) return EXIT_FAILURE;
            const float observation = strtof(o, &end);
            if (!consumed(o, end)) return EXIT_FAILURE;
            status = bbtc_vv_metric_exact_float(selected, reference, observation, &result);
        }
        else if (family == 'D')
        {
            const double reference = strtod(r, &end);
            if (!consumed(r, end)) return EXIT_FAILURE;
            const double observation = strtod(o, &end);
            if (!consumed(o, end)) return EXIT_FAILURE;
            status = bbtc_vv_metric_exact_double(selected, reference, observation, &result);
        }
        else if (family == 'L')
        {
            const long double reference = strtold(r, &end);
            if (!consumed(r, end)) return EXIT_FAILURE;
            const long double observation = strtold(o, &end);
            if (!consumed(o, end)) return EXIT_FAILURE;
            status = bbtc_vv_metric_exact_long_double(selected, reference, observation, &result);
        }
        else
        {
            fputs("metric oracle: invalid family\n", stderr);
            return EXIT_FAILURE;
        }
        reply(status, &result);
    }
    if (ferror(stdin))
        return EXIT_FAILURE;
    return EXIT_SUCCESS;
}
