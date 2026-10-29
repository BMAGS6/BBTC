/**
 * @file exact_round_oracle_driver.c
 * @brief Lossless, line-oriented transport for the independent rounding oracle.
 *
 * @details
 * Test input: F|D|L NUMERATOR_HEX E DENOMINATOR_HEX F.
 * Represents nonnegative N*2^E/(d*2^F), with N at most 520 64-bit limbs.
 * Replies: STATUS STATE HEX_FLOAT; hexadecimal float output preserves every
 * value bit for binary32/binary64/x87 without a decimal intermediary.
 * This driver is NOT itself a mathematical oracle: Python supplies all
 * reference classifications and correctly rounded expected values.
 */
#include "exact_round.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define ROUND_HEX_CAPACITY (BBTC_VV_METRIC_LIMBS * 16 + 2)

/** @brief Parse an unsigned hexadecimal integer into the exact limb layout. */
static bool parse_numerator(const char* text, bbtc_vv_exact_metric_t* out)
{
    const size_t length = strlen(text);
    if (length == 0 || length > BBTC_VV_METRIC_LIMBS * 16)
        return false;
    for (size_t i = 0; i < length; ++i)
    {
        const char ch = text[length - 1 - i];
        unsigned nibble = 0;
        if (ch >= '0' && ch <= '9') nibble = (unsigned)(ch - '0');
        else if (ch >= 'a' && ch <= 'f') nibble = (unsigned)(ch - 'a') + 10U;
        else if (ch >= 'A' && ch <= 'F') nibble = (unsigned)(ch - 'A') + 10U;
        else return false;
        out->word[i / 16] |= (uint64_t)nibble << (unsigned)((i % 16) * 4);
    }
    for (size_t i = BBTC_VV_METRIC_LIMBS; i != 0; --i)
    {
        if (out->word[i - 1] != 0)
        {
            out->used = (uint32_t)i;
            break;
        }
    }
    return true;
}

int main(void)
{
    char family = '\0';
    char numerator[ROUND_HEX_CAPACITY] = {0};
    int32_t num_exp = 0;
    uint64_t denominator = 0;
    int32_t den_exp = 0;

    /* Scan a complete record; sscanf-like partial records never count. */
    while (scanf(" %c %8320s %" SCNd32 " %" SCNx64 " %" SCNd32,
        &family, numerator, &num_exp, &denominator, &den_exp) == 5)
    {
        bbtc_vv_exact_metric_t exact = {0};
        if (!parse_numerator(numerator, &exact))
        {
            fputs("round driver: invalid numerator\n", stderr);
            return EXIT_FAILURE;
        }
        exact.numerator_exponent = num_exp;
        exact.denominator = denominator;
        exact.denominator_exponent = den_exp;
        bbtc_vv_metric_state_e state = BBTC_VV_METRIC_NOT_COMPUTED;
        if (family == 'F')
        {
            float value = 0;
            const bbtc_vv_operation_status_e status =
                bbtc_vv_metric_round_float(&exact, &state, &value);
            printf("%u %u %a\n", (unsigned)status, (unsigned)state, (double)value);
        }
        else if (family == 'D')
        {
            double value = 0;
            const bbtc_vv_operation_status_e status =
                bbtc_vv_metric_round_double(&exact, &state, &value);
            printf("%u %u %a\n", (unsigned)status, (unsigned)state, value);
        }
        else if (family == 'L')
        {
            long double value = 0;
            const bbtc_vv_operation_status_e status =
                bbtc_vv_metric_round_long_double(&exact, &state, &value);
            printf("%u %u %La\n", (unsigned)status, (unsigned)state, value);
        }
        else
        {
            fputs("round driver: unknown family\n", stderr);
            return EXIT_FAILURE;
        }
    }
    if (ferror(stdin))
        return EXIT_FAILURE;
    return EXIT_SUCCESS;
}
