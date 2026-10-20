/**
 * @file dyadic_oracle_driver.c
 * @brief Plain-text transport for an INDEPENDENT Python exact-rational oracle.
 *
 * @details
 * This executable is a test-only adapter, not an oracle and not a production
 * BBTC component. Python generates exactly representable binary operands,
 * computes its own expectations using fractions.Fraction, and passes each
 * operand as an exact hexadecimal C floating-point token of the form
 *     [-]0xSIGNIFICANDpBINARY_EXPONENT
 * The C library's strtof/strtod/strtold converts the dyadic input into the
 * target native family. Because each generated value is exactly representable,
 * parsing must not require rounding. Separate startup checks in Python and
 * this driver reject incompatible toolchains rather than pretending success.
 *
 * Protocol: one line per request, five whitespace-delimited fields:
 *     FAMILY METHOD REFERENCE OBSERVATION TOLERANCE
 * FAMILY is F/D/L; METHOD is 1/2/3. Output is exactly "STATUS PASS" per line,
 * both decimal integers. A fatal protocol/transport error exits nonzero.
 * A comparison returning unsupported-environment is a *test result* (and the
 * Python harness will fail unless it was deliberately expected).
 *
 * Do NOT use this text protocol as a validation-data fixture format. It
 * deliberately omits units, provenance, uncertainty, and decimal history.
 */
#include "exact_dyadic.h"

#include <float.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

/** @brief Maximum request line, including newline and terminator. */
#define BBTC_VV_LINE_SIZE 512
/** @brief Maximum length of one input's hexadecimal token plus NUL. */
#define BBTC_VV_TOKEN_SIZE 128

/**
 * @brief Verify parsing consumed an entire ASCII token.
 *
 * Floating-point parsers may legally set ERANGE for exactly representable
 * subnormals; errno is therefore not used to reject these values.
 */
static bool token_consumed(const char* token, const char* end)
{
    return end != token && *end == '\0';
}

/**
 * @brief Convert one C23-native scalar family without narrowing through double.
 *
 * `method` is validated by the kernel, so this adapter passes through even
 * unknown in-range method codes. Fatal parser failures never become ordinary
 * comparison statuses.
 */
static bool run_request(
    char family,
    unsigned int method,
    const char* reference_token,
    const char* observation_token,
    const char* tolerance_token,
    bbtc_vv_dyadic_status_e* out_status,
    bool* out_pass
)
{
    char* end = NULL;
    const bbtc_vv_dyadic_method_e selected = (bbtc_vv_dyadic_method_e)method;

    if (family == 'F')
    {
        const float r = strtof(reference_token, &end);
        if (!token_consumed(reference_token, end))
            return false;
        const float o = strtof(observation_token, &end);
        if (!token_consumed(observation_token, end))
            return false;
        const float t = strtof(tolerance_token, &end);
        if (!token_consumed(tolerance_token, end))
            return false;
        *out_status = bbtc_vv_dyadic_compare_float(selected, r, o, t, out_pass);
        return true;
    }
    if (family == 'D')
    {
        const double r = strtod(reference_token, &end);
        if (!token_consumed(reference_token, end))
            return false;
        const double o = strtod(observation_token, &end);
        if (!token_consumed(observation_token, end))
            return false;
        const double t = strtod(tolerance_token, &end);
        if (!token_consumed(tolerance_token, end))
            return false;
        *out_status = bbtc_vv_dyadic_compare_double(selected, r, o, t, out_pass);
        return true;
    }
    if (family == 'L')
    {
        const long double r = strtold(reference_token, &end);
        if (!token_consumed(reference_token, end))
            return false;
        const long double o = strtold(observation_token, &end);
        if (!token_consumed(observation_token, end))
            return false;
        const long double t = strtold(tolerance_token, &end);
        if (!token_consumed(tolerance_token, end))
            return false;
        *out_status = bbtc_vv_dyadic_compare_long_double(selected, r, o, t, out_pass);
        return true;
    }
    return false;
}

/**
 * @brief Transport entry point; stdout contains only machine-readable replies.
 *
 * The capability banner on stderr permits automated compatibility checking
 * independently of any test failure. This is intentionally limited to the
 * initial three native binary formats and is not an architecture certificate.
 */
int main(void)
{
    if (FLT_RADIX != 2 || FLT_MANT_DIG != 24 || DBL_MANT_DIG != 53 ||
        LDBL_MANT_DIG != 64 || LDBL_MAX_EXP != 16384)
    {
        fputs("oracle driver: unsupported native floating-point family\n", stderr);
        return EXIT_FAILURE;
    }

    fprintf(stderr,
        "oracle driver: float=%d double=%d long_double=%d radix=%d\n",
        FLT_MANT_DIG, DBL_MANT_DIG, LDBL_MANT_DIG, FLT_RADIX);

    char line[BBTC_VV_LINE_SIZE];
    unsigned long long count = 0;
    while (fgets(line, sizeof(line), stdin) != NULL)
    {
        char family = '\0';
        unsigned int method = 0;
        char r[BBTC_VV_TOKEN_SIZE];
        char o[BBTC_VV_TOKEN_SIZE];
        char t[BBTC_VV_TOKEN_SIZE];
        char extra = '\0';

        const int parsed = sscanf(
            line, " %c %u %127s %127s %127s %c",
            &family, &method, r, o, t, &extra
        );
        if (parsed != 5)
        {
            fprintf(stderr, "oracle driver: malformed line %llu\n", count + 1);
            return EXIT_FAILURE;
        }

        bool pass = false;
        bbtc_vv_dyadic_status_e status;
        if (!run_request(family, method, r, o, t, &status, &pass))
        {
            fprintf(stderr, "oracle driver: invalid scalar at line %llu\n", count + 1);
            return EXIT_FAILURE;
        }
        if (printf("%u %d\n", (unsigned int)status, pass ? 1 : 0) < 0)
        {
            fputs("oracle driver: output write failed\n", stderr);
            return EXIT_FAILURE;
        }
        ++count;
    }
    if (ferror(stdin))
    {
        fputs("oracle driver: input read failed\n", stderr);
        return EXIT_FAILURE;
    }
    if (fflush(stdout) != 0)
        return EXIT_FAILURE;
    fprintf(stderr, "oracle driver: processed %llu requests\n", count);
    return EXIT_SUCCESS;
}
