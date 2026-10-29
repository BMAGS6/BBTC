/**
 * @file comparison_api_oracle_driver.c
 * @brief Lossless, line-oriented driver for B1b.3d's complete-API oracle.
 *
 * @details
 * This is a TRANSPORT ADAPTER, not a mathematical oracle. Each input record
 * supplies native hexadecimal scalar values, a comparison method, whether
 * either required pointer is null, an incoming rounding mode, and a floating-
 * point exception preset. Python independently determines the expected API
 * status, exact finding, metric classification, and rounded native value.
 *
 * Input protocol (one record per line, whitespace-separated):
 *
 *   FAMILY METHOD NULL_POLICY NULL_RESULT ROUNDING FLAGS R O T
 *
 * FAMILY = F (float), D (double), L (long double).
 * METHOD = integer enumerator (0..255); nullable flags = 0 or 1.
 * ROUNDING = 0 nearest, 1 upward, 2 downward, 3 toward zero.
 * FLAGS = 0 none, 1 inexact, 2 invalid|underflow, 3 overflow|divbyzero.
 * R/O/T = exactly representable C hexadecimal floats, +/-inf, or nan.
 *
 * Output protocol:
 *
 *   STATUS FINDING METRIC_STATE NATIVE_HEX ENVIRONMENT_PRESERVED
 *
 * Native values are transported using %a/%La, never decimal. The F output
 * promotes float to double for printf (a mathematically exact promotion).
 * The final flag is 1 only when BOTH the original rounding mode and the
 * original sticky exception mask survive the complete API invocation.
 *
 * No C calculation here establishes an expected mathematical result. An
 * invalid input record or failure of the test apparatus exits nonzero rather
 * than being silently reinterpreted as a VVUQ operation failure.
 *
 * The harness is restricted to the previously qualified binary32, binary64,
 * and x87 long-double environments. Its parser reads precisely representable
 * hex tokens before the simulated caller's FP state is installed, keeping
 * strto* conversion from influencing the environment-preservation check.
 */
#include "comparison.h"

#include <fenv.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined(__FAST_MATH__)
    #error "Comparison API oracle driver requires rigorous FP semantics"
#endif

#if defined(__clang__)
    #pragma STDC FENV_ACCESS ON
#endif

/** Maximum bounded input record, with room for additional whitespace. */
#define VV_API_ORACLE_LINE_CAPACITY 1024
/** Input scalar token limit includes the null terminator. */
#define VV_API_ORACLE_TOKEN_CAPACITY 256

/**
 * @brief Translate the protocol's independent rounding-mode index.
 * @param index The documented protocol ordinal (0..3).
 * @param out_mode Required result for a supported index.
 * @return true for recognized indices; false for malformed records.
 */
static bool select_rounding(unsigned index, int* out_mode)
{
    switch (index)
    {
        case 0: *out_mode = FE_TONEAREST; return true;
        case 1: *out_mode = FE_UPWARD; return true;
        case 2: *out_mode = FE_DOWNWARD; return true;
        case 3: *out_mode = FE_TOWARDZERO; return true;
        default: return false;
    }
}

/**
 * @brief Select test exception presets without relying on numeric FE_* bits.
 *
 * FE_* macro values are implementation-defined, so Python sends ordinal
 * selections, and the C driver resolves them on the target platform.
 */
static bool select_flags(unsigned index, int* out_flags)
{
    switch (index)
    {
        case 0: *out_flags = 0; return true;
        case 1: *out_flags = FE_INEXACT; return true;
        case 2: *out_flags = FE_INVALID | FE_UNDERFLOW; return true;
        case 3: *out_flags = FE_OVERFLOW | FE_DIVBYZERO; return true;
        default: return false;
    }
}

/**
 * @brief Check a hexadecimal floating token reached its terminating NUL.
 *
 * The parser's syntactic success test also admits "inf" and "nan", which
 * are needed to validate API numeric-precedence rules. ERANGE is deliberately
 * NOT treated as a parsing error because an exactly representable smallest
 * subnormal can set errno while remaining a valid native floating value.
 */
static bool consumed_token(const char* end)
{
    return end != NULL && *end == '\0';
}

/**
 * @brief Instantiate a typed native-precision transport and API invocation.
 *
 * All token conversions happen while the harness retains its ordinary FP
 * environment. Only after parsing does the driver install the caller's
 * requested rounding mode and sticky flags. It checks environment equality
 * immediately after the API return, before any printf formatting.
 *
 * The prefilled result deliberately contains NONZERO fields. Thus failure
 * paths are checked for canonical field clearing by the Python oracle.
 * Null output paths use an independent zero-valued sentinel for transport:
 * the API is never required to write through an absent pointer.
 */
#define DEFINE_API_INVOCATION(NAME, TYPE, PARSE, PRINT_FORMAT, PRINT_VALUE)    \
    static bool invoke_##NAME(                                                 \
        const char* r_text, const char* o_text, const char* t_text,            \
        unsigned method, bool null_policy, bool null_result,                  \
        int requested_round, int preset_flags)                                 \
    {                                                                          \
        char* end = NULL;                                                      \
        const TYPE reference = PARSE(r_text, &end);                            \
        if (!consumed_token(end)) return false;                                \
        const TYPE observation = PARSE(o_text, &end);                          \
        if (!consumed_token(end)) return false;                                \
        const TYPE tolerance = PARSE(t_text, &end);                            \
        if (!consumed_token(end)) return false;                                \
        bbtc_vv_policy_##NAME##_t policy = {                                   \
            .method = (bbtc_vv_comparison_method_e)method,                    \
            .tolerance = tolerance                                            \
        };                                                                     \
        bbtc_vv_result_##NAME##_t result = {                                   \
            .finding = BBTC_VV_FINDING_FAIL,                                   \
            .metric_state = BBTC_VV_METRIC_FINITE_EXACT,                       \
            .metric = (TYPE)-1                                                 \
        };                                                                     \
        fenv_t original;                                                       \
        if (fegetenv(&original) != 0) return false;                            \
        if (fesetround(requested_round) != 0 ||                               \
            feclearexcept(FE_ALL_EXCEPT) != 0 ||                               \
            (preset_flags != 0 && feraiseexcept(preset_flags) != 0))          \
        {                                                                      \
            (void)fesetenv(&original);                                         \
            return false;                                                      \
        }                                                                      \
        const int before_round = fegetround();                                 \
        const int before_flags = fetestexcept(FE_ALL_EXCEPT);                  \
        const bbtc_vv_operation_status_e status = bbtc_vv_compare_##NAME(      \
            reference, observation, null_policy ? NULL : &policy,             \
            null_result ? NULL : &result);                                    \
        const bool preserved = fegetround() == before_round &&                \
            fetestexcept(FE_ALL_EXCEPT) == before_flags;                       \
        if (fesetenv(&original) != 0) return false;                            \
        if (null_result) result = (bbtc_vv_result_##NAME##_t){0};             \
        printf("%u %u %u " PRINT_FORMAT " %u\n",                            \
            (unsigned)status, (unsigned)result.finding,                        \
            (unsigned)result.metric_state, (PRINT_VALUE),                      \
            preserved ? 1U : 0U);                                             \
        return true;                                                           \
    }

DEFINE_API_INVOCATION(float, float, strtof, "%a", (double)result.metric)
DEFINE_API_INVOCATION(double, double, strtod, "%a", result.metric)
DEFINE_API_INVOCATION(long_double, long double, strtold, "%La", result.metric)

#undef DEFINE_API_INVOCATION

/**
 * @brief Process all oracle requests from standard input, one at a time.
 *
 * Bounded %s widths prevent the oracle itself from creating an out-of-bounds
 * write if test input is corrupt. The trailing %n and whitespace inspection
 * prohibit silently dropping extra fields. Unrecognized scalar families,
 * modes, presets, and pointer flags are harness errors, not API results.
 */
int main(void)
{
    char line[VV_API_ORACLE_LINE_CAPACITY] = {0};
    while (fgets(line, sizeof(line), stdin) != NULL)
    {
        char family = '\0';
        unsigned method = 0;
        unsigned null_policy = 0;
        unsigned null_result = 0;
        unsigned rounding_index = 0;
        unsigned flags_index = 0;
        char r_text[VV_API_ORACLE_TOKEN_CAPACITY] = {0};
        char o_text[VV_API_ORACLE_TOKEN_CAPACITY] = {0};
        char t_text[VV_API_ORACLE_TOKEN_CAPACITY] = {0};
        int consumed = 0;
        const int fields = sscanf(line,
            " %c %u %u %u %u %u %255s %255s %255s %n",
            &family, &method, &null_policy, &null_result,
            &rounding_index, &flags_index, r_text, o_text, t_text, &consumed);
        if (fields != 9 || consumed <= 0 ||
            method > UINT8_MAX || null_policy > 1 || null_result > 1)
        {
            fputs("comparison API oracle: malformed input record\n", stderr);
            return EXIT_FAILURE;
        }
        for (const char* tail = line + consumed; *tail != '\0'; ++tail)
        {
            if (*tail != ' ' && *tail != '\t' && *tail != '\r' && *tail != '\n')
            {
                fputs("comparison API oracle: unexpected trailing text\n", stderr);
                return EXIT_FAILURE;
            }
        }
        int requested_round = 0;
        int preset_flags = 0;
        if (!select_rounding(rounding_index, &requested_round) ||
            !select_flags(flags_index, &preset_flags))
        {
            fputs("comparison API oracle: invalid environment preset\n", stderr);
            return EXIT_FAILURE;
        }
        bool ok = false;
        if (family == 'F')
            ok = invoke_float(r_text, o_text, t_text, method,
                null_policy != 0, null_result != 0, requested_round, preset_flags);
        else if (family == 'D')
            ok = invoke_double(r_text, o_text, t_text, method,
                null_policy != 0, null_result != 0, requested_round, preset_flags);
        else if (family == 'L')
            ok = invoke_long_double(r_text, o_text, t_text, method,
                null_policy != 0, null_result != 0, requested_round, preset_flags);
        if (!ok)
        {
            fputs("comparison API oracle: native invocation failed\n", stderr);
            return EXIT_FAILURE;
        }
    }
    if (ferror(stdin))
        return EXIT_FAILURE;
    return EXIT_SUCCESS;
}
