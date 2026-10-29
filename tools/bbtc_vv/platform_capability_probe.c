/**
 * @file platform_capability_probe.c
 * @brief Report and check the host's candidate B1 numerical environment.
 *
 * @details
 * This developer-only acceptance probe is deliberately separate from the
 * production BBTC library and from the numerical comparison implementation.
 * Its purpose is to record the C implementation's compile-time floating-point
 * parameters and verify a small set of observable <fenv.h> operations.
 *
 * The probe does NOT establish universal arithmetic correctness, prove that
 * every compiler optimization respects the fenv, detect all unmasked traps,
 * or certify a new platform. Those are independent acceptance obligations.
 * It only admits the original binary32/binary64/x87 profile claimed by the
 * initial BBTC VVUQ verification campaign; other profiles fail closed.
 *
 * Compile independently with ISO C23, non-fast-math semantics, and
 * -frounding-math -ffp-contract=off. No BBTC library is linked.
 */
#include <fenv.h>
#include <float.h>
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#if defined(__FAST_MATH__)
    #error "The VVUQ platform probe forbids fast-math"
#endif

#if defined(__FINITE_MATH_ONLY__)
    #if __FINITE_MATH_ONLY__ != 0
        #error "The VVUQ platform probe requires NaN/Inf support"
    #endif
#endif

#if defined(__clang__)
    #pragma STDC FENV_ACCESS ON
#endif

/**
 * @brief Print the compiler-exposed scalar-family characteristics.
 *
 * These macro values constitute *evidence* of a candidate format, not an
 * exhaustive characterization of its representation. In particular,
 * sizeof(long double) by itself cannot establish an x87 encoding.
 */
static void print_format_profile(void)
{
    printf("C_STANDARD=%ld\n", (long)__STDC_VERSION__);
    printf("CHAR_BIT=%d\n", CHAR_BIT);
    printf("FLT_RADIX=%d\n", FLT_RADIX);
    printf("FLT_EVAL_METHOD=%d\n", FLT_EVAL_METHOD);
    printf("FLOAT_BITS=%zu\n", sizeof(float) * CHAR_BIT);
    printf("DOUBLE_BITS=%zu\n", sizeof(double) * CHAR_BIT);
    printf("LONG_DOUBLE_STORAGE_BITS=%zu\n", sizeof(long double) * CHAR_BIT);
    printf("FLOAT_MANT_DIG=%d MIN_EXP=%d MAX_EXP=%d\n",
        FLT_MANT_DIG, FLT_MIN_EXP, FLT_MAX_EXP);
    printf("DOUBLE_MANT_DIG=%d MIN_EXP=%d MAX_EXP=%d\n",
        DBL_MANT_DIG, DBL_MIN_EXP, DBL_MAX_EXP);
    printf("LONG_DOUBLE_MANT_DIG=%d MIN_EXP=%d MAX_EXP=%d\n",
        LDBL_MANT_DIG, LDBL_MIN_EXP, LDBL_MAX_EXP);
    printf("FLOAT_TRUE_MIN=%a\n", (double)FLT_TRUE_MIN);
    printf("DOUBLE_TRUE_MIN=%a\n", DBL_TRUE_MIN);
    printf("LONG_DOUBLE_TRUE_MIN=%La\n", LDBL_TRUE_MIN);
}

/**
 * @brief Enforce the single primary-host format envelope used in B1 tests.
 *
 * The exact dyadic, metric and rounding kernels claim initial support only
 * under a bounded set of format assumptions. A compiler that exposes a
 * different format must be reviewed and qualified separately, rather than
 * silently obtaining this host's acceptance record.
 *
 * @return true if C's reported type characteristics match the candidate
 *         binary32/binary64/x87 configuration, false otherwise.
 */
static bool profile_matches_primary_target(void)
{
    return CHAR_BIT == 8 && FLT_RADIX == 2 && FLT_EVAL_METHOD == 0 &&
        sizeof(float) == 4 && sizeof(double) == 8 &&
        sizeof(long double) == 16 &&
        FLT_MANT_DIG == 24 && FLT_MIN_EXP == -125 && FLT_MAX_EXP == 128 &&
        DBL_MANT_DIG == 53 && DBL_MIN_EXP == -1021 && DBL_MAX_EXP == 1024 &&
        LDBL_MANT_DIG == 64 && LDBL_MIN_EXP == -16381 &&
        LDBL_MAX_EXP == 16384;
}

/**
 * @brief Verify the host exposes the four rounding modes and fenv isolation.
 *
 * The check uses only <fenv.h> calls: it saves the original environment,
 * selects each supported rounding direction, raises a controlled sticky
 * flag, holds the environment (which must clear the observable flag), then
 * restores it. This is a smoke test, not a proof for every possible caller
 * environment or a test of unmasked-trap configurations.
 *
 * @return true on successful inspection and state restoration.
 */
static bool probe_fenv(void)
{
    fenv_t original;
    if (fegetenv(&original) != 0)
        return false;

    const int modes[] =
    {
        FE_TONEAREST,
        FE_UPWARD,
        FE_DOWNWARD,
        FE_TOWARDZERO
    };
    bool success = true;

    for (size_t i = 0; i < sizeof(modes) / sizeof(modes[0]); ++i)
    {
        if (fesetround(modes[i]) != 0 || fegetround() != modes[i] ||
            feclearexcept(FE_ALL_EXCEPT) != 0 ||
            feraiseexcept(FE_INEXACT) != 0 ||
            (fetestexcept(FE_INEXACT) & FE_INEXACT) == 0)
        {
            success = false;
            break;
        }

        fenv_t held;
        if (feholdexcept(&held) != 0)
        {
            success = false;
            break;
        }
        if (fetestexcept(FE_ALL_EXCEPT) != 0)
            success = false;
        if (fesetenv(&held) != 0 ||
            (fetestexcept(FE_INEXACT) & FE_INEXACT) == 0 ||
            fegetround() != modes[i])
        {
            success = false;
        }
        if (!success)
            break;
    }

    /* Even a failed probe must attempt to return the host to its caller state. */
    if (fesetenv(&original) != 0)
        return false;
    return success;
}

/**
 * @brief Emit host evidence and fail closed for an uncertified format profile.
 * @return EXIT_SUCCESS only when the primary profile and fenv smoke test pass.
 */
int main(void)
{
    print_format_profile();
    const bool format_ok = profile_matches_primary_target();
    const bool fenv_ok = probe_fenv();
    printf("PRIMARY_FORMAT_PROFILE_MATCH=%d\n", format_ok ? 1 : 0);
    printf("FENV_SMOKE_TEST_PASS=%d\n", fenv_ok ? 1 : 0);
    return format_ok && fenv_ok ? EXIT_SUCCESS : EXIT_FAILURE;
}
