/**
 * @file test_virial_fp_profile.c
 * @brief B2b.3c.2 native metadata and fenv preflight self-tests.
 *
 * @details
 * Unlike the independent mathematical reference matrix, this developer-only
 * executable tests whether the *environment screen* rejects contradictory
 * metadata, unsupported rounding modes, failed production metadata calls,
 * and invalid caller arguments. It never validates a gas model, certifies a
 * producer compilation, or converts a numerical PASS into a qualified profile.
 *
 * The positive test records the actual linked production API's native
 * metadata on stdout. CTest's `-V` switch preserves the report in the
 * developer's evidence transcript. Compiler commands must still be checked
 * separately using `compile_commands.json` and the accompanying capture tool.
 *
 * The negative tests use copies of actual observations. They do NOT create
 * counterfeit BBTC library objects, and ordinary classification tests do not
 * modify the host floating-point environment. The optional real rounding
 * test restores its original fenv *before* performing cmocka assertions.
 */
#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <fenv.h>

#include <cmocka.h>

#include "virial_fp_profile.h"

#if defined(__FAST_MATH__)
    #error "B2b.3c.2 profile tests forbid fast-math"
#endif
#if defined(__FINITE_MATH_ONLY__) && __FINITE_MATH_ONLY__ != 0
    #error "B2b.3c.2 requires finite-math-only disabled"
#endif
#if defined(__clang__)
    #pragma STDC FENV_ACCESS ON
#endif

/**
 * @brief Read an actual host observation and require our narrow screen to pass.
 *
 * @details
 * The currently configured developer workstation is expected to support the
 * screen; an unsupported configuration must result in a red test, not a
 * silently skipped reference. `out` is always initialized by the real
 * production metadata API, not from hand-written expected format numbers.
 *
 * @param out Caller-owned observation to fill.
 */
static void require_actual_supported_screen(bbtc_vv_virial_profile_observation_t* out)
{
    bbtc_vv_virial_profile_result_t result = {0};
    assert_non_null(out);
    assert_int_equal(bbtc_vv_virial_profile_screen(out, &result),
                     BBTC_VV_VIRIAL_PROFILE_SCREENED_UNQUALIFIED);
    assert_int_equal(result.state, BBTC_VV_VIRIAL_PROFILE_SCREENED_UNQUALIFIED);
    assert_int_equal(result.issue_mask, BBTC_VV_VIRIAL_ISSUE_NONE);
}

/**
 * @brief Print the exact format/fenv evidence observed at runtime.
 *
 * @details
 * The emitted values describe only the compilation unit and its linked
 * production library. They must be paired with the producer object's
 * compile-command evidence and a source/instruction review before a later
 * qualification decision. The screen has no QUALIFIED enumerator.
 *
 * @param state Unused cmocka state.
 */
static void test_host_profile_screen(void** state)
{
    (void)state;
    bbtc_vv_virial_profile_observation_t observation = {0};
    require_actual_supported_screen(&observation);

    printf("VVUQ B2b.3c.2 SCREENED_UNQUALIFIED; rounding=%d; FLT_EVAL_METHOD=%d\n",
           observation.rounding_mode, observation.evaluation_method);

    for (size_t i = 0u; i < BBTC_VV_VIRIAL_NATIVE_FAMILIES; ++i)
    {
        const bbtc_precision_info_t* info = &observation.production_info[i];
        printf("  precision=%u radix=%u mantissa=%u min_exp=%d max_exp=%d "
               "decimal_digits=%u bytes=%u metadata_status=%u\n",
               (unsigned)info->precision,
               (unsigned)info->radix,
               (unsigned)info->mantissa_digits,
               (int)info->minimum_normal_exponent,
               (int)info->maximum_finite_exponent,
               (unsigned)info->decimal_digits,
               (unsigned)info->storage_bytes,
               (unsigned)observation.production_status[i]);
    }
}

/**
 * @brief Reject invalid pointers without manufacturing SCREENED status.
 * @param state Unused cmocka state.
 */
static void test_invalid_observation_is_unsupported(void** state)
{
    (void)state;
    bbtc_vv_virial_profile_result_t result = {0};
    bbtc_vv_virial_profile_observation_t observation = {0};

    assert_int_equal(bbtc_vv_virial_profile_classify(NULL, &result),
                     BBTC_VV_VIRIAL_PROFILE_UNSUPPORTED);
    assert_int_equal(result.issue_mask, BBTC_VV_VIRIAL_ISSUE_INVALID_OBSERVATION);
    assert_int_equal(bbtc_vv_virial_profile_classify(&observation, NULL),
                     BBTC_VV_VIRIAL_PROFILE_UNSUPPORTED);
    assert_false(bbtc_vv_virial_profile_observe(NULL));
    assert_int_equal(bbtc_vv_virial_profile_screen(NULL, &result),
                     BBTC_VV_VIRIAL_PROFILE_UNSUPPORTED);
    assert_int_equal(result.issue_mask, BBTC_VV_VIRIAL_ISSUE_INVALID_OBSERVATION);
    assert_int_equal(bbtc_vv_virial_profile_screen(&observation, NULL),
                     BBTC_VV_VIRIAL_PROFILE_UNSUPPORTED);
}

/**
 * @brief Report non-nearest rounding and unsupported evaluation semantics.
 *
 * @details
 * Mutating an observation is deterministic even on a machine that cannot
 * switch to FE_UPWARD. The real `fesetround` control is checked separately.
 * It cannot affect the fixture matrix or cause its findings to be waived.
 *
 * @param state Unused cmocka state.
 */
static void test_rounding_and_eval_method_rejection(void** state)
{
    (void)state;
    bbtc_vv_virial_profile_observation_t valid = {0};
    bbtc_vv_virial_profile_result_t result = {0};
    require_actual_supported_screen(&valid);

    bbtc_vv_virial_profile_observation_t modified = valid;
#if defined(FE_UPWARD)
    modified.rounding_mode = FE_UPWARD;
#else
    modified.rounding_mode = FE_TONEAREST == 0 ? 1 : 0;
#endif
    assert_int_equal(bbtc_vv_virial_profile_classify(&modified, &result),
                     BBTC_VV_VIRIAL_PROFILE_UNSUPPORTED);
    assert_true((result.issue_mask & BBTC_VV_VIRIAL_ISSUE_ROUNDING_MODE) != 0u);

    modified = valid;
    modified.evaluation_method = -1;
    assert_int_equal(bbtc_vv_virial_profile_classify(&modified, &result),
                     BBTC_VV_VIRIAL_PROFILE_UNSUPPORTED);
    assert_true((result.issue_mask & BBTC_VV_VIRIAL_ISSUE_EVALUATION_METHOD) != 0u);
}

/**
 * @brief Reject one format or ABI discrepancy at a time, without narrowing.
 *
 * @details
 * This checks every scalar-family slot rather than assuming that agreement
 * for binary64 implies agreement for binary32 or long double. A wrong decimal
 * digit report, for example, must be rejected even if the significand width
 * happens to match. No bad observation is passed to the production evaluator.
 *
 * @param state Unused cmocka state.
 */
static void test_native_and_production_metadata_rejection(void** state)
{
    (void)state;
    bbtc_vv_virial_profile_observation_t valid = {0};
    bbtc_vv_virial_profile_result_t result = {0};
    require_actual_supported_screen(&valid);

    for (size_t i = 0u; i < BBTC_VV_VIRIAL_NATIVE_FAMILIES; ++i)
    {
        bbtc_vv_virial_profile_observation_t bad = valid;
        bad.compiler_info[i].radix = 10u;
        assert_int_equal(bbtc_vv_virial_profile_classify(&bad, &result),
                         BBTC_VV_VIRIAL_PROFILE_UNSUPPORTED);
        assert_true((result.issue_mask & BBTC_VV_VIRIAL_ISSUE_RADIX) != 0u);

        bad = valid;
        ++bad.compiler_info[i].mantissa_digits;
        assert_int_equal(bbtc_vv_virial_profile_classify(&bad, &result),
                         BBTC_VV_VIRIAL_PROFILE_UNSUPPORTED);
        assert_true((result.issue_mask & BBTC_VV_VIRIAL_ISSUE_NATIVE_FORMAT) != 0u);

        bad = valid;
        ++bad.compiler_info[i].storage_bytes;
        assert_int_equal(bbtc_vv_virial_profile_classify(&bad, &result),
                         BBTC_VV_VIRIAL_PROFILE_UNSUPPORTED);
        assert_true((result.issue_mask & BBTC_VV_VIRIAL_ISSUE_NATIVE_FORMAT) != 0u);

        bad = valid;
        ++bad.production_info[i].decimal_digits;
        assert_int_equal(bbtc_vv_virial_profile_classify(&bad, &result),
                         BBTC_VV_VIRIAL_PROFILE_UNSUPPORTED);
        assert_true((result.issue_mask & BBTC_VV_VIRIAL_ISSUE_PRODUCTION_METADATA) != 0u);

        bad = valid;
        bad.production_status[i] = BBTC_STATUS_INVALID_ARGUMENT;
        assert_int_equal(bbtc_vv_virial_profile_classify(&bad, &result),
                         BBTC_VV_VIRIAL_PROFILE_UNSUPPORTED);
        assert_true((result.issue_mask & BBTC_VV_VIRIAL_ISSUE_PRODUCTION_METADATA) != 0u);
    }
}

/**
 * @brief Ensure a real temporary rounding-mode change is refused and restored.
 *
 * @details
 * A platform without FE_UPWARD support still runs the required deterministic
 * negative test above. On a platform where `fesetround(FE_UPWARD)` succeeds,
 * the real profile screen MUST reject it. The original full environment is
 * restored BEFORE examining the report with cmocka, including on failures.
 *
 * @param state Unused cmocka state.
 */
static void test_actual_rounding_mode_is_restored(void** state)
{
    (void)state;
    fenv_t before;
    assert_int_equal(fegetenv(&before), 0);
    const int original_mode = fegetround();

#if defined(FE_UPWARD)
    const int changed = fesetround(FE_UPWARD);
#else
    const int changed = -1; /* No runtime FE_UPWARD capability is advertised. */
#endif
    bbtc_vv_virial_profile_observation_t observed = {0};
    bbtc_vv_virial_profile_result_t result = {0};
    bbtc_vv_virial_profile_state_e screened = BBTC_VV_VIRIAL_PROFILE_UNCHECKED;
    if (changed == 0)
        screened = bbtc_vv_virial_profile_screen(&observed, &result);

    const int restore_status = fesetenv(&before);
    assert_int_equal(restore_status, 0);
    assert_int_equal(fegetround(), original_mode);

    if (changed == 0)
    {
        assert_int_equal(screened, BBTC_VV_VIRIAL_PROFILE_UNSUPPORTED);
        assert_true((result.issue_mask & BBTC_VV_VIRIAL_ISSUE_ROUNDING_MODE) != 0u);
    }
}

/**
 * @brief Test that preflight reads but never clears sticky exception flags.
 *
 * @details
 * No exceptions are deliberately raised: a test process may have enabled
 * traps, and raising one could terminate the process. The preflight should
 * leave whatever flags the caller already had untouched.
 *
 * @param state Unused cmocka state.
 */
static void test_preflight_does_not_clear_fenv(void** state)
{
    (void)state;
    const int before_mode = fegetround();
    const int before_flags = fetestexcept(FE_ALL_EXCEPT);
    bbtc_vv_virial_profile_observation_t observation = {0};
    bbtc_vv_virial_profile_result_t result = {0};

    (void)bbtc_vv_virial_profile_screen(&observation, &result);

    assert_int_equal(fegetround(), before_mode);
    assert_int_equal(fetestexcept(FE_ALL_EXCEPT), before_flags);
}

/**
 * @brief Execute the independent developer-only host-profile checks.
 * @return cmocka nonzero error status on any failed required test.
 */
int main(void)
{
    const struct CMUnitTest tests[] =
    {
        cmocka_unit_test(test_host_profile_screen),
        cmocka_unit_test(test_invalid_observation_is_unsupported),
        cmocka_unit_test(test_rounding_and_eval_method_rejection),
        cmocka_unit_test(test_native_and_production_metadata_rejection),
        cmocka_unit_test(test_actual_rounding_mode_is_restored),
        cmocka_unit_test(test_preflight_does_not_clear_fenv)
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}
