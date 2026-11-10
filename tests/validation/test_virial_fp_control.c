/**
 * @file test_virial_fp_control.c
 * @brief Adversarial B2b.3c.3a floating-point hardware state self-tests.
 *
 * @details
 * Real control-word acquisition is read-only. All negative controls inject
 * deliberately invalid copies rather than calling `fesetround`, changing
 * MXCSR/x87 registers, enabling unmasked traps, or clearing sticky flags.
 * A successful classification is SCREENED_UNQUALIFIED, never qualified.
 *
 * These tests are deliberately separate from the mathematical-reference
 * runner: a negative control may pass because an unsupported state was
 * rejected, but that cannot count as any of the 36 required numerical PASS
 * results. The reference runner itself rejects unsupported real hardware.
 */
#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <fenv.h>

#include <cmocka.h>

#include "virial_fp_control.h"

#if defined(__FAST_MATH__)
    #error "B2b.3c.3a tests prohibit fast-math"
#endif

/**
 * @brief A known-good synthetic encoding, not evidence of the actual host.
 *
 * @details
 * x87=0x037f means all six exceptions masked, 64-bit significand
 * precision, and nearest rounding. MXCSR=0x1f80 means all six SSE exception
 * masks set, nearest rounding, FTZ/DAZ disabled, and no sticky flags.
 * The separate fegetround value is assigned the native FE_TONEAREST token.
 *
 * @return Deterministic synthetic fixture for classification-only tests.
 */
static bbtc_vv_virial_control_observation_t accepted_fixture(void)
{
    return (bbtc_vv_virial_control_observation_t)
    {
        .mxcsr = UINT32_C(0x00001f80),
        .rounding_mode = FE_TONEAREST,
        .x87_control = UINT16_C(0x037f),
        .available = true
    };
}

/**
 * @brief Require precisely one injected cause to be recognized as unsupported.
 * @param fixture Caller-owned synthetic or mutated observation.
 * @param issue Expected mandatory rejection-bit in the returned mask.
 */
static void require_issue(
    const bbtc_vv_virial_control_observation_t* fixture,
    uint32_t issue)
{
    bbtc_vv_virial_control_result_t result = {0};
    assert_int_equal(bbtc_vv_virial_control_classify(fixture, &result),
                     BBTC_VV_VIRIAL_CONTROL_UNSUPPORTED);
    assert_int_equal(result.state, BBTC_VV_VIRIAL_CONTROL_UNSUPPORTED);
    assert_true((result.issue_mask & issue) != 0u);
}

/**
 * @brief A valid synthetic profile may only be SCREENED_UNQUALIFIED.
 * @param state Unused cmocka context.
 */
static void test_synthetic_supported_screen(void** state)
{
    (void)state;
    const bbtc_vv_virial_control_observation_t fixture = accepted_fixture();
    bbtc_vv_virial_control_result_t result = {0};

    assert_int_equal(bbtc_vv_virial_control_classify(&fixture, &result),
                     BBTC_VV_VIRIAL_CONTROL_SCREENED_UNQUALIFIED);
    assert_int_equal(result.state, BBTC_VV_VIRIAL_CONTROL_SCREENED_UNQUALIFIED);
    assert_int_equal(result.issue_mask, BBTC_VV_VIRIAL_CONTROL_ISSUE_NONE);
}

/**
 * @brief Record live control words while refusing silent unsupported hosts.
 *
 * @details
 * The standalone control test can execute on a non-x86 build and prove that
 * acquisition is properly rejected. Required mathematical references, in
 * contrast, demand the screen on their executing host and fail if unsupported.
 *
 * @param state Unused cmocka context.
 */
static void test_live_read_only_screen(void** state)
{
    (void)state;
    bbtc_vv_virial_control_observation_t observation = {0};
    bbtc_vv_virial_control_result_t result = {0};
    const bbtc_vv_virial_control_state_e outcome =
        bbtc_vv_virial_control_screen(&observation, &result);

    printf("VVUQ B2b.3c.3a control=%s x87=0x%04x mxcsr=0x%08x fenv_round=%d "
           "issues=0x%08x (NOT QUALIFIED)\n",
           outcome == BBTC_VV_VIRIAL_CONTROL_SCREENED_UNQUALIFIED
               ? "SCREENED_UNQUALIFIED" : "UNSUPPORTED",
           (unsigned)observation.x87_control,
           (unsigned)observation.mxcsr,
           observation.rounding_mode,
           (unsigned)result.issue_mask);

    if (observation.available)
    {
        assert_int_equal(outcome, BBTC_VV_VIRIAL_CONTROL_SCREENED_UNQUALIFIED);
        assert_int_equal(result.issue_mask, BBTC_VV_VIRIAL_CONTROL_ISSUE_NONE);
    }
    else
    {
        assert_int_equal(outcome, BBTC_VV_VIRIAL_CONTROL_UNSUPPORTED);
        assert_true((result.issue_mask & BBTC_VV_VIRIAL_CONTROL_ISSUE_ARCH) != 0u);
    }
}

/**
 * @brief Required pointer and architecture rejection do not manufacture PASS.
 * @param state Unused cmocka context.
 */
static void test_invalid_arguments_and_architecture(void** state)
{
    (void)state;
    bbtc_vv_virial_control_result_t result = {0};
    bbtc_vv_virial_control_observation_t observation = accepted_fixture();

    assert_false(bbtc_vv_virial_control_observe(NULL));
    assert_int_equal(bbtc_vv_virial_control_classify(NULL, &result),
                     BBTC_VV_VIRIAL_CONTROL_UNSUPPORTED);
    assert_true((result.issue_mask & BBTC_VV_VIRIAL_CONTROL_ISSUE_INVALID) != 0u);
    assert_int_equal(bbtc_vv_virial_control_classify(&observation, NULL),
                     BBTC_VV_VIRIAL_CONTROL_UNSUPPORTED);
    assert_int_equal(bbtc_vv_virial_control_screen(NULL, &result),
                     BBTC_VV_VIRIAL_CONTROL_UNSUPPORTED);
    assert_true((result.issue_mask & BBTC_VV_VIRIAL_CONTROL_ISSUE_INVALID) != 0u);
    assert_int_equal(bbtc_vv_virial_control_screen(&observation, NULL),
                     BBTC_VV_VIRIAL_CONTROL_UNSUPPORTED);

    observation.available = false;
    require_issue(&observation, BBTC_VV_VIRIAL_CONTROL_ISSUE_ARCH);
}

/**
 * @brief Catch separate x87 rounding-control and precision-control defects.
 * @param state Unused cmocka context.
 */
static void test_x87_rounding_and_precision_rejection(void** state)
{
    (void)state;
    bbtc_vv_virial_control_observation_t bad = accepted_fixture();
    bad.rounding_mode = FE_TONEAREST == 0 ? 1 : 0;
    require_issue(&bad, BBTC_VV_VIRIAL_CONTROL_ISSUE_FENV_ROUND);

    for (unsigned bit = 10u; bit <= 11u; ++bit)
    {
        bad = accepted_fixture();
        bad.x87_control = (uint16_t)(bad.x87_control | (1u << bit));
        require_issue(&bad, BBTC_VV_VIRIAL_CONTROL_ISSUE_X87_ROUND);
    }

    for (unsigned bit = 8u; bit <= 9u; ++bit)
    {
        bad = accepted_fixture();
        bad.x87_control = (uint16_t)(bad.x87_control & ~(1u << bit));
        require_issue(&bad, BBTC_VV_VIRIAL_CONTROL_ISSUE_X87_PRECISION);
    }
}

/**
 * @brief Catch independently unmasked x87 exceptions without enabling traps.
 * @param state Unused cmocka context.
 */
static void test_x87_exception_mask_rejection(void** state)
{
    (void)state;
    for (unsigned bit = 0u; bit < 6u; ++bit)
    {
        bbtc_vv_virial_control_observation_t bad = accepted_fixture();
        bad.x87_control = (uint16_t)(bad.x87_control & ~(1u << bit));
        require_issue(&bad, BBTC_VV_VIRIAL_CONTROL_ISSUE_X87_MASKS);
    }
}

/**
 * @brief Catch SSE rounding-control and all six exception-mask defects.
 * @param state Unused cmocka context.
 */
static void test_mxcsr_rounding_and_masks_rejection(void** state)
{
    (void)state;
    for (unsigned bit = 13u; bit <= 14u; ++bit)
    {
        bbtc_vv_virial_control_observation_t bad = accepted_fixture();
        bad.mxcsr |= UINT32_C(1) << bit;
        require_issue(&bad, BBTC_VV_VIRIAL_CONTROL_ISSUE_MXCSR_ROUND);
    }

    for (unsigned bit = 7u; bit <= 12u; ++bit)
    {
        bbtc_vv_virial_control_observation_t bad = accepted_fixture();
        bad.mxcsr &= ~(UINT32_C(1) << bit);
        require_issue(&bad, BBTC_VV_VIRIAL_CONTROL_ISSUE_MXCSR_MASKS);
    }
}

/**
 * @brief Detect FTZ, DAZ, and impossible high bits without real-state edits.
 * @param state Unused cmocka context.
 */
static void test_mxcsr_subnormal_and_reserved_rejection(void** state)
{
    (void)state;
    bbtc_vv_virial_control_observation_t bad = accepted_fixture();
    bad.mxcsr |= UINT32_C(0x8000);
    require_issue(&bad, BBTC_VV_VIRIAL_CONTROL_ISSUE_MXCSR_FTZ);

    bad = accepted_fixture();
    bad.mxcsr |= UINT32_C(0x0040);
    require_issue(&bad, BBTC_VV_VIRIAL_CONTROL_ISSUE_MXCSR_DAZ);

    bad = accepted_fixture();
    bad.mxcsr |= UINT32_C(0x00010000);
    require_issue(&bad, BBTC_VV_VIRIAL_CONTROL_ISSUE_MXCSR_HIGH);
}

/**
 * @brief Sticky SSE status flags do not invalidate an otherwise valid profile.
 *
 * @details
 * The low six bits are sticky status indicators, not rounding or mask
 * settings. The synthetic test must accept them without clearing real flags.
 *
 * @param state Unused cmocka context.
 */
static void test_sticky_status_is_not_control_rejection(void** state)
{
    (void)state;
    bbtc_vv_virial_control_observation_t fixture = accepted_fixture();
    bbtc_vv_virial_control_result_t result = {0};
    fixture.mxcsr |= UINT32_C(0x003f);

    assert_int_equal(bbtc_vv_virial_control_classify(&fixture, &result),
                     BBTC_VV_VIRIAL_CONTROL_SCREENED_UNQUALIFIED);
    assert_int_equal(result.issue_mask, BBTC_VV_VIRIAL_CONTROL_ISSUE_NONE);
}

/**
 * @brief Prove acquiring and classifying real controls is observational only.
 *
 * @details
 * The test makes no floating-point calculations between observations. It
 * checks the x87 control word, all MXCSR *control* bits 6..15, C rounding
 * mode, and sticky exception indicators across screen calls. Any unsupported
 * architecture remains a valid rejection outcome, not a false qualification.
 *
 * @param state Unused cmocka context.
 */
static void test_control_state_preserved(void** state)
{
    (void)state;
    bbtc_vv_virial_control_observation_t before = {0};
    bbtc_vv_virial_control_observation_t after = {0};
    bbtc_vv_virial_control_result_t result = {0};

    const bool available = bbtc_vv_virial_control_observe(&before);
    const bbtc_vv_virial_control_state_e outcome =
        bbtc_vv_virial_control_screen(&after, &result);

    if (available)
    {
        assert_int_equal(outcome, BBTC_VV_VIRIAL_CONTROL_SCREENED_UNQUALIFIED);
        assert_int_equal(before.x87_control, after.x87_control);
        assert_int_equal(before.mxcsr, after.mxcsr);
        assert_int_equal(before.rounding_mode, after.rounding_mode);
    }
    else
    {
        assert_int_equal(outcome, BBTC_VV_VIRIAL_CONTROL_UNSUPPORTED);
        assert_true((result.issue_mask & BBTC_VV_VIRIAL_CONTROL_ISSUE_ARCH) != 0u);
    }
}

/**
 * @brief Execute read-only and adversarial control-state cmocka checks.
 * @return Nonzero when a required property fails; no tests are skipped.
 */
int main(void)
{
    const struct CMUnitTest tests[] =
    {
        cmocka_unit_test(test_synthetic_supported_screen),
        cmocka_unit_test(test_live_read_only_screen),
        cmocka_unit_test(test_invalid_arguments_and_architecture),
        cmocka_unit_test(test_x87_rounding_and_precision_rejection),
        cmocka_unit_test(test_x87_exception_mask_rejection),
        cmocka_unit_test(test_mxcsr_rounding_and_masks_rejection),
        cmocka_unit_test(test_mxcsr_subnormal_and_reserved_rejection),
        cmocka_unit_test(test_sticky_status_is_not_control_rejection),
        cmocka_unit_test(test_control_state_preserved)
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}
