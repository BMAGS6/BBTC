/**
 * @file test_virial_live_fp_rejection.c
 * @brief VV0.1-B2b.4b Stage 2: independently perturb real x87/MXCSR state.
 *
 * @details
 * The earlier B2b.3c.3a tests deliberately classify synthetic register
 * records and never write processor controls. This developer-only follow-up
 * exercises the actual instruction path on x86-64 Linux. Each challenge runs
 * inside a short-lived forked child; the parent never changes its fenv.
 *
 * Child procedure:
 *   (1) Save BOTH hardware control registers before changing anything.
 *   (2) Change exactly one control category, or use ISO C fesetround().
 *   (3) Read the REAL state using bbtc_vv_virial_control_screen().
 *   (4) Restore both registers; observe restoration before any pipe write.
 *   (5) Send a POD evidence record to the parent, then call _exit().
 *
 * The parent verifies the requested actual bit change, UNSUPPORTED state,
 * correct rejection issue, complete communication, normal child exit, and
 * unchanged parent controls. There are no unmasked exception/trap tests.
 *
 * This is a *screening* test, not formal compiler qualification and not a
 * mathematical-reference PASS/FAIL. No production function is modified.
 *
 * Restrict this source to GNU/Clang x86-64 Linux via its CMake gate. An
 * unsupported host must not quietly substitute fake observations.
 */
#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <stdbool.h>
#include <stdint.h>
#include <errno.h>
#include <fenv.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cmocka.h>

#include "virial_fp_control.h"
#include "virial_fp_profile.h"

#if !defined(__linux__) || !defined(__x86_64__) || \
    (!defined(__GNUC__) && !defined(__clang__))
    #error "Live FP rejection is restricted to GNU/Clang x86-64 Linux"
#endif
#if defined(__FAST_MATH__)
    #error "Live FP rejection requires strict floating-point compilation"
#endif
#if defined(__clang__)
    #pragma STDC FENV_ACCESS ON
#endif

/** @brief Architectural control bits; no exception masks are ever modified. */
enum
{
    X87_PRECISION_MASK = 0x0300u,
    X87_ROUNDING_UP    = 0x0800u,
    MXCSR_ROUNDING_UP  = 0x4000u,
    MXCSR_DAZ          = 0x0040u,
    MXCSR_FTZ          = 0x8000u,
    MXCSR_CONTROL_MASK = 0xffc0u
};

/** @brief Each named challenge affects an independently testable control. */
typedef enum challenge_e
{
    CHALLENGE_C_ROUND_UP,
    CHALLENGE_X87_ROUND_UP,
    CHALLENGE_MXCSR_ROUND_UP,
    CHALLENGE_X87_PRECISION,
    CHALLENGE_MXCSR_FTZ,
    CHALLENGE_MXCSR_DAZ
} challenge_e;

/**
 * @brief Child evidence transmitted only AFTER hardware state restoration.
 *
 * @details
 * This POD record contains no pointers, padding-sensitive equality tests,
 * or floating-point numbers. Only the named members are inspected. The
 * original, mutated, and restored values are retained to distinguish an
 * ineffective perturbation from a successfully rejected real perturbation.
 */
typedef struct child_evidence_t
{
    bbtc_vv_virial_control_observation_t original;
    bbtc_vv_virial_control_observation_t perturbed;
    bbtc_vv_virial_control_observation_t restored;
    bbtc_vv_virial_control_state_e control_state;
    uint32_t issue_mask;
    bbtc_vv_virial_profile_state_e profile_state;
    uint32_t profile_issues;
    bool setup_succeeded;
    bool restoration_succeeded;
} child_evidence_t;

/** @brief Load a saved x87 control word without executing FP arithmetic. */
static void write_x87_control(const uint16_t* value)
{
    __asm__ volatile("fldcw %0" : : "m"(*value) : "memory");
}

/** @brief Load an MXCSR image containing only supported control/status bits. */
static void write_mxcsr(const uint32_t* value)
{
    __asm__ volatile("ldmxcsr %0" : : "m"(*value) : "memory");
}

/**
 * @brief Compare meaningful control-state fields, ignoring sticky MXCSR flags.
 *
 * Sticky exception status bits may legitimately differ after other code runs.
 * The classifier's admitted profile is defined by the control fields, not
 * the history of sticky exception observations.
 */
static bool controls_equal(
    const bbtc_vv_virial_control_observation_t* lhs,
    const bbtc_vv_virial_control_observation_t* rhs)
{
    return lhs->available && rhs->available &&
           lhs->rounding_mode == rhs->rounding_mode &&
           lhs->x87_control == rhs->x87_control &&
           (lhs->mxcsr & MXCSR_CONTROL_MASK) ==
               (rhs->mxcsr & MXCSR_CONTROL_MASK);
}

/**
 * @brief Perform ONE real control mutation and collect raw screening evidence.
 *
 * @param challenge Independent control dimension to perturb.
 * @param evidence Caller-owned output, cleared on entry.
 * @return true only when mutation, screening, and restoration completed.
 *
 * @warning Must only run in a forked child. The caller must not use this
 *          function as a general-purpose floating-point configuration API.
 */
static bool perform_child_challenge(challenge_e challenge, child_evidence_t* evidence)
{
    *evidence = (child_evidence_t){0};
    if (!bbtc_vv_virial_control_observe(&evidence->original))
        return false;

    const uint16_t original_x87 = evidence->original.x87_control;
    const uint32_t original_mxcsr = evidence->original.mxcsr;
    uint16_t changed_x87 = original_x87;
    uint32_t changed_mxcsr = original_mxcsr;
    bool setup_ok = true;

    switch (challenge)
    {
        case CHALLENGE_C_ROUND_UP:
            setup_ok = fesetround(FE_UPWARD) == 0;
            break;
        case CHALLENGE_X87_ROUND_UP:
            changed_x87 = (uint16_t)((original_x87 & ~UINT16_C(0x0c00)) |
                                      X87_ROUNDING_UP);
            write_x87_control(&changed_x87);
            break;
        case CHALLENGE_MXCSR_ROUND_UP:
            changed_mxcsr = (original_mxcsr & ~UINT32_C(0x6000)) | MXCSR_ROUNDING_UP;
            write_mxcsr(&changed_mxcsr);
            break;
        case CHALLENGE_X87_PRECISION:
            changed_x87 = (uint16_t)(original_x87 & ~X87_PRECISION_MASK);
            write_x87_control(&changed_x87);
            break;
        case CHALLENGE_MXCSR_FTZ:
            changed_mxcsr = original_mxcsr | MXCSR_FTZ;
            write_mxcsr(&changed_mxcsr);
            break;
        case CHALLENGE_MXCSR_DAZ:
            changed_mxcsr = original_mxcsr | MXCSR_DAZ;
            write_mxcsr(&changed_mxcsr);
            break;
        default:
            setup_ok = false;
            break;
    }

    evidence->setup_succeeded = setup_ok;
    if (setup_ok)
    {
        bbtc_vv_virial_control_result_t control_result = {0};
        evidence->control_state = bbtc_vv_virial_control_screen(
            &evidence->perturbed, &control_result);
        evidence->issue_mask = control_result.issue_mask;

        /* Only this control challenge expects the broader ISO C metadata
         * preflight to reject. Other challenges are detected by the separate
         * hardware-control preflight, not by the format metadata screen. */
        if (challenge == CHALLENGE_C_ROUND_UP)
        {
            bbtc_vv_virial_profile_observation_t profile_observation = {0};
            bbtc_vv_virial_profile_result_t profile_result = {0};
            evidence->profile_state = bbtc_vv_virial_profile_screen(
                &profile_observation, &profile_result);
            evidence->profile_issues = profile_result.issue_mask;
        }
    }

    /* Restore even on setup or observation failure. The child is not allowed
     * to leave modified FP controls active while recording pipe evidence. */
    write_x87_control(&original_x87);
    write_mxcsr(&original_mxcsr);

    if (!bbtc_vv_virial_control_observe(&evidence->restored))
        return false;
    evidence->restoration_succeeded = controls_equal(
        &evidence->original, &evidence->restored);
    return setup_ok && evidence->restoration_succeeded;
}

/**
 * @brief Write all bytes despite EINTR/short POSIX pipe writes.
 * @return true iff the entire record was transmitted.
 */
static bool write_record(int fd, const child_evidence_t* evidence)
{
    const unsigned char* bytes = (const unsigned char*)evidence;
    size_t sent = 0u;
    while (sent < sizeof(*evidence))
    {
        const ssize_t n = write(fd, bytes + sent, sizeof(*evidence) - sent);
        if (n < 0 && errno == EINTR)
            continue;
        if (n <= 0)
            return false;
        sent += (size_t)n;
    }
    return true;
}

/**
 * @brief Read the complete child record without trusting partial pipe data.
 * @return true only for exactly the required record size.
 */
static bool read_record(int fd, child_evidence_t* evidence)
{
    unsigned char* bytes = (unsigned char*)evidence;
    size_t received = 0u;
    while (received < sizeof(*evidence))
    {
        const ssize_t n = read(fd, bytes + received, sizeof(*evidence) - received);
        if (n < 0 && errno == EINTR)
            continue;
        if (n <= 0)
            return false;
        received += (size_t)n;
    }
    return true;
}

/**
 * @brief Launch a single challenge in a forked process and verify its outcome.
 *
 * The expected mask can have additional reasons, especially when a C
 * rounding-mode change affects BOTH x87 and SSE. Requiring at least the
 * specific bit ensures that a generic rejection cannot hide a broken probe.
 *
 * @param challenge Control category that must be changed and rejected.
 * @param expected_issue Bit that must appear in the real hardware screen.
 */
static void require_live_rejection(challenge_e challenge, uint32_t expected_issue)
{
    bbtc_vv_virial_control_observation_t parent_before = {0};
    bbtc_vv_virial_control_result_t parent_result = {0};
    assert_int_equal(bbtc_vv_virial_control_screen(&parent_before, &parent_result),
                     BBTC_VV_VIRIAL_CONTROL_SCREENED_UNQUALIFIED);
    assert_int_equal(parent_result.issue_mask, BBTC_VV_VIRIAL_CONTROL_ISSUE_NONE);

    int fds[2] = {-1, -1};
    assert_int_equal(pipe(fds), 0);
    const pid_t child = fork();
    assert_true(child >= 0);

    if (child == 0)
    {
        (void)close(fds[0]);
        child_evidence_t evidence = {0};
        const bool ok = perform_child_challenge(challenge, &evidence);
        const bool reported = write_record(fds[1], &evidence);
        (void)close(fds[1]);
        _exit(ok && reported ? 0 : 3);
    }

    (void)close(fds[1]);
    child_evidence_t evidence = {0};
    const bool received = read_record(fds[0], &evidence);
    (void)close(fds[0]);

    int status = 0;
    pid_t waited;
    do
    {
        waited = waitpid(child, &status, 0);
    } while (waited < 0 && errno == EINTR);

    assert_true(received);
    assert_int_equal(waited, child);
    assert_true(WIFEXITED(status));
    assert_int_equal(WEXITSTATUS(status), 0);
    assert_true(evidence.setup_succeeded);
    assert_true(evidence.restoration_succeeded);
    assert_true(controls_equal(&evidence.original, &evidence.restored));
    assert_int_equal(evidence.control_state, BBTC_VV_VIRIAL_CONTROL_UNSUPPORTED);
    assert_true((evidence.issue_mask & expected_issue) != 0u);

    /* Prove a *real* control changed, rather than accepting a fake unsupported
     * status that might arise from an unrelated bad host or null pointer. */
    if (challenge == CHALLENGE_C_ROUND_UP)
    {
        assert_int_equal(evidence.perturbed.rounding_mode, FE_UPWARD);
        assert_int_equal(evidence.profile_state, BBTC_VV_VIRIAL_PROFILE_UNSUPPORTED);
        assert_true((evidence.profile_issues & BBTC_VV_VIRIAL_ISSUE_ROUNDING_MODE) != 0u);
    }
    else if (challenge == CHALLENGE_X87_ROUND_UP)
    {
        assert_true((evidence.perturbed.x87_control & UINT16_C(0x0c00)) != 0u);
    }
    else if (challenge == CHALLENGE_X87_PRECISION)
    {
        assert_true((evidence.perturbed.x87_control & X87_PRECISION_MASK) !=
                    (evidence.original.x87_control & X87_PRECISION_MASK));
    }
    else if (challenge == CHALLENGE_MXCSR_ROUND_UP)
    {
        assert_true((evidence.perturbed.mxcsr & UINT32_C(0x6000)) != 0u);
    }
    else if (challenge == CHALLENGE_MXCSR_FTZ)
    {
        assert_true((evidence.perturbed.mxcsr & MXCSR_FTZ) != 0u);
    }
    else if (challenge == CHALLENGE_MXCSR_DAZ)
    {
        assert_true((evidence.perturbed.mxcsr & MXCSR_DAZ) != 0u);
    }

    bbtc_vv_virial_control_observation_t parent_after = {0};
    bbtc_vv_virial_control_result_t parent_after_result = {0};
    assert_int_equal(bbtc_vv_virial_control_screen(&parent_after, &parent_after_result),
                     BBTC_VV_VIRIAL_CONTROL_SCREENED_UNQUALIFIED);
    assert_int_equal(parent_after_result.issue_mask, BBTC_VV_VIRIAL_CONTROL_ISSUE_NONE);
    assert_true(controls_equal(&parent_before, &parent_after));
}

/** @brief Test a real upward C rounding-mode change, including format screening. */
static void test_live_c_rounding(void** state)
{
    (void)state;
    require_live_rejection(CHALLENGE_C_ROUND_UP, BBTC_VV_VIRIAL_CONTROL_ISSUE_FENV_ROUND);
}

/** @brief Test independent x87 rounding control rejection. */
static void test_live_x87_rounding(void** state)
{
    (void)state;
    require_live_rejection(CHALLENGE_X87_ROUND_UP, BBTC_VV_VIRIAL_CONTROL_ISSUE_X87_ROUND);
}

/** @brief Test independent SSE rounding control rejection. */
static void test_live_mxcsr_rounding(void** state)
{
    (void)state;
    require_live_rejection(CHALLENGE_MXCSR_ROUND_UP, BBTC_VV_VIRIAL_CONTROL_ISSUE_MXCSR_ROUND);
}

/** @brief Test the unsupported x87 significand precision-control state. */
static void test_live_x87_precision(void** state)
{
    (void)state;
    require_live_rejection(CHALLENGE_X87_PRECISION, BBTC_VV_VIRIAL_CONTROL_ISSUE_X87_PRECISION);
}

/** @brief Test SSE flush-to-zero rejection without performing subnormal arithmetic. */
static void test_live_ftz(void** state)
{
    (void)state;
    require_live_rejection(CHALLENGE_MXCSR_FTZ, BBTC_VV_VIRIAL_CONTROL_ISSUE_MXCSR_FTZ);
}

/** @brief Test SSE denormals-are-zero rejection without performing arithmetic. */
static void test_live_daz(void** state)
{
    (void)state;
    require_live_rejection(CHALLENGE_MXCSR_DAZ, BBTC_VV_VIRIAL_CONTROL_ISSUE_MXCSR_DAZ);
}

/**
 * @brief Run all six mandatory live rejection challenges as separate cases.
 * @return Nonzero on any failure; no challenge is silently skipped.
 */
int main(void)
{
    const struct CMUnitTest tests[] =
    {
        cmocka_unit_test(test_live_c_rounding),
        cmocka_unit_test(test_live_x87_rounding),
        cmocka_unit_test(test_live_mxcsr_rounding),
        cmocka_unit_test(test_live_x87_precision),
        cmocka_unit_test(test_live_ftz),
        cmocka_unit_test(test_live_daz)
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}
