/**
 * @file virial_fp_control.h
 * @brief VV0.1-B2b.3c.3a read-only x87/MXCSR control-state evidence.
 *
 * @details
 * This developer-only interface observes the processor control words used by
 * x87 long-double and SSE float/double computations on supported x86 builds.
 * It deliberately neither configures nor repairs a floating-point environment.
 * Calling code must not describe SCREENED_UNQUALIFIED as scientific, compiler,
 * or numerical qualification. A compiled producer and its effective flags
 * still require the separately recorded B2b.3c.3 evidence review.
 *
 * Classification is separated from acquisition so adversarial tests can
 * corrupt copies of observations without changing a live control register,
 * enabling traps, or affecting unrelated tests. Raw control words are retained
 * so that an independent reviewer can inspect exact register values.
 *
 * Unsupported architectures, missing SSE support, and non-GNU-style compilers
 * do not execute architecture-specific instructions and remain UNSUPPORTED.
 * This module is never part of the installed production BBTC interface.
 */
#ifndef BBTC_VV_VIRIAL_FP_CONTROL_H
#define BBTC_VV_VIRIAL_FP_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Conservative runtime screening outcome, without a QUALIFIED state.
 */
typedef enum bbtc_vv_virial_control_state_e
{
    BBTC_VV_VIRIAL_CONTROL_UNCHECKED = 0,
    BBTC_VV_VIRIAL_CONTROL_UNSUPPORTED = 1,
    BBTC_VV_VIRIAL_CONTROL_SCREENED_UNQUALIFIED = 2
} bbtc_vv_virial_control_state_e;

/**
 * @brief Independent diagnostic causes returned as an OR-combinable mask.
 *
 * @details
 * Rejection is a precondition failure, NOT a failed B1 numerical comparison.
 * Separate masks allow evidence to retain several contradictory register
 * properties without short-circuiting into a misleading numerical result.
 */
typedef enum bbtc_vv_virial_control_issue_e
{
    BBTC_VV_VIRIAL_CONTROL_ISSUE_NONE         = 0u,
    BBTC_VV_VIRIAL_CONTROL_ISSUE_INVALID      = 1u << 0,
    BBTC_VV_VIRIAL_CONTROL_ISSUE_ARCH         = 1u << 1,
    BBTC_VV_VIRIAL_CONTROL_ISSUE_FENV_ROUND   = 1u << 2,
    BBTC_VV_VIRIAL_CONTROL_ISSUE_X87_ROUND    = 1u << 3,
    BBTC_VV_VIRIAL_CONTROL_ISSUE_X87_PRECISION = 1u << 4,
    BBTC_VV_VIRIAL_CONTROL_ISSUE_X87_MASKS    = 1u << 5,
    BBTC_VV_VIRIAL_CONTROL_ISSUE_MXCSR_ROUND  = 1u << 6,
    BBTC_VV_VIRIAL_CONTROL_ISSUE_MXCSR_MASKS  = 1u << 7,
    BBTC_VV_VIRIAL_CONTROL_ISSUE_MXCSR_FTZ    = 1u << 8,
    BBTC_VV_VIRIAL_CONTROL_ISSUE_MXCSR_DAZ    = 1u << 9,
    BBTC_VV_VIRIAL_CONTROL_ISSUE_MXCSR_HIGH   = 1u << 10
} bbtc_vv_virial_control_issue_e;

/**
 * @brief Raw evidence from one observation of the executing thread.
 *
 * @details
 * `available` means only that the read-only x86 instructions were compiled
 * for this host, not that the registers contain approved values. `x87_control`
 * is the 16-bit x87 control word; `mxcsr` is the 32-bit SSE control/status
 * register. The low six MXCSR sticky exception flags are *not* required to be
 * zero. `rounding_mode` is the separate ISO C `fegetround()` observation.
 *
 * Register values and the ISO C observation are recorded independently.
 * A result of UNSUPPORTED must not be converted into PASS or treated as a
 * skipped required mathematical reference. A run on another thread, after
 * any context-state mutation, needs its own observation.
 */
typedef struct bbtc_vv_virial_control_observation_t
{
    uint32_t mxcsr;
    int rounding_mode;
    uint16_t x87_control;
    bool available;
} bbtc_vv_virial_control_observation_t;

/** @brief Read-only classification output; cleared before examining input. */
typedef struct bbtc_vv_virial_control_result_t
{
    bbtc_vv_virial_control_state_e state;
    uint32_t issue_mask;
} bbtc_vv_virial_control_result_t;

/**
 * @brief Collect raw control state without changing rounding or exception bits.
 *
 * @param out Non-null caller-owned record, cleared before observation.
 * @return true when a supported instruction sequence sampled both registers;
 *         false for null output or a non-supported compilation target.
 */
bool bbtc_vv_virial_control_observe(
    bbtc_vv_virial_control_observation_t* out);

/**
 * @brief Classify a supplied raw observation under the restricted B2b profile.
 *
 * @details
 * The chosen profile requires nearest rounding in C, x87, and MXCSR; x87
 * 64-bit significand precision (PC=11); masked x87/SSE exceptions; and FTZ
 * and DAZ disabled. These requirements are conservative *test-profile*
 * restrictions, not claims that every deviation corrupts the four exactly
 * representable virial cases. No sticky exception flag is cleared or tested.
 *
 * @param observation Non-null fully initialized record to screen.
 * @param out_result Non-null result, cleared on entry.
 * @return SCREENED_UNQUALIFIED only when all restricted controls are present.
 */
bbtc_vv_virial_control_state_e bbtc_vv_virial_control_classify(
    const bbtc_vv_virial_control_observation_t* observation,
    bbtc_vv_virial_control_result_t* out_result);

/**
 * @brief Capture and screen the current thread without modifying its state.
 *
 * @param out_observation Caller-owned raw register observation.
 * @param out_result Caller-owned conservative screening result.
 * @return SCREENED_UNQUALIFIED or UNSUPPORTED, never a qualification verdict.
 */
bbtc_vv_virial_control_state_e bbtc_vv_virial_control_screen(
    bbtc_vv_virial_control_observation_t* out_observation,
    bbtc_vv_virial_control_result_t* out_result);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* BBTC_VV_VIRIAL_FP_CONTROL_H */
