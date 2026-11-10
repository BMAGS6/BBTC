/**
 * @file virial_fp_control.c
 * @brief VV0.1-B2b.3c.3a hardware floating-point control-state preflight.
 *
 * @details
 * The x87 control word and SSE MXCSR are separate registers. The ISO C
 * `fegetround()` result need not by itself establish the state of both units.
 * We therefore sample all three for the selected x86-64 / x86-with-SSE path.
 * Neither `fnstcw` nor `stmxcsr` changes the saved control state; in particular
 * this module does NOT use `fldcw`, `ldmxcsr`, `fesetround`, `feclearexcept`,
 * or `feenableexcept`. Adversarial checks operate on struct copies only.
 *
 * A successfully screened control state does NOT prove the independently
 * compiled production evaluator obeyed a particular compiler flag or that
 * these register values persisted across every instruction of the evaluation.
 * That review remains a distinct B2b.3c.3 qualification obligation.
 */
#include "virial_fp_control.h"

#include <fenv.h>
#include <stddef.h>

#if defined(__FAST_MATH__)
    #error "B2b.3c.3a requires non-fast-math floating-point semantics"
#endif
#if defined(__FINITE_MATH_ONLY__) && __FINITE_MATH_ONLY__ != 0
    #error "B2b.3c.3a requires finite-math-only disabled"
#endif

/*
 * 32-bit x86 only admits this probe if compiled with SSE enabled. On x86-64
 * the SSE baseline is part of the ABI. No x86 assembly is emitted elsewhere.
 * GNU/Clang inline asm is intentionally confined to this private module.
 */
#if (defined(__x86_64__) || (defined(__i386__) && defined(__SSE__))) && \
    (defined(__GNUC__) || defined(__clang__))
    #define BBTC_VV_CAN_READ_X86_FP_CONTROL 1
#else
    #define BBTC_VV_CAN_READ_X86_FP_CONTROL 0
#endif

/** @brief Bit fields from the architectural x87 and SSE control encodings. */
enum
{
    BBTC_VV_X87_EXCEPTION_MASK = 0x003f,
    BBTC_VV_X87_PRECISION_BITS = 0x0300,
    BBTC_VV_X87_ROUNDING_BITS  = 0x0c00,
    BBTC_VV_MXCSR_DAZ          = 0x0040,
    BBTC_VV_MXCSR_MASKS        = 0x1f80,
    BBTC_VV_MXCSR_ROUNDING     = 0x6000,
    BBTC_VV_MXCSR_FTZ          = 0x8000
};

/**
 * @brief Read x87 and MXCSR into caller storage; preserve the active state.
 * @param out Non-null, writable evidence record.
 * @return true if this compiled target has a supported read-only path.
 */
bool bbtc_vv_virial_control_observe(
    bbtc_vv_virial_control_observation_t* out)
{
    if (out == NULL)
        return false;

    *out = (bbtc_vv_virial_control_observation_t){0};
    out->rounding_mode = fegetround();

#if BBTC_VV_CAN_READ_X86_FP_CONTROL
    /* Separate memory outputs prevent narrowing, hidden FP operations, or a
     * compiler-generated dependency on a particular C floating-point type. */
    __asm__ volatile("fnstcw %0" : "=m"(out->x87_control) : : "memory");
    __asm__ volatile("stmxcsr %0" : "=m"(out->mxcsr) : : "memory");
    out->available = true;
    return true;
#else
    return false;
#endif
}

/**
 * @brief Validate a synthetic or real observation without touching hardware.
 * @param observation Readable observation or NULL.
 * @param out_result Writable outcome or NULL.
 * @return Restricted control-state screen status, never QUALIFIED.
 */
bbtc_vv_virial_control_state_e bbtc_vv_virial_control_classify(
    const bbtc_vv_virial_control_observation_t* observation,
    bbtc_vv_virial_control_result_t* out_result)
{
    if (out_result == NULL)
        return BBTC_VV_VIRIAL_CONTROL_UNSUPPORTED;

    *out_result = (bbtc_vv_virial_control_result_t){0};

    if (observation == NULL)
    {
        out_result->issue_mask = BBTC_VV_VIRIAL_CONTROL_ISSUE_INVALID;
        out_result->state = BBTC_VV_VIRIAL_CONTROL_UNSUPPORTED;
        return out_result->state;
    }

    uint32_t issues = BBTC_VV_VIRIAL_CONTROL_ISSUE_NONE;

    if (!observation->available)
        issues |= BBTC_VV_VIRIAL_CONTROL_ISSUE_ARCH;

    if (observation->rounding_mode != FE_TONEAREST)
        issues |= BBTC_VV_VIRIAL_CONTROL_ISSUE_FENV_ROUND;

    if ((observation->x87_control & BBTC_VV_X87_ROUNDING_BITS) != 0u)
        issues |= BBTC_VV_VIRIAL_CONTROL_ISSUE_X87_ROUND;

    if ((observation->x87_control & BBTC_VV_X87_PRECISION_BITS) !=
        BBTC_VV_X87_PRECISION_BITS)
    {
        issues |= BBTC_VV_VIRIAL_CONTROL_ISSUE_X87_PRECISION;
    }

    if ((observation->x87_control & BBTC_VV_X87_EXCEPTION_MASK) !=
        BBTC_VV_X87_EXCEPTION_MASK)
    {
        issues |= BBTC_VV_VIRIAL_CONTROL_ISSUE_X87_MASKS;
    }

    if ((observation->mxcsr & BBTC_VV_MXCSR_ROUNDING) != 0u)
        issues |= BBTC_VV_VIRIAL_CONTROL_ISSUE_MXCSR_ROUND;

    if ((observation->mxcsr & BBTC_VV_MXCSR_MASKS) != BBTC_VV_MXCSR_MASKS)
        issues |= BBTC_VV_VIRIAL_CONTROL_ISSUE_MXCSR_MASKS;

    if ((observation->mxcsr & BBTC_VV_MXCSR_FTZ) != 0u)
        issues |= BBTC_VV_VIRIAL_CONTROL_ISSUE_MXCSR_FTZ;

    if ((observation->mxcsr & BBTC_VV_MXCSR_DAZ) != 0u)
        issues |= BBTC_VV_VIRIAL_CONTROL_ISSUE_MXCSR_DAZ;

    /* No architectural MXCSR status/control fields exist above bit 15 in the
     * subset under review. A fabricated high bit is inconsistent evidence. */
    if ((observation->mxcsr & UINT32_C(0xffff0000)) != 0u)
        issues |= BBTC_VV_VIRIAL_CONTROL_ISSUE_MXCSR_HIGH;

    out_result->issue_mask = issues;
    out_result->state = issues == BBTC_VV_VIRIAL_CONTROL_ISSUE_NONE
        ? BBTC_VV_VIRIAL_CONTROL_SCREENED_UNQUALIFIED
        : BBTC_VV_VIRIAL_CONTROL_UNSUPPORTED;

    return out_result->state;
}

/**
 * @brief Obtain and immediately classify a raw observation from this thread.
 * @param out_observation Required raw-observation destination.
 * @param out_result Required classification destination.
 * @return Unsupported on invalid arguments or an unadmitted register state.
 */
bbtc_vv_virial_control_state_e bbtc_vv_virial_control_screen(
    bbtc_vv_virial_control_observation_t* out_observation,
    bbtc_vv_virial_control_result_t* out_result)
{
    if (out_result == NULL)
        return BBTC_VV_VIRIAL_CONTROL_UNSUPPORTED;

    *out_result = (bbtc_vv_virial_control_result_t){0};

    if (out_observation == NULL)
    {
        out_result->state = BBTC_VV_VIRIAL_CONTROL_UNSUPPORTED;
        out_result->issue_mask = BBTC_VV_VIRIAL_CONTROL_ISSUE_INVALID;
        return out_result->state;
    }

    if (!bbtc_vv_virial_control_observe(out_observation))
    {
        out_result->state = BBTC_VV_VIRIAL_CONTROL_UNSUPPORTED;
        out_result->issue_mask = BBTC_VV_VIRIAL_CONTROL_ISSUE_ARCH;
        return out_result->state;
    }

    return bbtc_vv_virial_control_classify(out_observation, out_result);
}
