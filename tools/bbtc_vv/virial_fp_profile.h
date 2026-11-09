/**
 * @file virial_fp_profile.h
 * @brief Developer-only B2b.3c.2 floating-point environment preflight.
 *
 * @details
 * This interface screens a narrowly described C execution environment for
 * the four independently established, dyadic virial temperature-law cases.
 * It compares the compiler's native scalar metadata with the metadata from
 * the *linked production BBTC library* and observes the caller's rounding
 * mode. The screen has no authority to certify compiled producer instructions,
 * compiler-option provenance, a physical equation of state, or safety.
 *
 * The state SCREENED_UNQUALIFIED is deliberately the strongest possible
 * result. An external, separately reviewed source/ABI/build-evidence decision
 * is required before any profile can be described as qualified. A successful
 * B1 comparison is never interpreted as a profile qualification decision.
 *
 * The observation/classification split exists so tests can inject erroneous
 * metadata without modifying the real floating-point environment or forging
 * production BBTC status codes. Neither type is installed with libbbtc.
 */
#ifndef BBTC_VV_VIRIAL_FP_PROFILE_H
#define BBTC_VV_VIRIAL_FP_PROFILE_H

#include <stdbool.h>
#include <stdint.h>

#include <bbtc/precision.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Screening state; no enumerator means scientific qualification. */
typedef enum bbtc_vv_virial_profile_state
{
    BBTC_VV_VIRIAL_PROFILE_UNCHECKED = 0,
    BBTC_VV_VIRIAL_PROFILE_UNSUPPORTED = 1,
    BBTC_VV_VIRIAL_PROFILE_SCREENED_UNQUALIFIED = 2
} bbtc_vv_virial_profile_state_e;

/**
 * @brief Independent rejection causes; multiple causes may coexist.
 *
 * The bits diagnose screening only. In particular, a zero mask is not an
 * assertion that producer flags, generated code, traps, or fenv are qualified.
 */
typedef enum bbtc_vv_virial_profile_issue
{
    BBTC_VV_VIRIAL_ISSUE_NONE                = 0,
    BBTC_VV_VIRIAL_ISSUE_INVALID_OBSERVATION = 1u << 0,
    BBTC_VV_VIRIAL_ISSUE_ROUNDING_MODE       = 1u << 1,
    BBTC_VV_VIRIAL_ISSUE_RADIX               = 1u << 2,
    BBTC_VV_VIRIAL_ISSUE_NATIVE_FORMAT       = 1u << 3,
    BBTC_VV_VIRIAL_ISSUE_PRODUCTION_METADATA = 1u << 4,
    BBTC_VV_VIRIAL_ISSUE_EVALUATION_METHOD   = 1u << 5
}
bbtc_vv_virial_profile_issue_e;

/** @brief Count of C scalar families, in public precision-identity order. */
enum { BBTC_VV_VIRIAL_NATIVE_FAMILIES = 3 };

/**
 * @brief Observations, *not* scientific inferences, from one active host.
 *
 * `compiler_info` is filled from `<float.h>` and `sizeof` in the caller's
 * compilation unit. `production_info` is returned by the linked public BBTC
 * metadata API, together with its separate raw status. Both sides are retained
 * to detect a different library ABI or an accidentally cross-linked binary.
 *
 * `rounding_mode` is copied from `fegetround()`, not normalized; a negative
 * result means it could not be observed. `evaluation_method` comes from
 * `FLT_EVAL_METHOD`. No member records the effective producer compile options:
 * a runtime program cannot reliably infer those options from a macro.
 */
typedef struct bbtc_vv_virial_profile_observation_t
{
    bbtc_precision_info_t compiler_info[BBTC_VV_VIRIAL_NATIVE_FAMILIES];
    bbtc_precision_info_t production_info[BBTC_VV_VIRIAL_NATIVE_FAMILIES];
    bbtc_status_e production_status[BBTC_VV_VIRIAL_NATIVE_FAMILIES];
    int rounding_mode;
    int evaluation_method;
}
bbtc_vv_virial_profile_observation_t;

/**
 * @brief Explicit outcome of screening; never contains QUALIFIED.
 *
 * `issue_mask` contains only `BBTC_VV_VIRIAL_ISSUE_*` bits and is zero iff
 * this limited screening succeeded. The caller may inspect the observation
 * separately even if it is unsupported, but must not run required reference
 * comparisons as admissible evidence in that unsupported state.
 */
typedef struct bbtc_vv_virial_profile_result_t
{
    bbtc_vv_virial_profile_state_e state;
    uint32_t issue_mask;
}
bbtc_vv_virial_profile_result_t;

/**
 * @brief Classify a supplied, fully initialized metadata observation.
 *
 * @details
 * This function performs no floating-point arithmetic, does not change the
 * caller's environment, and does not invoke BBTC. Its deterministic rejection
 * reasons permit independent negative controls for missing data, radix,
 * mantissa/exponent/storage discrepancies, unsupported evaluation methods,
 * rounding mode, and production metadata disagreement.
 *
 * @param observation Non-null, readable caller-owned observation.
 * @param out_result Non-null caller-owned result; cleared on entry.
 * @return SCREENED_UNQUALIFIED only for the supported *screening* subset;
 *         UNSUPPORTED for bad/missing data or arguments.
 */
bbtc_vv_virial_profile_state_e bbtc_vv_virial_profile_classify(
    const bbtc_vv_virial_profile_observation_t* observation,
    bbtc_vv_virial_profile_result_t* out_result);

/**
 * @brief Observe the current host, production-library metadata and fenv.
 *
 * @details
 * Does not call `fesetround`, clear sticky exceptions, enable traps, or
 * operate on values being numerically compared. It reports `fegetround()`
 * without changing it. If a production metadata query fails, its raw status
 * remains available and the classifier rejects the observation.
 *
 * @param out_observation Non-null caller-owned observation; cleared on entry.
 * @return true if an observation was written (not necessarily admissible).
 */
bool bbtc_vv_virial_profile_observe(
    bbtc_vv_virial_profile_observation_t* out_observation);

/**
 * @brief Perform the current-host preflight and preserve both evidence axes.
 *
 * @param out_observation Non-null observation record to populate.
 * @param out_result Non-null screening result to populate.
 * @return Limited preflight state, never a scientific qualification verdict.
 */
bbtc_vv_virial_profile_state_e bbtc_vv_virial_profile_screen(
    bbtc_vv_virial_profile_observation_t* out_observation,
    bbtc_vv_virial_profile_result_t* out_result);

#ifdef __cplusplus
  }    /* extern "C" */
#endif
#endif /* BBTC_VV_VIRIAL_FP_PROFILE_H */
