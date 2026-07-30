/**
 * @file
 * @brief Public API status codes and their stable text representation.
 */
#ifndef BBTC_STATUS_H
#define BBTC_STATUS_H

#include <stdint.h>

#ifdef __cplusplus
    extern "C" {
#endif

/**
 * @brief Reports whether a BBTC API operation completed successfully.
 *
 * These values describe API execution, not physical termination conditions,
 * warnings, model applicability, or firearm safety. Numeric values are part of
 * the public interface and must not be renumbered or reused.
 */
typedef enum bbtc_status_e : uint32_t
{
    /** The operation completed successfully. */
    BBTC_STATUS_SUCCESS = 0,

    /** A required pointer was null or another argument was invalid. */
    BBTC_STATUS_INVALID_ARGUMENT = 1,

    /** A floating-point input was NaN or infinite. */
    BBTC_STATUS_NONFINITE_INPUT = 2,

    /** A finite value was outside the mathematical domain of an operation. */
    BBTC_STATUS_OUTSIDE_DOMAIN = 3,

    /** Supplied geometry or configuration values contradicted one another. */
    BBTC_STATUS_INCONSISTENT_CONFIGURATION = 4,

    /** The requested model or option is not implemented or available. */
    BBTC_STATUS_UNSUPPORTED_MODEL_OR_OPTION = 5,

    /** Caller-provided storage was insufficient for a required result. */
    BBTC_STATUS_INSUFFICIENT_STORAGE = 6,

    /** A numerical operation failed to produce a valid result. */
    BBTC_STATUS_NUMERICAL_FAILURE = 7,

    /**
     * An internal iteration limit prevented API completion; this is not a
     * caller-configured simulation guard.
     */
    BBTC_STATUS_ITERATION_LIMIT = 8,

    /** An internal invariant was violated. */
    BBTC_STATUS_INTERNAL_INVARIANT_FAILURE = 9
} bbtc_status_e;

/**
 * @brief Returns immutable, nonlocalized text for a BBTC status value.
 *
 * The returned pointer is never null. It refers to static storage that remains
 * valid for the lifetime of the process, may be read concurrently, and must not
 * be modified or freed. Unrecognized numeric values return
 * `"unknown BBTC status"`.
 *
 * @param status Status value to describe.
 *
 * @return A null-terminated status string in immutable static storage.
 */
const char*
bbtc_status_string(bbtc_status_e status);

#ifdef __cplusplus
}   /* extern "C" */
#endif
#endif /* BBTC_STATUS_H */
