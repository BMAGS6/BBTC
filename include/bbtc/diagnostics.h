/**
 * @file
 * @brief Public simulation-termination and diagnostic metadata.
 */
#ifndef BBTC_DIAGNOSTICS_H
#define BBTC_DIAGNOSTICS_H

#include <stdint.h>

#ifdef __cplusplus
    extern "C" {
#endif

/**
 * @brief Describes why an internal-ballistics simulation stopped.
 *
 * Termination describes the modeled or numerical endpoint of a simulation. It
 * is separate from bbtc_status_e, warning flags, applicability flags, and any
 * firearm-safety judgment. The one-byte underlying representation and numeric
 * values are part of the public interface and must not be changed, renumbered,
 * or reused without an explicit ABI revision.
 */
typedef enum bbtc_ib_termination_e : uint8_t
{
    /** No internal-ballistics simulation has run. */
    BBTC_IB_TERMINATION_NOT_RUN = 0,

    /** The projectile reached the defined muzzle-exit position. */
    BBTC_IB_TERMINATION_MUZZLE_EXIT = 1,

    /** The modeled ignition process did not ignite the propellant. */
    BBTC_IB_TERMINATION_NO_IGNITION = 2,

    /** Pressure never overcame the modeled projectile-start resistance. */
    BBTC_IB_TERMINATION_PROJECTILE_NOT_STARTED = 3,

    /** The projectile began moving but stopped before muzzle exit. */
    BBTC_IB_TERMINATION_PROJECTILE_STOPPED_BEFORE_MUZZLE_EXIT = 4,

    /** The caller's maximum simulated-time guard was reached. */
    BBTC_IB_TERMINATION_TIME_GUARD_REACHED = 5,

    /** The caller's maximum simulated-pressure guard was reached. */
    BBTC_IB_TERMINATION_PRESSURE_GUARD_REACHED = 6,

    /** The caller's maximum accepted-step guard was reached. */
    BBTC_IB_TERMINATION_STEP_GUARD_REACHED = 7,

    /** A numerical failure prevented a coherent modeled endpoint. */
    BBTC_IB_TERMINATION_NUMERICAL_FAILURE = 8
}
bbtc_ib_termination_e;

/**
 * @brief Returns immutable, nonlocalized text for a termination value.
 *
 * The returned pointer is never null. It refers to static storage that remains
 * valid for the lifetime of the process, may be read concurrently, and must not
 * be modified or freed. Unrecognized numeric values return
 * `"unknown BBTC internal-ballistics termination"`.
 *
 * @param termination Termination value to describe.
 *
 * @return A null-terminated string in immutable static storage.
 */
const char*
bbtc_ib_termination_string(bbtc_ib_termination_e termination);

/**
 * @brief Storage type for zero or more BBTC warning bits.
 *
 * Warning bits report nonfatal computational or reporting conditions. A zero
 * mask means that no defined warning was reported; it does not establish model
 * applicability or firearm safety.
 */
typedef uint64_t bbtc_warning_flags_t;

/**
 * @brief Individual bits accepted by bbtc_warning_flags_t.
 *
 * Every nonzero enumerator is one independent bit. Numeric bit assignments are
 * part of the public interface and must not be renumbered or reused.
 */
typedef enum bbtc_warning_flag_e : uint64_t
{
    /** No defined warning was reported. */
    BBTC_WARNING_NONE                            = UINT64_C(0),

    /** Requested history storage filled before simulation termination. */
    BBTC_WARNING_HISTORY_TRUNCATED               = UINT64_C(1) << 0,

    /** The requested energy-accounting residual tolerance was exceeded. */
    BBTC_WARNING_ENERGY_RESIDUAL_EXCEEDED        = UINT64_C(1) << 1,

    /** Propellant burn remained incomplete at muzzle exit. */
    BBTC_WARNING_INCOMPLETE_BURN_AT_MUZZLE_EXIT  = UINT64_C(1) << 2,

    /** A documented fallback approximation replaced the preferred method. */
    BBTC_WARNING_FALLBACK_APPROXIMATION_USED     = UINT64_C(1) << 3,

    /** An event was located with reduced numerical accuracy. */
    BBTC_WARNING_REDUCED_EVENT_LOCATION_ACCURACY = UINT64_C(1) << 4,

    /** A supplied data record was evaluated outside its tabulated domain. */
    BBTC_WARNING_DATA_EXTRAPOLATED               = UINT64_C(1) << 5
}
bbtc_warning_flag_e;

/**
 * @brief Returns immutable, nonlocalized text for one warning value.
 *
 * This function describes one enumerator, not a combined warning mask.
 * Unrecognized values and combinations of multiple warning bits return
 * `"unknown or combined BBTC warning flag"`.
 *
 * The returned pointer is never null. It refers to immutable static storage,
 * may be read concurrently, and must not be modified or freed.
 *
 * @param flag Individual warning value to describe.
 *
 * @return A null-terminated string in immutable static storage.
 */
const char*
bbtc_warning_flag_string(bbtc_warning_flag_e flag);

/**
 * @brief Storage type for zero or more model-applicability bits.
 *
 * Applicability flags report limitations on how a computed result may be
 * interpreted. They are independent of API status and warning flags. A zero
 * mask means that no defined limitation was reported; it is not evidence of
 * validation, approval, or firearm safety.
 */
typedef uint64_t bbtc_applicability_flags_t;

/**
 * @brief Individual bits accepted by bbtc_applicability_flags_t.
 *
 * Every nonzero enumerator is one independent bit. Numeric bit assignments are
 * part of the public interface and must not be renumbered or reused.
 */
typedef enum bbtc_applicability_flag_e : uint64_t
{
    /** No defined applicability limitation was reported. */
    BBTC_APPLICABILITY_NONE_REPORTED                   = UINT64_C(0),

    /** A parameter set was used outside its documented calibration domain. */
    BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN      = UINT64_C(1) << 0,

    /** The selected combination of models lacks experimental validation. */
    BBTC_APPLICABILITY_MODEL_COMBINATION_UNVALIDATED   = UINT64_C(1) << 1,

    /** Supplied geometry or state materially stressed model assumptions. */
    BBTC_APPLICABILITY_ASSUMPTIONS_MATERIALLY_STRESSED = UINT64_C(1) << 2,

    /** A user-supplied data record had unknown provenance. */
    BBTC_APPLICABILITY_DATA_PROVENANCE_UNKNOWN         = UINT64_C(1) << 3,

    /** An approximation was used in place of a requested physical effect. */
    BBTC_APPLICABILITY_REQUESTED_EFFECT_APPROXIMATED   = UINT64_C(1) << 4
}
bbtc_applicability_flag_e;

/**
 * @brief Returns immutable, nonlocalized text for one applicability value.
 *
 * This function describes one enumerator, not a combined applicability mask.
 * Unrecognized values and combinations of multiple applicability bits return
 * `"unknown or combined BBTC applicability flag"`.
 *
 * The returned pointer is never null. It refers to immutable static storage,
 * may be read concurrently, and must not be modified or freed.
 *
 * @param flag Individual applicability value to describe.
 *
 * @return A null-terminated string in immutable static storage.
 */
const char*
bbtc_applicability_flag_string(bbtc_applicability_flag_e flag);

#ifdef __cplusplus
}   /* extern "C" */
#endif
#endif /* BBTC_DIAGNOSTICS_H */
