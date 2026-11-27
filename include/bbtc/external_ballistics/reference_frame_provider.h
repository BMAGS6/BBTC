/**
 * @file
 * @brief Caller-controlled external-ballistics reference-frame provider.
 *
 * EB0.1-B7 defines how later physics and integration layers request the B6
 * gravity/reference-frame sample at an arbitrary projectile center-of-mass
 * position and simulation time.
 *
 * The provider boundary is intentionally narrow. It receives only position,
 * time, and borrowed caller context. Projectile velocity is deliberately not
 * part of this interface because velocity-dependent non-inertial effects such
 * as Coriolis acceleration belong to a later consumer-side physics layer.
 *
 * Providers are borrowed interfaces. BBTC neither owns nor interprets provider
 * context storage, and a null context pointer is valid. Provider callbacks must
 * tolerate repeated and out-of-order sampling requests because multi-stage
 * numerical integrators may evaluate derivative stages in such an order.
 *
 * A successful callback must produce a complete EB0.1-B6 sample. Dispatch
 * revalidates that sample before returning success. BBTC does not infer whether
 * a finite vector has been physically mislabeled by a provider; semantic
 * correctness of gravity and frame kinematics remains the provider's
 * responsibility.
 */

#ifndef BBTC_EXTERNAL_BALLISTICS_REFERENCE_FRAME_PROVIDER_H
#define BBTC_EXTERNAL_BALLISTICS_REFERENCE_FRAME_PROVIDER_H

#include <bbtc/external_ballistics/reference_frame.h>

#ifdef __cplusplus
    extern "C" {
#endif

/**
 * @brief Native-float callback that samples gravity/reference-frame state.
 *
 * `position_reference_m` is the projectile center-of-mass position measured
 * from the active BBTC reference-frame origin and expressed in active-reference
 * axes, in metres. `time_s` is simulation time measured from the
 * external-ballistics initial state.
 *
 * The callback may use `context` to reach caller-owned configuration or a
 * deterministic external frame-model snapshot. It must not depend on mutable
 * sampling history, one-call-per-step behavior, or monotonic request order.
 *
 * A callback returning `BBTC_STATUS_SUCCESS` must initialize every named field
 * of `out_sample`. Provider dispatch subsequently validates that output through
 * `bbtc_eb_reference_frame_sample_validate_float()`.
 *
 * @param context Borrowed opaque caller context; may be null.
 * @param position_reference_m Finite active-reference position in metres.
 * @param time_s Finite, nonnegative simulation time in seconds.
 * @param out_sample Required destination for one complete B6 sample.
 *
 * @return `BBTC_STATUS_SUCCESS` after producing a complete sample, or another
 *         BBTC status describing provider failure.
 */
typedef bbtc_status_e
(*bbtc_eb_reference_frame_provider_float_fn)(
    const void* context,
    const bbtc_eb_vec3_float_t* position_reference_m,
    float time_s,
    bbtc_eb_reference_frame_sample_float_t* out_sample
);

/**
 * @brief Native-double reference-frame sampling callback.
 *
 * Semantics match `bbtc_eb_reference_frame_provider_float_fn` while position,
 * time, and output sample remain entirely native `double`.
 */
typedef bbtc_status_e
(*bbtc_eb_reference_frame_provider_double_fn)(
    const void* context,
    const bbtc_eb_vec3_double_t* position_reference_m,
    double time_s,
    bbtc_eb_reference_frame_sample_double_t* out_sample
);

/**
 * @brief Native-long-double reference-frame sampling callback.
 *
 * Semantics match `bbtc_eb_reference_frame_provider_float_fn` while position,
 * time, and output sample remain entirely native `long double`.
 */
typedef bbtc_status_e
(*bbtc_eb_reference_frame_provider_long_double_fn)(
    const void* context,
    const bbtc_eb_vec3_long_double_t* position_reference_m,
    long double time_s,
    bbtc_eb_reference_frame_sample_long_double_t* out_sample
);

/**
 * @brief Borrowed native-float reference-frame provider descriptor.
 */
typedef struct bbtc_eb_reference_frame_provider_float_t
{
    /** Matching native-float callback used to obtain one B6 sample. */
    bbtc_eb_reference_frame_provider_float_fn sample;

    /**
     * @brief Borrowed opaque callback context.
     *
     * May be null. BBTC does not allocate, free, retain, clone, mutate, or
     * interpret the pointed-to storage.
     */
    const void* context;
}
bbtc_eb_reference_frame_provider_float_t;

/**
 * @brief Borrowed native-double reference-frame provider descriptor.
 */
typedef struct bbtc_eb_reference_frame_provider_double_t
{
    /** Matching native-double callback used to obtain one B6 sample. */
    bbtc_eb_reference_frame_provider_double_fn sample;

    /** Borrowed opaque callback context; may be null. */
    const void* context;
}
bbtc_eb_reference_frame_provider_double_t;

/**
 * @brief Borrowed native-long-double reference-frame provider descriptor.
 */
typedef struct bbtc_eb_reference_frame_provider_long_double_t
{
    /** Matching native-long-double callback used to obtain one B6 sample. */
    bbtc_eb_reference_frame_provider_long_double_fn sample;

    /** Borrowed opaque callback context; may be null. */
    const void* context;
}
bbtc_eb_reference_frame_provider_long_double_t;

/**
 * @brief Dispatches one native-float reference-frame sampling request.
 *
 * A non-null `out_sample` is first overwritten with the exact all-zero B6
 * sample. Dispatch then validates provider, callback, position, and time,
 * invokes the callback exactly once for a valid request, propagates callback
 * failure unchanged, and revalidates successful callback output through the B6
 * validator.
 *
 * Every failure after a non-null output pointer has been established returns
 * with `*out_sample` restored to the exact all-zero B6 sample.
 *
 * The all-zero B6 sample is itself a valid physical sample representing
 * inertial, nonaccelerating, nonrotating, zero-gravity conditions. Therefore
 * the returned `bbtc_status_e` is the authoritative success/failure indicator.
 * Callers must not infer provider success merely because the returned output
 * validates successfully.
 *
 * @param provider Borrowed provider descriptor.
 * @param position_reference_m Finite center-of-mass position in metres.
 * @param time_s Finite, nonnegative simulation time in seconds.
 * @param out_sample Required destination sample.
 *
 * @return Deterministic request-validation, callback, or B6 validation status.
 */
bbtc_status_e
bbtc_eb_reference_frame_provider_sample_float(
    const bbtc_eb_reference_frame_provider_float_t* provider,
    const bbtc_eb_vec3_float_t* position_reference_m,
    float time_s,
    bbtc_eb_reference_frame_sample_float_t* out_sample
);

/**
 * @brief Dispatches one native-double reference-frame sampling request.
 *
 * Failure-state, validation, callback, B6 revalidation, and authoritative
 * status semantics are identical to
 * `bbtc_eb_reference_frame_provider_sample_float()`.
 */
bbtc_status_e
bbtc_eb_reference_frame_provider_sample_double(
    const bbtc_eb_reference_frame_provider_double_t* provider,
    const bbtc_eb_vec3_double_t* position_reference_m,
    double time_s,
    bbtc_eb_reference_frame_sample_double_t* out_sample
);

/**
 * @brief Dispatches one native-long-double reference-frame sampling request.
 *
 * Failure-state, validation, callback, B6 revalidation, and authoritative
 * status semantics are identical to
 * `bbtc_eb_reference_frame_provider_sample_float()`.
 */
bbtc_status_e
bbtc_eb_reference_frame_provider_sample_long_double(
    const bbtc_eb_reference_frame_provider_long_double_t* provider,
    const bbtc_eb_vec3_long_double_t* position_reference_m,
    long double time_s,
    bbtc_eb_reference_frame_sample_long_double_t* out_sample
);

#ifdef __cplusplus
    }    /* extern "C" */
#endif

#endif /* BBTC_EXTERNAL_BALLISTICS_REFERENCE_FRAME_PROVIDER_H */
