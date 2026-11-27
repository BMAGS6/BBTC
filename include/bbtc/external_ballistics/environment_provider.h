/**
 * @file
 * @brief Caller-controlled external-ballistics environment-provider interface.
 *
 * EB0.1-B5 defines how later physics and integration layers request the B4
 * sampled environment at an arbitrary projectile center-of-mass position and
 * simulation time. The provider mechanism deliberately does not prescribe a
 * standard atmosphere, weather implementation, interpolation scheme, cache,
 * gravity model, or host application.
 *
 * Providers are borrowed interfaces. BBTC neither owns nor interprets provider
 * context storage, and a null context pointer is valid. Provider callbacks must
 * tolerate repeated and out-of-order sampling requests because multi-stage
 * numerical integrators may evaluate derivative stages in such an order.
 */

#ifndef BBTC_EXTERNAL_BALLISTICS_ENVIRONMENT_PROVIDER_H
#define BBTC_EXTERNAL_BALLISTICS_ENVIRONMENT_PROVIDER_H

#include <bbtc/external_ballistics/environment.h>

#ifdef __cplusplus
    extern "C" {
#endif

/**
 * @brief Native-float callback that samples one local environment.
 *
 * The callback receives the projectile center-of-mass position in metres in
 * the active BBTC world/reference frame and simulation time in seconds from
 * the external-ballistics initial state. It may use the borrowed opaque
 * `context` pointer to reach caller-owned configuration or host services.
 *
 * A successful callback must initialize every named field of `out_sample`.
 * Provider dispatch subsequently validates that sample through the matching
 * EB0.1-B4 validator. Callbacks must not assume monotonic call order or one
 * invocation per integration step.
 *
 * @param context Borrowed opaque caller context; may be null.
 * @param position_world_m Finite world/reference-frame position in metres.
 * @param time_s Finite, nonnegative simulation time in seconds.
 * @param out_sample Required destination for one B4 environment sample.
 *
 * @return `BBTC_STATUS_SUCCESS` when a complete sample was produced, or
 *         another BBTC status describing provider failure.
 */
typedef bbtc_status_e
(*bbtc_eb_environment_provider_float_fn)(
    const void* context,
    const bbtc_eb_vec3_float_t* position_world_m,
    float time_s,
    bbtc_eb_environment_sample_float_t* out_sample
);

/**
 * @brief Native-double environment-sampling callback.
 *
 * Semantics match `bbtc_eb_environment_provider_float_fn` while position,
 * time, and output sample remain entirely native `double`.
 */
typedef bbtc_status_e
(*bbtc_eb_environment_provider_double_fn)(
    const void* context,
    const bbtc_eb_vec3_double_t* position_world_m,
    double time_s,
    bbtc_eb_environment_sample_double_t* out_sample
);

/**
 * @brief Native-long-double environment-sampling callback.
 *
 * Semantics match `bbtc_eb_environment_provider_float_fn` while position,
 * time, and output sample remain entirely native `long double`.
 */
typedef bbtc_status_e
(*bbtc_eb_environment_provider_long_double_fn)(
    const void* context,
    const bbtc_eb_vec3_long_double_t* position_world_m,
    long double time_s,
    bbtc_eb_environment_sample_long_double_t* out_sample
);

/**
 * @brief Borrowed native-float environment provider descriptor.
 */
typedef struct bbtc_eb_environment_provider_float_t
{
    /** Matching native-float callback used to obtain a local sample. */
    bbtc_eb_environment_provider_float_fn sample;

    /**
     * @brief Borrowed opaque callback context.
     *
     * May be null. BBTC does not allocate, free, retain, clone, mutate, or
     * interpret the pointed-to storage.
     */
    const void* context;
}
bbtc_eb_environment_provider_float_t;

/**
 * @brief Borrowed native-double environment provider descriptor.
 */
typedef struct bbtc_eb_environment_provider_double_t
{
    /** Matching native-double callback used to obtain a local sample. */
    bbtc_eb_environment_provider_double_fn sample;

    /** Borrowed opaque callback context; may be null. */
    const void* context;
}
bbtc_eb_environment_provider_double_t;

/**
 * @brief Borrowed native-long-double environment provider descriptor.
 */
typedef struct bbtc_eb_environment_provider_long_double_t
{
    /** Matching native-long-double callback used to obtain a local sample. */
    bbtc_eb_environment_provider_long_double_fn sample;

    /** Borrowed opaque callback context; may be null. */
    const void* context;
}
bbtc_eb_environment_provider_long_double_t;

/**
 * @brief Dispatches one native-float environment sampling request.
 *
 * When `out_sample` is non-null, dispatch first establishes an all-zero B4
 * sample as the deterministic failure state. It then validates provider,
 * callback, position, and time; invokes the callback exactly once for a valid
 * request; propagates callback failure unchanged; and revalidates successful
 * callback output through the B4 sample validator.
 *
 * A null output pointer returns `BBTC_STATUS_INVALID_ARGUMENT` and naturally
 * carries no output-state guarantee. No callback is invoked for invalid request
 * data. Any later failure with a non-null output leaves the sample all-zero.
 *
 * @param provider Borrowed provider descriptor.
 * @param position_world_m Finite center-of-mass position in metres.
 * @param time_s Finite, nonnegative simulation time in seconds.
 * @param out_sample Required destination sample.
 *
 * @return Deterministic validation, provider, or B4 sample status.
 */
bbtc_status_e
bbtc_eb_environment_provider_sample_float(
    const bbtc_eb_environment_provider_float_t* provider,
    const bbtc_eb_vec3_float_t* position_world_m,
    float time_s,
    bbtc_eb_environment_sample_float_t* out_sample
);

/**
 * @brief Dispatches one native-double environment sampling request.
 *
 * Failure-state, validation, callback, and B4 revalidation semantics are
 * identical to `bbtc_eb_environment_provider_sample_float()`.
 */
bbtc_status_e
bbtc_eb_environment_provider_sample_double(
    const bbtc_eb_environment_provider_double_t* provider,
    const bbtc_eb_vec3_double_t* position_world_m,
    double time_s,
    bbtc_eb_environment_sample_double_t* out_sample
);

/**
 * @brief Dispatches one native-long-double environment sampling request.
 *
 * Failure-state, validation, callback, and B4 revalidation semantics are
 * identical to `bbtc_eb_environment_provider_sample_float()`.
 */
bbtc_status_e
bbtc_eb_environment_provider_sample_long_double(
    const bbtc_eb_environment_provider_long_double_t* provider,
    const bbtc_eb_vec3_long_double_t* position_world_m,
    long double time_s,
    bbtc_eb_environment_sample_long_double_t* out_sample
);

#ifdef __cplusplus
    }    /* extern "C" */
#endif

#endif /* BBTC_EXTERNAL_BALLISTICS_ENVIRONMENT_PROVIDER_H */
