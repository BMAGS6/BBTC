/**
 * @file
 * @brief Inertial-vacuum composition for the generic state-derivative provider.
 *
 * EB1.2 binds immutable-during-use projectile/loading configuration and a B7
 * reference-frame provider to the generic B10 derivative-provider interface.
 * Each derivative request samples B7 at the candidate state's exact position
 * and time, then delegates physical equations to the accepted EB1.1 evaluator.
 */

#ifndef BBTC_EXTERNAL_BALLISTICS_INERTIAL_VACUUM_PROVIDER_H
#define BBTC_EXTERNAL_BALLISTICS_INERTIAL_VACUUM_PROVIDER_H

#include <bbtc/external_ballistics/force_moment.h>
#include <bbtc/external_ballistics/projectile.h>
#include <bbtc/external_ballistics/reference_frame_provider.h>
#include <bbtc/external_ballistics/state_derivative_provider.h>
#include <bbtc/status.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Native-float EB1.2 inertial-vacuum provider configuration. */
typedef struct bbtc_eb_inertial_vacuum_provider_config_float_t
{
    /** Rigid-projectile physical properties stored by value. */
    bbtc_eb_projectile_properties_float_t projectile;

    /** Constant physical applied force/moment stored by value. */
    bbtc_eb_force_moment_float_t constant_force_moment;

    /** B7 descriptor stored by value; its context remains borrowed. */
    bbtc_eb_reference_frame_provider_float_t reference_frame_provider;
}
bbtc_eb_inertial_vacuum_provider_config_float_t;

/** @brief Native-double EB1.2 inertial-vacuum provider configuration. */
typedef struct bbtc_eb_inertial_vacuum_provider_config_double_t
{
    /** Rigid-projectile physical properties stored by value. */
    bbtc_eb_projectile_properties_double_t projectile;

    /** Constant physical applied force/moment stored by value. */
    bbtc_eb_force_moment_double_t constant_force_moment;

    /** B7 descriptor stored by value; its context remains borrowed. */
    bbtc_eb_reference_frame_provider_double_t reference_frame_provider;
}
bbtc_eb_inertial_vacuum_provider_config_double_t;

/** @brief Native-long-double EB1.2 inertial-vacuum provider configuration. */
typedef struct bbtc_eb_inertial_vacuum_provider_config_long_double_t
{
    /** Rigid-projectile physical properties stored by value. */
    bbtc_eb_projectile_properties_long_double_t projectile;

    /** Constant physical applied force/moment stored by value. */
    bbtc_eb_force_moment_long_double_t constant_force_moment;

    /** B7 descriptor stored by value; its context remains borrowed. */
    bbtc_eb_reference_frame_provider_long_double_t reference_frame_provider;
}
bbtc_eb_inertial_vacuum_provider_config_long_double_t;

/**
 * @brief Validates one native-float EB1.2 provider configuration.
 *
 * Validation order is B3 projectile, B8 constant force/moment, then structural
 * B7 callback validity. The B7 callback is never invoked by validation and a
 * null nested B7 context is valid.
 */
bbtc_status_e
bbtc_eb_inertial_vacuum_provider_config_validate_float(
    const bbtc_eb_inertial_vacuum_provider_config_float_t* config
);

/** @brief Native-double validation with identical EB1.2 semantics. */
bbtc_status_e
bbtc_eb_inertial_vacuum_provider_config_validate_double(
    const bbtc_eb_inertial_vacuum_provider_config_double_t* config
);

/** @brief Native-long-double validation with identical EB1.2 semantics. */
bbtc_status_e
bbtc_eb_inertial_vacuum_provider_config_validate_long_double(
    const bbtc_eb_inertial_vacuum_provider_config_long_double_t* config
);

/**
 * @brief Binds one native-float configuration to an ordinary B10 descriptor.
 *
 * On success, `out_provider->context` borrows exactly `config`. The caller must
 * keep that configuration alive and immutable during provider evaluation or
 * B11 stepping. On failure with nonnull output, both descriptor fields compare
 * equal to null.
 */
bbtc_status_e
bbtc_eb_inertial_vacuum_provider_bind_float(
    const bbtc_eb_inertial_vacuum_provider_config_float_t* config,
    bbtc_eb_state_derivative_provider_float_t* out_provider
);

/** @brief Native-double binding with identical EB1.2 semantics. */
bbtc_status_e
bbtc_eb_inertial_vacuum_provider_bind_double(
    const bbtc_eb_inertial_vacuum_provider_config_double_t* config,
    bbtc_eb_state_derivative_provider_double_t* out_provider
);

/** @brief Native-long-double binding with identical EB1.2 semantics. */
bbtc_status_e
bbtc_eb_inertial_vacuum_provider_bind_long_double(
    const bbtc_eb_inertial_vacuum_provider_config_long_double_t* config,
    bbtc_eb_state_derivative_provider_long_double_t* out_provider
);

#ifdef __cplusplus
}   /* extern "C" */
#endif

#endif /* BBTC_EXTERNAL_BALLISTICS_INERTIAL_VACUUM_PROVIDER_H */
