/**
 * @file
 * @brief Precision-qualified composed internal-ballistics loading states.
 */
#ifndef BBTC_INTERNAL_BALLISTICS_LOADING_STATE_H
#define BBTC_INTERNAL_BALLISTICS_LOADING_STATE_H

#include <bbtc/internal_ballistics/geometry.h>
#include <bbtc/internal_ballistics/projectile.h>
#include <bbtc/internal_ballistics/propellant_charge.h>
#include <bbtc/status.h>

#ifdef __cplusplus
    extern "C" {
#endif

/**
 * @brief Composed internal-ballistics loading state using native `float`.
 *
 * The record owns one geometry, projectile, and propellant-charge component by
 * value. It introduces no alternate source for a component field and contains
 * no solver options or integration state. A zero-initialized record is
 * deliberately invalid.
 */
typedef struct bbtc_ib_loading_state_float_t
{
    /** Initial chamber, bore, and projectile-reference geometry. */
    bbtc_ib_geometry_float_t geometry;

    /** Projectile physical identity used by internal ballistics. */
    bbtc_ib_projectile_float_t projectile;

    /** Initial propellant charge and condensed-phase density. */
    bbtc_ib_propellant_charge_float_t propellant_charge;
}
bbtc_ib_loading_state_float_t;


/**
 * @brief Composed internal-ballistics loading state using native `double`.
 */
typedef struct bbtc_ib_loading_state_double_t
{
    /** Initial chamber, bore, and projectile-reference geometry. */
    bbtc_ib_geometry_double_t geometry;

    /** Projectile physical identity used by internal ballistics. */
    bbtc_ib_projectile_double_t projectile;

    /** Initial propellant charge and condensed-phase density. */
    bbtc_ib_propellant_charge_double_t propellant_charge;
}
bbtc_ib_loading_state_double_t;


/**
 * @brief Composed internal-ballistics loading state using native `long double`.
 */
typedef struct bbtc_ib_loading_state_long_double_t
{
    /** Initial chamber, bore, and projectile-reference geometry. */
    bbtc_ib_geometry_long_double_t geometry;

    /** Projectile physical identity used by internal ballistics. */
    bbtc_ib_projectile_long_double_t projectile;

    /** Initial propellant charge and condensed-phase density. */
    bbtc_ib_propellant_charge_long_double_t propellant_charge;
}
bbtc_ib_loading_state_long_double_t;


/**
 * @brief Derived initial volumes for a native-float loading state.
 *
 * Both members are outputs derived from loading-state primitives. They are not
 * independent caller inputs and do not establish that a cartridge or firearm is
 * safe.
 */
typedef struct bbtc_ib_loading_state_volumes_float_t
{
    /** Condensed propellant material volume, in cubic meters. */
    float condensed_propellant_volume_m3;

    /** Initial free-gas volume behind the projectile, in cubic meters. */
    float initial_free_gas_volume_m3;
}
bbtc_ib_loading_state_volumes_float_t;


/**
 * @brief Derived initial volumes for a native-double loading state.
 */
typedef struct bbtc_ib_loading_state_volumes_double_t
{
    /** Condensed propellant material volume, in cubic meters. */
    double condensed_propellant_volume_m3;

    /** Initial free-gas volume behind the projectile, in cubic meters. */
    double initial_free_gas_volume_m3;
}
bbtc_ib_loading_state_volumes_double_t;


/**
 * @brief Derived initial volumes for a native-long-double loading state.
 */
typedef struct bbtc_ib_loading_state_volumes_long_double_t
{
    /** Condensed propellant material volume, in cubic meters. */
    long double condensed_propellant_volume_m3;

    /** Initial free-gas volume behind the projectile, in cubic meters. */
    long double initial_free_gas_volume_m3;
}
bbtc_ib_loading_state_volumes_long_double_t;


/**
 * @brief Validates and evaluates one native-float loading state.
 *
 * The function validates every component, derives condensed propellant volume
 * from charge mass and condensed-phase density, and derives initial free-gas
 * volume by subtracting condensed volume from initial behind-projectile volume.
 * The condensed volume must be representable, positive, and strictly smaller
 * than the geometric volume. The output is cleared before any failure return.
 *
 * @param loading_state Loading state to validate and evaluate.
 * @param out_volumes Derived-volume destination.
 *
 * @return `BBTC_STATUS_SUCCESS` on success; otherwise the first component or
 *         cross-record validation status.
 */
bbtc_status_e
bbtc_ib_loading_state_evaluate_float(const bbtc_ib_loading_state_float_t*   loading_state,
                                     bbtc_ib_loading_state_volumes_float_t* out_volumes);


/**
 * @brief Validates and evaluates one native-double loading state.
 *
 * Validation, derivation, ownership, and output-clearing semantics match
 * `bbtc_ib_loading_state_evaluate_float()`.
 *
 * @param loading_state Loading state to validate and evaluate.
 * @param out_volumes Derived-volume destination.
 *
 * @return `BBTC_STATUS_SUCCESS` on success; otherwise a nonzero status.
 */
bbtc_status_e
bbtc_ib_loading_state_evaluate_double(const bbtc_ib_loading_state_double_t*   loading_state,
                                      bbtc_ib_loading_state_volumes_double_t* out_volumes);


/**
 * @brief Validates and evaluates one native-long-double loading state.
 *
 * Validation, derivation, ownership, and output-clearing semantics match
 * `bbtc_ib_loading_state_evaluate_float()`.
 *
 * @param loading_state Loading state to validate and evaluate.
 * @param out_volumes Derived-volume destination.
 *
 * @return `BBTC_STATUS_SUCCESS` on success; otherwise a nonzero status.
 */
bbtc_status_e
bbtc_ib_loading_state_evaluate_long_double(const bbtc_ib_loading_state_long_double_t*   loading_state,
                                           bbtc_ib_loading_state_volumes_long_double_t* out_volumes);

#ifdef __cplusplus
}   /* extern "C" */
#endif
#endif /* BBTC_INTERNAL_BALLISTICS_LOADING_STATE_H */
