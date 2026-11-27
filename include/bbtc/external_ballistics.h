/**
 * @file
 * @brief Umbrella header for BBTC's public external-ballistics API.
 *
 * This header intentionally exposes only external-ballistics facilities that
 * have reached a reviewed public contract. EB0.1-B5 includes mathematical
 * value records, rigid-body-state validation, projectile physical properties,
 * validated local environment samples, and caller-controlled environment
 * provider dispatch. Atmosphere models, gravity/reference-frame physics,
 * force/moment models, derivatives, and numerical integration remain later
 * checkpoints.
 */

#ifndef BBTC_EXTERNAL_BALLISTICS_H
#define BBTC_EXTERNAL_BALLISTICS_H

#include <bbtc/external_ballistics/environment.h>
#include <bbtc/external_ballistics/environment_provider.h>
#include <bbtc/external_ballistics/projectile.h>
#include <bbtc/external_ballistics/quaternion.h>
#include <bbtc/external_ballistics/state.h>
#include <bbtc/external_ballistics/vector.h>

#endif /* BBTC_EXTERNAL_BALLISTICS_H */
