/**
 * @file
 * @brief Umbrella header for BBTC's public external-ballistics API.
 *
 * This header intentionally exposes only external-ballistics facilities that
 * have reached a reviewed public contract. EB0.1-B10 includes mathematical
 * value records, rigid-body-state validation, projectile physical properties,
 * validated local environment samples, caller-controlled environment and
 * reference-frame providers, validated applied force/moment values, validated
 * rigid-body state-derivative values, and a generic caller-controlled derivative
 * provider boundary. Physical derivative evaluation, non-inertial acceleration
 * evaluation, atmosphere and force/moment models, and numerical integration
 * remain later checkpoints.
 */

#ifndef BBTC_EXTERNAL_BALLISTICS_H
#define BBTC_EXTERNAL_BALLISTICS_H

#include <bbtc/external_ballistics/environment.h>
#include <bbtc/external_ballistics/environment_provider.h>
#include <bbtc/external_ballistics/force_moment.h>
#include <bbtc/external_ballistics/projectile.h>
#include <bbtc/external_ballistics/quaternion.h>
#include <bbtc/external_ballistics/reference_frame.h>
#include <bbtc/external_ballistics/reference_frame_provider.h>
#include <bbtc/external_ballistics/state.h>
#include <bbtc/external_ballistics/state_derivative.h>
#include <bbtc/external_ballistics/state_derivative_provider.h>
#include <bbtc/external_ballistics/vector.h>

#endif /* BBTC_EXTERNAL_BALLISTICS_H */
