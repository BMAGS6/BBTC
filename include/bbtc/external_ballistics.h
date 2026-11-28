/**
 * @file
 * @brief Umbrella header for BBTC's public external-ballistics API.
 *
 * This header intentionally exposes only external-ballistics facilities that
 * have reached a reviewed public contract: the EB0 mathematical/state/provider
 * foundation, EB1 inertial-vacuum propagation, and EB2.1
 * projectile-relative-air derived-flow evaluation. Drag force, attitude
 * aerodynamics, and non-inertial acceleration remain later checkpoints.
 */

#ifndef BBTC_EXTERNAL_BALLISTICS_H
#define BBTC_EXTERNAL_BALLISTICS_H

#include <bbtc/external_ballistics/environment.h>
#include <bbtc/external_ballistics/environment_provider.h>
#include <bbtc/external_ballistics/force_moment.h>
#include <bbtc/external_ballistics/inertial_vacuum.h>
#include <bbtc/external_ballistics/inertial_vacuum_provider.h>
#include <bbtc/external_ballistics/integrator.h>
#include <bbtc/external_ballistics/projectile.h>
#include <bbtc/external_ballistics/quaternion.h>
#include <bbtc/external_ballistics/reference_frame.h>
#include <bbtc/external_ballistics/reference_frame_provider.h>
#include <bbtc/external_ballistics/relative_air.h>
#include <bbtc/external_ballistics/state.h>
#include <bbtc/external_ballistics/state_derivative.h>
#include <bbtc/external_ballistics/state_derivative_provider.h>
#include <bbtc/external_ballistics/vector.h>

#endif /* BBTC_EXTERNAL_BALLISTICS_H */
