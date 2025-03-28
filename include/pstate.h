/**
 * @file pstate.h
 * @brief Defines the projectile state in 3D (position, velocity, orientation, etc.).
 */

#ifndef PSTATE_H
#define PSTATE_H

#ifdef __cplusplus
	extern "C" {
#endif

#include "quat.h"
#include "compiler_macros.h"

/**
 * @defgroup ProjectileState Projectile State Tracking
 * @ingroup AllFiles
 * @{
 */

/**
 * @brief Represents the full projectile state in 3D space.
 *
 * Fields:
 *  - \c x, \c y, \c z for position (meters)
 *  - \c vx, \c vy, \c vz for velocity (m/s)
 *  - \c wx, \c wy, \c wz for angular velocity (rad/s)
 *  - \c ori for quaternion orientation
 */
typedef struct ALIGN(64)
{
	double x; 	///< position X
	double y; 	///< position Y
	double z; 	///< position Z
	double vx;  ///< velocity in X
	double vy;  ///< velocity in Y
	double vz;  ///< velocity in Z
	double wx;  ///< angular vel about local X
	double wy;  ///< angular vel about local Y
	double wz;  ///< angular vel about local Z

	Quat   ori; ///< Orientation in 3D, as a quaternion
} PState;

/**
 * @brief Derivatives used in 3D integration steps (RK4, etc.).
 *
 * Typically:
 * - \c dx, \c dy, \c dz = velocity,
 * - \c dvx, \c dvy, \c dvz = acceleration,
 * - \c dwx, \c dwy, \c dwz = angular acceleration,
 * - \c dOri = orientation derivative = 0.5 * (0, w) * q
 */
typedef struct ALIGN(64)
{
    double dx,  dy,  dz;
    double dvx, dvy, dvz;
    double dwx, dwy, dwz;

    Quat   dOri;
} StateDeriv3D;

/** @} */
// end of Doxygen Parsing group "ProjectileState"

#ifdef __cplusplus
}
#endif
#endif /* PSTATE_H */


