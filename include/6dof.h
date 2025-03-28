/**
 * @file 6dof.h
 * @brief 6 degrees-of-freedom (6-DOF) ballistic solver interface and data structures.
 *
 * @details
 * This header defines the data structures and function prototypes for simulating a full six-degrees-of-freedom projectile.
 * It models translation (3 DOF) and rotation (3 DOF) using quaternion-based orientation, angular velocities,
 * and an inertia tensor. Supports Magnus force, Coriolis effect, spin damping, yaw of repose, and other aerodynamic torques.
 *
 *	See @ref core_6dof for how these operations are used in the simulator.
 */

#ifndef SIX_DOF_H
#define SIX_DOF_H

#ifdef __cplusplus
	extern "C" {
#endif

#include <stddef.h>
#include "pstate.h"
#include "environment.h"
#include "compiler_macros.h"
#include "solver_options.h"

/**
 * @defgroup SixDOF_Solver 6-DOF Solver
 * @ingroup AllFiles
 * @{
 */
 
// fallback macros
#ifndef STANDARD_EARTH_ROTATION
	#define STANDARD_EARTH_ROTATION 7.292115e-5
#endif
#ifndef M_PI
	#define M_PI 3.14159265358979323846
#endif
#ifndef PI_OVER_180
	#define PI_OVER_180 (M_PI / 180.0)
#endif

/**
 * @brief Moments of inertia for the projectile in principal axes.
 *
 * Assumes diagonal inertia tensor:
 *
 * - \f$ I = \mathrm{diag}(I_{xx}, I_{yy}, I_{zz}) \f$
 */
typedef struct ALIGN(64)
{
    double Ixx; // Inertia about local X axis
    double Iyy; // Inertia about local Y axis
    double Izz; // Inertia about local Z axis (spin axis if axisymmetric)
} Inertia;

/**
 * @brief Aerodynamic coefficients for advanced torque/force modeling.
 *
 * Includes empirical coefficients for effects like Magnus, spin damping, and more.
 */
typedef struct ALIGN(64)
{
    double Cm;         ///< Magnus coefficient (lift from spin)
    double CspinDamp;  ///< Damping term proportional to spin rate
    double CyawRepose; ///< Drift force due to yaw-of-repose
    double Ctilt;      ///< Torque from bullet tilt during flight
} AeroCoeffs;

/**
 * @brief Extended projectile specifications for 6-DOF.
 *
 * This structure defines the bullet's physical properties needed for accurate 6DOF simulation.
 */
typedef struct ALIGN(64)
{
	double diam;         ///< Bullet diameter (caliber) in meters
	double area;         ///< Cross-sectional area in m^2
	double areaOverMass; ///< Precomputed area / mass ratio
    Inertia inertia;     ///< Diagonal inertia tensor
    AeroCoeffs aero;     ///< Aerodynamic coefficients
    double mass;         ///< Mass in kilograms
    double rAC_local[3]; ///< Aerodynamic center offset vector in local bullet coordinates
} SixDOFSpecs;

/**
 * @brief Computes 6-DOF ballistic derivatives with optional advanced effects.
 *
 * This function evaluates the time derivatives of:
 * - Position: \( \dot{x}, \dot{y}, \dot{z} \)
 * - Velocity: \( \dot{v}_x, \dot{v}_y, \dot{v}_z \)
 * - Orientation: \( \dot{q} = 0.5 (0, \vec{\omega}) q \)
 * - Angular velocity: \( \dot{\vec{\omega}} = I^{-1}(\vec{\tau} - \vec{\omega} \times I\vec{\omega}) \)
 *
 * Effects included depending on solver flags:
 * - **Magnus force:** \( \vec{F}_M \propto \vec{\omega} \times \vec{v} \)
 * - **Coriolis effect:** \( \vec{a}_C = -2 \vec{\Omega} \times \vec{v} \)
 * - **Eötvös effect:** alters vertical acceleration based on eastward velocity
 *
 * @anchor func_compute_6dof_derivatives
 *
 * @param state   Current projectile state
 * @param env     Environmental parameters (wind, density, etc.)
 * @param specs   Projectile 6DOF parameters (mass, inertia, aero)
 * @param dState  Output state derivatives
 * @param options Solver feature flags
 */
void compute_6dof_derivatives(
    const PState        *state,
    const Environment   *env,
    const SixDOFSpecs   *specs,
    StateDeriv3D        *dState,
    const SolverOptions *options
);

/**
 * @brief Performs one RK4 (Runge-Kutta 4th Order) integration step on a projectile.
 *
 * RK4 approximates the next state of the system \( y(t + h) \) using weighted slopes:
 * \f[
 * y(t+h) = y(t) + \frac{h}{6}(k_1 + 2k_2 + 2k_3 + k_4)
 * \f]
 *
 * Each \( k_i \) represents a derivative estimate:
 * - \( k_1 = f(t, y) \)
 * - \( k_2 = f(t + h/2, y + h/2 k_1) \)
 * - \( k_3 = f(t + h/2, y + h/2 k_2) \)
 * - \( k_4 = f(t + h, y + h k_3) \)
 *
 * This method is preferred for its stability and accuracy in simulating real-time 6DOF dynamics.
 *
 * @param state   Input/output projectile state
 * @param dt      Time step in seconds
 * @param env     Current environmental state
 * @param specs   Projectile characteristics
 * @param options Solver toggles (Magnus, Coriolis, etc.)
 */
HOT void rk4_step_6dof(
    PState              *state,
    double               dt,
    const Environment    *env,
    const SixDOFSpecs    *specs,
    const SolverOptions  *options
);

/** @} */
// end of Doxygen Parsing group "SixDOF_Solver"

#ifdef __cplusplus
}
#endif

#endif /* SIX_DOF_H */

