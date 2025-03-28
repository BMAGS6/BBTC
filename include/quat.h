/**
 * @file quat.h
 * @brief Declarations for quaternion-based rotation utilities.
 */

#ifndef QUAT_H
#define QUAT_H

#ifdef __cplusplus
	extern "C"{
#endif

#include "compiler_macros.h"

/**
 * @defgroup QuaternionUtilities Quaternion Utilities
 * @{
 */
 
/**
 * @brief A quaternion representing orientation in 3D, with fields w, x, y, z.
 *
 * Typically normalized so that \f$ w^2 + x^2 + y^2 + z^2 = 1 \f$.
 */
typedef struct ALIGN(64)
{
	double w; 	///< Real part
	double x; 	///< i component
	double y; 	///< j component
	double z; 	///< k component
} Quat;

/**
 * @brief Normalize the quaternion in-place.
 *
 * Ensures \f$q.w^2 + q.x^2 + q.y^2 + q.z^2 = 1\f$.
 * @param q Pointer to the quaternion.
 */
void quat_normalize(Quat *q);

/**
 * @brief Multiply two quaternions: result = q1 * q2
 * @param q1 First quaternion
 * @param q2 Second quaternion
 * @return The product quaternion
 */
Quat quat_multiply(const Quat *q1, const Quat *q2);

/**
 * @brief Conjugate of a quaternion: \f$q^* = (w, -x, -y, -z)\f$.
 * @param q The input quaternion
 * @return The conjugate
 */
Quat quat_conjugate(const Quat *q);

/**
 * @brief Rotates a 3D vector using the quaternion \p q.
 *
 * @param q     Rotation quaternion
 * @param vect  The input vector (x,y,z)
 * @param outVect The rotated vector (x,y,z)
 *
 * This uses an optimized formula for quaternion * vect multiplication:
 * \f[
 *   \mathbf{v}_{out} = \mathbf{v} + 2.0 \Bigl( q_w (\mathbf{q}_v \times \mathbf{v}) 
 *         + \mathbf{q}_v \times ( \mathbf{q}_v \times \mathbf{v} ) \Bigr).
 * \f]
 */
void quat_rotate_vector(const Quat *q, const double vect[3], double outVect[3]);

/**
 * @brief Create a pure rotation quaternion from an axis (x,y,z) and an angle (radians).
 * @param axis_x X component of axis
 * @param axis_y Y component of axis
 * @param axis_z Z component of axis
 * @param angleRad Rotation angle in radians
 * @return The resulting quaternion
 */
Quat quat_from_axis_angle(double axis_x, double axis_y, double axis_z, double angleRad);

/**
 * @brief Update orientation by angular velocity w (in rad/s) over time dt.
 *
 * This uses dq/dt = 0.5 * (0, w) * q, then normalizes q.
 * @param q Pointer to the quaternion
 * @param w The angular velocity (3D)
 * @param dt The small time step
 */
void quat_integrate_angular_velocity(Quat *q, const double w[3], double dt);

/**
 * @brief Add a scaled quaternion dq to a base quaternion q: result = q + scale*dq
 * @param q Base quaternion
 * @param dq Delta quaternion
 * @param scale Scale factor
 * @return The resulting quaternion
 *
 * This effectively does:
 * \f$ q \leftarrow q + 0.5\, (0, \omega)\, q \, dt,\f$
 * then normalizes the quaternion to unit length.
 *
 * Typically used in RK4 steps, e.g.:
 * @code
 * midState.ori = quat_add_scaled(&state->ori, &k1.dOri, 0.5 * dt);
 * @endcode
 */
Quat quat_add_scaled(const Quat *q, const Quat *dq, double scale);

/** @} */
// end of group "QuaternionUtilities"

#ifdef __cplusplus
}
#endif

#endif /* QUAT_H */


