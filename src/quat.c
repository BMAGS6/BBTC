/**
 * @file quat.c
 * @brief Implements quaternion-based rotation utilities.
 */

#include <math.h>
#include "../include/quat.h"
#include "../include/pstate.h"
#include "../include/environment.h"
#include "../include/load_drag_tables.h"

/**
 * @ingroup QuaternionUtilities
 * @{
 */

/**
 * @brief Normalizes a quaternion to unit length.
 *
 * @param q Quaternion to normalize (modified in place).
 */
void quat_normalize(Quat *restrict q)
{
    double mag2 = q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z;
    if (mag2 > 1e-14)
    {
        double invMag = 1.0 / sqrt(mag2);
        q->w *= invMag;
        q->x *= invMag;
        q->y *= invMag;
        q->z *= invMag;
    }
}

/**
 * @brief Multiplies two quaternions.
 *
 * This performs Hamilton product: \f$ q = q_1 \cdot q_2 \f$
 *
 * @param a First quaternion (left-hand operand).
 * @param b Second quaternion (right-hand operand).
 * @return Result of quaternion multiplication.
 */
Quat quat_multiply(const Quat *q1, const Quat *q2)
{
    Quat result;
    result.w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
    result.x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
    result.y = q1->w * q2->y + q1->y * q2->w + q1->z * q2->x - q1->x * q2->z;
    result.z = q1->w * q2->z + q1->z * q2->w + q1->x * q2->y - q1->y * q2->x;
    return result;
}

/**
 * @brief Computes the conjugate of a quaternion.
 *
 * @param q Input quaternion.
 * @return Quaternion conjugate \f$ q^* \f$ such that \f$ q^* = (w, -x, -y, -z) \f$
 */
Quat quat_conjugate(const Quat *restrict q)
{
    Quat conj;
    conj.w =  q->w;
    conj.x = -q->x;
    conj.y = -q->y;
    conj.z = -q->z;
    return conj;
}

/**
 * @brief Rotates a 3D vector using the quaternion \p q.
 */
void quat_rotate_vector(const Quat *restrict q, const double vect[3], double outVect[3])
{
	double qw = q->w, qx = q->x, qy = q->y, qz = q->z;
	double vx = vect[0], vy = vect[1], vz = vect[2];

	// cross1 = qv x v
	double cross1_x = qy * vz - qz * vy;
	double cross1_y = qz * vx - qx * vz;
	double cross1_z = qx * vy - qy * vx;

	// cross2 = qv x cross1
	double cross2_x = qy * cross1_z - qz * cross1_y;
	double cross2_y = qz * cross1_x - qx * cross1_z;
	double cross2_z = qx * cross1_y - qy * cross1_x;

	outVect[0] = vx + 2.0 * (qw * cross1_x + cross2_x);
	outVect[1] = vy + 2.0 * (qw * cross1_y + cross2_y);
	outVect[2] = vz + 2.0 * (qw * cross1_z + cross2_z);
}

/**
 * @brief Creates a quaternion from axis-angle representation.
 *
 * @param axis Rotation axis (unit vector).
 * @param angle Rotation angle in radians.
 * @return Quaternion representing the rotation.
 */
Quat quat_from_axis_angle(double axis_x, double axis_y, double axis_z, double angleRad)
{
    Quat q;
    double halfAngle = angleRad * 0.5;
    double s = sin(halfAngle);

    // Normalize axis
    double mag = sqrt(axis_x*axis_x + axis_y*axis_y + axis_z*axis_z);
    if (mag < 1e-15)
    {
        q.w = 1.0; q.x = 0.0; q.y = 0.0; q.z = 0.0;
        return q;
    }
    double invMag = 1.0 / mag;
    axis_x *= invMag;
    axis_y *= invMag;
    axis_z *= invMag;

    q.w = cos(halfAngle);
    q.x = axis_x * s;
    q.y = axis_y * s;
    q.z = axis_z * s;
    return q;
}

/**
 * @brief Adds \p dq scaled by \p scale to quaternion \p q.
 */
Quat quat_add_scaled(const Quat *q, const Quat *dq, double scale)
{
    Quat result;
    result.w = q->w + dq->w * scale;
    result.x = q->x + dq->x * scale;
    result.y = q->y + dq->y * scale;
    result.z = q->z + dq->z * scale;
    return result;
}

/**
 * @brief Integrates quaternion orientation by angular velocity over time dt.
 */
void quat_integrate_angular_velocity(Quat *restrict q, const double w[3], double dt)
{
    Quat wq = {0.0, w[0], w[1], w[2]};
    Quat dq = quat_multiply(&wq, q);

    dq.w *= 0.5 * dt;
    dq.x *= 0.5 * dt;
    dq.y *= 0.5 * dt;
    dq.z *= 0.5 * dt;

    q->w += dq.w;
    q->x += dq.x;
    q->y += dq.y;
    q->z += dq.z;

    quat_normalize(q);
}

/** @} */
// end of QuaternionUtilities
