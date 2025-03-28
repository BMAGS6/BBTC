/**
 * @file 6dof.c
 * @brief 6 degrees-of-freedom (6 D.O.F.) ballistic solver implementation.
 *
 * This file implements an advanced torque-based 6-DOF solver, building on the existing 3D
 * ballistic framework. It uses an inertia tensor and aerodynamic coefficients to compute
 * rotational accelerations, optionally including Coriolis, Eötvös, Magnus force & torque,
 * and spin damping.
 *
 * For a complete overview of the physics modeled here, refer to @ref core_6dof.
 */
#include <stdio.h>
#include <math.h>
#include "../include/6dof.h"
#include "../include/quat.h"
#include "../include/compiler_macros.h"
#include "../include/load_drag_tables.h"
#include "../include/solver_options.h"  // for SOLVER_ENABLE_MAGNUS, CORIOLIS, etc.

#ifndef M_PI
  #define M_PI 3.14159265358979323846
#endif

#ifndef STANDARD_EARTH_ROTATION
  #define STANDARD_EARTH_ROTATION 7.292115e-5
#endif

#ifndef PI_TIMES_0_25
  #define PI_TIMES_0_25 (M_PI * 0.25)
#endif

/**
 * @brief Cross product: \f$ \mathbf{out} = \mathbf{a} \times \mathbf{b} \f$
 * @ingroup SixDOF_Solver
 *
 * @param a     Left-hand 3D vector
 * @param b     Right-hand 3D vector
 * @param out   Output vector (\f$ \mathbf{a} \times \mathbf{b} \f$)
 */
static inline void cross3(double a[3], double b[3], double out[3])
{
    out[0] = a[1]*b[2] - a[2]*b[1];
    out[1] = a[2]*b[0] - a[0]*b[2];
    out[2] = a[0]*b[1] - a[1]*b[0];
}

/**
 * @brief Clamps Magnus force to a safe maximum based on bullet weight.
 * @ingroup SixDOF_Solver
 *
 * @param x     Input force value to clamp
 * @param min   Minimum allowable value
 * @param max   Maximum allowable value
 * @return      Clamped value in the range [min, max]
 */
static inline double clamp_magnus(double x, double min, double max)
{
    return (x < min) ? min : (x > max) ? max : x;
}

/**
 * @brief Computes \f$ \dot{q} = \frac{1}{2} \mathbf{w}_q \cdot q \f$ for quaternion integration.
 * @ingroup QuaternionUtilities
 *
 * @param q     Input orientation quaternion
 * @param w     Angular velocity vector in radians per second
 * @return      Quaternion derivative \f$ \dot{q} \f$
 */
static inline Quat quat_derivatives(const Quat *q, const double w[3])
{
    Quat wq = {0.0, w[0], w[1], w[2]};
    Quat dq = quat_multiply(&wq, q);
    dq.w *= 0.5;
    dq.x *= 0.5;
    dq.y *= 0.5;
    dq.z *= 0.5;
    return dq;
}

/**
 * @brief Computes bullet's forward, up, and right unit vectors in world coordinates.
 *
 * Useful for determining spin-axis orientation, side forces, and yaw-of-repose effects.
 *
 * @ingroup SixDOF_Solver
 *
 * @param ori         Bullet orientation quaternion
 * @param forwardW    Output forward vector in world space
 * @param upW         Output up vector in world space
 * @param rightW      Output right vector in world space
 */
static inline void compute_bullet_axes_world(const Quat *ori,
                                             double forwardW[3],
                                             double upW[3],
                                             double rightW[3])
{
    static const double forwardLocal[3] = {1.0, 0.0, 0.0};
    static const double upLocal[3] = {0.0, 1.0, 0.0};

    quat_rotate_vector(ori, forwardLocal, forwardW);
    quat_rotate_vector(ori, upLocal, upW);

    double fMag = sqrt(forwardW[0]*forwardW[0] + forwardW[1]*forwardW[1] + forwardW[2]*forwardW[2]);
    double uMag = sqrt(upW[0]*upW[0] + upW[1]*upW[1] + upW[2]*upW[2]);

    if (fMag > 1e-8)
    {
        double invF = 1.0 / fMag;
        forwardW[0] *= invF;
        forwardW[1] *= invF;
        forwardW[2] *= invF;
    }

    if (uMag > 1e-8)
    {
        double invU = 1.0 / uMag;
        upW[0] *= invU;
        upW[1] *= invU;
        upW[2] *= invU;
    }

    rightW[0] = forwardW[1] * upW[2] - forwardW[2] * upW[1];
    rightW[1] = forwardW[2] * upW[0] - forwardW[0] * upW[2];
    rightW[2] = forwardW[0] * upW[1] - forwardW[1] * upW[0];

    double rMag = sqrt(rightW[0]*rightW[0] + rightW[1]*rightW[1] + rightW[2]*rightW[2]);
    if (rMag > 1e-8)
    {
        double invR = 1.0 / rMag;
        rightW[0] *= invR;
        rightW[1] *= invR;
        rightW[2] *= invR;
    }
}

/**
 * @brief Computes the full set of 6DOF derivatives for simulation.
 *
 * @param state Current physical state of the bullet.
 * @param env Environmental parameters.
 * @param options Solver control options.
 * @param deriv Output derivatives of the bullet's state.
 */
HOT void compute_6dof_derivatives(
    const PState        *state,
    const Environment   *env,
    const SixDOFSpecs   *specs,
    StateDeriv3D        *dState,
    const SolverOptions *options
)
{
    // Position derivatives = velocity
    dState->dx = state->vx;
    dState->dy = state->vy;
    dState->dz = state->vz;

	double v_sq = 0.0;
	double speed = 0.0;
	double mach = 0.0;
	double Cd = 0.0;

    // Orientation derivative from angular velocity
    {
        double wvec[3] = { state->wx, state->wy, state->wz };
        dState->dOri = quat_derivatives(&state->ori, wvec);
    }

    // Basic translational logic: drag, gravity
    double vrel_x = state->vx - env->windX;
    double vrel_y = state->vy - env->windY;
    double vrel_z = state->vz - env->windZ;

    v_sq = vrel_x*vrel_x + vrel_y*vrel_y + vrel_z*vrel_z;
    double ax=0.0, ay=0.0, az=0.0;

    if (v_sq < 1e-8)
    {
        // very low speed => only gravity
        ax = 0.0;
        ay = -GRAVITY;
        az = 0.0;
    }
    else
    {
        speed  = sqrt(v_sq);
        mach   = speed / env->localSpdOfSnd;
        Cd     = fast_interpolate_cd(lookupTable, mach);

        double dragFactor = 0.5 * env->localDensity * v_sq * Cd * (specs->areaOverMass);
        double inv_speed = 1.0 / speed;

        double drag_x = -dragFactor * (vrel_x*inv_speed);
        double drag_y = -dragFactor * (vrel_y*inv_speed);
        double drag_z = -dragFactor * (vrel_z*inv_speed);

        ax = drag_x;
        ay = drag_y - GRAVITY;
        az = drag_z;
    }

    // Coriolis effect
    if (options->effects & SOLVER_ENABLE_CORIOLIS)
    {
        double Omega[3] = {0.0, STANDARD_EARTH_ROTATION, 0.0};
        double crossCor[3] = {0.0, 0.0, 0.0};
        double v[3] = { state->vx, state->vy, state->vz };
        cross3(Omega, v, crossCor);

        ax += -2.0 * crossCor[0];
        ay += -2.0 * crossCor[1];
        az += -2.0 * crossCor[2];
    }

    // Eötvös effect
    if (options->effects & SOLVER_ENABLE_EOTVOS)
    {
        double latRad = (M_PI/180.0) * env->latitude;
        double cosLat = cos(latRad);
        double eotvos = 2.0 * STANDARD_EARTH_ROTATION * state->vx * cosLat;
        ay += eotvos;
    }


    // Magnus Effect Computation
    double F_magnus[3] = {0.0,0.0,0.0};
    if (options->effects & SOLVER_ENABLE_MAGNUS)
    {
        if (v_sq > 1e-8)
        {
            speed = sqrt(v_sq);
            double spin[3]  = { state->wx, state->wy, state->wz };
            double vrel[3]  = { vrel_x, vrel_y, vrel_z };
            double crossM[3] = {0.0, 0.0, 0.0};
            cross3(spin, vrel, crossM);  // compute spin x velocity

            double spinRate = sqrt(state->wx*state->wx + state->wy*state->wy + state->wz*state->wz);

            // Compute spin ratio and dynamically compute Cl with a lower multiplier and cap:
            double spinRatio = (spinRate * specs->diam) / (2.0 * speed);
            double Cl = 0.00003 * spinRatio;								// DO NOT CHANGE; This number was fine-tuned to 0.000005, and magnus force will become less accurate if this number changes
            if (Cl > 0.08)
                Cl = 0.08;
            // Compute force magnitude factor (without direction)
            double factor = 0.5 * env->localDensity * specs->area * Cl;

            // Normalize the cross product so that its magnitude doesn't affect the force magnitude
            double crossM_mag = sqrt(crossM[0]*crossM[0] + crossM[1]*crossM[1] + crossM[2]*crossM[2]);
            if (crossM_mag > 1e-8)
            {
                crossM[0] /= crossM_mag;
                crossM[1] /= crossM_mag;
                crossM[2] /= crossM_mag;
            }
            else
            {
                crossM[0] = crossM[1] = crossM[2] = 0.0;
            }

            // Compute the force components using the normalized direction:
            double Fx = factor * speed * speed * crossM[0];
            double Fy = factor * speed * speed * crossM[1];
            double Fz = factor * speed * speed * crossM[2];

            // Clamp the Magnus force so that its magnitude does not exceed a certain multiple of the bullet's weight.
            // Here we cap it at 3*g (three times gravity) times the bullet mass.
            double maxMagnusForce = 3.0 * specs->mass * GRAVITY;  // 3g limit
            Fx = clamp_magnus(Fx, -maxMagnusForce, maxMagnusForce);
            Fy = clamp_magnus(Fy, -maxMagnusForce, maxMagnusForce);
            Fz = clamp_magnus(Fz, -maxMagnusForce, maxMagnusForce);

            // Add the Magnus force contribution to net acceleration:
            ax += Fx / specs->mass;
            ay += Fy / specs->mass;
            az += Fz / specs->mass;

            // Store for torque calculations (if needed)
            F_magnus[0] = Fx;
            F_magnus[1] = Fy;
            F_magnus[2] = Fz;

            // Debugging output:
            printf("Magnus Force: (%.9lf, %.9lf, %.9lf) Spin=%.9lf rpm\n",
                   Fx, Fy, Fz, spinRate*(60.0/(2.0*M_PI)));
        }
        else
        {
            F_magnus[0] = 0.0;
            F_magnus[1] = 0.0;
            F_magnus[2] = 0.0;
        }
    }/* End of Magnus Calculations section */


	double
		forwardW[3],
		upW[3],
		rightW[3];
	compute_bullet_axes_world(&state->ori, forwardW, upW, rightW);

	if (options->effects & SOLVER_ENABLE_YAW_REPOSE)
	{
		// Spin rate
		double
			wx = state->wx,
			wy = state->wy,
			wz = state->wz;
		double spinRate = sqrt(wx*wx + wy*wy + wz*wz);

		if (spinRate > 1e-8)	// if bullet is still spinning
		{
			// Example side-force magnitude:
			double speed = sqrt(v_sq);
			double F_side = specs->aero.CyawRepose * spinRate * speed * specs->mass;

			if (spinRate > 104719.755114) spinRate = 104719.755114;		// Clamp spin rate at 104,719.755114 rad/s (1,000,000 rpm)

			// Direction is bulletRight axis:
			//   sign would also depend on bullet’s spin direction, etc.
			double Fx_side = F_side * rightW[0];
			double Fy_side = F_side * rightW[1];
			double Fz_side = F_side * rightW[2];

			ax += Fx_side / specs->mass;
			ay += Fy_side / specs->mass;
			az += Fz_side / specs->mass;
		}
	}

    // Store final translational derivatives
    dState->dvx = ax;
    dState->dvy = ay;
    dState->dvz = az;

    // net torque accumulation
    double torque[3] = {0.0,0.0,0.0};

    if (options->effects & SOLVER_ENABLE_BULLET_TILT)
    {
		double tiltFactor = specs->aero.Ctilt;

		double spinRate = sqrt(state->wx*state->wx + state->wy*state->wy + state->wz*state->wz);

		if (spinRate > 1e-8)
		{
			double torqueTilt[3];

			if (spinRate > 10000.0) spinRate = 10000.0;		// Clamp spin rate at 10,000 rad/s

			torqueTilt[0] = -tiltFactor * state->wx * 0.1;
			torqueTilt[1] = -tiltFactor * state->wy * 0.1;
			torqueTilt[2] = -tiltFactor * state->wz * 0.1;

			torque[0] += torqueTilt[0];
			torque[1] += torqueTilt[1];
			torque[2] += torqueTilt[2];
		}
    }

    // Spin damping: -CspinDamp * speed * omega
    {
        double speedApprox = (v_sq < 1e-8) ? 0.0 : sqrt(v_sq);
        double Cspin = specs->aero.CspinDamp;
        torque[0] += -Cspin * speedApprox * state->wx;
        torque[1] += -Cspin * speedApprox * state->wy;
        torque[2] += -Cspin * speedApprox * state->wz;
    }

    // (B) if there's a nonzero aerodynamic center offset, produce torque from F_magnus
    {
        double rAC2 = specs->rAC_local[0]*specs->rAC_local[0]
                    + specs->rAC_local[1]*specs->rAC_local[1]
                    + specs->rAC_local[2]*specs->rAC_local[2];

        // only if user sets a non-zero offset and F_magnus is non-zero
        if ((rAC2 > 1e-8) && ( (F_magnus[0]!=0)||(F_magnus[1]!=0)||(F_magnus[2]!=0) ))
        {
            double r_ac_world[3];
            quat_rotate_vector(&state->ori, specs->rAC_local, r_ac_world);

            double torqueMagnus[3];
            cross3(r_ac_world, F_magnus, torqueMagnus);

            torque[0] += torqueMagnus[0];
            torque[1] += torqueMagnus[1];
            torque[2] += torqueMagnus[2];
        }
    }

    // Gyroscopic term: \omega x (I \omega)
    double wx = state->wx;
    double wy = state->wy;
    double wz = state->wz;

    double Ixx = specs->inertia.Ixx;
    double Iyy = specs->inertia.Iyy;
    double Izz = specs->inertia.Izz;

    double Iomega[3] = { Ixx*wx, Iyy*wy, Izz*wz };
    double crossGyro[3];
    cross3((double[3]){wx, wy, wz}, Iomega, crossGyro);

    double netTorque[3];
    netTorque[0] = torque[0] - crossGyro[0];
    netTorque[1] = torque[1] - crossGyro[1];
    netTorque[2] = torque[2] - crossGyro[2];


    // \dot{\omega} = I^{-1} netTorque
    double invIxx = 1.0 / Ixx;
    double invIyy = 1.0 / Iyy;
    double invIzz = 1.0 / Izz;

    dState->dwx = invIxx * netTorque[0];
    dState->dwy = invIyy * netTorque[1];
    dState->dwz = invIzz * netTorque[2];
}

/**
 * @brief Performs a single 6-DOF RK4 integration step on the projectile state.
 *
 * @param state     Mutable pointer to projectile state (position, velocity, orientation, etc.)
 * @param dt        Time step in seconds
 * @param env       Pointer to environmental conditions
 * @param specs     Pointer to 6DOF specifications (e.g. bullet parameters)
 * @param options   Pointer to simulation solver options
 */
HOT void rk4_step_6dof(
    PState              *state,
    double               dt,
    const Environment    *env,
    const SixDOFSpecs    *specs,
    const SolverOptions  *options
)
{
    StateDeriv3D k1, k2, k3, k4;
    PState midState;

    // --- K1
    compute_6dof_derivatives(state, env, specs, &k1, options);

    midState.x  = state->x  + 0.5 * dt * k1.dx;
    midState.y  = state->y  + 0.5 * dt * k1.dy;
    midState.z  = state->z  + 0.5 * dt * k1.dz;
    midState.vx = state->vx + 0.5 * dt * k1.dvx;
    midState.vy = state->vy + 0.5 * dt * k1.dvy;
    midState.vz = state->vz + 0.5 * dt * k1.dvz;

    midState.ori = quat_add_scaled(&state->ori, &k1.dOri, 0.5 * dt);
    quat_normalize(&midState.ori);

    midState.wx = state->wx + 0.5 * dt * k1.dwx;
    midState.wy = state->wy + 0.5 * dt * k1.dwy;
    midState.wz = state->wz + 0.5 * dt * k1.dwz;

    // --- K2
    compute_6dof_derivatives(&midState, env, specs, &k2, options);

    midState.x  = state->x  + 0.5 * dt * k2.dx;
    midState.y  = state->y  + 0.5 * dt * k2.dy;
    midState.z  = state->z  + 0.5 * dt * k2.dz;
    midState.vx = state->vx + 0.5 * dt * k2.dvx;
    midState.vy = state->vy + 0.5 * dt * k2.dvy;
    midState.vz = state->vz + 0.5 * dt * k2.dvz;

    midState.ori = quat_add_scaled(&state->ori, &k2.dOri, 0.5 * dt);
    quat_normalize(&midState.ori);

    midState.wx = state->wx + 0.5 * dt * k2.dwx;
    midState.wy = state->wy + 0.5 * dt * k2.dwy;
    midState.wz = state->wz + 0.5 * dt * k2.dwz;

    // --- K3
    compute_6dof_derivatives(&midState, env, specs, &k3, options);

    midState.x  = state->x  + 0.5 * dt * k3.dx;
    midState.y  = state->y  + 0.5 * dt * k3.dy;
    midState.z  = state->z  + 0.5 * dt * k3.dz;
    midState.vx = state->vx + 0.5 * dt * k3.dvx;
    midState.vy = state->vy + 0.5 * dt * k3.dvy;
    midState.vz = state->vz + 0.5 * dt * k3.dvz;

    midState.ori = quat_add_scaled(&state->ori, &k3.dOri, 0.5 * dt);
    quat_normalize(&midState.ori);

    midState.wx = state->wx + 0.5 * dt * k3.dwx;
    midState.wy = state->wy + 0.5 * dt * k3.dwy;
    midState.wz = state->wz + 0.5 * dt * k3.dwz;

    // --- K4
    compute_6dof_derivatives(&midState, env, specs, &k4, options);

    // Combine results
    state->x  += (dt/6.0)*(k1.dx  + 2.0*k2.dx  + 2.0*k3.dx  + k4.dx);
    state->y  += (dt/6.0)*(k1.dy  + 2.0*k2.dy  + 2.0*k3.dy  + k4.dy);
    state->z  += (dt/6.0)*(k1.dz  + 2.0*k2.dz  + 2.0*k3.dz  + k4.dz);

    state->vx += (dt/6.0)*(k1.dvx + 2.0*k2.dvx + 2.0*k3.dvx + k4.dvx);
    state->vy += (dt/6.0)*(k1.dvy + 2.0*k2.dvy + 2.0*k3.dvy + k4.dvy);
    state->vz += (dt/6.0)*(k1.dvz + 2.0*k2.dvz + 2.0*k3.dvz + k4.dvz);

    // Orientation aggregator
    {
        double scale = dt / 6.0;
        Quat tmpQ;
        tmpQ.w = state->ori.w + scale*(k1.dOri.w + 2.0*k2.dOri.w + 2.0*k3.dOri.w + k4.dOri.w);
        tmpQ.x = state->ori.x + scale*(k1.dOri.x + 2.0*k2.dOri.x + 2.0*k3.dOri.x + k4.dOri.x);
        tmpQ.y = state->ori.y + scale*(k1.dOri.y + 2.0*k2.dOri.y + 2.0*k3.dOri.y + k4.dOri.y);
        tmpQ.z = state->ori.z + scale*(k1.dOri.z + 2.0*k2.dOri.z + 2.0*k3.dOri.z + k4.dOri.z);

        state->ori = tmpQ;
		quat_normalize(&state->ori);
    }

    state->wx += (dt/6.0)*(k1.dwx + 2.0*k2.dwx + 2.0*k3.dwx + k4.dwx);
    state->wy += (dt/6.0)*(k1.dwy + 2.0*k2.dwy + 2.0*k3.dwy + k4.dwy);
    state->wz += (dt/6.0)*(k1.dwz + 2.0*k2.dwz + 2.0*k3.dwz + k4.dwz);
}