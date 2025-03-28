/**
 *	@file main.c
 *	@brief Prototype interface for the 6-DOF ballistic solver, now with actual drag table usage.
 *
 * This file:
 * - Loads a CSV drag table into a global array,
 * - Precomputes the finer-resolution lookup table,
 * - Initializes the environment (weather, altitude, muzzle conditions),
 * - Initializes projectile specs (mass, inertia, aero, etc.),
 * - Runs the 6-DOF solver in a loop until the projectile hits the ground or times out.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>  			// For memset()
#include <getopt.h> 			// For user input on the command line
#include <locale.h>
#include "../include/6dof.h"
#include "../include/solver_options.h"
#include "../include/environment.h"
#include "../include/pstate.h"
#include "../include/quat.h"
#include "../include/load_drag_tables.h"
#include "../include/cli_opts.h"

/**
 *	@defgroup MainProgram Main Program
 *	@{
 */

/**
 *	@brief A macro for controlling the main simulation loop logic. Pure syntactic sugar.
 */
#define BULLET_IS_AIRBORNE 1

#ifndef M_PI
  #define M_PI 3.14159265358979323846	///< Pi calculated to 20 decimal places
#endif

#ifndef PI_OVER_180
  #define PI_OVER_180 (M_PI / 180.0)  ///< Helper constant for converting degrees to radians
#endif

#ifndef SEA_LVL
  #define SEA_LVL 0.0                ///< Value for sea level in meters
#endif

#define MAX_TIME  60.0  			///< 1 minute simulation time-limit
#define GROUND_LEVEL 0.0  			///< y=0 means ground

/**
 *	@brief Returns a string name corresponding to the user's selected drag model.
 *	@param model The chosen DragModel enum
 *	@return A readable string such as "G1", "G7", etc.
 */
static const char* get_drag_model_string(DragModel model)
{
	switch (model)
	{
		case G1:	return "G1";
		case G7:	return "G7";
		case G2:	return "G2";
		case G5:	return "G5";
		case G6:	return "G6";
		case G8:	return "G8";
		case GL:	return "GL";
		case GS:	return "GS";
		case GI:	return "GI";
		case RA4:	return "RA4";
		default:	return "G1";	// fallback
	}
}

/**
 *	@brief The main entry point for the 6-DOF ballistic solver.
 *
 *	@param[in] argc The number of command-line arguments
 *	@param[in] argv The array of command-line arguments
 *	@return 0 on success, non-zero if error or user requested help
 */
int main(int argc, char *argv[])
{
	// Initialize CLI inputs
	CLI_inputs cli;
	initialize_CLI_inputs(&cli);

	// Parse CLI Arguments
	if (parse_options(&cli, argc, argv) != 0)
		return 1; 												// Early exit on help or error
	fprintf(stderr, "LOG: Inputs parsed and initialized.\n");

	// Drag Table Selection
	DragModel userDragModel  = cli.dragModelChoice;
	const char *dragTableFile = get_drag_model_file(userDragModel);
	fprintf(stderr, "Drag File Name: %s\n", dragTableFile);

	// Load drag CSV
	size_t entriesLoaded = load_drag_table_CSV(dragTableFile);
	if (entriesLoaded <= 0)
	{
		fprintf(stderr, "ERROR: No drag data could be loaded. Exiting.\n");
		return 1;
	}
	// Precompute finer resolution array
	precompute_drag_lookup_table(lookupTable);

	const char* dragModelString = get_drag_model_string(userDragModel);
	SolverOptions options = cli.options;

	// Initialize with user + default values
	double muzzleSpeed   = cli.muzzleSpeed;
	double muzzleAngleDeg= cli.muzzleAngleDeg;
	double muzzleAngleRad= 0.0;
	double t             = 0.0; 						// Simulation time
	double peakHeight    = 0.0; 						// Track max altitude
	double azimuthDeg    = cli.azimuthDeg;
	double azimuthRad    = azimuthDeg * (PI_OVER_180);
	double twistInches   = cli.twistInches;
	double twistMeters   = twistInches * 0.0254;
	double rotPerMeter   = 1.0 / twistMeters;
	double spin_rot_s    = muzzleSpeed * rotPerMeter;   // Spin rate in rotations/sec
	double spin_rad_s    = spin_rot_s * 2.0 * M_PI;     // Convert to rad/s
	double timeStep      = cli.timeStep;

	// Projectile Specs
	SixDOFSpecs specs;
	memset(&specs, 0, sizeof(SixDOFSpecs));
	specs.mass = cli.mass;
	specs.aero.Cm       = 0.05;
	specs.aero.CspinDamp= 1e-9;
	specs.inertia.Ixx   = 1e-5;
	specs.inertia.Iyy   = 1e-5;
	specs.inertia.Izz   = 2e-6;
	specs.diam          = cli.diam;
	specs.rAC_local[0]  = 0.0;
	specs.rAC_local[1]  = 0.0001;
	specs.rAC_local[2]  = 0.0;

	// Environment
	Environment env;
	memset(&env, 0, sizeof(Environment));
	env.muzzleTempK   = 288.15;
	env.muzzlePressure= 101325.0;
	env.muzzleAlt     = cli.altitude;
	env.latitude      = cli.latitude;
	env.relHumidity   = 0.5;
	env.windX         = 0.0;
	env.windY         = 0.0;
	env.windZ         = 0.0;
	env.groundLvl     = SEA_LVL;
	env.localDensity  = 1.225;
	env.localSpdOfSnd = 343.0;

	// Projectile State
	PState state;
	memset(&state, 0, sizeof(PState));
	state.ori.w = 1.0; 						// Identity quaternion
	state.ori.x = 0.0001;
	state.ori.y = 0.0;
	state.ori.z = 0.0;
	state.wx    = spin_rad_s;
	state.wy    = 0.0;
	state.wz    = 0.0;

	// Recalculate angles
	muzzleAngleRad = muzzleAngleDeg * (PI_OVER_180);
	azimuthRad     = azimuthDeg     * (PI_OVER_180);

	// Recalculate spin
	twistMeters    = twistInches * 0.0254;
	rotPerMeter    = 1.0 / twistMeters;
	spin_rot_s     = muzzleSpeed * rotPerMeter;
	spin_rad_s     = spin_rot_s * 2.0 * M_PI;
	state.wx       = spin_rad_s;

	// Define bullet area, etc.
	specs.area         = M_PI * 0.25 * (specs.diam * specs.diam);
	specs.areaOverMass = specs.area / specs.mass;

	// Initial Position & velocity
	state.x  = 0.0;
	state.y  = env.muzzleAlt;
	state.z  = 0.0;
	state.vx = muzzleSpeed * cos(muzzleAngleRad) * cos(azimuthRad);
	state.vy = muzzleSpeed * sin(muzzleAngleRad);
	state.vz = muzzleSpeed * cos(muzzleAngleRad) * sin(azimuthRad);

	// Print solver toggles
	puts("=== Solver Toggles ===");
	if (options.effects == SOLVER_ENABLE_NONE)
		puts("No advanced effects enabled.");
	else
	{
		if (options.effects & SOLVER_ENABLE_MAGNUS)   puts("Magnus enabled");
		if (options.effects & SOLVER_ENABLE_CORIOLIS) puts("Coriolis enabled");
		if (options.effects & SOLVER_ENABLE_EOTVOS)   puts("Eotvos enabled");
		if (options.effects & SOLVER_ENABLE_YAW_REPOSE) puts("Yaw-of-Repose enabled");
		if (options.effects & SOLVER_ENABLE_BULLET_TILT) puts("Bullet Tilt enabled");
	}
	puts(" ");

	// Set certain advanced bullet factors
	specs.aero.CyawRepose = 3e-6;
	specs.aero.Ctilt      = 2e-6;

	// Prepare .csv log
	FILE *csv_log_file = fopen("trajectory_log.csv", "w");
	if (!csv_log_file)
	{
		perror("ERROR: Could not open trajectory_log.csv. Exiting.");
		return 1;
	}
	// CSV headers
	fprintf(csv_log_file,
	        "TIME (s),East Distance (m),Distance Above Sea Level (m),North Distance (m),"
	        "Velocity East (m/s),Velocity Up (m/s),Velocity North (m/s),Spin Rate (rpm),"
	        "Air Density (kg/m^3),Air Temperature (K)\n");

	update_environment(&env, &state);

	// Main Simulation Loop
	while (BULLET_IS_AIRBORNE)
	{
		// track peak altitude
		if (state.y > peakHeight)
			peakHeight = state.y;

		// environment update
		update_environment(&env, &state);

		// log
		double spinRateRPM = (sqrt(state.wx*state.wx + state.wy*state.wy + state.wz*state.wz)) * (60 / (2*M_PI));
		fprintf(csv_log_file, "%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf\n",
		    t, state.x, state.y, state.z,
		    state.vx, state.vy, state.vz,
		    spinRateRPM,
		    env.localDensity, env.localTempK);

		// stop condition
		if (state.y < env.groundLvl)
		{
			printf("Projectile hit the ground after %.4lf seconds.\n", t);
			break;
		}
		if (t > MAX_TIME)
		{
			printf("Simulation timed-out after %.1f seconds of simulation time.\n", MAX_TIME);
			break;
		}

		rk4_step_6dof(&state, timeStep, &env, &specs, &options);
		t += timeStep;
	}

	double groundDistance = sqrt(state.x*state.x + state.y*state.y + state.z*state.z);


	// Print Final Results:
	puts
	(
		"\n================================\n"
		 "| + + +  FINAL RESULTS  + + +  |\n"
		 "================================\n"
		 "|    Input Parameters:         |\n"
		 "--------------------------------"
	);

	printf
	(
		"| Bullet Mass:\t\t%.4lf grams\n"
		"| Bullet Diameter:\t%.4lf mm\n"
		"| Angle of Fire:\t%.2lf degrees\n"
		"| Heading of Fire:\t%.2lf degrees\n"
		"| Muzzle Velocity:\t%.3lf m/s\n"
		"| Yaw-of-Repose factor:\t%g\n"
		"| Bullet Tilt factor:\t%g\n"
		"| Twist Rate:\t\t1:%.0lf\n"
		"| Time-Step length:\t%lf seconds\n"
		"| Ground Level:\t%lf meters from sea level\n"
		"| Drag Model:\t%s\n",
			specs.mass * 1000.0,
			specs.diam * 1000.0,
			muzzleAngleDeg,
			azimuthDeg,
			muzzleSpeed,
			specs.aero.CyawRepose,
			specs.aero.Ctilt,
			twistInches,
			timeStep,
			env.groundLvl,
			dragModelString
	);

	puts
	(
		"--------------------------------\n"
		"|    Simulation calculations:  |\n"
		"--------------------------------"
	);

	printf
	(
		"| Flight Time:\t\t%.3lf seconds\n"
		"| Impact Position (m):\tx = %.3lf\ty = %.3lf\tz = %.3lf\n"
		"| Final Velocity (m/s):\tvx = %.3lf\tvy = %.3lf\tvz = %.3lf\n"
		"| Orientation\t\tq = (%g, %g, %g, %g)\n",
		t, state.x, state.y, state.z, state.vx, state.vy, state.vz, state.ori.w, state.ori.x, state.ori.y, state.ori.z
	);


	// Convert spin rate from rad/s to rpm and display that to user
	double spinRateRpm = (sqrt(state.wx*state.wx + state.wy*state.wy + state.wz*state.wz)) * (60 / (2*M_PI));
	printf
	(
		"| Impact Spin Rate (rpm):\t%g\n"
		"| Peak Height:\t\t%.3lf meters from sea level\n"
		"| Range:\t\t%.3lf meters away from muzzle\n",
		spinRateRpm, peakHeight, groundDistance
	);

	fclose(csv_log_file);

	printf("Trajectory data saved to \"trajectory_log.csv\".\n");

	return 0;
}

/** @} */ // end of MainProgram group