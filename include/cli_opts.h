/**
 * @file cli_opts.h
 * @brief Function declarations and datatypes for user input through the command line.
 */

#ifndef CLI_OPTS_H
#define CLI_OPTS_H

#include <stdio.h>
#include <getopt.h>
#include "solver_options.h"
#include "load_drag_tables.h"

#ifdef __cplusplus
	extern "C"{
#endif

/**
 * @defgroup CLI_Parsing CLI Option Parsing
 * @ingroup AllFiles
 * @{
  */

/**
 * @brief Composite type that holds the command-line input parameters specified by the user.
 *
 * This structure aggregates ballistic parameters such as muzzle velocity, angle,
 * bullet mass, solver toggles, drag model selection, etc.
 */
typedef struct
{
	SolverOptions options;   /* Toggles/flags (Magnus, Coriolis, etc.) from solver_options.h */

	// Ballistic Parameters:
	double muzzleSpeed;      /**< Muzzle velocity in m/s */
	double muzzleAngleDeg;   /**< Muzzle angle in degrees above horizontal */
	double tiltFactor;       /**< Bullet tilt factor (advanced) */
	double yawFactor;        /**< Yaw-of-repose factor (advanced) */
	double diam;             /**< Bullet diameter in meters */
	double shotAngle;        /**< Not used in the code snippet, but could represent shot angle in degrees? */
	double twistInches;      /**< Barrel twist rate (inches per rotation) */
	double timeStep;         /**< Integration time step in seconds */
	double azimuthDeg;       /**< Heading in degrees (0 = North) */
	double ambientTemp;      /**< Ambient temperature at muzzle in Kelvin */
	double mass;             /**< Bullet mass in kg */
	double latitude;         /**< Firing latitude in degrees */
	double altitude;         /**< Muzzle altitude above sea level in meters */

	DragModel dragModelChoice; /**< Drag model enum specifying which ballistic drag curve is used */
} CLI_inputs;

/**
 * @brief Initializes defaults for CLI_Inputs, so we have a baseline.
 * @param cli pointer to CLI_Inputs struct
 *
 * This function zeroes out the struct and then sets default values
 * such as muzzleSpeed = 800.0, muzzleAngleDeg = 1.0, mass = 0.0065, etc.
 */
void initialize_CLI_inputs(CLI_inputs *cli);

/**
 * @brief Parses the command-line arguments for the solver.
 *
 * @param[in] cli  Pointer to CLI_Inputs to store the user choices
 * @param[in] argc Argument count
 * @param[in] argv Argument vector
 * @return 0 on success, non-zero on error or if user requested help
 *
 * This function uses \c getopt_long to handle short and long options.
 * The recognized options and their effect are documented in print_usage_instructions().
 */
int parse_options(CLI_inputs *cli, int argc, char *argv[]);

/** @} */
// end of CLI_Parsing group

#ifdef __cplusplus
}
#endif
#endif /* CLI_OPTS_H */
