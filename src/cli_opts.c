/**
 * @file cli_opts.c
 * @brief Definitions of functions for user input through the command line.
 *
 *
 * Provides a command-line interface for specifying muzzle velocity, angle,
 * bullet mass, and advanced solver toggles like Magnus effect, bullet tilt, etc.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <getopt.h>
#include "../include/compiler_macros.h"
#include "../include/cli_opts.h"
#include "../include/environment.h"

/**
 * @ingroup AllFiles
 * @{
 */

/**
 *	@brief The global array describing our long options.
 *	@ingroup CLI_Parsing
 */
struct option longOpts[] =
{
	{"magnus",		no_argument,		0, 'm'},
	{"coriolis",	no_argument,		0, 'c'},
	{"eotvos",		no_argument,		0, 'e'},
	{"yawRepose",	no_argument,		0, 'y'},
	{"mass",		required_argument,	0, 'M'},
	{"tilt",		no_argument,		0, 't'},
	{"yawFactor",	required_argument,	0, 'Y'},
	{"tiltFactor",	required_argument,	0, 'T'},
	{"velocity",	required_argument,	0, 'v'},
	{"diameter",	required_argument,	0, 'd'},
	{"angle",		required_argument,	0, 'a'},
	{"direction",	required_argument,	0, 'D'},
	{"twistRate",	required_argument,	0, 'w'},
	{"timeStep",	required_argument,	0, 's'},
	{"shotTemp",	required_argument,	0, 'x'},
	{"dragModel",	required_argument,	0, 'G'},
	{"latitude",	required_argument,	0, 'L'},
	{"altitude",	required_argument,	0, 'A'},
	{"help",		no_argument,		0, 'h'},
	{0, 0, 0, 0} // sentinel
};

/**
 *	@brief Prints usage instructions for the user, listing all recognized options.
 *	@ingroup CLI_Parsing
 *	@param progName The name of the executable (argv[0])
 */
COLD void print_usage_instructions(const char *progName)
{
	fprintf(stderr,
		"|\tUsage: %s [OPTIONS]\n"
		"|---------------------------------------------------------------------------------------------------\n"
		"|\t-v | --velocity - - -| Set muzzle velocity (m/s);\n"
		"|\t-d | --diameter - - -| Set diameter (caliber) of bullet (m);\n"
		"|\t-y | --yawRepose  - -| Enable yaw-of-repose in calculations---default value is preset (advanced);\n"
		"|\t-M | --mass - - - - -| Mass of the bullet in kilograms (kg);\n"
		"|\t-Y | --yawFactor  - -| Enter in a custom yaw factor for bullet                        (advanced);\n"
		"|\t-t | --tilt - - - - -| Enable bullet tilt in calculations---default value is preset   (advanced);\n"
		"|\t-T | --tiltFactor - -| Set the tilt factor of the bullet (advanced);\n"
		"|\t-a | --angle  - - - -| Set vertical angle of muzzle, where 0 = straight ahead (deg);\n"
		"|\t-D | --direction  - -| Set heading of muzzle, where 0 = North (degrees);\n"
		"|\t-m | --magnus - - - -| Enable Magnus effect in calculations   (advanced);\n"
		"|\t-c | --coriolis - - -| Enable Coriolis effect in calculations (advanced);\n"
		"|\t-e | --eotvos - - - -| Enable Eotvos effect in calculations   (advanced);\n"
		"|\t-w | --twistRate  - -| Change the twist rate of barrel        (advanced);\n"
		"|\t-s | --timeStep  - - | Change the solver time-step (advanced);\n"
		"|\t   | --shotTemp  - - | Set the ambient temperature at muzzle;\n"
		"|\t-G | --dragModel - - | Select the drag model (\"G function\") to use;\n"
		"|\t-L | --latitude  - - | Set the latitude of the muzzle in degrees;\n"
		"|\t-A | --altitude  - - | Set the altitude of the muzzle from sea level (m);\n"
		"|\t-h | --help - - - - -| Show this message.\n\n"
		"|Example:\n"
		"|\t%s -mc -a 5\n"
		"|\t(Enables Magnus and Coriolis, muzzle altitude = 5m, etc.)\n\n",
		progName, progName
	);
}

/**
 *	@brief Case-insensitive string compare.
 *	@ingroup CLI_Parsing
 *	@param str1 First string
 *	@param str2 Second string
 *	@return 1 if equal ignoring case, 0 if mismatch, -1 if either is NULL
 */
static int str_cmp_ignoring_case(const char *str1, const char *str2)
{
	if (!str1 || !str2)
		return -1;

	while (*str1 && *str2)
	{
		char ch1 = (char) tolower((unsigned char)*str1);
		char ch2 = (char) tolower((unsigned char)*str2);
		if (ch1 != ch2)
			return 0; // mismatch
		str1++;
		str2++;
	}
	return (*str1 == '\0' && *str2 == '\0'); // 1 if both ended
}

/**
 * @brief Character array of possible short options
 * @ingroup CLI_Parsing
 */
static const char *shortOpts = "mceyM:tY:T:v:d:a:D:w:s:x:G:L:A:h";

/**
 * @brief Initializes CLI_Inputs with default toggles and ballistic parameters.
 */
void initialize_CLI_inputs(CLI_inputs *cli)
{
	memset(cli, 0, sizeof(*cli));
	cli->options.effects = SOLVER_ENABLE_NONE;

	cli->muzzleSpeed     = 800.0;
	cli->muzzleAngleDeg  = 1.0;
	cli->azimuthDeg      = 90.0;
	cli->timeStep        = 0.001;
	cli->twistInches     = 7.0;
	cli->diam            = 0.00556;
	cli->ambientTemp     = 288.15;
	cli->mass            = 0.0065;
	cli->dragModelChoice = G1; // default drag model
}

/**
 *	@brief Parses the command-line arguments for the solver.
 */
int parse_options(CLI_inputs *cli, int argc, char *argv[])
{
	int opt       = 0;
	int longIndex = 0;

	while((opt = getopt_long(argc, argv, shortOpts, longOpts, &longIndex)) != -1)
	{
		fprintf(stderr, "In While loop\n");

		switch (opt)
		{
			case 'm':
				cli->options.effects |= SOLVER_ENABLE_MAGNUS;
				break;
			case 'c':
				cli->options.effects |= SOLVER_ENABLE_CORIOLIS;
				break;
			case 'e':
				cli->options.effects |= SOLVER_ENABLE_EOTVOS;
				break;
			case 'y':
				cli->options.effects |= SOLVER_ENABLE_YAW_REPOSE;
				break;
			case 't':
				cli->options.effects |= SOLVER_ENABLE_BULLET_TILT;
				break;
			case 'Y':
				cli->yawFactor = atof(optarg);
				break;
			case 'T':
				cli->tiltFactor = atof(optarg);
				break;
			case 'v':
				cli->muzzleSpeed = atof(optarg);
				break;
			case 'd':
				cli->diam = atof(optarg);
				break;
			case 'a':
				cli->muzzleAngleDeg = atof(optarg);
				break;
			case 'D':
				cli->azimuthDeg = atof(optarg);
				break;
			case 'w':
				cli->twistInches = atof(optarg);
				break;
			case 's':
				cli->timeStep = atof(optarg);
				break;
			case 'G':
			{
				// Support G1, G2, G5, G6, G7, G8, GL, GS, GI, RA4
				if (str_cmp_ignoring_case(optarg, "G1") || str_cmp_ignoring_case(optarg, "1"))
					cli->dragModelChoice = G1;
				else if (str_cmp_ignoring_case(optarg, "G2") || str_cmp_ignoring_case(optarg, "2"))
					cli->dragModelChoice = G2;
				else if (str_cmp_ignoring_case(optarg, "G5") || str_cmp_ignoring_case(optarg, "5"))
					cli->dragModelChoice = G5;
				else if (str_cmp_ignoring_case(optarg, "G6") || str_cmp_ignoring_case(optarg, "6"))
					cli->dragModelChoice = G6;
				else if (str_cmp_ignoring_case(optarg, "G7") || str_cmp_ignoring_case(optarg, "7"))
					cli->dragModelChoice = G7;
				else if (str_cmp_ignoring_case(optarg, "G8") || str_cmp_ignoring_case(optarg, "8"))
					cli->dragModelChoice = G8;
				else if (str_cmp_ignoring_case(optarg, "GL") || str_cmp_ignoring_case(optarg, "L"))
					cli->dragModelChoice = GL;
				else if (str_cmp_ignoring_case(optarg, "GS") || str_cmp_ignoring_case(optarg, "S"))
					cli->dragModelChoice = GS;
				else if (str_cmp_ignoring_case(optarg, "GI") || str_cmp_ignoring_case(optarg, "I"))
					cli->dragModelChoice = GI;
				else if (str_cmp_ignoring_case(optarg, "RA4") || str_cmp_ignoring_case(optarg, "A4"))
					cli->dragModelChoice = RA4;
				else {
					fprintf(stderr, "WARNING: Unknown drag model entered, (%s). Defaulting to G1.\n", optarg);
					cli->dragModelChoice = G1;
				}
				break;
			}
			case 'L':
				cli->latitude = atof(optarg);
				break;
			case 'A':
				cli->altitude = atof(optarg);
				break;
			case 'h':
				print_usage_instructions(argv[0]);
				return (opt == 'h' ? 0 : 1);
			default:
				print_usage_instructions(argv[0]);
				return 0;
		}
	}

	return 0; // success
}
/** @} */ // end of AllFiles parsing group
