/**
 * @file environment.h
 * @brief Declarations for environment & weather calculations.
 *
 */

#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#ifdef __cplusplus
	extern "C"{
#endif

#include "pstate.h"
#include "compiler_macros.h"

/**
 * @defgroup environment Environment & Weather
 * @ingroup AllFiles
 * @{
 */

// Macros for standard environment constants
#define TEMP_SEA_LVL_K	  288.15		// Kelvin
#define TEMP_SEA_LVL_C	  15.0065		// Celsius
#define LAPSE_RATE		  0.0065		// K/m
#define STANDARD_PRESSURE 101325.0065	// Pa
#define GRAVITY			  9.807			// m/s^2
#define MOL_M_AIR		  0.0289644		// kg/mol
#define R_DRY_AIR		  287.052874	// J/(kg*K)
#define R_H2O_VAPOR		  461.52		// J/(kg*K)
#define R_UNIV			  8.3144598		// J/(mol*K)
#define GAMMA_DRY_AIR	  1.461
#define GAMMA_H2O_VAPOR   1.32

/**
 * @brief Holds environment data such as temperature, pressure, wind, etc.
 */
typedef struct ALIGN(64)
{
	double	muzzleTempK;	///< Temperature of the air at the muzzle in K
	double	muzzlePressure;	///< Air pressure at the muzzle
	double	muzzleAlt;		///< Altitude of the muzzle
	double	latitude;		///< Latitude in meters
	double	relHumidity;	///< Humidity of the air at projectile
	double	windX;			///< Speed of wind, east-west, in m/s
	double	windY;			///< Speed of wind, vertically, in m/s
	double	localSpdOfSnd;	///< Speed of sound at projectile
	double	windZ;			///< Speed of wind, north-south, in m/s
	double	localDensity;	///< Density of air at projectile
	double	localTempK;		///< Temperature of air at projectile in K
	double	spdOfSndInv;	///< Inverse of the local speed of sound
	double	localPressure;	///< Air pressure at projectile
	double	windDirDeg; 	///< Direction of horizontal wind from 0 - 359 degrees
	double	groundLvl;  	///< Elevation of the ground from sea level
	double	massFrac;   	///< Mass fraction of water vapor
	double	p_vap;			///< Partial pressure of water vapor
	double	p_sat;			///< Saturation vapor pressure
	double	p_dry;			///< Partial pressure of dry air
} Environment;

/**
 * @brief Computes the saturation vapor pressure at temperature tK (Kelvin).
 * @param tK Temperature in Kelvin
 * @return Saturation vapor pressure in Pa
 */
double saturation_vapor_pressure(double tK);

/**
 * @brief Updates the environment based on projectile state (e.g., altitude).
 * @param env [in,out] The environment to update
 * @param state [in]   Current projectile state
 */
void update_environment(Environment *restrict env, PState *restrict state);

/**
 * @brief Computes the speed of sound in the environment
 *
 * @param env The environment
 */
void compute_speed_of_sound(Environment *restrict env);

/** @} */ // end of environment group

#ifdef __cplusplus
}
#endif
#endif /* ENVIRONMENT_H */
