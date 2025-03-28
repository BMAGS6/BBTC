/**
 * @file environment.c
 * @brief Implements functions for atmospheric and environmental calculations.
 *
 *	Integrated into the main loop described in @ref core_6dof.
 *
 */

#include <stdio.h>
#include <math.h>
#include "../include/environment.h"
#include "../include/pstate.h"
#include "../include/compiler_macros.h"

/**
 * @ingroup environment
 * @{
 */

/**
 * @brief A constant factor used for exponent calculation in some atmospheric models.
 *
 * Defined as:
 * \f[
 *   \mathrm{PRECOMPUTED\_EXPONENT} = \frac{GRAVITY \times MOL\_M\_AIR}{R\_UNIV \times LAPSE\_RATE}.
 * \f]
 */
#define PRECOMPUTED_EXPONENT (GRAVITY * MOL_M_AIR) / (R_UNIV * LAPSE_RATE)

/**
 * @brief Computes saturation vapor pressure (Pa) for a given temperature in Kelvin.
 *
 * Internally converts to Celsius (\f$t_{\mathrm{C}} = t_{\mathrm{K}} - 273.15\f$), then uses an approximation:
 * \f[
 *   p_{\mathrm{sat}} = 6.1078 \times \exp\Bigl(\frac{17.08085 \times t_{\mathrm{C}}}{234.175 + t_{\mathrm{C}}}\Bigr)\times 100.0 \mathrm{Pa}.
 * \f]
 *
 * @param[in] tK Temperature in Kelvin
 * @return Saturation vapor pressure in Pascals
 */
inline double saturation_vapor_pressure(double tK)
{
	double tC = tK - 273.15;
	double p_sat_hPa = 6.1078 * exp(17.08085 * tC / (234.175 + tC));
	return p_sat_hPa * 100.0;
}

/**
 * @brief Computes an effective ratio of specific heats \f$\gamma_{\mathrm{effective}}\f$
 * given the fraction of water vapor in air.
 *
 * Weighted average:
 * \f[
 *    \gamma_{\mathrm{effective}} = 1.4 \times (1 - mass\_frac\_h2o) + 1.33 \times mass\_frac\_h2o.
 * \f]
 *
 * @param[in] mass_frac_h2o Fraction of water vapor by pressure
 * @return Effective gamma
 */
static inline double compute_gamma(double mass_frac_h2o)
{
	return ((1.4 * (1.0 - mass_frac_h2o)) + (1.33 * mass_frac_h2o));
}

/**
 * @brief Updates the environment based on projectile state (e.g., altitude).
 *
 * Modifies \c env->localTempK, \c env->localPressure, \c env->localDensity,
 * and \c env->localSpdOfSnd according to various atmospheric layers:
 * - Troposphere
 * - Stratosphere (lower, mid, upper)
 * - Mesosphere (lower, mid, upper)
 * - Thermosphere
 * - Exosphere
 *
 * Each layer has a distinct temperature/pressure model:
 * - Some layers have a constant temperature, with exponential decay in pressure.
 * - Others have linear or variable lapse rates in temperature.
 *
 * After computing local temperature and pressure, the function calculates partial
 * pressures of water vapor, total density, and speed of sound.
 *
 * @param env [in,out] The environment to update
 * @param state [in]   Current projectile state for altitude, \c state->y
 */
HOT void update_environment(Environment *env, PState *state)
{
	double
		alt = fmax(state->y, -413.0),	// Current bullet altitude
		T0  = env->muzzleTempK,			// Initial temperature at muzzle
		P0  = env->muzzlePressure;		// Initial pressure at muzzle

	// Standard atmosphere layer boundaries
	const double h1 = 11000.0, T1 = 216.65;		// Start of Stratosphere
	const double h2 = 20000.0, T2 = 216.65;		// Lower Stratosphere
	const double h3 = 32000.0, T3 = 228.65;		// Mid Stratosphere
	const double h4 = 47000.0, T4 = 270.65;		// Upper Stratosphere
	const double h5 = 51000.0, T5 = 270.65;		// Start of Mesosphere
	const double h6 = 71000.0, T6 = 214.65;		// Mid Mesosphere
	const double h7 = 85000.0, T7 = 186.65;		// Upper Mesosphere/Thermosphere
	const double h8 = 600000.0,T8 = 1500.0;		// Approximate Exosphere

	// -------- Temperature & Pressure Model for a Changing Atmosphere --------
	if (LIKELY(alt <= h1))	// Troposphere
	{
		env->localTempK = T0 - LAPSE_RATE * (alt - env->muzzleAlt);
		env->localPressure = P0 * pow((env->localTempK / T0), (-GRAVITY * MOL_M_AIR / (R_DRY_AIR * LAPSE_RATE)));
	}
	else if (UNLIKELY(alt <= h2))	// Lower Stratosphere (Constant Temp)
	{
		env->localTempK = T1;
		env->localPressure *= exp((-GRAVITY * MOL_M_AIR * (alt - h1)) / (R_DRY_AIR * T1));
	}
	else if (UNLIKELY(alt <= h3))	// Mid Stratosphere (Lapse Rate: +1 K/km)
	{
		env->localTempK = T2 + (0.001 * (alt - h2));
		env->localPressure *= pow((env->localTempK / T2), (-GRAVITY * MOL_M_AIR / (R_DRY_AIR * 0.001)));
	}
	else if (UNLIKELY(alt <= h4))	// Upper Stratosphere (Lapse Rate: +2.8 K/km)
	{
		env->localTempK = T3 + (0.0028 * (alt - h3));
		env->localPressure *= pow((env->localTempK / T3), (-GRAVITY * MOL_M_AIR / (R_DRY_AIR * 0.0028)));
	}
	else if (UNLIKELY(alt <= h5))	// Upper Stratosphere (Constant Temp)
	{
		env->localTempK = T4;
		env->localPressure *= exp((-GRAVITY * MOL_M_AIR * (alt - h4)) / (R_DRY_AIR * T4));
	}
	else if (UNLIKELY(alt <= h6))	// Lower Mesosphere (lapse Rate: -2.8 K/km)
	{
		env->localTempK = T5 - (0.0028 * (alt - h5));
		env->localPressure *= pow((env->localTempK / T5), (-GRAVITY * MOL_M_AIR / (R_DRY_AIR * -0.0028)));
	}
	else if (UNLIKELY(alt <= h7))	// Mid Mesosphere (Lapse Rate: -2.0 K/km)
	{
		env->localTempK = T6 - (0.002 * (alt - h6));
		env->localPressure *= pow((env->localTempK / T6), (-GRAVITY * MOL_M_AIR / (R_DRY_AIR * -0.002)));
	}
	else if (UNLIKELY(alt <= h8))	// Thermosphere (Linear increase)
	{
		env->localTempK = T7 + (4.0 * (alt - h7) / 1000.0);		// +4.0 K/km
		env->localPressure *= exp((-GRAVITY * MOL_M_AIR * (alt - h7)) / (R_DRY_AIR * T7));
	}
	else	// Exosphere (exponential decay)
	{
		env->localTempK = T8;									// Keep temp at ~1500 Kelvin
		env->localPressure *= exp(-0.00001 * (alt - h8));		// Extreme pressure decay (bullet is in space. This is only really in here cuz it was easy to add and... yeah. Fun little edge case :D)
	}

	// --- Humidity Effects ---
	env->p_sat = saturation_vapor_pressure(env->localTempK);
	env->p_vap = fmin(env->relHumidity * env->p_sat, env->localPressure * 0.99999999);
	env->p_dry = env->localPressure - env->p_vap;

	// --- Compute Mass Fraction of Water Vapor ---
	env->massFrac = env->p_vap / env->localPressure;

	// --- Air Density Calculation ---
	double rho_dry = env->p_dry / (R_DRY_AIR * env->localTempK);
	double rho_vap = env->p_vap / (R_H2O_VAPOR * env->localTempK);
	env->localDensity = rho_dry + rho_vap;

	// --- Speed of Sound ---
	double gamma_effective = compute_gamma(env->massFrac);
	env->localSpdOfSnd = sqrt(gamma_effective * R_DRY_AIR * env->localTempK);
}
/** @} */ // end of environment group
