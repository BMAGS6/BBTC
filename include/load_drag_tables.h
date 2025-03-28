/**
 *	@file load_drag_tables.h
 *	@brief Drag model support and CSV loading for standard ballistic drag functions.
 */

#ifndef LOAD_DRAG_TABLES_H
#define LOAD_DRAG_TABLES_H

#ifdef __cplusplus
	extern "C" {
#endif

#include <stdlib.h>
#include "compiler_macros.h"

/**
 *	@defgroup DragTableFunctions Drag Table Infrastructure
 *	@{
 *	This module handles:
 *	- Defining standard drag models (G1, G7, etc.),
 *	- Parsing CSV tables of Mach/Cd values,
 *	- Interpolating drag coefficients at arbitrary Mach numbers,
 *	- Precomputing fast lookup tables for runtime efficiency.
  */

/**
 * @brief Standard drag function types used in external ballistics.
 *
 * These refer to common empirical drag models such as G1, G7, etc.
 * Each is based on different reference projectile shapes:
 * - G1: Flat-base, blunt nose (classic bullet)
 * - G7: Boat-tail projectile (modern long-range)
 * - G5, G6, etc.: Specialized shapes
 * - RA4, GS, GL, GI: Less commonly used or experimental
 *
 * These are used to select which drag function file to load.
 */
typedef enum
{
	G1,			///< G1 Drag Model (Most used drag model today)
	G2,			///< G2 Drag Model
	G5,			///< G5 Drag Model
	G6,			///< G6 Drag Model
	G7,			///< G7 Drag Model (Second-most used drag model today; most accurate for modern, boat-tailed bullets)
	G8,			///< G8 Drag Model
	GL,			///< GL Drag Model
	GS,			/// GS Drag Model (used for spheres, e.g., cannon and musket balls)
	GI,			///< GI Drag Model
	RA4,		///< RA4 Drag Model (used rarely and only for "heeled" bullets, only seen in air guns and small rimfire rounds)
	G_UNKNOWN  ///< Unknown or unsupported drag model
} DragModel;

/**
 * @brief One entry in a drag function table (Mach vs. Cd)
 *
 * This structure holds a single pair:
 * - `mach`: The Mach number
 * - `Cd`:   The corresponding drag coefficient
 *
 * Used for interpolation in drag functions.
 */
typedef struct ALIGN(64)
{
	double mach;	///< Mach number (ratio of bullet speed to local speed of sound)
	double Cd;		///< Corresponding drag coefficient
} DragEntry;

/**
 * @brief Global drag coefficient lookup table.
 *
 * This global buffer is filled by `load_drag_table_CSV()` or `precompute_drag_lookup_table()`
 * and used by the other functions for interpolation.
 */
extern double lookupTable[];

/**
 * @brief Loads a drag table from a CSV file into the global lookup table.
 *
 * Parses a file formatted as "Mach,Cd" on each line, stores values in a global table.
 *
 * @param filename Path to the CSV file to load.
 * @return The number of entries successfully loaded.
 *
 * @note This function allocates and populates the global `lookupTable[]`.
 * The file must be correctly formatted as a .csv file with no spaces, where mach is the first column
 *	and Cd is the right column and there is no header, or the program will not work.
 */
size_t load_drag_table_CSV(const char *filename);

/**
 * @brief Linearly interpolates a drag coefficient from the global lookup table.
 *
 * Given a Mach number, this function searches for two nearby entries in the
 * `lookupTable[]` and performs linear interpolation. If `mach` is outside the
 * table range, it clamps to the first or last available value.
 *
 * @param[in] mach Mach number to query.
 * @return The interpolated or clamped drag coefficient.
 */
double interpolate_cd_from_table(double mach);

/**
 * @brief Quickly retrieves a drag coefficient using a precomputed table.
 *
 * Similar to `interpolate_cd_from_table()`, but optimized using fixed-resolution
 * sampling of the Mach domain and avoiding runtime search.
 *
 * @param lookupTable Pointer to the precomputed Cd lookup array.
 * @param mach Mach number.
 * @return Drag coefficient approximated from the precomputed table.
 */
double fast_interpolate_cd(const double *restrict lookupTable, double mach);

/**
 * @brief Precomputes a lookup table of Cd values for fast runtime access.
 *
 * This function builds a uniformly sampled array of Cd values across a
 * typical Mach number range (e.g., 0.0 to 5.0). Used by `fast_interpolate_cd()`.
 *
 * @param lookupTable Pointer to the buffer to fill with precomputed Cd values.
 */
void precompute_drag_lookup_table(double *restrict lookupTable);

/**
 * @brief Gets the file name for a given drag model enum.
 *
 * Maps a `DragModel` enum to a corresponding CSV filename string.
 *
 * @param model Drag model to query.
 * @return Pointer to string containing the file path, or NULL if unknown.
 */
const char* get_drag_model_file(DragModel model);

/** @} */
// end of Doxygen parsing group "DragTableFunctions"

#ifdef __cplusplus
}
#endif
#endif /* LOAD_DRAG_TABLES_H */


