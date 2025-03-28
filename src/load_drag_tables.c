/**
 * @file load_drag_tables.c
 * @brief Contains functions to load and interpolate drag tables from CSV files.
 *
 * These functions handle reading drag data from CSV, building a finer
 * resolution array, and providing interpolation for Mach-based drag.
 */

#include <stdio.h>
#include <stdlib.h>
#include "../include/load_drag_tables.h"
#include "../include/compiler_macros.h"

/**
 * @ingroup DragTableFunctions
 * @{
 */
#define MAX_TABLE_SIZE 500      ///< Max # of entries in the CSV table
#define LOOKUP_STEP 0.001       ///< Step for the finer resolution array
#define LOOKUP_MAX_MACH 5.0     ///< Maximum Mach # in the finer resolution array
#define LOOKUP_SIZE 5001        ///< \f$ \frac{\mathrm{LOOKUP\_MAX\_MACH}}{\mathrm{LOOKUP\_STEP}} + 1 \f$

static ALIGN(64) DragEntry g_dragTable[MAX_TABLE_SIZE];
double ALIGN(64) lookupTable[LOOKUP_SIZE]; ///< The precomputed finer array

static size_t g_tableSize = 0;

/**
 * @brief Compare function for qsort, sorting DragEntry by Mach.
 * @param a Pointer to first DragEntry
 * @param b Pointer to second DragEntry
 * @return negative/zero/positive if a < b, a == b, a > b
 */
static inline int compare_drag_entries(const void *a, const void *b)
{
    double machA = ((const DragEntry *) a)->mach;
    double machB = ((const DragEntry *) b)->mach;
    return (machA > machB) - (machA < machB);
}

/**
 * @brief Loads a CSV file of (Mach,Cd) pairs and sorts them by Mach.
 *
 * @param[in] filename Path to the CSV file
 * @return The number of entries loaded. Returns 0 if the file could not be opened.
 *
 * The CSV is expected to have lines of the form:
 * @code
 * 0.1,0.23
 * 0.2,0.30
 * ...
 * @endcode
 * where `mach` is the left column, and `Cd` is the right.
 */
size_t load_drag_table_CSV(const char* filename)
{
    FILE *dragTableFile = fopen(filename, "r");
    if (!dragTableFile)
    {
        fprintf(stderr, "ERROR: Could not open drag table file \"%s\".\n", filename);
        return 0;
    }

    g_tableSize = 0;
    double m, c;
    while (fscanf(dragTableFile, "%lf,%lf", &m, &c) == 2)
    {
        if (g_tableSize < MAX_TABLE_SIZE)
        {
            g_dragTable[g_tableSize].mach = m;
            g_dragTable[g_tableSize].Cd   = c;
            g_tableSize++;
        }
        else
        {
            fprintf(stderr, "WARN: Table too large; ignoring extra data.\n");
            break;
        }
    }
    fclose(dragTableFile);

    qsort(g_dragTable, g_tableSize, sizeof(DragEntry), compare_drag_entries);
    fprintf(stderr, "Loaded and sorted %zu drag entries from \"%s\".\n", g_tableSize, filename);
    return g_tableSize;
}

/**
 * @brief Interpolates Cd for a given Mach # using linear search in g_dragTable[].
 *
 * Internally does a binary search to find the bracketed region. Then performs
 * linear interpolation:
 *
 * \f[
 *   \mathrm{Cd}(mach) = \mathrm{Cd}(m_1) + \frac{mach - m_1}{m_2 - m_1}\, \bigl(\mathrm{Cd}(m_2) - \mathrm{Cd}(m_1)\bigr).
 * \f]
 *
 * \par Complexity
 * \f$ \mathcal{O}(\log n) \f$ due to binary search.
 *
 * @param[in] mach Current Mach number.
 * @param mach_table Sorted array of Mach numbers.
 * @param cd_table Corresponding array of drag coefficients.
 * @param table_size Number of elements in the tables.
 * @return Interpolated drag coefficient for the given Mach number.
 */
double interpolate_cd_from_table(double mach)
{
    size_t low = 0, high = g_tableSize - 1;
    while (low < high)
    {
        size_t mid = (low + high) / 2;
        if (g_dragTable[mid].mach < mach)
            low = mid + 1;
        else
            high = mid;
    }
    if (low == 0) return g_dragTable[0].Cd;
    if (low == g_tableSize) return g_dragTable[g_tableSize - 1].Cd;

    double m1 = g_dragTable[low - 1].mach;
    double m2 = g_dragTable[low].mach;
    double c1 = g_dragTable[low - 1].Cd;
    double c2 = g_dragTable[low].Cd;

    double t = (mach - m1) / (m2 - m1);
    return c1 + t * (c2 - c1);
}

/**
 * @brief Fast interpolation from the finer resolution array for a given Mach.
 * @param[in] lookupTable A pointer to the precomputed array
 * @param[in] mach The Mach number to sample
 * @return Drag coefficient
 *
 * We clamp \f$\mathrm{mach}\f$ to \f$[0, \mathrm{LOOKUP\_MAX\_MACH}]\f$, then compute:
 * \f[
 *     \mathrm{index} = \left\lfloor \frac{\mathrm{mach}}{\mathrm{LOOKUP\_STEP}} \right\rfloor
 * \f]
 * and linearly interpolate between index and index+1.
 */
double fast_interpolate_cd(const double *restrict lookupTable, double mach)
{
    if (mach <= 0.0) return lookupTable[0];
    if (mach >= LOOKUP_MAX_MACH) return lookupTable[LOOKUP_SIZE - 1];

    double index_f = mach / LOOKUP_STEP; 
    size_t index   = (size_t)index_f;
    double t       = index_f - index;

    return lookupTable[index] + t * (lookupTable[index + 1] - lookupTable[index]);
}

/**
 * @brief Builds a finer-resolution array from the loaded table for fast lookup.
 * @param[in,out] lookupTable A pointer to the array of size LOOKUP_SIZE
 *
 * For each index \f$i\f$, we map:
 * \f[
 *     i \cdot \mathrm{LOOKUP\_STEP} \mapsto \mathrm{interpolate\_cd\_from\_table}
 * \f]
 */
void precompute_drag_lookup_table(double *restrict lookupTable)
{
    size_t i;
    for (i = 0; i < LOOKUP_SIZE; i++)
    {
        double mach = i * LOOKUP_STEP;
        lookupTable[i] = interpolate_cd_from_table(mach);
    }
    fprintf(stderr, "Expanded Drag Table's resolution to %zu entries.\n", i - 1);
}

/**
 * @brief Returns the file path for each DragModel enumeration.
 * @param[in] model The selected drag model
 * @return The corresponding CSV file path. e.g. "CSV_Files/G1.csv" for G1
 */
const char* get_drag_model_file(DragModel model)
{
	switch (model)
	{
		case G1:	return "CSV_Files/G1.csv";
		case G7:	return "CSV_Files/G7.csv";
		case G2:	return "CSV_Files/G2.csv";
		case G5:	return "CSV_Files/G5.csv";
		case G6:	return "CSV_Files/G6.csv";
		case G8:	return "CSV_Files/G8.csv";
		case GL:	return "CSV_Files/GL.csv";
		case GS:	return "CSV_Files/GS.csv";
		case GI:	return "CSV_Files/GI.csv";
		case RA4:	return "CSV_Files/RA4.csv";
		default:	return "CSV_Files/G1.csv"; // fallback
	}
}
/** @} */ // end of DragTableFunctions group
