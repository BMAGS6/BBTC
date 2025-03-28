/**
 * @file solver_options.h
 * @brief Defines toggles/flags controlling advanced solver effects.
 */

#ifndef SOLVER_OPTIONS_H
#define SOLVER_OPTIONS_H

#ifdef __cplusplus
	extern "C" {
#endif

#include "compiler_macros.h"

/**
 * @defgroup SolverOptions Solver Options
 * @ingroup AllFiles
 * @{
 */

/**
 * @brief Bitmask flags for advanced physics effects in the 6-DOF solver.
 */
typedef enum
{
    SOLVER_ENABLE_NONE        = 0x00,
    SOLVER_ENABLE_MAGNUS      = 0x01,
    SOLVER_ENABLE_CORIOLIS    = 0x02,
    SOLVER_ENABLE_EOTVOS      = 0x04,
    SOLVER_ENABLE_YAW_REPOSE  = 0x08,
    SOLVER_ENABLE_BULLET_TILT = 0x10,
    // etc...
} SolverEffects;

/**
 * @brief Defines which advanced physics effects are accounted for in the simulation.
 */
typedef struct ALIGN(64)
{
    int effects;  ///< bitmask from @ref SolverEffects
} SolverOptions;

/** @} */ // end group

#ifdef __cplusplus
}
#endif
#endif /* SOLVER_OPTIONS_H */
