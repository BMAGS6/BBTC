/**
 * @file
 * @brief Validation of EB0.1-B8 applied force/moment records.
 *
 * B8 validation is intentionally representational rather than dynamical. The
 * validator checks only pointer validity and scalar finiteness. It does not
 * infer mass, inertia, orientation, gravity, force application points, or any
 * physical relationship between the force and moment vectors.
 *
 * To preserve deterministic diagnostic precedence, each native precision
 * family performs one complete NaN pass across all six scalars before a second
 * complete pass checks for positive or negative infinity.
 */

#include <bbtc/external_ballistics/force_moment.h>

#include <math.h>
#include <stddef.h>

#define BBTC_EB_DEFINE_FORCE_MOMENT_VALIDATOR(SUFFIX, SCALAR, RECORD_T)    \
    bbtc_status_e                                                         \
    bbtc_eb_force_moment_validate_##SUFFIX(                              \
        const RECORD_T* const force_moment                               \
    )                                                                     \
    {                                                                     \
        if (force_moment == NULL)                                         \
            return BBTC_STATUS_INVALID_ARGUMENT;                          \
                                                                          \
        const SCALAR values[] =                                           \
        {                                                                 \
            force_moment->applied_force_reference_n.x,                    \
            force_moment->applied_force_reference_n.y,                    \
            force_moment->applied_force_reference_n.z,                    \
            force_moment->applied_moment_body_n_m.x,                      \
            force_moment->applied_moment_body_n_m.y,                      \
            force_moment->applied_moment_body_n_m.z                       \
        };                                                                \
                                                                          \
        for (size_t i = 0; i < sizeof(values) / sizeof(values[0]); ++i)  \
        {                                                                 \
            if (isnan(values[i]))                                         \
                return BBTC_STATUS_NAN_INPUT;                             \
        }                                                                 \
                                                                          \
        for (size_t i = 0; i < sizeof(values) / sizeof(values[0]); ++i)  \
        {                                                                 \
            if (isinf(values[i]))                                         \
                return BBTC_STATUS_NONFINITE_INPUT;                       \
        }                                                                 \
                                                                          \
        return BBTC_STATUS_SUCCESS;                                       \
    }

BBTC_EB_DEFINE_FORCE_MOMENT_VALIDATOR(
    float,
    float,
    bbtc_eb_force_moment_float_t
)

BBTC_EB_DEFINE_FORCE_MOMENT_VALIDATOR(
    double,
    double,
    bbtc_eb_force_moment_double_t
)

BBTC_EB_DEFINE_FORCE_MOMENT_VALIDATOR(
    long_double,
    long double,
    bbtc_eb_force_moment_long_double_t
)

#undef BBTC_EB_DEFINE_FORCE_MOMENT_VALIDATOR
