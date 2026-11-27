/**
 * @file
 * @brief Fail-closed validation of precision-qualified EB rigid-body states.
 *
 * Validation deliberately performs complete NaN and infinity scans before
 * checking domain constraints. This makes the reported status deterministic
 * and independent of structure declaration order.
 *
 * The validator never normalizes a caller's orientation. A near-unit
 * quaternion is either accepted within the documented interoperability
 * tolerance or rejected; repair belongs to explicit mathematical operations,
 * not to a validation boundary.
 */

#include <bbtc/external_ballistics/state.h>

#include "math_internal.h"

#include <float.h>
#include <math.h>
#include <stddef.h>

#define BBTC_EB_DEFINE_STATE_VALIDATOR(                                  \
    SUFFIX, SCALAR, STATE_T, QUAT_NORM_FN, FABS_FN                      \
)                                                                        \
    bbtc_status_e                                                        \
    bbtc_eb_rigid_body_state_validate_##SUFFIX(                         \
        const STATE_T* const state                                       \
    )                                                                    \
    {                                                                    \
        if (state == NULL)                                               \
            return BBTC_STATUS_INVALID_ARGUMENT;                         \
                                                                          \
        /*                                                               \
         * Keep this explicit flattened list in physical record order.    \
         * The two full scans below deliberately decouple error            \
         * precedence from that order.                                    \
         */                                                              \
        const SCALAR values[] =                                          \
        {                                                                \
            state->orientation_world_from_body.x,                        \
            state->orientation_world_from_body.y,                        \
            state->orientation_world_from_body.z,                        \
            state->orientation_world_from_body.w,                        \
            state->position_m.x,                                         \
            state->position_m.y,                                         \
            state->position_m.z,                                         \
            state->velocity_m_per_s.x,                                   \
            state->velocity_m_per_s.y,                                   \
            state->velocity_m_per_s.z,                                   \
            state->angular_velocity_body_rad_per_s.x,                    \
            state->angular_velocity_body_rad_per_s.y,                    \
            state->angular_velocity_body_rad_per_s.z,                    \
            state->time_s                                                \
        };                                                               \
                                                                          \
        for (size_t i = 0; i < sizeof(values) / sizeof(values[0]); ++i) \
        {                                                                \
            if (isnan(values[i]))                                        \
                return BBTC_STATUS_NAN_INPUT;                            \
        }                                                                \
                                                                          \
        for (size_t i = 0; i < sizeof(values) / sizeof(values[0]); ++i) \
        {                                                                \
            if (isinf(values[i]))                                        \
                return BBTC_STATUS_NONFINITE_INPUT;                      \
        }                                                                \
                                                                          \
        if (state->time_s < (SCALAR)0)                                   \
            return BBTC_STATUS_OUTSIDE_DOMAIN;                           \
                                                                          \
        const SCALAR orientation_norm = QUAT_NORM_FN(                    \
            state->orientation_world_from_body                           \
        );                                                               \
        const SCALAR orientation_norm_squared =                          \
            orientation_norm * orientation_norm;                         \
        const SCALAR unit_tolerance =                                    \
            (SCALAR)(64.0f * FLT_EPSILON);                               \
                                                                          \
        if (FABS_FN(orientation_norm_squared - (SCALAR)1)               \
            > unit_tolerance)                                            \
        {                                                                \
            return BBTC_STATUS_OUTSIDE_DOMAIN;                           \
        }                                                                \
                                                                          \
        return BBTC_STATUS_SUCCESS;                                      \
    }

BBTC_EB_DEFINE_STATE_VALIDATOR(
    float,
    float,
    bbtc_eb_rigid_body_state_float_t,
    bbtc_eb_internal_quat_norm_float,
    fabsf
)

BBTC_EB_DEFINE_STATE_VALIDATOR(
    double,
    double,
    bbtc_eb_rigid_body_state_double_t,
    bbtc_eb_internal_quat_norm_double,
    fabs
)

BBTC_EB_DEFINE_STATE_VALIDATOR(
    long_double,
    long double,
    bbtc_eb_rigid_body_state_long_double_t,
    bbtc_eb_internal_quat_norm_long_double,
    fabsl
)

#undef BBTC_EB_DEFINE_STATE_VALIDATOR
