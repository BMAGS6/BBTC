/**
 * @file
 * @brief Validation of EB0.1-B9 rigid-body state derivatives.
 *
 * B9 validation checks only pointer validity and scalar finiteness. A
 * derivative record is not itself a physical state, so the quaternion-shaped
 * orientation derivative is intentionally not normalized or subjected to the
 * rigid-body state's unit-orientation rule.
 *
 * Each native precision family performs one complete NaN scan across all
 * thirteen scalar components before a second complete scan checks positive or
 * negative infinity. This preserves deterministic diagnostic precedence
 * independently of member order.
 */

#include <bbtc/external_ballistics/state_derivative.h>

#include <math.h>
#include <stddef.h>

#define BBTC_EB_DEFINE_STATE_DERIVATIVE_VALIDATOR(SUFFIX, SCALAR, RECORD_T) \
    bbtc_status_e                                                          \
    bbtc_eb_rigid_body_state_derivative_validate_##SUFFIX(                \
        const RECORD_T* const derivative                                  \
    )                                                                      \
    {                                                                      \
        if (derivative == NULL)                                            \
            return BBTC_STATUS_INVALID_ARGUMENT;                           \
                                                                           \
        const SCALAR values[] =                                            \
        {                                                                  \
            derivative->orientation_world_from_body_derivative_per_s.x,   \
            derivative->orientation_world_from_body_derivative_per_s.y,   \
            derivative->orientation_world_from_body_derivative_per_s.z,   \
            derivative->orientation_world_from_body_derivative_per_s.w,   \
            derivative->position_derivative_reference_m_per_s.x,          \
            derivative->position_derivative_reference_m_per_s.y,          \
            derivative->position_derivative_reference_m_per_s.z,          \
            derivative->velocity_derivative_reference_m_per_s2.x,         \
            derivative->velocity_derivative_reference_m_per_s2.y,         \
            derivative->velocity_derivative_reference_m_per_s2.z,         \
            derivative->angular_velocity_derivative_body_rad_per_s2.x,    \
            derivative->angular_velocity_derivative_body_rad_per_s2.y,    \
            derivative->angular_velocity_derivative_body_rad_per_s2.z     \
        };                                                                 \
                                                                           \
        for (size_t i = 0; i < sizeof(values) / sizeof(values[0]); ++i)   \
        {                                                                  \
            if (isnan(values[i]))                                          \
                return BBTC_STATUS_NAN_INPUT;                              \
        }                                                                  \
                                                                           \
        for (size_t i = 0; i < sizeof(values) / sizeof(values[0]); ++i)   \
        {                                                                  \
            if (isinf(values[i]))                                          \
                return BBTC_STATUS_NONFINITE_INPUT;                        \
        }                                                                  \
                                                                           \
        return BBTC_STATUS_SUCCESS;                                        \
    }

BBTC_EB_DEFINE_STATE_DERIVATIVE_VALIDATOR(
    float,
    float,
    bbtc_eb_rigid_body_state_derivative_float_t
)

BBTC_EB_DEFINE_STATE_DERIVATIVE_VALIDATOR(
    double,
    double,
    bbtc_eb_rigid_body_state_derivative_double_t
)

BBTC_EB_DEFINE_STATE_DERIVATIVE_VALIDATOR(
    long_double,
    long double,
    bbtc_eb_rigid_body_state_derivative_long_double_t
)

#undef BBTC_EB_DEFINE_STATE_DERIVATIVE_VALIDATOR
