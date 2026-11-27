/**
 * @file
 * @brief Transactional fixed-step RK4 integration for EB0.1-B11.
 *
 * This implementation deliberately performs no equations of motion. It asks a
 * B10 provider for each derivative and advances only the B1 state components
 * represented by B9 derivatives.
 *
 * All candidate stage states live in local storage. K2/K3/K4 orientations are
 * normalized before B10 dispatch, and the final orientation is normalized
 * before commit. Invalid internally generated states map to
 * `BBTC_STATUS_NUMERICAL_FAILURE`.
 */

#include <bbtc/external_ballistics/integrator.h>

#include "math_internal.h"

#include <math.h>
#include <stddef.h>

#define BBTC_EB_DEFINE_RK4_FAMILY(                                         \
    SUFFIX, SCALAR, STATE_T, DERIVATIVE_T, PROVIDER_T,                    \
    VALIDATE_STATE, DISPATCH, NORMALIZE_QUAT                              \
)                                                                          \
    static bbtc_status_e                                                   \
    bbtc_eb_internal_rk4_stage_##SUFFIX(                                  \
        const STATE_T* const base,                                         \
        const DERIVATIVE_T* const derivative,                              \
        const SCALAR scale,                                                \
        const SCALAR stage_time,                                           \
        STATE_T* const out_stage                                           \
    )                                                                      \
    {                                                                      \
        STATE_T stage = *base;                                             \
                                                                           \
        stage.orientation_world_from_body.x =                              \
            base->orientation_world_from_body.x                            \
            + scale                                                        \
                * derivative->orientation_world_from_body_derivative_per_s.x;\
        stage.orientation_world_from_body.y =                              \
            base->orientation_world_from_body.y                            \
            + scale                                                        \
                * derivative->orientation_world_from_body_derivative_per_s.y;\
        stage.orientation_world_from_body.z =                              \
            base->orientation_world_from_body.z                            \
            + scale                                                        \
                * derivative->orientation_world_from_body_derivative_per_s.z;\
        stage.orientation_world_from_body.w =                              \
            base->orientation_world_from_body.w                            \
            + scale                                                        \
                * derivative->orientation_world_from_body_derivative_per_s.w;\
                                                                           \
        stage.position_m.x = base->position_m.x                            \
            + scale * derivative->position_derivative_reference_m_per_s.x; \
        stage.position_m.y = base->position_m.y                            \
            + scale * derivative->position_derivative_reference_m_per_s.y; \
        stage.position_m.z = base->position_m.z                            \
            + scale * derivative->position_derivative_reference_m_per_s.z; \
                                                                           \
        stage.velocity_m_per_s.x = base->velocity_m_per_s.x                \
            + scale * derivative->velocity_derivative_reference_m_per_s2.x;\
        stage.velocity_m_per_s.y = base->velocity_m_per_s.y                \
            + scale * derivative->velocity_derivative_reference_m_per_s2.y;\
        stage.velocity_m_per_s.z = base->velocity_m_per_s.z                \
            + scale * derivative->velocity_derivative_reference_m_per_s2.z;\
                                                                           \
        stage.angular_velocity_body_rad_per_s.x =                          \
            base->angular_velocity_body_rad_per_s.x                        \
            + scale                                                        \
                * derivative->angular_velocity_derivative_body_rad_per_s2.x;\
        stage.angular_velocity_body_rad_per_s.y =                          \
            base->angular_velocity_body_rad_per_s.y                        \
            + scale                                                        \
                * derivative->angular_velocity_derivative_body_rad_per_s2.y;\
        stage.angular_velocity_body_rad_per_s.z =                          \
            base->angular_velocity_body_rad_per_s.z                        \
            + scale                                                        \
                * derivative->angular_velocity_derivative_body_rad_per_s2.z;\
        stage.time_s = stage_time;                                         \
                                                                           \
        const bbtc_status_e normalize_status = NORMALIZE_QUAT(             \
            &stage.orientation_world_from_body,                            \
            &stage.orientation_world_from_body                            \
        );                                                                 \
                                                                           \
        if (normalize_status != BBTC_STATUS_SUCCESS)                       \
            return BBTC_STATUS_NUMERICAL_FAILURE;                          \
                                                                           \
        if (VALIDATE_STATE(&stage) != BBTC_STATUS_SUCCESS)                 \
            return BBTC_STATUS_NUMERICAL_FAILURE;                          \
                                                                           \
        *out_stage = stage;                                                \
        return BBTC_STATUS_SUCCESS;                                        \
    }                                                                      \
                                                                           \
    static bbtc_status_e                                                   \
    bbtc_eb_internal_rk4_final_##SUFFIX(                                  \
        const STATE_T* const base,                                         \
        const DERIVATIVE_T* const k1,                                      \
        const DERIVATIVE_T* const k2,                                      \
        const DERIVATIVE_T* const k3,                                      \
        const DERIVATIVE_T* const k4,                                      \
        const SCALAR step_s,                                               \
        const SCALAR final_time,                                           \
        STATE_T* const out_final                                           \
    )                                                                      \
    {                                                                      \
        const SCALAR one_sixth_step = step_s / (SCALAR)6;                 \
        STATE_T result = *base;                                            \
                                                                           \
        result.orientation_world_from_body.x =                             \
            base->orientation_world_from_body.x                            \
            + one_sixth_step * (                                          \
                k1->orientation_world_from_body_derivative_per_s.x         \
                + (SCALAR)2                                                \
                    * k2->orientation_world_from_body_derivative_per_s.x   \
                + (SCALAR)2                                                \
                    * k3->orientation_world_from_body_derivative_per_s.x   \
                + k4->orientation_world_from_body_derivative_per_s.x       \
            );                                                             \
        result.orientation_world_from_body.y =                             \
            base->orientation_world_from_body.y                            \
            + one_sixth_step * (                                          \
                k1->orientation_world_from_body_derivative_per_s.y         \
                + (SCALAR)2                                                \
                    * k2->orientation_world_from_body_derivative_per_s.y   \
                + (SCALAR)2                                                \
                    * k3->orientation_world_from_body_derivative_per_s.y   \
                + k4->orientation_world_from_body_derivative_per_s.y       \
            );                                                             \
        result.orientation_world_from_body.z =                             \
            base->orientation_world_from_body.z                            \
            + one_sixth_step * (                                          \
                k1->orientation_world_from_body_derivative_per_s.z         \
                + (SCALAR)2                                                \
                    * k2->orientation_world_from_body_derivative_per_s.z   \
                + (SCALAR)2                                                \
                    * k3->orientation_world_from_body_derivative_per_s.z   \
                + k4->orientation_world_from_body_derivative_per_s.z       \
            );                                                             \
        result.orientation_world_from_body.w =                             \
            base->orientation_world_from_body.w                            \
            + one_sixth_step * (                                          \
                k1->orientation_world_from_body_derivative_per_s.w         \
                + (SCALAR)2                                                \
                    * k2->orientation_world_from_body_derivative_per_s.w   \
                + (SCALAR)2                                                \
                    * k3->orientation_world_from_body_derivative_per_s.w   \
                + k4->orientation_world_from_body_derivative_per_s.w       \
            );                                                             \
                                                                           \
        result.position_m.x = base->position_m.x + one_sixth_step * (     \
            k1->position_derivative_reference_m_per_s.x                    \
            + (SCALAR)2 * k2->position_derivative_reference_m_per_s.x     \
            + (SCALAR)2 * k3->position_derivative_reference_m_per_s.x     \
            + k4->position_derivative_reference_m_per_s.x                  \
        );                                                                 \
        result.position_m.y = base->position_m.y + one_sixth_step * (     \
            k1->position_derivative_reference_m_per_s.y                    \
            + (SCALAR)2 * k2->position_derivative_reference_m_per_s.y     \
            + (SCALAR)2 * k3->position_derivative_reference_m_per_s.y     \
            + k4->position_derivative_reference_m_per_s.y                  \
        );                                                                 \
        result.position_m.z = base->position_m.z + one_sixth_step * (     \
            k1->position_derivative_reference_m_per_s.z                    \
            + (SCALAR)2 * k2->position_derivative_reference_m_per_s.z     \
            + (SCALAR)2 * k3->position_derivative_reference_m_per_s.z     \
            + k4->position_derivative_reference_m_per_s.z                  \
        );                                                                 \
                                                                           \
        result.velocity_m_per_s.x = base->velocity_m_per_s.x              \
            + one_sixth_step * (                                          \
                k1->velocity_derivative_reference_m_per_s2.x               \
                + (SCALAR)2 * k2->velocity_derivative_reference_m_per_s2.x\
                + (SCALAR)2 * k3->velocity_derivative_reference_m_per_s2.x\
                + k4->velocity_derivative_reference_m_per_s2.x             \
            );                                                             \
        result.velocity_m_per_s.y = base->velocity_m_per_s.y              \
            + one_sixth_step * (                                          \
                k1->velocity_derivative_reference_m_per_s2.y               \
                + (SCALAR)2 * k2->velocity_derivative_reference_m_per_s2.y\
                + (SCALAR)2 * k3->velocity_derivative_reference_m_per_s2.y\
                + k4->velocity_derivative_reference_m_per_s2.y             \
            );                                                             \
        result.velocity_m_per_s.z = base->velocity_m_per_s.z              \
            + one_sixth_step * (                                          \
                k1->velocity_derivative_reference_m_per_s2.z               \
                + (SCALAR)2 * k2->velocity_derivative_reference_m_per_s2.z\
                + (SCALAR)2 * k3->velocity_derivative_reference_m_per_s2.z\
                + k4->velocity_derivative_reference_m_per_s2.z             \
            );                                                             \
                                                                           \
        result.angular_velocity_body_rad_per_s.x =                         \
            base->angular_velocity_body_rad_per_s.x                        \
            + one_sixth_step * (                                          \
                k1->angular_velocity_derivative_body_rad_per_s2.x          \
                + (SCALAR)2                                                \
                    * k2->angular_velocity_derivative_body_rad_per_s2.x    \
                + (SCALAR)2                                                \
                    * k3->angular_velocity_derivative_body_rad_per_s2.x    \
                + k4->angular_velocity_derivative_body_rad_per_s2.x        \
            );                                                             \
        result.angular_velocity_body_rad_per_s.y =                         \
            base->angular_velocity_body_rad_per_s.y                        \
            + one_sixth_step * (                                          \
                k1->angular_velocity_derivative_body_rad_per_s2.y          \
                + (SCALAR)2                                                \
                    * k2->angular_velocity_derivative_body_rad_per_s2.y    \
                + (SCALAR)2                                                \
                    * k3->angular_velocity_derivative_body_rad_per_s2.y    \
                + k4->angular_velocity_derivative_body_rad_per_s2.y        \
            );                                                             \
        result.angular_velocity_body_rad_per_s.z =                         \
            base->angular_velocity_body_rad_per_s.z                        \
            + one_sixth_step * (                                          \
                k1->angular_velocity_derivative_body_rad_per_s2.z          \
                + (SCALAR)2                                                \
                    * k2->angular_velocity_derivative_body_rad_per_s2.z    \
                + (SCALAR)2                                                \
                    * k3->angular_velocity_derivative_body_rad_per_s2.z    \
                + k4->angular_velocity_derivative_body_rad_per_s2.z        \
            );                                                             \
        result.time_s = final_time;                                        \
                                                                           \
        const bbtc_status_e normalize_status = NORMALIZE_QUAT(             \
            &result.orientation_world_from_body,                           \
            &result.orientation_world_from_body                            \
        );                                                                 \
                                                                           \
        if (normalize_status != BBTC_STATUS_SUCCESS)                       \
            return BBTC_STATUS_NUMERICAL_FAILURE;                          \
                                                                           \
        if (VALIDATE_STATE(&result) != BBTC_STATUS_SUCCESS)                \
            return BBTC_STATUS_NUMERICAL_FAILURE;                          \
                                                                           \
        *out_final = result;                                               \
        return BBTC_STATUS_SUCCESS;                                        \
    }                                                                      \
                                                                           \
    bbtc_status_e                                                          \
    bbtc_eb_rk4_step_##SUFFIX(                                            \
        const PROVIDER_T* const provider,                                  \
        const SCALAR step_s,                                               \
        STATE_T* const state                                               \
    )                                                                      \
    {                                                                      \
        if (state == NULL)                                                 \
            return BBTC_STATUS_INVALID_ARGUMENT;                           \
                                                                           \
        if (provider == NULL || provider->evaluate == NULL)                \
            return BBTC_STATUS_INVALID_ARGUMENT;                           \
                                                                           \
        const bbtc_status_e state_status = VALIDATE_STATE(state);          \
                                                                           \
        if (state_status == BBTC_STATUS_NAN_INPUT || isnan(step_s))        \
            return BBTC_STATUS_NAN_INPUT;                                  \
                                                                           \
        if (state_status == BBTC_STATUS_NONFINITE_INPUT || isinf(step_s))  \
            return BBTC_STATUS_NONFINITE_INPUT;                            \
                                                                           \
        if (state_status != BBTC_STATUS_SUCCESS)                           \
            return state_status;                                           \
                                                                           \
        if (step_s <= (SCALAR)0)                                           \
            return BBTC_STATUS_OUTSIDE_DOMAIN;                             \
                                                                           \
        const STATE_T initial = *state;                                    \
        const SCALAR final_time = initial.time_s + step_s;                 \
                                                                           \
        if (!isfinite(final_time) || final_time <= initial.time_s)          \
            return BBTC_STATUS_NUMERICAL_FAILURE;                          \
                                                                           \
        const SCALAR half_step = step_s / (SCALAR)2;                      \
        const SCALAR half_time = initial.time_s + half_step;               \
                                                                           \
        DERIVATIVE_T k1 = {0};                                             \
        DERIVATIVE_T k2 = {0};                                             \
        DERIVATIVE_T k3 = {0};                                             \
        DERIVATIVE_T k4 = {0};                                             \
        STATE_T k1_state = initial;                                        \
        STATE_T stage = {0};                                               \
                                                                           \
        bbtc_status_e status = DISPATCH(provider, &k1_state, &k1);         \
                                                                           \
        if (status != BBTC_STATUS_SUCCESS)                                 \
            return status;                                                 \
                                                                           \
        status = bbtc_eb_internal_rk4_stage_##SUFFIX(                     \
            &initial, &k1, half_step, half_time, &stage                    \
        );                                                                 \
                                                                           \
        if (status != BBTC_STATUS_SUCCESS)                                 \
            return status;                                                 \
                                                                           \
        status = DISPATCH(provider, &stage, &k2);                          \
                                                                           \
        if (status != BBTC_STATUS_SUCCESS)                                 \
            return status;                                                 \
                                                                           \
        status = bbtc_eb_internal_rk4_stage_##SUFFIX(                     \
            &initial, &k2, half_step, half_time, &stage                    \
        );                                                                 \
                                                                           \
        if (status != BBTC_STATUS_SUCCESS)                                 \
            return status;                                                 \
                                                                           \
        status = DISPATCH(provider, &stage, &k3);                          \
                                                                           \
        if (status != BBTC_STATUS_SUCCESS)                                 \
            return status;                                                 \
                                                                           \
        status = bbtc_eb_internal_rk4_stage_##SUFFIX(                     \
            &initial, &k3, step_s, final_time, &stage                     \
        );                                                                 \
                                                                           \
        if (status != BBTC_STATUS_SUCCESS)                                 \
            return status;                                                 \
                                                                           \
        status = DISPATCH(provider, &stage, &k4);                          \
                                                                           \
        if (status != BBTC_STATUS_SUCCESS)                                 \
            return status;                                                 \
                                                                           \
        STATE_T final_state = {0};                                         \
        status = bbtc_eb_internal_rk4_final_##SUFFIX(                     \
            &initial, &k1, &k2, &k3, &k4, step_s, final_time,             \
            &final_state                                                   \
        );                                                                 \
                                                                           \
        if (status != BBTC_STATUS_SUCCESS)                                 \
            return status;                                                 \
                                                                           \
        *state = final_state;                                              \
        return BBTC_STATUS_SUCCESS;                                        \
    }

BBTC_EB_DEFINE_RK4_FAMILY(
    float,
    float,
    bbtc_eb_rigid_body_state_float_t,
    bbtc_eb_rigid_body_state_derivative_float_t,
    bbtc_eb_state_derivative_provider_float_t,
    bbtc_eb_rigid_body_state_validate_float,
    bbtc_eb_state_derivative_provider_evaluate_float,
    bbtc_eb_internal_quat_normalize_float
)

BBTC_EB_DEFINE_RK4_FAMILY(
    double,
    double,
    bbtc_eb_rigid_body_state_double_t,
    bbtc_eb_rigid_body_state_derivative_double_t,
    bbtc_eb_state_derivative_provider_double_t,
    bbtc_eb_rigid_body_state_validate_double,
    bbtc_eb_state_derivative_provider_evaluate_double,
    bbtc_eb_internal_quat_normalize_double
)

BBTC_EB_DEFINE_RK4_FAMILY(
    long_double,
    long double,
    bbtc_eb_rigid_body_state_long_double_t,
    bbtc_eb_rigid_body_state_derivative_long_double_t,
    bbtc_eb_state_derivative_provider_long_double_t,
    bbtc_eb_rigid_body_state_validate_long_double,
    bbtc_eb_state_derivative_provider_evaluate_long_double,
    bbtc_eb_internal_quat_normalize_long_double
)

#undef BBTC_EB_DEFINE_RK4_FAMILY
