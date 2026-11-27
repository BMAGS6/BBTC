/**
 * @file
 * @brief EB1.1 inertial-vacuum six-degree-of-freedom derivative evaluation.
 *
 * This translation unit is intentionally a pure equations-of-motion layer. It
 * consumes already-sampled B6 frame/gravity data and already-assembled B8
 * physical loading, produces one B9 derivative, performs no provider dispatch,
 * and never advances state or time.
 */

#include <bbtc/external_ballistics/inertial_vacuum.h>

#include "math_internal.h"

#include <stddef.h>

#define BBTC_EB_DEFINE_INERTIAL_VACUUM_EVALUATOR(SUFFIX, SCALAR)          \
    bbtc_status_e                                                         \
    bbtc_eb_inertial_vacuum_derivative_evaluate_##SUFFIX(                \
        const bbtc_eb_rigid_body_state_##SUFFIX##_t* const state,        \
        const bbtc_eb_projectile_properties_##SUFFIX##_t* const projectile,\
        const bbtc_eb_reference_frame_sample_##SUFFIX##_t* const          \
            reference_frame,                                              \
        const bbtc_eb_force_moment_##SUFFIX##_t* const force_moment,     \
        bbtc_eb_rigid_body_state_derivative_##SUFFIX##_t* const          \
            out_derivative                                                \
    )                                                                     \
    {                                                                     \
        if (out_derivative == NULL)                                       \
            return BBTC_STATUS_INVALID_ARGUMENT;                          \
                                                                          \
        *out_derivative =                                                 \
            (bbtc_eb_rigid_body_state_derivative_##SUFFIX##_t){0};       \
                                                                          \
        if (state == NULL || projectile == NULL                           \
            || reference_frame == NULL || force_moment == NULL)          \
        {                                                                 \
            return BBTC_STATUS_INVALID_ARGUMENT;                          \
        }                                                                 \
                                                                          \
        bbtc_status_e status =                                            \
            bbtc_eb_rigid_body_state_validate_##SUFFIX(state);           \
        if (status != BBTC_STATUS_SUCCESS)                                \
            return status;                                                \
                                                                          \
        status = bbtc_eb_projectile_properties_validate_##SUFFIX(         \
            projectile                                                    \
        );                                                                \
        if (status != BBTC_STATUS_SUCCESS)                                \
            return status;                                                \
                                                                          \
        status = bbtc_eb_reference_frame_sample_validate_##SUFFIX(        \
            reference_frame                                               \
        );                                                                \
        if (status != BBTC_STATUS_SUCCESS)                                \
            return status;                                                \
                                                                          \
        status = bbtc_eb_force_moment_validate_##SUFFIX(force_moment);   \
        if (status != BBTC_STATUS_SUCCESS)                                \
            return status;                                                \
                                                                          \
        const SCALAR zero = (SCALAR)0;                                    \
        if (reference_frame                                               \
                ->reference_frame_origin_acceleration_reference_m_per_s2.x\
                != zero                                                   \
            || reference_frame                                            \
                ->reference_frame_origin_acceleration_reference_m_per_s2.y\
                != zero                                                   \
            || reference_frame                                            \
                ->reference_frame_origin_acceleration_reference_m_per_s2.z\
                != zero                                                   \
            || reference_frame                                            \
                ->reference_frame_angular_velocity_reference_rad_per_s.x  \
                != zero                                                   \
            || reference_frame                                            \
                ->reference_frame_angular_velocity_reference_rad_per_s.y  \
                != zero                                                   \
            || reference_frame                                            \
                ->reference_frame_angular_velocity_reference_rad_per_s.z  \
                != zero                                                   \
            || reference_frame                                            \
                ->reference_frame_angular_acceleration_reference_rad_per_s2.x\
                != zero                                                   \
            || reference_frame                                            \
                ->reference_frame_angular_acceleration_reference_rad_per_s2.y\
                != zero                                                   \
            || reference_frame                                            \
                ->reference_frame_angular_acceleration_reference_rad_per_s2.z\
                != zero)                                                  \
        {                                                                 \
            return BBTC_STATUS_UNSUPPORTED_MODEL_OR_OPTION;              \
        }                                                                 \
                                                                          \
        bbtc_eb_rigid_body_state_derivative_##SUFFIX##_t derivative = {0};\
                                                                          \
        derivative.position_derivative_reference_m_per_s =                \
            state->velocity_m_per_s;                                      \
                                                                          \
        const SCALAR mass = projectile->mass_kg;                          \
        derivative.velocity_derivative_reference_m_per_s2 =               \
            (bbtc_eb_vec3_##SUFFIX##_t)                                   \
            {                                                             \
                force_moment->applied_force_reference_n.x / mass          \
                    + reference_frame                                     \
                        ->gravitational_acceleration_reference_m_per_s2.x, \
                force_moment->applied_force_reference_n.y / mass          \
                    + reference_frame                                     \
                        ->gravitational_acceleration_reference_m_per_s2.y, \
                force_moment->applied_force_reference_n.z / mass          \
                    + reference_frame                                     \
                        ->gravitational_acceleration_reference_m_per_s2.z  \
            };                                                            \
                                                                          \
        const bbtc_eb_quat_##SUFFIX##_t omega_body_quaternion =           \
        {                                                                 \
            state->angular_velocity_body_rad_per_s.x,                     \
            state->angular_velocity_body_rad_per_s.y,                     \
            state->angular_velocity_body_rad_per_s.z,                     \
            zero                                                          \
        };                                                                \
        const bbtc_eb_quat_##SUFFIX##_t orientation_product =             \
            bbtc_eb_internal_quat_multiply_##SUFFIX(                      \
                state->orientation_world_from_body,                       \
                omega_body_quaternion                                     \
            );                                                            \
        const SCALAR half = (SCALAR)0.5;                                  \
        derivative.orientation_world_from_body_derivative_per_s =         \
            (bbtc_eb_quat_##SUFFIX##_t)                                   \
            {                                                             \
                orientation_product.x * half,                             \
                orientation_product.y * half,                             \
                orientation_product.z * half,                             \
                orientation_product.w * half                              \
            };                                                            \
                                                                          \
        const SCALAR ix =                                                 \
            projectile->principal_inertia.moment_about_x_kg_m2;          \
        const SCALAR iy =                                                 \
            projectile->principal_inertia.moment_about_y_kg_m2;          \
        const SCALAR iz =                                                 \
            projectile->principal_inertia.moment_about_z_kg_m2;          \
        const SCALAR omega_x = state->angular_velocity_body_rad_per_s.x;  \
        const SCALAR omega_y = state->angular_velocity_body_rad_per_s.y;  \
        const SCALAR omega_z = state->angular_velocity_body_rad_per_s.z;  \
        const SCALAR gyroscopic_x = (iz - iy) * omega_y * omega_z;       \
        const SCALAR gyroscopic_y = (ix - iz) * omega_z * omega_x;       \
        const SCALAR gyroscopic_z = (iy - ix) * omega_x * omega_y;       \
                                                                          \
        derivative.angular_velocity_derivative_body_rad_per_s2 =          \
            (bbtc_eb_vec3_##SUFFIX##_t)                                   \
            {                                                             \
                (force_moment->applied_moment_body_n_m.x - gyroscopic_x) \
                    / ix,                                                 \
                (force_moment->applied_moment_body_n_m.y - gyroscopic_y) \
                    / iy,                                                 \
                (force_moment->applied_moment_body_n_m.z - gyroscopic_z) \
                    / iz                                                  \
            };                                                            \
                                                                          \
        status = bbtc_eb_rigid_body_state_derivative_validate_##SUFFIX(   \
            &derivative                                                   \
        );                                                                \
        if (status != BBTC_STATUS_SUCCESS)                                \
            return BBTC_STATUS_NUMERICAL_FAILURE;                         \
                                                                          \
        *out_derivative = derivative;                                     \
        return BBTC_STATUS_SUCCESS;                                       \
    }

BBTC_EB_DEFINE_INERTIAL_VACUUM_EVALUATOR(float, float)
BBTC_EB_DEFINE_INERTIAL_VACUUM_EVALUATOR(double, double)
BBTC_EB_DEFINE_INERTIAL_VACUUM_EVALUATOR(long_double, long double)

#undef BBTC_EB_DEFINE_INERTIAL_VACUUM_EVALUATOR
