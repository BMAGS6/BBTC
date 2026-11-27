/**
 * @file
 * @brief Private precision-qualified vector and quaternion mathematics for EB.
 *
 * The implementation deliberately keeps these operations out of the public API
 * while giving later physics code one verified mathematical foundation.
 *
 * Normalization deserves special care. Computing `1 / norm` directly can
 * overflow for tiny but nonzero vectors even when each normalized component is
 * perfectly representable. The routines below therefore scale by the largest
 * component first, normalize the scaled value, and divide component-by-
 * component. The same strategy also avoids overflowing the sum of squares for
 * very large finite inputs.
 */

#include "math_internal.h"

#include <math.h>
#include <stddef.h>

/*
 * Generate one complete native scalar family. The macro is intentionally local
 * to this translation unit: callers see ordinary precision-qualified C
 * functions, while the common definition prevents subtle differences between
 * float, double, and long-double implementations.
 */
#define BBTC_EB_DEFINE_INTERNAL_MATH_FAMILY(                              \
    SUFFIX, SCALAR, VEC_T, QUAT_T, FABS_FN, FMAX_FN, HYPOT_FN           \
)                                                                        \
    VEC_T                                                                \
    bbtc_eb_internal_vec3_add_##SUFFIX(VEC_T a, VEC_T b)                \
    {                                                                    \
        return (VEC_T){ a.x + b.x, a.y + b.y, a.z + b.z };              \
    }                                                                    \
                                                                          \
    VEC_T                                                                \
    bbtc_eb_internal_vec3_subtract_##SUFFIX(VEC_T a, VEC_T b)           \
    {                                                                    \
        return (VEC_T){ a.x - b.x, a.y - b.y, a.z - b.z };              \
    }                                                                    \
                                                                          \
    VEC_T                                                                \
    bbtc_eb_internal_vec3_scale_##SUFFIX(VEC_T value, SCALAR scale)     \
    {                                                                    \
        return (VEC_T){                                                  \
            value.x * scale, value.y * scale, value.z * scale           \
        };                                                               \
    }                                                                    \
                                                                          \
    SCALAR                                                               \
    bbtc_eb_internal_vec3_dot_##SUFFIX(VEC_T a, VEC_T b)                \
    {                                                                    \
        return a.x * b.x + a.y * b.y + a.z * b.z;                       \
    }                                                                    \
                                                                          \
    VEC_T                                                                \
    bbtc_eb_internal_vec3_cross_##SUFFIX(VEC_T a, VEC_T b)              \
    {                                                                    \
        return (VEC_T){                                                  \
            a.y * b.z - a.z * b.y,                                      \
            a.z * b.x - a.x * b.z,                                      \
            a.x * b.y - a.y * b.x                                       \
        };                                                               \
    }                                                                    \
                                                                          \
    SCALAR                                                               \
    bbtc_eb_internal_vec3_norm_squared_##SUFFIX(VEC_T value)            \
    {                                                                    \
        return bbtc_eb_internal_vec3_dot_##SUFFIX(value, value);        \
    }                                                                    \
                                                                          \
    SCALAR                                                               \
    bbtc_eb_internal_vec3_norm_##SUFFIX(VEC_T value)                    \
    {                                                                    \
        return HYPOT_FN(HYPOT_FN(value.x, value.y), value.z);           \
    }                                                                    \
                                                                          \
    bbtc_status_e                                                        \
    bbtc_eb_internal_vec3_normalize_##SUFFIX(                           \
        const VEC_T* const input,                                        \
        VEC_T* const output                                              \
    )                                                                    \
    {                                                                    \
        if (output == NULL)                                              \
            return BBTC_STATUS_INVALID_ARGUMENT;                         \
                                                                          \
        if (input == NULL)                                               \
        {                                                                \
            *output = (VEC_T){0};                                        \
            return BBTC_STATUS_INVALID_ARGUMENT;                         \
        }                                                                \
                                                                          \
        const VEC_T value = *input;                                      \
        *output = (VEC_T){0};                                            \
                                                                          \
        if (isnan(value.x) || isnan(value.y) || isnan(value.z))         \
            return BBTC_STATUS_NAN_INPUT;                                \
                                                                          \
        if (isinf(value.x) || isinf(value.y) || isinf(value.z))         \
            return BBTC_STATUS_NONFINITE_INPUT;                          \
                                                                          \
        const SCALAR maximum = FMAX_FN(                                  \
            FABS_FN(value.x),                                            \
            FMAX_FN(FABS_FN(value.y), FABS_FN(value.z))                 \
        );                                                               \
                                                                          \
        if (maximum == (SCALAR)0)                                        \
            return BBTC_STATUS_OUTSIDE_DOMAIN;                           \
                                                                          \
        const VEC_T scaled =                                             \
        {                                                                \
            value.x / maximum,                                           \
            value.y / maximum,                                           \
            value.z / maximum                                            \
        };                                                               \
        const SCALAR scaled_norm = HYPOT_FN(                             \
            HYPOT_FN(scaled.x, scaled.y),                                \
            scaled.z                                                     \
        );                                                               \
                                                                          \
        output->x = scaled.x / scaled_norm;                              \
        output->y = scaled.y / scaled_norm;                              \
        output->z = scaled.z / scaled_norm;                              \
        return BBTC_STATUS_SUCCESS;                                      \
    }                                                                    \
                                                                          \
    QUAT_T                                                               \
    bbtc_eb_internal_quat_identity_##SUFFIX(void)                        \
    {                                                                    \
        return (QUAT_T){                                                 \
            (SCALAR)0, (SCALAR)0, (SCALAR)0, (SCALAR)1                  \
        };                                                               \
    }                                                                    \
                                                                          \
    QUAT_T                                                               \
    bbtc_eb_internal_quat_multiply_##SUFFIX(QUAT_T a, QUAT_T b)         \
    {                                                                    \
        return (QUAT_T){                                                 \
            a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,             \
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,             \
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,             \
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z              \
        };                                                               \
    }                                                                    \
                                                                          \
    QUAT_T                                                               \
    bbtc_eb_internal_quat_conjugate_##SUFFIX(QUAT_T value)              \
    {                                                                    \
        return (QUAT_T){ -value.x, -value.y, -value.z, value.w };       \
    }                                                                    \
                                                                          \
    SCALAR                                                               \
    bbtc_eb_internal_quat_norm_squared_##SUFFIX(QUAT_T value)           \
    {                                                                    \
        return value.x * value.x + value.y * value.y                    \
             + value.z * value.z + value.w * value.w;                   \
    }                                                                    \
                                                                          \
    SCALAR                                                               \
    bbtc_eb_internal_quat_norm_##SUFFIX(QUAT_T value)                   \
    {                                                                    \
        return HYPOT_FN(                                                 \
            HYPOT_FN(value.x, value.y),                                  \
            HYPOT_FN(value.z, value.w)                                   \
        );                                                               \
    }                                                                    \
                                                                          \
    bbtc_status_e                                                        \
    bbtc_eb_internal_quat_normalize_##SUFFIX(                           \
        const QUAT_T* const input,                                       \
        QUAT_T* const output                                             \
    )                                                                    \
    {                                                                    \
        if (output == NULL)                                              \
            return BBTC_STATUS_INVALID_ARGUMENT;                         \
                                                                          \
        if (input == NULL)                                               \
        {                                                                \
            *output = (QUAT_T){0};                                       \
            return BBTC_STATUS_INVALID_ARGUMENT;                         \
        }                                                                \
                                                                          \
        const QUAT_T value = *input;                                     \
        *output = (QUAT_T){0};                                           \
                                                                          \
        if (isnan(value.x) || isnan(value.y)                            \
            || isnan(value.z) || isnan(value.w))                        \
        {                                                                \
            return BBTC_STATUS_NAN_INPUT;                                \
        }                                                                \
                                                                          \
        if (isinf(value.x) || isinf(value.y)                            \
            || isinf(value.z) || isinf(value.w))                        \
        {                                                                \
            return BBTC_STATUS_NONFINITE_INPUT;                          \
        }                                                                \
                                                                          \
        const SCALAR maximum = FMAX_FN(                                  \
            FMAX_FN(FABS_FN(value.x), FABS_FN(value.y)),                \
            FMAX_FN(FABS_FN(value.z), FABS_FN(value.w))                 \
        );                                                               \
                                                                          \
        if (maximum == (SCALAR)0)                                        \
            return BBTC_STATUS_OUTSIDE_DOMAIN;                           \
                                                                          \
        const QUAT_T scaled =                                            \
        {                                                                \
            value.x / maximum,                                           \
            value.y / maximum,                                           \
            value.z / maximum,                                           \
            value.w / maximum                                            \
        };                                                               \
        const SCALAR scaled_norm = HYPOT_FN(                             \
            HYPOT_FN(scaled.x, scaled.y),                                \
            HYPOT_FN(scaled.z, scaled.w)                                 \
        );                                                               \
                                                                          \
        output->x = scaled.x / scaled_norm;                              \
        output->y = scaled.y / scaled_norm;                              \
        output->z = scaled.z / scaled_norm;                              \
        output->w = scaled.w / scaled_norm;                              \
        return BBTC_STATUS_SUCCESS;                                      \
    }                                                                    \
                                                                          \
    VEC_T                                                                \
    bbtc_eb_internal_quat_rotate_vec3_##SUFFIX(                          \
        QUAT_T rotation,                                                 \
        VEC_T vector                                                     \
    )                                                                    \
    {                                                                    \
        const VEC_T qv = { rotation.x, rotation.y, rotation.z };         \
        const VEC_T first_cross =                                        \
            bbtc_eb_internal_vec3_cross_##SUFFIX(qv, vector);           \
        const VEC_T second_cross =                                       \
            bbtc_eb_internal_vec3_cross_##SUFFIX(qv, first_cross);      \
        const VEC_T first_term =                                         \
            bbtc_eb_internal_vec3_scale_##SUFFIX(                       \
                first_cross,                                             \
                (SCALAR)2 * rotation.w                                   \
            );                                                           \
        const VEC_T second_term =                                        \
            bbtc_eb_internal_vec3_scale_##SUFFIX(                       \
                second_cross,                                            \
                (SCALAR)2                                                \
            );                                                           \
        return bbtc_eb_internal_vec3_add_##SUFFIX(                      \
            vector,                                                      \
            bbtc_eb_internal_vec3_add_##SUFFIX(first_term, second_term)\
        );                                                               \
    }

BBTC_EB_DEFINE_INTERNAL_MATH_FAMILY(
    float,
    float,
    bbtc_eb_vec3_float_t,
    bbtc_eb_quat_float_t,
    fabsf,
    fmaxf,
    hypotf
)

BBTC_EB_DEFINE_INTERNAL_MATH_FAMILY(
    double,
    double,
    bbtc_eb_vec3_double_t,
    bbtc_eb_quat_double_t,
    fabs,
    fmax,
    hypot
)

BBTC_EB_DEFINE_INTERNAL_MATH_FAMILY(
    long_double,
    long double,
    bbtc_eb_vec3_long_double_t,
    bbtc_eb_quat_long_double_t,
    fabsl,
    fmaxl,
    hypotl
)

#undef BBTC_EB_DEFINE_INTERNAL_MATH_FAMILY
