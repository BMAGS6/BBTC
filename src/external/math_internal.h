/**
 * @file
 * @brief Private precision-qualified vector and quaternion operations for EB.
 *
 * This header is an implementation boundary, not BBTC public API. It exists so
 * the solver and dedicated white-box tests can share the exact same
 * mathematical operations without exposing a general-purpose math library
 * through <bbtc/external_ballistics.h>.
 *
 * All functions operate natively in their named scalar family. No float family
 * is routed through double, and no long-double family is reduced to double.
 */

#ifndef BBTC_EXTERNAL_MATH_INTERNAL_H
#define BBTC_EXTERNAL_MATH_INTERNAL_H

#include <bbtc/external_ballistics/quaternion.h>
#include <bbtc/external_ballistics/vector.h>
#include <bbtc/status.h>

/*
 * The declarations are generated from one signature template so the three
 * precision families cannot silently drift apart. The generated function names
 * remain ordinary C symbols implemented in math.c.
 */
#define BBTC_EB_DECLARE_INTERNAL_MATH_FAMILY(SUFFIX, SCALAR, VEC_T, QUAT_T) \
    VEC_T                                                               \
    bbtc_eb_internal_vec3_add_##SUFFIX(VEC_T a, VEC_T b);              \
                                                                         \
    VEC_T                                                               \
    bbtc_eb_internal_vec3_subtract_##SUFFIX(VEC_T a, VEC_T b);         \
                                                                         \
    VEC_T                                                               \
    bbtc_eb_internal_vec3_scale_##SUFFIX(VEC_T value, SCALAR scale);   \
                                                                         \
    SCALAR                                                              \
    bbtc_eb_internal_vec3_dot_##SUFFIX(VEC_T a, VEC_T b);              \
                                                                         \
    VEC_T                                                               \
    bbtc_eb_internal_vec3_cross_##SUFFIX(VEC_T a, VEC_T b);            \
                                                                         \
    SCALAR                                                              \
    bbtc_eb_internal_vec3_norm_squared_##SUFFIX(VEC_T value);          \
                                                                         \
    SCALAR                                                              \
    bbtc_eb_internal_vec3_norm_##SUFFIX(VEC_T value);                  \
                                                                         \
    bbtc_status_e                                                       \
    bbtc_eb_internal_vec3_normalize_##SUFFIX(                           \
        const VEC_T* input,                                             \
        VEC_T* output                                                   \
    );                                                                  \
                                                                         \
    QUAT_T                                                              \
    bbtc_eb_internal_quat_identity_##SUFFIX(void);                      \
                                                                         \
    QUAT_T                                                              \
    bbtc_eb_internal_quat_multiply_##SUFFIX(QUAT_T left, QUAT_T right);\
                                                                         \
    QUAT_T                                                              \
    bbtc_eb_internal_quat_conjugate_##SUFFIX(QUAT_T value);            \
                                                                         \
    SCALAR                                                              \
    bbtc_eb_internal_quat_norm_squared_##SUFFIX(QUAT_T value);         \
                                                                         \
    SCALAR                                                              \
    bbtc_eb_internal_quat_norm_##SUFFIX(QUAT_T value);                 \
                                                                         \
    bbtc_status_e                                                       \
    bbtc_eb_internal_quat_normalize_##SUFFIX(                           \
        const QUAT_T* input,                                            \
        QUAT_T* output                                                  \
    );                                                                  \
                                                                         \
    VEC_T                                                               \
    bbtc_eb_internal_quat_rotate_vec3_##SUFFIX(                         \
        QUAT_T rotation,                                                \
        VEC_T vector                                                    \
    )

/**
 * @brief Declares the complete native-float private math family.
 *
 * Vector cross products are right-handed. Quaternion multiplication is the
 * Hamilton product. Quaternion-vector rotation is the active rotation
 * `q * (0, v) * conjugate(q)` for unit `q`.
 */
BBTC_EB_DECLARE_INTERNAL_MATH_FAMILY(
    float,
    float,
    bbtc_eb_vec3_float_t,
    bbtc_eb_quat_float_t
);

/**
 * @brief Declares the complete native-double private math family.
 */
BBTC_EB_DECLARE_INTERNAL_MATH_FAMILY(
    double,
    double,
    bbtc_eb_vec3_double_t,
    bbtc_eb_quat_double_t
);

/**
 * @brief Declares the complete native-long-double private math family.
 */
BBTC_EB_DECLARE_INTERNAL_MATH_FAMILY(
    long_double,
    long double,
    bbtc_eb_vec3_long_double_t,
    bbtc_eb_quat_long_double_t
);

#undef BBTC_EB_DECLARE_INTERNAL_MATH_FAMILY

#endif /* BBTC_EXTERNAL_MATH_INTERNAL_H */
