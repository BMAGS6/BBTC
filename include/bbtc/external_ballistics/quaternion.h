/**
 * @file
 * @brief Precision-qualified external-ballistics quaternion value records.
 *
 * BBTC uses active, right-handed Hamilton quaternions. The public record order
 * is `(x, y, z, w)`: `x`, `y`, and `z` are the vector part and `w` is the
 * scalar part.
 *
 * When a quaternion is used as a projectile orientation, BBTC interprets it as
 * world/reference-from-body. For a unit quaternion q_WB and body-frame vector
 * v_B, the corresponding world/reference-frame vector is conceptually
 *
 *     v_W = q_WB * (0, v_B) * conjugate(q_WB)
 *
 * EB0.1-B1 defines storage only. The type does not imply unit length, finite
 * components, or a valid physical orientation; those checks belong to a later
 * validation contract.
 */

#ifndef BBTC_EXTERNAL_BALLISTICS_QUATERNION_H
#define BBTC_EXTERNAL_BALLISTICS_QUATERNION_H

/**
 * @brief Native-float BBTC quaternion value.
 *
 * An all-zero record is representable storage but is not a valid unit
 * orientation. No identity fallback or implicit normalization occurs merely by
 * constructing this type.
 */
typedef struct bbtc_eb_quat_float_t
{
    /** X component of the quaternion vector part. */
    float x;

    /** Y component of the quaternion vector part. */
    float y;

    /** Z component of the quaternion vector part. */
    float z;

    /** Scalar component of the quaternion. */
    float w;
}
bbtc_eb_quat_float_t;


/**
 * @brief Native-double BBTC quaternion value.
 *
 * The Hamilton-product and active-rotation conventions match
 * `bbtc_eb_quat_float_t`; only the native scalar family differs.
 */
typedef struct bbtc_eb_quat_double_t
{
    /** X component of the quaternion vector part. */
    double x;

    /** Y component of the quaternion vector part. */
    double y;

    /** Z component of the quaternion vector part. */
    double z;

    /** Scalar component of the quaternion. */
    double w;
}
bbtc_eb_quat_double_t;


/**
 * @brief Native-long-double BBTC quaternion value.
 *
 * This record preserves `long double` components without routing storage
 * through another precision family. The platform is not required to provide a
 * `long double` format wider than `double`.
 */
typedef struct bbtc_eb_quat_long_double_t
{
    /** X component of the quaternion vector part. */
    long double x;

    /** Y component of the quaternion vector part. */
    long double y;

    /** Z component of the quaternion vector part. */
    long double z;

    /** Scalar component of the quaternion. */
    long double w;
}
bbtc_eb_quat_long_double_t;

#endif /* BBTC_EXTERNAL_BALLISTICS_QUATERNION_H */
