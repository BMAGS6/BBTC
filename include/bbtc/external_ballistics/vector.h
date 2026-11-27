/**
 * @file
 * @brief Precision-qualified three-dimensional external-ballistics vectors.
 *
 * These records are intentionally small physics-interface value types. They
 * carry three scalar components but do not, by themselves, assign units or a
 * coordinate frame. The containing field supplies that physical meaning.
 *
 * BBTC owns these types so the standalone library does not depend on a host
 * application's vector library. A host such as Cubes of Honor converts values
 * explicitly at the adapter boundary instead of aliasing unrelated structure
 * types.
 */

#ifndef BBTC_EXTERNAL_BALLISTICS_VECTOR_H
#define BBTC_EXTERNAL_BALLISTICS_VECTOR_H

/**
 * @brief Native-float three-dimensional BBTC vector value.
 *
 * The `x`, `y`, and `z` members are declared in that order. Their units and
 * frame are intentionally unspecified here; for example, the same record type
 * may represent metres, metres per second, or radians per second when a
 * containing API field documents those semantics.
 *
 * EB0.1-B1 defines representation only. No normalization, arithmetic, or
 * validity guarantee is implied by this type.
 */
typedef struct bbtc_eb_vec3_float_t
{
    /** First vector component in the units and frame defined by the owner. */
    float x;

    /** Second vector component in the units and frame defined by the owner. */
    float y;

    /** Third vector component in the units and frame defined by the owner. */
    float z;
}
bbtc_eb_vec3_float_t;


/**
 * @brief Native-double three-dimensional BBTC vector value.
 *
 * This record has the same representation semantics as
 * `bbtc_eb_vec3_float_t` while preserving native `double` components.
 */
typedef struct bbtc_eb_vec3_double_t
{
    /** First vector component in the units and frame defined by the owner. */
    double x;

    /** Second vector component in the units and frame defined by the owner. */
    double y;

    /** Third vector component in the units and frame defined by the owner. */
    double z;
}
bbtc_eb_vec3_double_t;


/**
 * @brief Native-long-double three-dimensional BBTC vector value.
 *
 * This record has the same representation semantics as the other BBTC vector
 * families. It does not assume that `long double` is wider than `double` on
 * every supported platform.
 */
typedef struct bbtc_eb_vec3_long_double_t
{
    /** First vector component in the units and frame defined by the owner. */
    long double x;

    /** Second vector component in the units and frame defined by the owner. */
    long double y;

    /** Third vector component in the units and frame defined by the owner. */
    long double z;
}
bbtc_eb_vec3_long_double_t;

#endif /* BBTC_EXTERNAL_BALLISTICS_VECTOR_H */
