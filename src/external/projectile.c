/**
 * @file
 * @brief Fail-closed validation of EB projectile physical properties.
 *
 * Validation deliberately scans the complete record for NaN before scanning
 * for infinity, then applies physical-domain rules. This keeps diagnostic
 * precedence independent of declaration order.
 *
 * The principal-inertia triangle tests avoid forming a sum when that sum would
 * overflow the selected finite scalar family. If b + c exceeds the largest
 * finite value, no finite positive a can violate a <= b + c.
 */

#include <bbtc/external_ballistics/projectile.h>

#include <float.h>
#include <math.h>
#include <stddef.h>

#define BBTC_EB_DEFINE_PROJECTILE_PROPERTIES_VALIDATOR(                    \
    SUFFIX, SCALAR, PROPERTIES_T, MAX_VALUE                               \
)                                                                          \
    bbtc_status_e                                                          \
    bbtc_eb_projectile_properties_validate_##SUFFIX(                      \
        const PROPERTIES_T* const properties                              \
    )                                                                      \
    {                                                                      \
        if (properties == NULL)                                            \
            return BBTC_STATUS_INVALID_ARGUMENT;                           \
                                                                           \
        const SCALAR values[] =                                            \
        {                                                                  \
            properties->principal_inertia.moment_about_x_kg_m2,           \
            properties->principal_inertia.moment_about_y_kg_m2,           \
            properties->principal_inertia.moment_about_z_kg_m2,           \
            properties->mass_kg,                                          \
            properties->reference_diameter_m,                             \
            properties->reference_area_m2                                 \
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
        const SCALAR ix =                                                  \
            properties->principal_inertia.moment_about_x_kg_m2;           \
        const SCALAR iy =                                                  \
            properties->principal_inertia.moment_about_y_kg_m2;           \
        const SCALAR iz =                                                  \
            properties->principal_inertia.moment_about_z_kg_m2;           \
                                                                           \
        if (properties->mass_kg <= (SCALAR)0                              \
            || ix <= (SCALAR)0                                             \
            || iy <= (SCALAR)0                                             \
            || iz <= (SCALAR)0                                             \
            || properties->reference_diameter_m <= (SCALAR)0              \
            || properties->reference_area_m2 <= (SCALAR)0)                \
        {                                                                  \
            return BBTC_STATUS_OUTSIDE_DOMAIN;                             \
        }                                                                  \
                                                                           \
        if ((iy <= (SCALAR)(MAX_VALUE) - iz && ix > iy + iz)             \
            || (ix <= (SCALAR)(MAX_VALUE) - iz && iy > ix + iz)          \
            || (ix <= (SCALAR)(MAX_VALUE) - iy && iz > ix + iy))         \
        {                                                                  \
            return BBTC_STATUS_OUTSIDE_DOMAIN;                             \
        }                                                                  \
                                                                           \
        return BBTC_STATUS_SUCCESS;                                        \
    }

BBTC_EB_DEFINE_PROJECTILE_PROPERTIES_VALIDATOR(
    float,
    float,
    bbtc_eb_projectile_properties_float_t,
    FLT_MAX
)

BBTC_EB_DEFINE_PROJECTILE_PROPERTIES_VALIDATOR(
    double,
    double,
    bbtc_eb_projectile_properties_double_t,
    DBL_MAX
)

BBTC_EB_DEFINE_PROJECTILE_PROPERTIES_VALIDATOR(
    long_double,
    long double,
    bbtc_eb_projectile_properties_long_double_t,
    LDBL_MAX
)

#undef BBTC_EB_DEFINE_PROJECTILE_PROPERTIES_VALIDATOR
