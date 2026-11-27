/**
 * @file
 * @brief Fail-closed validation of sampled EB local environments.
 *
 * Validation scans the complete record for NaN before scanning for infinity.
 * Physical-domain rules are applied only after both nonfinite passes. This
 * preserves the deterministic public diagnostic precedence independently of
 * declaration order or of which field first contains an invalid value.
 *
 * EB0.1-B4 intentionally performs no thermodynamic closure. Pressure, density,
 * temperature, and speed of sound are treated as caller-supplied model outputs.
 * Their cross-field consistency belongs to the environment provider rather
 * than to this value-validation boundary.
 */

#include <bbtc/external_ballistics/environment.h>

#include <math.h>
#include <stddef.h>

#define BBTC_EB_DEFINE_ENVIRONMENT_SAMPLE_VALIDATOR(                       \
    SUFFIX, SCALAR, SAMPLE_T                                              \
)                                                                          \
    bbtc_status_e                                                          \
    bbtc_eb_environment_sample_validate_##SUFFIX(                         \
        const SAMPLE_T* const sample                                      \
    )                                                                      \
    {                                                                      \
        if (sample == NULL)                                                \
            return BBTC_STATUS_INVALID_ARGUMENT;                           \
                                                                           \
        const SCALAR values[] =                                            \
        {                                                                  \
            sample->air_velocity_world_m_per_s.x,                         \
            sample->air_velocity_world_m_per_s.y,                         \
            sample->air_velocity_world_m_per_s.z,                         \
            sample->temperature_k,                                         \
            sample->absolute_pressure_pa,                                  \
            sample->air_density_kg_per_m3,                                 \
            sample->speed_of_sound_m_per_s                                 \
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
        if (sample->temperature_k <= (SCALAR)0                            \
            || sample->absolute_pressure_pa < (SCALAR)0                   \
            || sample->air_density_kg_per_m3 < (SCALAR)0                  \
            || sample->speed_of_sound_m_per_s < (SCALAR)0                 \
            || (sample->air_density_kg_per_m3 > (SCALAR)0                 \
                && sample->speed_of_sound_m_per_s == (SCALAR)0))          \
        {                                                                  \
            return BBTC_STATUS_OUTSIDE_DOMAIN;                             \
        }                                                                  \
                                                                           \
        return BBTC_STATUS_SUCCESS;                                        \
    }

BBTC_EB_DEFINE_ENVIRONMENT_SAMPLE_VALIDATOR(
    float,
    float,
    bbtc_eb_environment_sample_float_t
)

BBTC_EB_DEFINE_ENVIRONMENT_SAMPLE_VALIDATOR(
    double,
    double,
    bbtc_eb_environment_sample_double_t
)

BBTC_EB_DEFINE_ENVIRONMENT_SAMPLE_VALIDATOR(
    long_double,
    long double,
    bbtc_eb_environment_sample_long_double_t
)

#undef BBTC_EB_DEFINE_ENVIRONMENT_SAMPLE_VALIDATOR
