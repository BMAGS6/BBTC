/**
 * @file
 * @brief Validation of sampled EB0.1-B6 gravity/reference-frame values.
 *
 * Validation scans the complete record for NaN before scanning the complete
 * record for infinity. Every finite vector value is otherwise valid: B6 does
 * not infer Earth geometry, derive frame kinematics, normalize vectors, or
 * impose a relationship between gravity and reference-frame motion.
 */

#include <bbtc/external_ballistics/reference_frame.h>

#include <math.h>
#include <stddef.h>

#define BBTC_EB_DEFINE_REFERENCE_FRAME_SAMPLE_VALIDATOR(                   \
    SUFFIX, SCALAR, SAMPLE_T                                              \
)                                                                          \
    bbtc_status_e                                                          \
    bbtc_eb_reference_frame_sample_validate_##SUFFIX(                     \
        const SAMPLE_T* const sample                                      \
    )                                                                      \
    {                                                                      \
        if (sample == NULL)                                                \
            return BBTC_STATUS_INVALID_ARGUMENT;                           \
                                                                           \
        const SCALAR values[] =                                            \
        {                                                                  \
            sample->gravitational_acceleration_reference_m_per_s2.x,      \
            sample->gravitational_acceleration_reference_m_per_s2.y,      \
            sample->gravitational_acceleration_reference_m_per_s2.z,      \
            sample                                                         \
                ->reference_frame_origin_acceleration_reference_m_per_s2.x,\
            sample                                                         \
                ->reference_frame_origin_acceleration_reference_m_per_s2.y,\
            sample                                                         \
                ->reference_frame_origin_acceleration_reference_m_per_s2.z,\
            sample                                                         \
                ->reference_frame_angular_velocity_reference_rad_per_s.x, \
            sample                                                         \
                ->reference_frame_angular_velocity_reference_rad_per_s.y, \
            sample                                                         \
                ->reference_frame_angular_velocity_reference_rad_per_s.z, \
            sample                                                         \
                ->reference_frame_angular_acceleration_reference_rad_per_s2.x,\
            sample                                                         \
                ->reference_frame_angular_acceleration_reference_rad_per_s2.y,\
            sample                                                         \
                ->reference_frame_angular_acceleration_reference_rad_per_s2.z \
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

BBTC_EB_DEFINE_REFERENCE_FRAME_SAMPLE_VALIDATOR(
    float,
    float,
    bbtc_eb_reference_frame_sample_float_t
)

BBTC_EB_DEFINE_REFERENCE_FRAME_SAMPLE_VALIDATOR(
    double,
    double,
    bbtc_eb_reference_frame_sample_double_t
)

BBTC_EB_DEFINE_REFERENCE_FRAME_SAMPLE_VALIDATOR(
    long_double,
    long double,
    bbtc_eb_reference_frame_sample_long_double_t
)

#undef BBTC_EB_DEFINE_REFERENCE_FRAME_SAMPLE_VALIDATOR
