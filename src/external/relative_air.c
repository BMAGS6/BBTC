/**
 * @file
 * @brief EB2.1 projectile-relative-air derived-flow evaluation.
 *
 * The public equations are intentionally simple:
 *
 *   v_PA = v_PW - v_AW
 *   V    = |v_PA|
 *   M    = V / c                  for rho > 0
 *   M    = 0                      for rho == 0
 *   q    = 0.5 * rho * V^2
 *
 * The implementation work is mostly numerical and diagnostic discipline:
 *
 * - NaN must outrank infinity globally across both complete inputs;
 * - generated overflow must become BBTC_STATUS_NUMERICAL_FAILURE;
 * - the Euclidean norm must not overflow merely because x*x would overflow;
 * - dynamic pressure must not overflow or underflow merely because a poor
 *   multiplication order creates an avoidable intermediate; and
 * - caller output remains exact-zero until every derived quantity succeeds.
 *
 * Dynamic pressure is evaluated by decomposing positive density and speed into
 * native-family binary significands and exponents. The bounded significands are
 * multiplied safely, while powers of two are combined in integer exponent
 * space and applied once with scalbn(). This preserves representable high and
 * low results without routing arithmetic through another precision family.
 */

#include <bbtc/external_ballistics/relative_air.h>

#include <limits.h>
#include <math.h>
#include <stddef.h>


/**
 * @brief Define one native-family dynamic-pressure helper.
 *
 * `frexp()` represents a positive finite value as `m * 2^e` with
 * `0.5 <= m < 1`. Therefore:
 *
 *   0.5 * rho * V^2
 *     = (0.5 * m_rho * m_V * m_V) * 2^(e_rho + 2*e_V)
 *
 * The parenthesized significand lies safely between 1/16 and 1/2, so its own
 * multiplications cannot overflow or underflow. Only the final scalbn() decides
 * whether the mathematical result is representable. Final underflow to +0 is
 * valid EB2.1 success; final overflow to infinity is numerical failure.
 */
#define BBTC_EB_DEFINE_DYNAMIC_PRESSURE_HELPER(                            \
    SUFFIX, SCALAR, FREXP_FN, SCALBN_FN                                   \
)                                                                          \
    static bbtc_status_e                                                   \
    bbtc_eb_relative_air_dynamic_pressure_##SUFFIX(                        \
        SCALAR density_kg_per_m3,                                          \
        SCALAR relative_speed_m_per_s,                                     \
        SCALAR* const out_dynamic_pressure_pa                              \
    )                                                                      \
    {                                                                      \
        if (density_kg_per_m3 == (SCALAR)0                                \
            || relative_speed_m_per_s == (SCALAR)0)                       \
        {                                                                  \
            *out_dynamic_pressure_pa = (SCALAR)0;                          \
            return BBTC_STATUS_SUCCESS;                                    \
        }                                                                  \
                                                                           \
        int density_exponent = 0;                                          \
        int speed_exponent = 0;                                            \
        const SCALAR density_fraction =                                    \
            FREXP_FN(density_kg_per_m3, &density_exponent);                \
        const SCALAR speed_fraction =                                      \
            FREXP_FN(relative_speed_m_per_s, &speed_exponent);             \
                                                                           \
        const SCALAR pressure_fraction =                                   \
            ((SCALAR)0.5 * density_fraction)                              \
            * speed_fraction                                               \
            * speed_fraction;                                              \
                                                                           \
        const long long pressure_exponent =                                \
            (long long)density_exponent                                    \
            + 2LL * (long long)speed_exponent;                             \
                                                                           \
        if (pressure_exponent > (long long)INT_MAX)                        \
            return BBTC_STATUS_NUMERICAL_FAILURE;                          \
                                                                           \
        if (pressure_exponent < (long long)INT_MIN)                        \
        {                                                                  \
            *out_dynamic_pressure_pa = (SCALAR)0;                          \
            return BBTC_STATUS_SUCCESS;                                    \
        }                                                                  \
                                                                           \
        const SCALAR dynamic_pressure_pa =                                 \
            SCALBN_FN(                                                     \
                pressure_fraction,                                         \
                (int)pressure_exponent                                     \
            );                                                             \
                                                                           \
        if (!isfinite(dynamic_pressure_pa))                                \
            return BBTC_STATUS_NUMERICAL_FAILURE;                          \
                                                                           \
        *out_dynamic_pressure_pa = dynamic_pressure_pa;                    \
        return BBTC_STATUS_SUCCESS;                                        \
    }

BBTC_EB_DEFINE_DYNAMIC_PRESSURE_HELPER(float, float, frexpf, scalbnf)
BBTC_EB_DEFINE_DYNAMIC_PRESSURE_HELPER(double, double, frexp, scalbn)
BBTC_EB_DEFINE_DYNAMIC_PRESSURE_HELPER(
    long_double,
    long double,
    frexpl,
    scalbnl
)

#undef BBTC_EB_DEFINE_DYNAMIC_PRESSURE_HELPER


/**
 * @brief Define one complete native-family EB2.1 evaluator.
 *
 * The evaluator scans both complete inputs before selecting a diagnostic so an
 * environment NaN is not hidden by projectile-velocity infinity (or vice
 * versa). B4 validation is then reused for physical-domain policy only.
 *
 * All generated values are held in a local result. The caller's output is
 * committed only after relative components, norm, Mach, and dynamic pressure
 * have all succeeded.
 */
#define BBTC_EB_DEFINE_RELATIVE_AIR_EVALUATOR(                             \
    SUFFIX, SCALAR, VEC_T, ENV_T, RESULT_T, HYPOT_FN                      \
)                                                                          \
    bbtc_status_e                                                          \
    bbtc_eb_relative_air_evaluate_##SUFFIX(                                \
        const VEC_T* const projectile_velocity_world_m_per_s,              \
        const ENV_T* const environment,                                    \
        RESULT_T* const out_relative_air                                   \
    )                                                                      \
    {                                                                      \
        if (out_relative_air == NULL)                                      \
            return BBTC_STATUS_INVALID_ARGUMENT;                           \
                                                                           \
        *out_relative_air = (RESULT_T){0};                                 \
                                                                           \
        if (projectile_velocity_world_m_per_s == NULL                      \
            || environment == NULL)                                        \
        {                                                                  \
            return BBTC_STATUS_INVALID_ARGUMENT;                           \
        }                                                                  \
                                                                           \
        const SCALAR input_values[] =                                      \
        {                                                                  \
            projectile_velocity_world_m_per_s->x,                          \
            projectile_velocity_world_m_per_s->y,                          \
            projectile_velocity_world_m_per_s->z,                          \
            environment->air_velocity_world_m_per_s.x,                    \
            environment->air_velocity_world_m_per_s.y,                    \
            environment->air_velocity_world_m_per_s.z,                    \
            environment->temperature_k,                                    \
            environment->absolute_pressure_pa,                             \
            environment->air_density_kg_per_m3,                            \
            environment->speed_of_sound_m_per_s                            \
        };                                                                 \
                                                                           \
        for (size_t i = 0;                                                 \
             i < sizeof(input_values) / sizeof(input_values[0]);           \
             ++i)                                                          \
        {                                                                  \
            if (isnan(input_values[i]))                                    \
                return BBTC_STATUS_NAN_INPUT;                              \
        }                                                                  \
                                                                           \
        for (size_t i = 0;                                                 \
             i < sizeof(input_values) / sizeof(input_values[0]);           \
             ++i)                                                          \
        {                                                                  \
            if (isinf(input_values[i]))                                    \
                return BBTC_STATUS_NONFINITE_INPUT;                        \
        }                                                                  \
                                                                           \
        const bbtc_status_e environment_status =                           \
            bbtc_eb_environment_sample_validate_##SUFFIX(environment);     \
                                                                           \
        if (environment_status != BBTC_STATUS_SUCCESS)                     \
            return environment_status;                                     \
                                                                           \
        RESULT_T result = {0};                                             \
                                                                           \
        result.projectile_relative_to_air_velocity_world_m_per_s.x =       \
            projectile_velocity_world_m_per_s->x                           \
            - environment->air_velocity_world_m_per_s.x;                  \
        result.projectile_relative_to_air_velocity_world_m_per_s.y =       \
            projectile_velocity_world_m_per_s->y                           \
            - environment->air_velocity_world_m_per_s.y;                  \
        result.projectile_relative_to_air_velocity_world_m_per_s.z =       \
            projectile_velocity_world_m_per_s->z                           \
            - environment->air_velocity_world_m_per_s.z;                  \
                                                                           \
        if (!isfinite(                                                     \
                result                                                     \
                    .projectile_relative_to_air_velocity_world_m_per_s.x   \
            )                                                              \
            || !isfinite(                                                  \
                result                                                     \
                    .projectile_relative_to_air_velocity_world_m_per_s.y   \
            )                                                              \
            || !isfinite(                                                  \
                result                                                     \
                    .projectile_relative_to_air_velocity_world_m_per_s.z   \
            ))                                                             \
        {                                                                  \
            return BBTC_STATUS_NUMERICAL_FAILURE;                          \
        }                                                                  \
                                                                           \
        const SCALAR xy_norm = HYPOT_FN(                                  \
            result.projectile_relative_to_air_velocity_world_m_per_s.x,    \
            result.projectile_relative_to_air_velocity_world_m_per_s.y     \
        );                                                                 \
        const SCALAR relative_speed = HYPOT_FN(                            \
            xy_norm,                                                       \
            result.projectile_relative_to_air_velocity_world_m_per_s.z     \
        );                                                                 \
                                                                           \
        if (!isfinite(relative_speed))                                     \
            return BBTC_STATUS_NUMERICAL_FAILURE;                          \
                                                                           \
        result.projectile_relative_to_air_speed_m_per_s = relative_speed;  \
                                                                           \
        if (environment->air_density_kg_per_m3 == (SCALAR)0)              \
        {                                                                  \
            result.mach_number = (SCALAR)0;                                \
        }                                                                  \
        else                                                               \
        {                                                                  \
            result.mach_number =                                           \
                relative_speed / environment->speed_of_sound_m_per_s;      \
                                                                           \
            if (!isfinite(result.mach_number))                             \
                return BBTC_STATUS_NUMERICAL_FAILURE;                      \
        }                                                                  \
                                                                           \
        const bbtc_status_e pressure_status =                              \
            bbtc_eb_relative_air_dynamic_pressure_##SUFFIX(                \
                environment->air_density_kg_per_m3,                        \
                relative_speed,                                            \
                &result.dynamic_pressure_pa                                \
            );                                                             \
                                                                           \
        if (pressure_status != BBTC_STATUS_SUCCESS)                        \
            return pressure_status;                                        \
                                                                           \
        *out_relative_air = result;                                        \
        return BBTC_STATUS_SUCCESS;                                        \
    }

BBTC_EB_DEFINE_RELATIVE_AIR_EVALUATOR(
    float,
    float,
    bbtc_eb_vec3_float_t,
    bbtc_eb_environment_sample_float_t,
    bbtc_eb_relative_air_float_t,
    hypotf
)

BBTC_EB_DEFINE_RELATIVE_AIR_EVALUATOR(
    double,
    double,
    bbtc_eb_vec3_double_t,
    bbtc_eb_environment_sample_double_t,
    bbtc_eb_relative_air_double_t,
    hypot
)

BBTC_EB_DEFINE_RELATIVE_AIR_EVALUATOR(
    long_double,
    long double,
    bbtc_eb_vec3_long_double_t,
    bbtc_eb_environment_sample_long_double_t,
    bbtc_eb_relative_air_long_double_t,
    hypotl
)

#undef BBTC_EB_DEFINE_RELATIVE_AIR_EVALUATOR
