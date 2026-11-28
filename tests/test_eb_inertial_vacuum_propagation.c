/**
 * @file
 * @brief EB1.3 end-to-end analytic propagation validation.
 *
 * This file is deliberately more heavily documented than an ordinary unit
 * test. EB1.3 is a validation milestone: the purpose is not merely to execute
 * code paths, but to preserve the reasoning that connects BBTC output to
 * independent analytic solutions.
 *
 * The complete production pipeline exercised here is:
 *
 *   B11 fixed-step RK4
 *       -> B10 state-derivative dispatch
 *           -> EB1.2 inertial-vacuum provider composition
 *               -> B7 reference-frame sampling
 *               -> EB1.1 inertial-vacuum 6DOF equations
 *
 * Four independent analytic oracles are used:
 *
 * 1. zero-acceleration inertial drift;
 * 2. constant translational acceleration from B7 gravity plus B8 force;
 * 3. the simple harmonic oscillator produced by g_x = -x; and
 * 4. noncommuting constant body-axis spin with an exact quaternion solution.
 *
 * The first two are polynomial solutions and therefore use the tolerance
 * frozen in contract section 44.4. The latter two use three fresh fixed-step
 * runs and verify the fourth-order error decrease required by sections 44.7
 * and 44.8.
 *
 * Expected states are never manufactured by EB1.1, B10, or B11. Every oracle
 * is calculated directly from elementary mechanics and the matching native C
 * math functions. Float, double, and long double remain separate throughout.
 */

#include <bbtc/bbtc.h>

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>


/**
 * @brief Selects the deterministic gravitational field returned by test B7.
 *
 * The diagnostic sample counter is intentionally independent of this mode.
 * Returned physics therefore never depends on callback order or callback
 * history; the counter only records how many stage-local requests occurred.
 */
typedef enum frame_field_mode_e
{
    FRAME_FIELD_ZERO = 0,
    FRAME_FIELD_CONSTANT = 1,
    FRAME_FIELD_RESTORING_X = 2
}
frame_field_mode_e;


/**
 * @brief Native-float B7 context used by the analytic validation providers.
 */
typedef struct frame_context_float_t
{
    /** Number of B7 sampling requests observed during the current run. */
    size_t sample_count;

    /** Deterministic field model selected for this run. */
    frame_field_mode_e mode;

    /** Constant true gravity used only by FRAME_FIELD_CONSTANT. */
    bbtc_eb_vec3_float_t constant_gravity_m_per_s2;
}
frame_context_float_t;


/**
 * @brief Native-double B7 context used by the analytic validation providers.
 */
typedef struct frame_context_double_t
{
    /** Number of B7 sampling requests observed during the current run. */
    size_t sample_count;

    /** Deterministic field model selected for this run. */
    frame_field_mode_e mode;

    /** Constant true gravity used only by FRAME_FIELD_CONSTANT. */
    bbtc_eb_vec3_double_t constant_gravity_m_per_s2;
}
frame_context_double_t;


/**
 * @brief Native-long-double B7 context used by the analytic validation providers.
 */
typedef struct frame_context_long_double_t
{
    /** Number of B7 sampling requests observed during the current run. */
    size_t sample_count;

    /** Deterministic field model selected for this run. */
    frame_field_mode_e mode;

    /** Constant true gravity used only by FRAME_FIELD_CONSTANT. */
    bbtc_eb_vec3_long_double_t constant_gravity_m_per_s2;
}
frame_context_long_double_t;


/**
 * @brief Produce one native-float inertial reference-frame sample.
 *
 * Only true gravity is populated. Frame-origin acceleration, frame angular
 * velocity, and frame angular acceleration remain exactly zero so every sample
 * satisfies the accepted EB1.1 inertial-frame restriction.
 *
 * For FRAME_FIELD_RESTORING_X, the field is g_x = -x s^-2. This makes the
 * translational equation x'' = -x and therefore gives the independent exact
 * solution x(t)=cos(t), v_x(t)=-sin(t) for x(0)=1 and v_x(0)=0.
 */
static bbtc_status_e
sample_frame_float(
    const void* context,
    const bbtc_eb_vec3_float_t* position_reference_m,
    float time_s,
    bbtc_eb_reference_frame_sample_float_t* out_sample
)
{
    (void)time_s;

    if (context == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    frame_context_float_t* const frame =
        (frame_context_float_t*)context;

    ++frame->sample_count;
    *out_sample = (bbtc_eb_reference_frame_sample_float_t){0};

    switch (frame->mode)
    {
        case FRAME_FIELD_ZERO:
            return BBTC_STATUS_SUCCESS;

        case FRAME_FIELD_CONSTANT:
            out_sample->gravitational_acceleration_reference_m_per_s2 =
                frame->constant_gravity_m_per_s2;
            return BBTC_STATUS_SUCCESS;

        case FRAME_FIELD_RESTORING_X:
            out_sample->gravitational_acceleration_reference_m_per_s2.x =
                -position_reference_m->x;
            return BBTC_STATUS_SUCCESS;
    }

    return BBTC_STATUS_INTERNAL_INVARIANT_FAILURE;
}


/**
 * @brief Native-double counterpart of sample_frame_float().
 *
 * No value is converted through float or long double; the callback samples and
 * returns only native double values.
 */
static bbtc_status_e
sample_frame_double(
    const void* context,
    const bbtc_eb_vec3_double_t* position_reference_m,
    double time_s,
    bbtc_eb_reference_frame_sample_double_t* out_sample
)
{
    (void)time_s;

    if (context == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    frame_context_double_t* const frame =
        (frame_context_double_t*)context;

    ++frame->sample_count;
    *out_sample = (bbtc_eb_reference_frame_sample_double_t){0};

    switch (frame->mode)
    {
        case FRAME_FIELD_ZERO:
            return BBTC_STATUS_SUCCESS;

        case FRAME_FIELD_CONSTANT:
            out_sample->gravitational_acceleration_reference_m_per_s2 =
                frame->constant_gravity_m_per_s2;
            return BBTC_STATUS_SUCCESS;

        case FRAME_FIELD_RESTORING_X:
            out_sample->gravitational_acceleration_reference_m_per_s2.x =
                -position_reference_m->x;
            return BBTC_STATUS_SUCCESS;
    }

    return BBTC_STATUS_INTERNAL_INVARIANT_FAILURE;
}


/**
 * @brief Native-long-double counterpart of sample_frame_float().
 *
 * Long-double sampling remains native even on platforms where long double has
 * the same representation as double.
 */
static bbtc_status_e
sample_frame_long_double(
    const void* context,
    const bbtc_eb_vec3_long_double_t* position_reference_m,
    long double time_s,
    bbtc_eb_reference_frame_sample_long_double_t* out_sample
)
{
    (void)time_s;

    if (context == NULL)
        return BBTC_STATUS_INVALID_ARGUMENT;

    frame_context_long_double_t* const frame =
        (frame_context_long_double_t*)context;

    ++frame->sample_count;
    *out_sample = (bbtc_eb_reference_frame_sample_long_double_t){0};

    switch (frame->mode)
    {
        case FRAME_FIELD_ZERO:
            return BBTC_STATUS_SUCCESS;

        case FRAME_FIELD_CONSTANT:
            out_sample->gravitational_acceleration_reference_m_per_s2 =
                frame->constant_gravity_m_per_s2;
            return BBTC_STATUS_SUCCESS;

        case FRAME_FIELD_RESTORING_X:
            out_sample->gravitational_acceleration_reference_m_per_s2.x =
                -position_reference_m->x;
            return BBTC_STATUS_SUCCESS;
    }

    return BBTC_STATUS_INTERNAL_INVARIANT_FAILURE;
}


/**
 * @brief Construct the native-float spherical projectile used by EB1.3.
 *
 * Spherical principal inertia makes zero applied moment imply constant body
 * angular velocity. Mass is exactly 2 kg so Oracle B's F/m term is simple and
 * independently auditable. Reference geometry is positive solely to satisfy
 * the accepted B3 validity contract; aerodynamics are absent from EB1.3.
 */
static bbtc_eb_projectile_properties_float_t
projectile_float(void)
{
    bbtc_eb_projectile_properties_float_t projectile = {0};

    projectile.principal_inertia.moment_about_x_kg_m2 = 2.0f;
    projectile.principal_inertia.moment_about_y_kg_m2 = 2.0f;
    projectile.principal_inertia.moment_about_z_kg_m2 = 2.0f;
    projectile.mass_kg = 2.0f;
    projectile.reference_diameter_m = 1.0f;
    projectile.reference_area_m2 = 1.0f;

    return projectile;
}


/**
 * @brief Native-double counterpart of projectile_float().
 */
static bbtc_eb_projectile_properties_double_t
projectile_double(void)
{
    bbtc_eb_projectile_properties_double_t projectile = {0};

    projectile.principal_inertia.moment_about_x_kg_m2 = 2.0;
    projectile.principal_inertia.moment_about_y_kg_m2 = 2.0;
    projectile.principal_inertia.moment_about_z_kg_m2 = 2.0;
    projectile.mass_kg = 2.0;
    projectile.reference_diameter_m = 1.0;
    projectile.reference_area_m2 = 1.0;

    return projectile;
}


/**
 * @brief Native-long-double counterpart of projectile_float().
 */
static bbtc_eb_projectile_properties_long_double_t
projectile_long_double(void)
{
    bbtc_eb_projectile_properties_long_double_t projectile = {0};

    projectile.principal_inertia.moment_about_x_kg_m2 = 2.0L;
    projectile.principal_inertia.moment_about_y_kg_m2 = 2.0L;
    projectile.principal_inertia.moment_about_z_kg_m2 = 2.0L;
    projectile.mass_kg = 2.0L;
    projectile.reference_diameter_m = 1.0L;
    projectile.reference_area_m2 = 1.0L;

    return projectile;
}


/**
 * @brief Construct one native-float EB1.2 configuration for a test run.
 *
 * The configuration owns projectile and B8 values by value while borrowing the
 * test-owned B7 context exactly as required by the accepted EB1.2 contract.
 */
static bbtc_eb_inertial_vacuum_provider_config_float_t
config_float(
    frame_context_float_t* context,
    float force_x_n,
    float force_y_n,
    float force_z_n
)
{
    bbtc_eb_inertial_vacuum_provider_config_float_t config = {0};

    config.projectile = projectile_float();
    config.constant_force_moment.applied_force_reference_n.x = force_x_n;
    config.constant_force_moment.applied_force_reference_n.y = force_y_n;
    config.constant_force_moment.applied_force_reference_n.z = force_z_n;
    config.reference_frame_provider.sample = sample_frame_float;
    config.reference_frame_provider.context = context;

    return config;
}


/**
 * @brief Native-double counterpart of config_float().
 */
static bbtc_eb_inertial_vacuum_provider_config_double_t
config_double(
    frame_context_double_t* context,
    double force_x_n,
    double force_y_n,
    double force_z_n
)
{
    bbtc_eb_inertial_vacuum_provider_config_double_t config = {0};

    config.projectile = projectile_double();
    config.constant_force_moment.applied_force_reference_n.x = force_x_n;
    config.constant_force_moment.applied_force_reference_n.y = force_y_n;
    config.constant_force_moment.applied_force_reference_n.z = force_z_n;
    config.reference_frame_provider.sample = sample_frame_double;
    config.reference_frame_provider.context = context;

    return config;
}


/**
 * @brief Native-long-double counterpart of config_float().
 */
static bbtc_eb_inertial_vacuum_provider_config_long_double_t
config_long_double(
    frame_context_long_double_t* context,
    long double force_x_n,
    long double force_y_n,
    long double force_z_n
)
{
    bbtc_eb_inertial_vacuum_provider_config_long_double_t config = {0};

    config.projectile = projectile_long_double();
    config.constant_force_moment.applied_force_reference_n.x = force_x_n;
    config.constant_force_moment.applied_force_reference_n.y = force_y_n;
    config.constant_force_moment.applied_force_reference_n.z = force_z_n;
    config.reference_frame_provider.sample = sample_frame_long_double;
    config.reference_frame_provider.context = context;

    return config;
}


/**
 * @brief Advance one native-float state through a fixed number of B11 steps.
 *
 * This is intentionally test-local infrastructure rather than a BBTC public
 * propagation API. It stops immediately on the first B11 failure and propagates
 * that status unchanged, matching contract section 44.9.
 */
static bbtc_status_e
propagate_float(
    const bbtc_eb_state_derivative_provider_float_t* provider,
    float step_s,
    size_t step_count,
    bbtc_eb_rigid_body_state_float_t* state
)
{
    for (size_t step = 0; step < step_count; ++step)
    {
        const bbtc_status_e status =
            bbtc_eb_rk4_step_float(provider, step_s, state);

        if (status != BBTC_STATUS_SUCCESS)
            return status;
    }

    return BBTC_STATUS_SUCCESS;
}


/**
 * @brief Native-double counterpart of propagate_float().
 */
static bbtc_status_e
propagate_double(
    const bbtc_eb_state_derivative_provider_double_t* provider,
    double step_s,
    size_t step_count,
    bbtc_eb_rigid_body_state_double_t* state
)
{
    for (size_t step = 0; step < step_count; ++step)
    {
        const bbtc_status_e status =
            bbtc_eb_rk4_step_double(provider, step_s, state);

        if (status != BBTC_STATUS_SUCCESS)
            return status;
    }

    return BBTC_STATUS_SUCCESS;
}


/**
 * @brief Native-long-double counterpart of propagate_float().
 */
static bbtc_status_e
propagate_long_double(
    const bbtc_eb_state_derivative_provider_long_double_t* provider,
    long double step_s,
    size_t step_count,
    bbtc_eb_rigid_body_state_long_double_t* state
)
{
    for (size_t step = 0; step < step_count; ++step)
    {
        const bbtc_status_e status =
            bbtc_eb_rk4_step_long_double(provider, step_s, state);

        if (status != BBTC_STATUS_SUCCESS)
            return status;
    }

    return BBTC_STATUS_SUCCESS;
}


/**
 * @brief Apply the contract section 44.4 native-float polynomial tolerance.
 */
static int
polynomial_close_float(float actual, float expected)
{
    if (!isfinite(actual) || !isfinite(expected))
        return 0;

    const float scale = fmaxf(1.0f, fabsf(expected));
    const float tolerance = 4096.0f * FLT_EPSILON * scale;

    return fabsf(actual - expected) <= tolerance;
}


/**
 * @brief Apply the contract section 44.4 native-double polynomial tolerance.
 */
static int
polynomial_close_double(double actual, double expected)
{
    if (!isfinite(actual) || !isfinite(expected))
        return 0;

    const double scale = fmax(1.0, fabs(expected));
    const double tolerance = 4096.0 * DBL_EPSILON * scale;

    return fabs(actual - expected) <= tolerance;
}


/**
 * @brief Apply the contract section 44.4 native-long-double tolerance.
 */
static int
polynomial_close_long_double(long double actual, long double expected)
{
    if (!isfinite(actual) || !isfinite(expected))
        return 0;

    const long double scale = fmaxl(1.0L, fabsl(expected));
    const long double tolerance = 4096.0L * LDBL_EPSILON * scale;

    return fabsl(actual - expected) <= tolerance;
}


/**
 * @brief Validate Oracle A for the native-float pipeline.
 *
 * With zero gravity, zero force, zero moment, and zero angular velocity, the
 * exact solution is constant velocity and linear position:
 *
 *   r(T) = r0 + v0*T
 *   v(T) = v0
 *
 * Eight h=0.25 s RK4 steps cover T=2 s and must trigger exactly 32 B7 samples.
 */
static int
test_drift_float(void)
{
    frame_context_float_t frame =
    {
        .sample_count = 0,
        .mode = FRAME_FIELD_ZERO,
        .constant_gravity_m_per_s2 = {0}
    };
    const bbtc_eb_inertial_vacuum_provider_config_float_t config =
        config_float(&frame, 0.0f, 0.0f, 0.0f);
    bbtc_eb_state_derivative_provider_float_t provider = {0};
    bbtc_eb_rigid_body_state_float_t state = {0};

    state.orientation_world_from_body.w = 1.0f;
    state.position_m.x = 1.0f;
    state.position_m.y = -2.0f;
    state.position_m.z = 3.0f;
    state.velocity_m_per_s.x = 4.0f;
    state.velocity_m_per_s.y = -6.0f;
    state.velocity_m_per_s.z = 8.0f;

    if (bbtc_eb_inertial_vacuum_provider_bind_float(&config, &provider)
            != BBTC_STATUS_SUCCESS
        || propagate_float(&provider, 0.25f, 8, &state)
            != BBTC_STATUS_SUCCESS
        || frame.sample_count != 32
        || bbtc_eb_rigid_body_state_validate_float(&state)
            != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    return polynomial_close_float(state.position_m.x, 9.0f)
            && polynomial_close_float(state.position_m.y, -14.0f)
            && polynomial_close_float(state.position_m.z, 19.0f)
            && polynomial_close_float(state.velocity_m_per_s.x, 4.0f)
            && polynomial_close_float(state.velocity_m_per_s.y, -6.0f)
            && polynomial_close_float(state.velocity_m_per_s.z, 8.0f)
            && polynomial_close_float(
                state.orientation_world_from_body.x,
                0.0f
            )
            && polynomial_close_float(
                state.orientation_world_from_body.y,
                0.0f
            )
            && polynomial_close_float(
                state.orientation_world_from_body.z,
                0.0f
            )
            && polynomial_close_float(
                state.orientation_world_from_body.w,
                1.0f
            )
            && polynomial_close_float(
                state.angular_velocity_body_rad_per_s.x,
                0.0f
            )
            && polynomial_close_float(
                state.angular_velocity_body_rad_per_s.y,
                0.0f
            )
            && polynomial_close_float(
                state.angular_velocity_body_rad_per_s.z,
                0.0f
            )
            && polynomial_close_float(state.time_s, 2.0f)
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Validate Oracle A for the native-double pipeline.
 */
static int
test_drift_double(void)
{
    frame_context_double_t frame =
    {
        .sample_count = 0,
        .mode = FRAME_FIELD_ZERO,
        .constant_gravity_m_per_s2 = {0}
    };
    const bbtc_eb_inertial_vacuum_provider_config_double_t config =
        config_double(&frame, 0.0, 0.0, 0.0);
    bbtc_eb_state_derivative_provider_double_t provider = {0};
    bbtc_eb_rigid_body_state_double_t state = {0};

    state.orientation_world_from_body.w = 1.0;
    state.position_m.x = 1.0;
    state.position_m.y = -2.0;
    state.position_m.z = 3.0;
    state.velocity_m_per_s.x = 4.0;
    state.velocity_m_per_s.y = -6.0;
    state.velocity_m_per_s.z = 8.0;

    if (bbtc_eb_inertial_vacuum_provider_bind_double(&config, &provider)
            != BBTC_STATUS_SUCCESS
        || propagate_double(&provider, 0.25, 8, &state)
            != BBTC_STATUS_SUCCESS
        || frame.sample_count != 32
        || bbtc_eb_rigid_body_state_validate_double(&state)
            != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    return polynomial_close_double(state.position_m.x, 9.0)
            && polynomial_close_double(state.position_m.y, -14.0)
            && polynomial_close_double(state.position_m.z, 19.0)
            && polynomial_close_double(state.velocity_m_per_s.x, 4.0)
            && polynomial_close_double(state.velocity_m_per_s.y, -6.0)
            && polynomial_close_double(state.velocity_m_per_s.z, 8.0)
            && polynomial_close_double(
                state.orientation_world_from_body.x,
                0.0
            )
            && polynomial_close_double(
                state.orientation_world_from_body.y,
                0.0
            )
            && polynomial_close_double(
                state.orientation_world_from_body.z,
                0.0
            )
            && polynomial_close_double(
                state.orientation_world_from_body.w,
                1.0
            )
            && polynomial_close_double(
                state.angular_velocity_body_rad_per_s.x,
                0.0
            )
            && polynomial_close_double(
                state.angular_velocity_body_rad_per_s.y,
                0.0
            )
            && polynomial_close_double(
                state.angular_velocity_body_rad_per_s.z,
                0.0
            )
            && polynomial_close_double(state.time_s, 2.0)
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Validate Oracle A for the native-long-double pipeline.
 */
static int
test_drift_long_double(void)
{
    frame_context_long_double_t frame =
    {
        .sample_count = 0,
        .mode = FRAME_FIELD_ZERO,
        .constant_gravity_m_per_s2 = {0}
    };
    const bbtc_eb_inertial_vacuum_provider_config_long_double_t config =
        config_long_double(&frame, 0.0L, 0.0L, 0.0L);
    bbtc_eb_state_derivative_provider_long_double_t provider = {0};
    bbtc_eb_rigid_body_state_long_double_t state = {0};

    state.orientation_world_from_body.w = 1.0L;
    state.position_m.x = 1.0L;
    state.position_m.y = -2.0L;
    state.position_m.z = 3.0L;
    state.velocity_m_per_s.x = 4.0L;
    state.velocity_m_per_s.y = -6.0L;
    state.velocity_m_per_s.z = 8.0L;

    if (bbtc_eb_inertial_vacuum_provider_bind_long_double(
            &config,
            &provider
        ) != BBTC_STATUS_SUCCESS
        || propagate_long_double(&provider, 0.25L, 8, &state)
            != BBTC_STATUS_SUCCESS
        || frame.sample_count != 32
        || bbtc_eb_rigid_body_state_validate_long_double(&state)
            != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    return polynomial_close_long_double(state.position_m.x, 9.0L)
            && polynomial_close_long_double(state.position_m.y, -14.0L)
            && polynomial_close_long_double(state.position_m.z, 19.0L)
            && polynomial_close_long_double(
                state.velocity_m_per_s.x,
                4.0L
            )
            && polynomial_close_long_double(
                state.velocity_m_per_s.y,
                -6.0L
            )
            && polynomial_close_long_double(
                state.velocity_m_per_s.z,
                8.0L
            )
            && polynomial_close_long_double(
                state.orientation_world_from_body.x,
                0.0L
            )
            && polynomial_close_long_double(
                state.orientation_world_from_body.y,
                0.0L
            )
            && polynomial_close_long_double(
                state.orientation_world_from_body.z,
                0.0L
            )
            && polynomial_close_long_double(
                state.orientation_world_from_body.w,
                1.0L
            )
            && polynomial_close_long_double(
                state.angular_velocity_body_rad_per_s.x,
                0.0L
            )
            && polynomial_close_long_double(
                state.angular_velocity_body_rad_per_s.y,
                0.0L
            )
            && polynomial_close_long_double(
                state.angular_velocity_body_rad_per_s.z,
                0.0L
            )
            && polynomial_close_long_double(state.time_s, 2.0L)
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Validate Oracle B for native float.
 *
 * Gravity and force are intentionally supplied through different accepted
 * boundaries so this test validates their EB1.1 composition:
 *
 *   a = g + F/m
 *     = (0.5, 1, -1) + (2, -4, 6)/2
 *     = (1.5, -1, 2) m/s^2.
 *
 * The independent exact solution at T=2 s is:
 *
 *   v(T) = (7, -4, 5) m/s
 *   r(T) = (12, -4, 9) m.
 */
static int
test_constant_acceleration_float(void)
{
    frame_context_float_t frame =
    {
        .sample_count = 0,
        .mode = FRAME_FIELD_CONSTANT,
        .constant_gravity_m_per_s2 = {.x = 0.5f, .y = 1.0f, .z = -1.0f}
    };
    const bbtc_eb_inertial_vacuum_provider_config_float_t config =
        config_float(&frame, 2.0f, -4.0f, 6.0f);
    bbtc_eb_state_derivative_provider_float_t provider = {0};
    bbtc_eb_rigid_body_state_float_t state = {0};

    state.orientation_world_from_body.w = 1.0f;
    state.position_m.x = 1.0f;
    state.position_m.y = 2.0f;
    state.position_m.z = 3.0f;
    state.velocity_m_per_s.x = 4.0f;
    state.velocity_m_per_s.y = -2.0f;
    state.velocity_m_per_s.z = 1.0f;

    if (bbtc_eb_inertial_vacuum_provider_bind_float(&config, &provider)
            != BBTC_STATUS_SUCCESS
        || propagate_float(&provider, 0.25f, 8, &state)
            != BBTC_STATUS_SUCCESS
        || frame.sample_count != 32
        || bbtc_eb_rigid_body_state_validate_float(&state)
            != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    return polynomial_close_float(state.position_m.x, 12.0f)
            && polynomial_close_float(state.position_m.y, -4.0f)
            && polynomial_close_float(state.position_m.z, 9.0f)
            && polynomial_close_float(state.velocity_m_per_s.x, 7.0f)
            && polynomial_close_float(state.velocity_m_per_s.y, -4.0f)
            && polynomial_close_float(state.velocity_m_per_s.z, 5.0f)
            && polynomial_close_float(
                state.orientation_world_from_body.x,
                0.0f
            )
            && polynomial_close_float(
                state.orientation_world_from_body.y,
                0.0f
            )
            && polynomial_close_float(
                state.orientation_world_from_body.z,
                0.0f
            )
            && polynomial_close_float(
                state.orientation_world_from_body.w,
                1.0f
            )
            && polynomial_close_float(
                state.angular_velocity_body_rad_per_s.x,
                0.0f
            )
            && polynomial_close_float(
                state.angular_velocity_body_rad_per_s.y,
                0.0f
            )
            && polynomial_close_float(
                state.angular_velocity_body_rad_per_s.z,
                0.0f
            )
            && polynomial_close_float(state.time_s, 2.0f)
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Validate Oracle B for native double.
 */
static int
test_constant_acceleration_double(void)
{
    frame_context_double_t frame =
    {
        .sample_count = 0,
        .mode = FRAME_FIELD_CONSTANT,
        .constant_gravity_m_per_s2 = {.x = 0.5, .y = 1.0, .z = -1.0}
    };
    const bbtc_eb_inertial_vacuum_provider_config_double_t config =
        config_double(&frame, 2.0, -4.0, 6.0);
    bbtc_eb_state_derivative_provider_double_t provider = {0};
    bbtc_eb_rigid_body_state_double_t state = {0};

    state.orientation_world_from_body.w = 1.0;
    state.position_m.x = 1.0;
    state.position_m.y = 2.0;
    state.position_m.z = 3.0;
    state.velocity_m_per_s.x = 4.0;
    state.velocity_m_per_s.y = -2.0;
    state.velocity_m_per_s.z = 1.0;

    if (bbtc_eb_inertial_vacuum_provider_bind_double(&config, &provider)
            != BBTC_STATUS_SUCCESS
        || propagate_double(&provider, 0.25, 8, &state)
            != BBTC_STATUS_SUCCESS
        || frame.sample_count != 32
        || bbtc_eb_rigid_body_state_validate_double(&state)
            != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    return polynomial_close_double(state.position_m.x, 12.0)
            && polynomial_close_double(state.position_m.y, -4.0)
            && polynomial_close_double(state.position_m.z, 9.0)
            && polynomial_close_double(state.velocity_m_per_s.x, 7.0)
            && polynomial_close_double(state.velocity_m_per_s.y, -4.0)
            && polynomial_close_double(state.velocity_m_per_s.z, 5.0)
            && polynomial_close_double(
                state.orientation_world_from_body.x,
                0.0
            )
            && polynomial_close_double(
                state.orientation_world_from_body.y,
                0.0
            )
            && polynomial_close_double(
                state.orientation_world_from_body.z,
                0.0
            )
            && polynomial_close_double(
                state.orientation_world_from_body.w,
                1.0
            )
            && polynomial_close_double(
                state.angular_velocity_body_rad_per_s.x,
                0.0
            )
            && polynomial_close_double(
                state.angular_velocity_body_rad_per_s.y,
                0.0
            )
            && polynomial_close_double(
                state.angular_velocity_body_rad_per_s.z,
                0.0
            )
            && polynomial_close_double(state.time_s, 2.0)
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Validate Oracle B for native long double.
 */
static int
test_constant_acceleration_long_double(void)
{
    frame_context_long_double_t frame =
    {
        .sample_count = 0,
        .mode = FRAME_FIELD_CONSTANT,
        .constant_gravity_m_per_s2 = {.x = 0.5L, .y = 1.0L, .z = -1.0L}
    };
    const bbtc_eb_inertial_vacuum_provider_config_long_double_t config =
        config_long_double(&frame, 2.0L, -4.0L, 6.0L);
    bbtc_eb_state_derivative_provider_long_double_t provider = {0};
    bbtc_eb_rigid_body_state_long_double_t state = {0};

    state.orientation_world_from_body.w = 1.0L;
    state.position_m.x = 1.0L;
    state.position_m.y = 2.0L;
    state.position_m.z = 3.0L;
    state.velocity_m_per_s.x = 4.0L;
    state.velocity_m_per_s.y = -2.0L;
    state.velocity_m_per_s.z = 1.0L;

    if (bbtc_eb_inertial_vacuum_provider_bind_long_double(
            &config,
            &provider
        ) != BBTC_STATUS_SUCCESS
        || propagate_long_double(&provider, 0.25L, 8, &state)
            != BBTC_STATUS_SUCCESS
        || frame.sample_count != 32
        || bbtc_eb_rigid_body_state_validate_long_double(&state)
            != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    return polynomial_close_long_double(state.position_m.x, 12.0L)
            && polynomial_close_long_double(state.position_m.y, -4.0L)
            && polynomial_close_long_double(state.position_m.z, 9.0L)
            && polynomial_close_long_double(
                state.velocity_m_per_s.x,
                7.0L
            )
            && polynomial_close_long_double(
                state.velocity_m_per_s.y,
                -4.0L
            )
            && polynomial_close_long_double(
                state.velocity_m_per_s.z,
                5.0L
            )
            && polynomial_close_long_double(
                state.orientation_world_from_body.x,
                0.0L
            )
            && polynomial_close_long_double(
                state.orientation_world_from_body.y,
                0.0L
            )
            && polynomial_close_long_double(
                state.orientation_world_from_body.z,
                0.0L
            )
            && polynomial_close_long_double(
                state.orientation_world_from_body.w,
                1.0L
            )
            && polynomial_close_long_double(
                state.angular_velocity_body_rad_per_s.x,
                0.0L
            )
            && polynomial_close_long_double(
                state.angular_velocity_body_rad_per_s.y,
                0.0L
            )
            && polynomial_close_long_double(
                state.angular_velocity_body_rad_per_s.z,
                0.0L
            )
            && polynomial_close_long_double(state.time_s, 2.0L)
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Execute one fresh native-float restoring-gravity convergence run.
 *
 * Each invocation constructs an independent state, provider configuration, and
 * B7 context. This prevents a finer run from inheriting numerical error from a
 * coarser run and satisfies the freshness requirement in contract section 44.3.
 */
static int
run_restoring_float(float step_s, size_t step_count, float* out_error)
{
    frame_context_float_t frame =
    {
        .sample_count = 0,
        .mode = FRAME_FIELD_RESTORING_X,
        .constant_gravity_m_per_s2 = {0}
    };
    const bbtc_eb_inertial_vacuum_provider_config_float_t config =
        config_float(&frame, 0.0f, 0.0f, 0.0f);
    bbtc_eb_state_derivative_provider_float_t provider = {0};
    bbtc_eb_rigid_body_state_float_t state = {0};

    state.orientation_world_from_body.w = 1.0f;
    state.position_m.x = 1.0f;

    if (out_error == NULL)
        return EXIT_FAILURE;

    if (bbtc_eb_inertial_vacuum_provider_bind_float(&config, &provider)
            != BBTC_STATUS_SUCCESS
        || propagate_float(&provider, step_s, step_count, &state)
            != BBTC_STATUS_SUCCESS
        || frame.sample_count != 4 * step_count
        || bbtc_eb_rigid_body_state_validate_float(&state)
            != BBTC_STATUS_SUCCESS
        || !polynomial_close_float(state.position_m.y, 0.0f)
        || !polynomial_close_float(state.position_m.z, 0.0f)
        || !polynomial_close_float(state.velocity_m_per_s.y, 0.0f)
        || !polynomial_close_float(state.velocity_m_per_s.z, 0.0f)
        || !polynomial_close_float(
            state.angular_velocity_body_rad_per_s.x,
            0.0f
        )
        || !polynomial_close_float(
            state.angular_velocity_body_rad_per_s.y,
            0.0f
        )
        || !polynomial_close_float(
            state.angular_velocity_body_rad_per_s.z,
            0.0f
        )
        || !polynomial_close_float(state.time_s, 1.0f))
    {
        return EXIT_FAILURE;
    }

    const float exact_x = cosf(1.0f);
    const float exact_vx = -sinf(1.0f);

    *out_error = fmaxf(
        fabsf(state.position_m.x - exact_x),
        fabsf(state.velocity_m_per_s.x - exact_vx)
    );

    return isfinite(*out_error) && *out_error > 0.0f
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Native-double counterpart of run_restoring_float().
 */
static int
run_restoring_double(double step_s, size_t step_count, double* out_error)
{
    frame_context_double_t frame =
    {
        .sample_count = 0,
        .mode = FRAME_FIELD_RESTORING_X,
        .constant_gravity_m_per_s2 = {0}
    };
    const bbtc_eb_inertial_vacuum_provider_config_double_t config =
        config_double(&frame, 0.0, 0.0, 0.0);
    bbtc_eb_state_derivative_provider_double_t provider = {0};
    bbtc_eb_rigid_body_state_double_t state = {0};

    state.orientation_world_from_body.w = 1.0;
    state.position_m.x = 1.0;

    if (out_error == NULL)
        return EXIT_FAILURE;

    if (bbtc_eb_inertial_vacuum_provider_bind_double(&config, &provider)
            != BBTC_STATUS_SUCCESS
        || propagate_double(&provider, step_s, step_count, &state)
            != BBTC_STATUS_SUCCESS
        || frame.sample_count != 4 * step_count
        || bbtc_eb_rigid_body_state_validate_double(&state)
            != BBTC_STATUS_SUCCESS
        || !polynomial_close_double(state.position_m.y, 0.0)
        || !polynomial_close_double(state.position_m.z, 0.0)
        || !polynomial_close_double(state.velocity_m_per_s.y, 0.0)
        || !polynomial_close_double(state.velocity_m_per_s.z, 0.0)
        || !polynomial_close_double(
            state.angular_velocity_body_rad_per_s.x,
            0.0
        )
        || !polynomial_close_double(
            state.angular_velocity_body_rad_per_s.y,
            0.0
        )
        || !polynomial_close_double(
            state.angular_velocity_body_rad_per_s.z,
            0.0
        )
        || !polynomial_close_double(state.time_s, 1.0))
    {
        return EXIT_FAILURE;
    }

    const double exact_x = cos(1.0);
    const double exact_vx = -sin(1.0);

    *out_error = fmax(
        fabs(state.position_m.x - exact_x),
        fabs(state.velocity_m_per_s.x - exact_vx)
    );

    return isfinite(*out_error) && *out_error > 0.0
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Native-long-double counterpart of run_restoring_float().
 */
static int
run_restoring_long_double(
    long double step_s,
    size_t step_count,
    long double* out_error
)
{
    frame_context_long_double_t frame =
    {
        .sample_count = 0,
        .mode = FRAME_FIELD_RESTORING_X,
        .constant_gravity_m_per_s2 = {0}
    };
    const bbtc_eb_inertial_vacuum_provider_config_long_double_t config =
        config_long_double(&frame, 0.0L, 0.0L, 0.0L);
    bbtc_eb_state_derivative_provider_long_double_t provider = {0};
    bbtc_eb_rigid_body_state_long_double_t state = {0};

    state.orientation_world_from_body.w = 1.0L;
    state.position_m.x = 1.0L;

    if (out_error == NULL)
        return EXIT_FAILURE;

    if (bbtc_eb_inertial_vacuum_provider_bind_long_double(
            &config,
            &provider
        ) != BBTC_STATUS_SUCCESS
        || propagate_long_double(&provider, step_s, step_count, &state)
            != BBTC_STATUS_SUCCESS
        || frame.sample_count != 4 * step_count
        || bbtc_eb_rigid_body_state_validate_long_double(&state)
            != BBTC_STATUS_SUCCESS
        || !polynomial_close_long_double(state.position_m.y, 0.0L)
        || !polynomial_close_long_double(state.position_m.z, 0.0L)
        || !polynomial_close_long_double(
            state.velocity_m_per_s.y,
            0.0L
        )
        || !polynomial_close_long_double(
            state.velocity_m_per_s.z,
            0.0L
        )
        || !polynomial_close_long_double(
            state.angular_velocity_body_rad_per_s.x,
            0.0L
        )
        || !polynomial_close_long_double(
            state.angular_velocity_body_rad_per_s.y,
            0.0L
        )
        || !polynomial_close_long_double(
            state.angular_velocity_body_rad_per_s.z,
            0.0L
        )
        || !polynomial_close_long_double(state.time_s, 1.0L))
    {
        return EXIT_FAILURE;
    }

    const long double exact_x = cosl(1.0L);
    const long double exact_vx = -sinl(1.0L);

    *out_error = fmaxl(
        fabsl(state.position_m.x - exact_x),
        fabsl(state.velocity_m_per_s.x - exact_vx)
    );

    return isfinite(*out_error) && *out_error > 0.0L
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Validate Oracle C's native-float fourth-order error decrease.
 */
static int
test_restoring_convergence_float(void)
{
    float e0 = 0.0f;
    float e1 = 0.0f;
    float e2 = 0.0f;

    if (run_restoring_float(1.0f, 1, &e0) != EXIT_SUCCESS
        || run_restoring_float(0.5f, 2, &e1) != EXIT_SUCCESS
        || run_restoring_float(0.25f, 4, &e2) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    return e0 > e1
            && e1 > e2
            && e0 / e1 >= 10.0f
            && e1 / e2 >= 10.0f
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Validate Oracle C's native-double fourth-order error decrease.
 */
static int
test_restoring_convergence_double(void)
{
    double e0 = 0.0;
    double e1 = 0.0;
    double e2 = 0.0;

    if (run_restoring_double(1.0, 1, &e0) != EXIT_SUCCESS
        || run_restoring_double(0.5, 2, &e1) != EXIT_SUCCESS
        || run_restoring_double(0.25, 4, &e2) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    return e0 > e1
            && e1 > e2
            && e0 / e1 >= 10.0
            && e1 / e2 >= 10.0
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Validate Oracle C's native-long-double fourth-order error decrease.
 */
static int
test_restoring_convergence_long_double(void)
{
    long double e0 = 0.0L;
    long double e1 = 0.0L;
    long double e2 = 0.0L;

    if (run_restoring_long_double(1.0L, 1, &e0) != EXIT_SUCCESS
        || run_restoring_long_double(0.5L, 2, &e1) != EXIT_SUCCESS
        || run_restoring_long_double(0.25L, 4, &e2) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    return e0 > e1
            && e1 > e2
            && e0 / e1 >= 10.0L
            && e1 / e2 >= 10.0L
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Compute sign-invariant native-float quaternion component error.
 *
 * Unit quaternions q and -q encode the same physical rotation. Comparing only
 * q_numeric-q_exact would therefore reject a mathematically equivalent sign.
 * The EB1.3 contract instead takes the smaller infinity-norm component error
 * between q_numeric-q_exact and q_numeric+q_exact.
 */
static float
quaternion_error_float(
    bbtc_eb_quat_float_t numeric,
    bbtc_eb_quat_float_t exact
)
{
    const float plus = fmaxf(
        fmaxf(
            fabsf(numeric.x - exact.x),
            fabsf(numeric.y - exact.y)
        ),
        fmaxf(
            fabsf(numeric.z - exact.z),
            fabsf(numeric.w - exact.w)
        )
    );
    const float minus = fmaxf(
        fmaxf(
            fabsf(numeric.x + exact.x),
            fabsf(numeric.y + exact.y)
        ),
        fmaxf(
            fabsf(numeric.z + exact.z),
            fabsf(numeric.w + exact.w)
        )
    );

    return fminf(plus, minus);
}


/**
 * @brief Native-double counterpart of quaternion_error_float().
 */
static double
quaternion_error_double(
    bbtc_eb_quat_double_t numeric,
    bbtc_eb_quat_double_t exact
)
{
    const double plus = fmax(
        fmax(
            fabs(numeric.x - exact.x),
            fabs(numeric.y - exact.y)
        ),
        fmax(
            fabs(numeric.z - exact.z),
            fabs(numeric.w - exact.w)
        )
    );
    const double minus = fmax(
        fmax(
            fabs(numeric.x + exact.x),
            fabs(numeric.y + exact.y)
        ),
        fmax(
            fabs(numeric.z + exact.z),
            fabs(numeric.w + exact.w)
        )
    );

    return fmin(plus, minus);
}


/**
 * @brief Native-long-double counterpart of quaternion_error_float().
 */
static long double
quaternion_error_long_double(
    bbtc_eb_quat_long_double_t numeric,
    bbtc_eb_quat_long_double_t exact
)
{
    const long double plus = fmaxl(
        fmaxl(
            fabsl(numeric.x - exact.x),
            fabsl(numeric.y - exact.y)
        ),
        fmaxl(
            fabsl(numeric.z - exact.z),
            fabsl(numeric.w - exact.w)
        )
    );
    const long double minus = fmaxl(
        fmaxl(
            fabsl(numeric.x + exact.x),
            fabsl(numeric.y + exact.y)
        ),
        fmaxl(
            fabsl(numeric.z + exact.z),
            fabsl(numeric.w + exact.w)
        )
    );

    return fminl(plus, minus);
}


/**
 * @brief Execute one fresh native-float noncommuting spin run.
 *
 * The initial orientation q0=(0,0,1,0) is a 180-degree +Z rotation. It does
 * not commute with the pure +X angular-velocity quaternion. Under the accepted
 * Hamilton right-multiplied kinematics,
 *
 *   q(T) = q0 * (sin(T/2),0,0,cos(T/2))
 *
 * and for T=1 s the exact result is
 *
 *   (0, sin(0.5), cos(0.5), 0).
 *
 * The historical wrong left-multiplied ordering flips the sign of the y
 * component without producing a globally sign-equivalent quaternion, so the
 * convergence oracle genuinely detects that regression.
 */
static int
run_spin_float(float step_s, size_t step_count, float* out_error)
{
    frame_context_float_t frame =
    {
        .sample_count = 0,
        .mode = FRAME_FIELD_ZERO,
        .constant_gravity_m_per_s2 = {0}
    };
    const bbtc_eb_inertial_vacuum_provider_config_float_t config =
        config_float(&frame, 0.0f, 0.0f, 0.0f);
    bbtc_eb_state_derivative_provider_float_t provider = {0};
    bbtc_eb_rigid_body_state_float_t state = {0};

    state.orientation_world_from_body.z = 1.0f;
    state.angular_velocity_body_rad_per_s.x = 1.0f;

    if (out_error == NULL)
        return EXIT_FAILURE;

    if (bbtc_eb_inertial_vacuum_provider_bind_float(&config, &provider)
            != BBTC_STATUS_SUCCESS
        || propagate_float(&provider, step_s, step_count, &state)
            != BBTC_STATUS_SUCCESS
        || frame.sample_count != 4 * step_count
        || bbtc_eb_rigid_body_state_validate_float(&state)
            != BBTC_STATUS_SUCCESS
        || !polynomial_close_float(state.position_m.x, 0.0f)
        || !polynomial_close_float(state.position_m.y, 0.0f)
        || !polynomial_close_float(state.position_m.z, 0.0f)
        || !polynomial_close_float(state.velocity_m_per_s.x, 0.0f)
        || !polynomial_close_float(state.velocity_m_per_s.y, 0.0f)
        || !polynomial_close_float(state.velocity_m_per_s.z, 0.0f)
        || !polynomial_close_float(
            state.angular_velocity_body_rad_per_s.x,
            1.0f
        )
        || !polynomial_close_float(
            state.angular_velocity_body_rad_per_s.y,
            0.0f
        )
        || !polynomial_close_float(
            state.angular_velocity_body_rad_per_s.z,
            0.0f
        )
        || !polynomial_close_float(state.time_s, 1.0f))
    {
        return EXIT_FAILURE;
    }

    const bbtc_eb_quat_float_t exact =
    {
        .x = 0.0f,
        .y = sinf(0.5f),
        .z = cosf(0.5f),
        .w = 0.0f
    };

    *out_error = quaternion_error_float(
        state.orientation_world_from_body,
        exact
    );

    return isfinite(*out_error) && *out_error > 0.0f
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Native-double counterpart of run_spin_float().
 */
static int
run_spin_double(double step_s, size_t step_count, double* out_error)
{
    frame_context_double_t frame =
    {
        .sample_count = 0,
        .mode = FRAME_FIELD_ZERO,
        .constant_gravity_m_per_s2 = {0}
    };
    const bbtc_eb_inertial_vacuum_provider_config_double_t config =
        config_double(&frame, 0.0, 0.0, 0.0);
    bbtc_eb_state_derivative_provider_double_t provider = {0};
    bbtc_eb_rigid_body_state_double_t state = {0};

    state.orientation_world_from_body.z = 1.0;
    state.angular_velocity_body_rad_per_s.x = 1.0;

    if (out_error == NULL)
        return EXIT_FAILURE;

    if (bbtc_eb_inertial_vacuum_provider_bind_double(&config, &provider)
            != BBTC_STATUS_SUCCESS
        || propagate_double(&provider, step_s, step_count, &state)
            != BBTC_STATUS_SUCCESS
        || frame.sample_count != 4 * step_count
        || bbtc_eb_rigid_body_state_validate_double(&state)
            != BBTC_STATUS_SUCCESS
        || !polynomial_close_double(state.position_m.x, 0.0)
        || !polynomial_close_double(state.position_m.y, 0.0)
        || !polynomial_close_double(state.position_m.z, 0.0)
        || !polynomial_close_double(state.velocity_m_per_s.x, 0.0)
        || !polynomial_close_double(state.velocity_m_per_s.y, 0.0)
        || !polynomial_close_double(state.velocity_m_per_s.z, 0.0)
        || !polynomial_close_double(
            state.angular_velocity_body_rad_per_s.x,
            1.0
        )
        || !polynomial_close_double(
            state.angular_velocity_body_rad_per_s.y,
            0.0
        )
        || !polynomial_close_double(
            state.angular_velocity_body_rad_per_s.z,
            0.0
        )
        || !polynomial_close_double(state.time_s, 1.0))
    {
        return EXIT_FAILURE;
    }

    const bbtc_eb_quat_double_t exact =
    {
        .x = 0.0,
        .y = sin(0.5),
        .z = cos(0.5),
        .w = 0.0
    };

    *out_error = quaternion_error_double(
        state.orientation_world_from_body,
        exact
    );

    return isfinite(*out_error) && *out_error > 0.0
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Native-long-double counterpart of run_spin_float().
 */
static int
run_spin_long_double(
    long double step_s,
    size_t step_count,
    long double* out_error
)
{
    frame_context_long_double_t frame =
    {
        .sample_count = 0,
        .mode = FRAME_FIELD_ZERO,
        .constant_gravity_m_per_s2 = {0}
    };
    const bbtc_eb_inertial_vacuum_provider_config_long_double_t config =
        config_long_double(&frame, 0.0L, 0.0L, 0.0L);
    bbtc_eb_state_derivative_provider_long_double_t provider = {0};
    bbtc_eb_rigid_body_state_long_double_t state = {0};

    state.orientation_world_from_body.z = 1.0L;
    state.angular_velocity_body_rad_per_s.x = 1.0L;

    if (out_error == NULL)
        return EXIT_FAILURE;

    if (bbtc_eb_inertial_vacuum_provider_bind_long_double(
            &config,
            &provider
        ) != BBTC_STATUS_SUCCESS
        || propagate_long_double(&provider, step_s, step_count, &state)
            != BBTC_STATUS_SUCCESS
        || frame.sample_count != 4 * step_count
        || bbtc_eb_rigid_body_state_validate_long_double(&state)
            != BBTC_STATUS_SUCCESS
        || !polynomial_close_long_double(state.position_m.x, 0.0L)
        || !polynomial_close_long_double(state.position_m.y, 0.0L)
        || !polynomial_close_long_double(state.position_m.z, 0.0L)
        || !polynomial_close_long_double(
            state.velocity_m_per_s.x,
            0.0L
        )
        || !polynomial_close_long_double(
            state.velocity_m_per_s.y,
            0.0L
        )
        || !polynomial_close_long_double(
            state.velocity_m_per_s.z,
            0.0L
        )
        || !polynomial_close_long_double(
            state.angular_velocity_body_rad_per_s.x,
            1.0L
        )
        || !polynomial_close_long_double(
            state.angular_velocity_body_rad_per_s.y,
            0.0L
        )
        || !polynomial_close_long_double(
            state.angular_velocity_body_rad_per_s.z,
            0.0L
        )
        || !polynomial_close_long_double(state.time_s, 1.0L))
    {
        return EXIT_FAILURE;
    }

    const bbtc_eb_quat_long_double_t exact =
    {
        .x = 0.0L,
        .y = sinl(0.5L),
        .z = cosl(0.5L),
        .w = 0.0L
    };

    *out_error = quaternion_error_long_double(
        state.orientation_world_from_body,
        exact
    );

    return isfinite(*out_error) && *out_error > 0.0L
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Validate Oracle D's native-float fourth-order quaternion convergence.
 */
static int
test_spin_convergence_float(void)
{
    float e0 = 0.0f;
    float e1 = 0.0f;
    float e2 = 0.0f;

    if (run_spin_float(1.0f, 1, &e0) != EXIT_SUCCESS
        || run_spin_float(0.5f, 2, &e1) != EXIT_SUCCESS
        || run_spin_float(0.25f, 4, &e2) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    return e0 > e1
            && e1 > e2
            && e0 / e1 >= 10.0f
            && e1 / e2 >= 10.0f
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Validate Oracle D's native-double fourth-order quaternion convergence.
 */
static int
test_spin_convergence_double(void)
{
    double e0 = 0.0;
    double e1 = 0.0;
    double e2 = 0.0;

    if (run_spin_double(1.0, 1, &e0) != EXIT_SUCCESS
        || run_spin_double(0.5, 2, &e1) != EXIT_SUCCESS
        || run_spin_double(0.25, 4, &e2) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    return e0 > e1
            && e1 > e2
            && e0 / e1 >= 10.0
            && e1 / e2 >= 10.0
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Validate Oracle D's native-long-double quaternion convergence.
 */
static int
test_spin_convergence_long_double(void)
{
    long double e0 = 0.0L;
    long double e1 = 0.0L;
    long double e2 = 0.0L;

    if (run_spin_long_double(1.0L, 1, &e0) != EXIT_SUCCESS
        || run_spin_long_double(0.5L, 2, &e1) != EXIT_SUCCESS
        || run_spin_long_double(0.25L, 4, &e2) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    return e0 > e1
            && e1 > e2
            && e0 / e1 >= 10.0L
            && e1 / e2 >= 10.0L
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Run the complete EB1.3 analytic validation matrix.
 *
 * A single CTest executable keeps the milestone atomic while each helper above
 * preserves one independently reviewable proof obligation. Any failed oracle
 * returns EXIT_FAILURE immediately.
 */
int
main(void)
{
    if (test_drift_float() != EXIT_SUCCESS
        || test_drift_double() != EXIT_SUCCESS
        || test_drift_long_double() != EXIT_SUCCESS
        || test_constant_acceleration_float() != EXIT_SUCCESS
        || test_constant_acceleration_double() != EXIT_SUCCESS
        || test_constant_acceleration_long_double() != EXIT_SUCCESS
        || test_restoring_convergence_float() != EXIT_SUCCESS
        || test_restoring_convergence_double() != EXIT_SUCCESS
        || test_restoring_convergence_long_double() != EXIT_SUCCESS
        || test_spin_convergence_float() != EXIT_SUCCESS
        || test_spin_convergence_double() != EXIT_SUCCESS
        || test_spin_convergence_long_double() != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
