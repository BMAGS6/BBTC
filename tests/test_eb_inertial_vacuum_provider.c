/**
 * @file
 * @brief Contract tests for EB1.2 inertial-vacuum provider composition.
 */

#include <bbtc/bbtc.h>

#include <math.h>
#include <stddef.h>
#include <stdlib.h>

typedef bbtc_status_e (*config_validate_float_fn)(
    const bbtc_eb_inertial_vacuum_provider_config_float_t*
);
typedef bbtc_status_e (*config_validate_double_fn)(
    const bbtc_eb_inertial_vacuum_provider_config_double_t*
);
typedef bbtc_status_e (*config_validate_long_double_fn)(
    const bbtc_eb_inertial_vacuum_provider_config_long_double_t*
);

typedef bbtc_status_e (*provider_bind_float_fn)(
    const bbtc_eb_inertial_vacuum_provider_config_float_t*,
    bbtc_eb_state_derivative_provider_float_t*
);
typedef bbtc_status_e (*provider_bind_double_fn)(
    const bbtc_eb_inertial_vacuum_provider_config_double_t*,
    bbtc_eb_state_derivative_provider_double_t*
);
typedef bbtc_status_e (*provider_bind_long_double_fn)(
    const bbtc_eb_inertial_vacuum_provider_config_long_double_t*,
    bbtc_eb_state_derivative_provider_long_double_t*
);

static_assert(_Generic(
    &bbtc_eb_inertial_vacuum_provider_config_validate_float,
    config_validate_float_fn: 1,
    default: 0
));
static_assert(_Generic(
    &bbtc_eb_inertial_vacuum_provider_config_validate_double,
    config_validate_double_fn: 1,
    default: 0
));
static_assert(_Generic(
    &bbtc_eb_inertial_vacuum_provider_config_validate_long_double,
    config_validate_long_double_fn: 1,
    default: 0
));
static_assert(_Generic(
    &bbtc_eb_inertial_vacuum_provider_bind_float,
    provider_bind_float_fn: 1,
    default: 0
));
static_assert(_Generic(
    &bbtc_eb_inertial_vacuum_provider_bind_double,
    provider_bind_double_fn: 1,
    default: 0
));
static_assert(_Generic(
    &bbtc_eb_inertial_vacuum_provider_bind_long_double,
    provider_bind_long_double_fn: 1,
    default: 0
));

static_assert(_Generic(
    ((bbtc_eb_inertial_vacuum_provider_config_float_t*)0)->projectile,
    bbtc_eb_projectile_properties_float_t: 1,
    default: 0
));
static_assert(_Generic(
    ((bbtc_eb_inertial_vacuum_provider_config_float_t*)0)
        ->constant_force_moment,
    bbtc_eb_force_moment_float_t: 1,
    default: 0
));
static_assert(_Generic(
    ((bbtc_eb_inertial_vacuum_provider_config_float_t*)0)
        ->reference_frame_provider,
    bbtc_eb_reference_frame_provider_float_t: 1,
    default: 0
));

static_assert(
    offsetof(bbtc_eb_inertial_vacuum_provider_config_float_t, projectile)
        < offsetof(
            bbtc_eb_inertial_vacuum_provider_config_float_t,
            constant_force_moment
        )
);
static_assert(
    offsetof(
        bbtc_eb_inertial_vacuum_provider_config_float_t,
        constant_force_moment
    ) < offsetof(
        bbtc_eb_inertial_vacuum_provider_config_float_t,
        reference_frame_provider
    )
);

static_assert(_Generic(
    ((bbtc_eb_inertial_vacuum_provider_config_double_t*)0)->projectile,
    bbtc_eb_projectile_properties_double_t: 1,
    default: 0
));
static_assert(_Generic(
    ((bbtc_eb_inertial_vacuum_provider_config_double_t*)0)
        ->constant_force_moment,
    bbtc_eb_force_moment_double_t: 1,
    default: 0
));
static_assert(_Generic(
    ((bbtc_eb_inertial_vacuum_provider_config_double_t*)0)
        ->reference_frame_provider,
    bbtc_eb_reference_frame_provider_double_t: 1,
    default: 0
));

static_assert(
    offsetof(bbtc_eb_inertial_vacuum_provider_config_double_t, projectile)
        < offsetof(
            bbtc_eb_inertial_vacuum_provider_config_double_t,
            constant_force_moment
        )
);
static_assert(
    offsetof(
        bbtc_eb_inertial_vacuum_provider_config_double_t,
        constant_force_moment
    ) < offsetof(
        bbtc_eb_inertial_vacuum_provider_config_double_t,
        reference_frame_provider
    )
);

static_assert(_Generic(
    ((bbtc_eb_inertial_vacuum_provider_config_long_double_t*)0)->projectile,
    bbtc_eb_projectile_properties_long_double_t: 1,
    default: 0
));
static_assert(_Generic(
    ((bbtc_eb_inertial_vacuum_provider_config_long_double_t*)0)
        ->constant_force_moment,
    bbtc_eb_force_moment_long_double_t: 1,
    default: 0
));
static_assert(_Generic(
    ((bbtc_eb_inertial_vacuum_provider_config_long_double_t*)0)
        ->reference_frame_provider,
    bbtc_eb_reference_frame_provider_long_double_t: 1,
    default: 0
));

static_assert(
    offsetof(
        bbtc_eb_inertial_vacuum_provider_config_long_double_t,
        projectile
    ) < offsetof(
        bbtc_eb_inertial_vacuum_provider_config_long_double_t,
        constant_force_moment
    )
);
static_assert(
    offsetof(
        bbtc_eb_inertial_vacuum_provider_config_long_double_t,
        constant_force_moment
    ) < offsetof(
        bbtc_eb_inertial_vacuum_provider_config_long_double_t,
        reference_frame_provider
    )
);

typedef struct sample_recorder_double_t
{
    size_t count;
    bbtc_eb_vec3_double_t positions[8];
    double times_s[8];
    bbtc_status_e unconditional_status;
    int fail_at_or_after_time;
    double fail_time_s;
    bbtc_status_e timed_failure_status;
    bbtc_eb_reference_frame_sample_double_t sample;
}
sample_recorder_double_t;

static bbtc_eb_projectile_properties_double_t
valid_projectile_double(void)
{
    return (bbtc_eb_projectile_properties_double_t)
    {
        .principal_inertia =
        {
            .moment_about_x_kg_m2 = 3.0,
            .moment_about_y_kg_m2 = 3.0,
            .moment_about_z_kg_m2 = 3.0
        },
        .mass_kg = 2.0,
        .reference_diameter_m = 1.0,
        .reference_area_m2 = 1.0
    };
}

static bbtc_eb_rigid_body_state_double_t
valid_state_double(void)
{
    return (bbtc_eb_rigid_body_state_double_t)
    {
        .orientation_world_from_body = {.w = 1.0},
        .position_m = {.x = 2.0, .y = 4.0, .z = 6.0},
        .velocity_m_per_s = {.x = 8.0, .y = -4.0, .z = 2.0},
        .angular_velocity_body_rad_per_s = {0},
        .time_s = 1.0
    };
}

static bbtc_status_e
zero_frame_sample_float(
    const void* context,
    const bbtc_eb_vec3_float_t* position_reference_m,
    float time_s,
    bbtc_eb_reference_frame_sample_float_t* out_sample
)
{
    (void)context;
    (void)position_reference_m;
    (void)time_s;
    *out_sample = (bbtc_eb_reference_frame_sample_float_t){0};
    return BBTC_STATUS_SUCCESS;
}

static bbtc_status_e
zero_frame_sample_long_double(
    const void* context,
    const bbtc_eb_vec3_long_double_t* position_reference_m,
    long double time_s,
    bbtc_eb_reference_frame_sample_long_double_t* out_sample
)
{
    (void)context;
    (void)position_reference_m;
    (void)time_s;
    *out_sample = (bbtc_eb_reference_frame_sample_long_double_t){0};
    return BBTC_STATUS_SUCCESS;
}

static bbtc_status_e
recording_frame_sample_double(
    const void* context,
    const bbtc_eb_vec3_double_t* position_reference_m,
    double time_s,
    bbtc_eb_reference_frame_sample_double_t* out_sample
)
{
    const sample_recorder_double_t* const const_recorder = context;
    if (const_recorder == NULL)
    {
        *out_sample = (bbtc_eb_reference_frame_sample_double_t){0};
        return BBTC_STATUS_SUCCESS;
    }

    sample_recorder_double_t* const recorder =
        (sample_recorder_double_t*)const_recorder;

    if (recorder->count < 8)
    {
        recorder->positions[recorder->count] = *position_reference_m;
        recorder->times_s[recorder->count] = time_s;
    }
    ++recorder->count;

    if (recorder->fail_at_or_after_time && time_s >= recorder->fail_time_s)
        return recorder->timed_failure_status;

    if (recorder->unconditional_status != BBTC_STATUS_SUCCESS)
        return recorder->unconditional_status;

    *out_sample = recorder->sample;
    return BBTC_STATUS_SUCCESS;
}

static bbtc_eb_inertial_vacuum_provider_config_double_t
valid_config_double(sample_recorder_double_t* recorder)
{
    return (bbtc_eb_inertial_vacuum_provider_config_double_t)
    {
        .projectile = valid_projectile_double(),
        .reference_frame_provider =
        {
            .sample = recording_frame_sample_double,
            .context = recorder
        }
    };
}

static int
derivative_double_is_zero(
    const bbtc_eb_rigid_body_state_derivative_double_t* derivative
)
{
    return derivative->orientation_world_from_body_derivative_per_s.x == 0.0
        && derivative->orientation_world_from_body_derivative_per_s.y == 0.0
        && derivative->orientation_world_from_body_derivative_per_s.z == 0.0
        && derivative->orientation_world_from_body_derivative_per_s.w == 0.0
        && derivative->position_derivative_reference_m_per_s.x == 0.0
        && derivative->position_derivative_reference_m_per_s.y == 0.0
        && derivative->position_derivative_reference_m_per_s.z == 0.0
        && derivative->velocity_derivative_reference_m_per_s2.x == 0.0
        && derivative->velocity_derivative_reference_m_per_s2.y == 0.0
        && derivative->velocity_derivative_reference_m_per_s2.z == 0.0
        && derivative->angular_velocity_derivative_body_rad_per_s2.x == 0.0
        && derivative->angular_velocity_derivative_body_rad_per_s2.y == 0.0
        && derivative->angular_velocity_derivative_body_rad_per_s2.z == 0.0;
}

static bbtc_status_e
poison_derivative_callback(
    const void* context,
    const bbtc_eb_rigid_body_state_double_t* state,
    bbtc_eb_rigid_body_state_derivative_double_t* out_derivative
)
{
    (void)context;
    (void)state;
    (void)out_derivative;
    return BBTC_STATUS_INTERNAL_INVARIANT_FAILURE;
}

static int
test_configuration_validation(void)
{
    sample_recorder_double_t recorder = {0};
    bbtc_eb_inertial_vacuum_provider_config_double_t config =
        valid_config_double(&recorder);

    if (bbtc_eb_inertial_vacuum_provider_config_validate_double(NULL)
        != BBTC_STATUS_INVALID_ARGUMENT)
    {
        return EXIT_FAILURE;
    }

    config.projectile.mass_kg = 0.0;
    config.constant_force_moment.applied_force_reference_n.x = NAN;
    config.reference_frame_provider.sample = NULL;

    if (bbtc_eb_inertial_vacuum_provider_config_validate_double(&config)
        != BBTC_STATUS_OUTSIDE_DOMAIN)
    {
        return EXIT_FAILURE;
    }

    config.projectile = valid_projectile_double();
    if (bbtc_eb_inertial_vacuum_provider_config_validate_double(&config)
        != BBTC_STATUS_NAN_INPUT)
    {
        return EXIT_FAILURE;
    }

    config.constant_force_moment = (bbtc_eb_force_moment_double_t){0};
    if (bbtc_eb_inertial_vacuum_provider_config_validate_double(&config)
        != BBTC_STATUS_INVALID_ARGUMENT)
    {
        return EXIT_FAILURE;
    }

    config.reference_frame_provider.sample = recording_frame_sample_double;
    config.reference_frame_provider.context = NULL;
    if (bbtc_eb_inertial_vacuum_provider_config_validate_double(&config)
            != BBTC_STATUS_SUCCESS
        || recorder.count != 0)
    {
        return EXIT_FAILURE;
    }

    config.reference_frame_provider.context = &recorder;
    const bbtc_eb_inertial_vacuum_provider_config_double_t before = config;

    return bbtc_eb_inertial_vacuum_provider_config_validate_double(&config)
                == BBTC_STATUS_SUCCESS
            && recorder.count == 0
            && config.projectile.mass_kg == before.projectile.mass_kg
            && config.constant_force_moment.applied_force_reference_n.x
                == before.constant_force_moment.applied_force_reference_n.x
            && config.reference_frame_provider.sample
                == before.reference_frame_provider.sample
            && config.reference_frame_provider.context
                == before.reference_frame_provider.context
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}

static int
test_by_value_storage(void)
{
    sample_recorder_double_t first = {0};
    sample_recorder_double_t second = {0};

    bbtc_eb_projectile_properties_double_t projectile =
        valid_projectile_double();
    bbtc_eb_force_moment_double_t force_moment =
    {
        .applied_force_reference_n = {.x = 4.0}
    };
    bbtc_eb_reference_frame_provider_double_t frame_provider =
    {
        .sample = recording_frame_sample_double,
        .context = &first
    };

    const bbtc_eb_inertial_vacuum_provider_config_double_t config =
    {
        .projectile = projectile,
        .constant_force_moment = force_moment,
        .reference_frame_provider = frame_provider
    };

    projectile.mass_kg = 99.0;
    force_moment.applied_force_reference_n.x = 99.0;
    frame_provider.sample = NULL;
    frame_provider.context = &second;

    return config.projectile.mass_kg == 2.0
            && config.constant_force_moment.applied_force_reference_n.x == 4.0
            && config.reference_frame_provider.sample
                == recording_frame_sample_double
            && config.reference_frame_provider.context == &first
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}

static int
test_binding(void)
{
    sample_recorder_double_t recorder = {0};
    bbtc_eb_inertial_vacuum_provider_config_double_t config =
        valid_config_double(&recorder);

    if (bbtc_eb_inertial_vacuum_provider_bind_double(&config, NULL)
        != BBTC_STATUS_INVALID_ARGUMENT)
    {
        return EXIT_FAILURE;
    }

    bbtc_eb_state_derivative_provider_double_t provider =
    {
        .evaluate = poison_derivative_callback,
        .context = &recorder
    };

    if (bbtc_eb_inertial_vacuum_provider_bind_double(NULL, &provider)
            != BBTC_STATUS_INVALID_ARGUMENT
        || provider.evaluate != NULL
        || provider.context != NULL)
    {
        return EXIT_FAILURE;
    }

    config.projectile.mass_kg = 0.0;
    provider.evaluate = poison_derivative_callback;
    provider.context = &recorder;

    if (bbtc_eb_inertial_vacuum_provider_bind_double(&config, &provider)
            != BBTC_STATUS_OUTSIDE_DOMAIN
        || provider.evaluate != NULL
        || provider.context != NULL)
    {
        return EXIT_FAILURE;
    }

    config.projectile = valid_projectile_double();
    if (bbtc_eb_inertial_vacuum_provider_bind_double(&config, &provider)
            != BBTC_STATUS_SUCCESS
        || provider.evaluate == NULL
        || provider.context != &config)
    {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

static int
test_one_derivative_request(void)
{
    sample_recorder_double_t recorder = {0};
    recorder.sample.gravitational_acceleration_reference_m_per_s2.z = -8.0;

    bbtc_eb_inertial_vacuum_provider_config_double_t config =
        valid_config_double(&recorder);
    config.constant_force_moment.applied_force_reference_n.x = 4.0;
    config.constant_force_moment.applied_moment_body_n_m.x = 6.0;

    const bbtc_eb_inertial_vacuum_provider_config_double_t config_before =
        config;
    bbtc_eb_rigid_body_state_double_t state = valid_state_double();
    const bbtc_eb_rigid_body_state_double_t state_before = state;

    bbtc_eb_state_derivative_provider_double_t provider = {0};
    bbtc_eb_rigid_body_state_derivative_double_t derivative = {0};

    if (bbtc_eb_inertial_vacuum_provider_bind_double(&config, &provider)
            != BBTC_STATUS_SUCCESS
        || bbtc_eb_state_derivative_provider_evaluate_double(
            &provider,
            &state,
            &derivative
        ) != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    return recorder.count == 1
            && recorder.positions[0].x == state.position_m.x
            && recorder.positions[0].y == state.position_m.y
            && recorder.positions[0].z == state.position_m.z
            && recorder.times_s[0] == state.time_s
            && derivative.position_derivative_reference_m_per_s.x == 8.0
            && derivative.position_derivative_reference_m_per_s.y == -4.0
            && derivative.position_derivative_reference_m_per_s.z == 2.0
            && derivative.velocity_derivative_reference_m_per_s2.x == 2.0
            && derivative.velocity_derivative_reference_m_per_s2.z == -8.0
            && derivative.angular_velocity_derivative_body_rad_per_s2.x == 2.0
            && state.position_m.x == state_before.position_m.x
            && state.time_s == state_before.time_s
            && config.projectile.mass_kg == config_before.projectile.mass_kg
            && config.constant_force_moment.applied_force_reference_n.x
                == config_before.constant_force_moment
                       .applied_force_reference_n.x
            && config.reference_frame_provider.sample
                == config_before.reference_frame_provider.sample
            && config.reference_frame_provider.context
                == config_before.reference_frame_provider.context
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}

static int
test_failure_propagation(void)
{
    sample_recorder_double_t recorder = {0};
    recorder.unconditional_status = BBTC_STATUS_OUTSIDE_DOMAIN;

    bbtc_eb_inertial_vacuum_provider_config_double_t config =
        valid_config_double(&recorder);
    bbtc_eb_state_derivative_provider_double_t provider = {0};
    bbtc_eb_rigid_body_state_double_t state = valid_state_double();
    bbtc_eb_rigid_body_state_derivative_double_t derivative =
    {
        .position_derivative_reference_m_per_s = {.x = 1.0}
    };

    if (bbtc_eb_inertial_vacuum_provider_bind_double(&config, &provider)
            != BBTC_STATUS_SUCCESS
        || bbtc_eb_state_derivative_provider_evaluate_double(
            &provider,
            &state,
            &derivative
        ) != BBTC_STATUS_OUTSIDE_DOMAIN
        || recorder.count != 1
        || !derivative_double_is_zero(&derivative))
    {
        return EXIT_FAILURE;
    }

    recorder = (sample_recorder_double_t){0};
    recorder.sample.reference_frame_angular_velocity_reference_rad_per_s.z =
        1.0;
    config = valid_config_double(&recorder);
    derivative.position_derivative_reference_m_per_s.x = 1.0;

    if (bbtc_eb_inertial_vacuum_provider_bind_double(&config, &provider)
            != BBTC_STATUS_SUCCESS
        || bbtc_eb_state_derivative_provider_evaluate_double(
            &provider,
            &state,
            &derivative
        ) != BBTC_STATUS_UNSUPPORTED_MODEL_OR_OPTION
        || recorder.count != 1
        || !derivative_double_is_zero(&derivative))
    {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

static int
test_other_precision_families(void)
{
    const bbtc_eb_inertial_vacuum_provider_config_float_t float_config =
    {
        .projectile =
        {
            .principal_inertia =
            {
                .moment_about_x_kg_m2 = 2.0f,
                .moment_about_y_kg_m2 = 2.0f,
                .moment_about_z_kg_m2 = 2.0f
            },
            .mass_kg = 2.0f,
            .reference_diameter_m = 1.0f,
            .reference_area_m2 = 1.0f
        },
        .constant_force_moment =
        {
            .applied_force_reference_n = {.x = 2.0f}
        },
        .reference_frame_provider =
        {
            .sample = zero_frame_sample_float,
            .context = NULL
        }
    };
    const bbtc_eb_rigid_body_state_float_t float_state =
    {
        .orientation_world_from_body = {.w = 1.0f},
        .velocity_m_per_s = {.x = 3.0f},
        .time_s = 0.0f
    };
    bbtc_eb_state_derivative_provider_float_t float_provider = {0};
    bbtc_eb_rigid_body_state_derivative_float_t float_derivative = {0};

    if (bbtc_eb_inertial_vacuum_provider_config_validate_float(&float_config)
            != BBTC_STATUS_SUCCESS
        || bbtc_eb_inertial_vacuum_provider_bind_float(
            &float_config,
            &float_provider
        ) != BBTC_STATUS_SUCCESS
        || bbtc_eb_state_derivative_provider_evaluate_float(
            &float_provider,
            &float_state,
            &float_derivative
        ) != BBTC_STATUS_SUCCESS
        || float_derivative.position_derivative_reference_m_per_s.x != 3.0f
        || float_derivative.velocity_derivative_reference_m_per_s2.x != 1.0f)
    {
        return EXIT_FAILURE;
    }

    const bbtc_eb_inertial_vacuum_provider_config_long_double_t long_config =
    {
        .projectile =
        {
            .principal_inertia =
            {
                .moment_about_x_kg_m2 = 2.0L,
                .moment_about_y_kg_m2 = 2.0L,
                .moment_about_z_kg_m2 = 2.0L
            },
            .mass_kg = 2.0L,
            .reference_diameter_m = 1.0L,
            .reference_area_m2 = 1.0L
        },
        .constant_force_moment =
        {
            .applied_force_reference_n = {.x = 2.0L}
        },
        .reference_frame_provider =
        {
            .sample = zero_frame_sample_long_double,
            .context = NULL
        }
    };
    const bbtc_eb_rigid_body_state_long_double_t long_state =
    {
        .orientation_world_from_body = {.w = 1.0L},
        .velocity_m_per_s = {.x = 3.0L},
        .time_s = 0.0L
    };
    bbtc_eb_state_derivative_provider_long_double_t long_provider = {0};
    bbtc_eb_rigid_body_state_derivative_long_double_t long_derivative = {0};

    if (bbtc_eb_inertial_vacuum_provider_config_validate_long_double(
            &long_config
        ) != BBTC_STATUS_SUCCESS
        || bbtc_eb_inertial_vacuum_provider_bind_long_double(
            &long_config,
            &long_provider
        ) != BBTC_STATUS_SUCCESS
        || bbtc_eb_state_derivative_provider_evaluate_long_double(
            &long_provider,
            &long_state,
            &long_derivative
        ) != BBTC_STATUS_SUCCESS
        || long_derivative.position_derivative_reference_m_per_s.x != 3.0L
        || long_derivative.velocity_derivative_reference_m_per_s2.x != 1.0L)
    {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

static int
test_rk4_stage_sampling(void)
{
    sample_recorder_double_t recorder = {0};
    bbtc_eb_inertial_vacuum_provider_config_double_t config =
        valid_config_double(&recorder);
    bbtc_eb_state_derivative_provider_double_t provider = {0};
    bbtc_eb_rigid_body_state_double_t state = valid_state_double();

    if (bbtc_eb_inertial_vacuum_provider_bind_double(&config, &provider)
            != BBTC_STATUS_SUCCESS
        || bbtc_eb_rk4_step_double(&provider, 0.5, &state)
            != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (recorder.count != 4)
        return EXIT_FAILURE;

    const bbtc_eb_vec3_double_t expected_positions[4] =
    {
        {.x = 2.0, .y = 4.0, .z = 6.0},
        {.x = 4.0, .y = 3.0, .z = 6.5},
        {.x = 4.0, .y = 3.0, .z = 6.5},
        {.x = 6.0, .y = 2.0, .z = 7.0}
    };
    const double expected_times[4] = {1.0, 1.25, 1.25, 1.5};

    for (size_t index = 0; index < 4; ++index)
    {
        if (recorder.positions[index].x != expected_positions[index].x
            || recorder.positions[index].y != expected_positions[index].y
            || recorder.positions[index].z != expected_positions[index].z
            || recorder.times_s[index] != expected_times[index])
        {
            return EXIT_FAILURE;
        }
    }

    return state.position_m.x == 6.0
            && state.position_m.y == 2.0
            && state.position_m.z == 7.0
            && state.velocity_m_per_s.x == 8.0
            && state.velocity_m_per_s.y == -4.0
            && state.velocity_m_per_s.z == 2.0
            && state.time_s == 1.5
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}

static int
test_rk4_failure_transaction(void)
{
    sample_recorder_double_t recorder = {0};
    recorder.fail_at_or_after_time = 1;
    recorder.fail_time_s = 1.5;
    recorder.timed_failure_status = BBTC_STATUS_OUTSIDE_DOMAIN;

    bbtc_eb_inertial_vacuum_provider_config_double_t config =
        valid_config_double(&recorder);
    bbtc_eb_state_derivative_provider_double_t provider = {0};
    bbtc_eb_rigid_body_state_double_t state = valid_state_double();
    const bbtc_eb_rigid_body_state_double_t before = state;

    if (bbtc_eb_inertial_vacuum_provider_bind_double(&config, &provider)
            != BBTC_STATUS_SUCCESS
        || bbtc_eb_rk4_step_double(&provider, 0.5, &state)
            != BBTC_STATUS_OUTSIDE_DOMAIN)
    {
        return EXIT_FAILURE;
    }

    return recorder.count == 4
            && state.orientation_world_from_body.x
                == before.orientation_world_from_body.x
            && state.orientation_world_from_body.y
                == before.orientation_world_from_body.y
            && state.orientation_world_from_body.z
                == before.orientation_world_from_body.z
            && state.orientation_world_from_body.w
                == before.orientation_world_from_body.w
            && state.position_m.x == before.position_m.x
            && state.position_m.y == before.position_m.y
            && state.position_m.z == before.position_m.z
            && state.velocity_m_per_s.x == before.velocity_m_per_s.x
            && state.velocity_m_per_s.y == before.velocity_m_per_s.y
            && state.velocity_m_per_s.z == before.velocity_m_per_s.z
            && state.angular_velocity_body_rad_per_s.x
                == before.angular_velocity_body_rad_per_s.x
            && state.angular_velocity_body_rad_per_s.y
                == before.angular_velocity_body_rad_per_s.y
            && state.angular_velocity_body_rad_per_s.z
                == before.angular_velocity_body_rad_per_s.z
            && state.time_s == before.time_s
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}

int
main(void)
{
    if (test_configuration_validation() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_by_value_storage() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_binding() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_one_derivative_request() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_failure_propagation() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_other_precision_families() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_rk4_stage_sampling() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    return test_rk4_failure_transaction();
}
