/**
 * @file
 * @brief Tests EB0.1-B5 environment-provider representation and dispatch.
 */

#include <bbtc/bbtc.h>

#include <math.h>
#include <stddef.h>
#include <stdlib.h>

static_assert(_Generic(((bbtc_eb_environment_provider_float_t*)0)->sample,
                       bbtc_eb_environment_provider_float_fn: 1,
                       default: 0));
static_assert(_Generic(((bbtc_eb_environment_provider_double_t*)0)->sample,
                       bbtc_eb_environment_provider_double_fn: 1,
                       default: 0));
static_assert(_Generic(
    ((bbtc_eb_environment_provider_long_double_t*)0)->sample,
    bbtc_eb_environment_provider_long_double_fn: 1,
    default: 0
));
static_assert(_Generic(((bbtc_eb_environment_provider_double_t*)0)->context,
                       const void*: 1,
                       default: 0));

#define BBTC_EB_ASSERT_PROVIDER_ORDER(TYPE)                               \
    static_assert(offsetof(TYPE, sample) < offsetof(TYPE, context))

BBTC_EB_ASSERT_PROVIDER_ORDER(bbtc_eb_environment_provider_float_t);
BBTC_EB_ASSERT_PROVIDER_ORDER(bbtc_eb_environment_provider_double_t);
BBTC_EB_ASSERT_PROVIDER_ORDER(bbtc_eb_environment_provider_long_double_t);

#undef BBTC_EB_ASSERT_PROVIDER_ORDER

/**
 * @brief Expected callback-observation state for native-double provider tests.
 */
typedef struct tracking_context_t
{
    unsigned* call_count;
    const void* expected_context;
    double expected_x;
    double expected_y;
    double expected_z;
    double expected_time_s;
}
tracking_context_t;

/**
 * @brief Reports whether every named field of a native-double sample is zero.
 */
static int
double_sample_is_zero(const bbtc_eb_environment_sample_double_t* sample)
{
    return sample->air_velocity_world_m_per_s.x == 0.0
        && sample->air_velocity_world_m_per_s.y == 0.0
        && sample->air_velocity_world_m_per_s.z == 0.0
        && sample->temperature_k == 0.0
        && sample->absolute_pressure_pa == 0.0
        && sample->air_density_kg_per_m3 == 0.0
        && sample->speed_of_sound_m_per_s == 0.0;
}

/**
 * @brief Writes one representative valid positive-density double sample.
 */
static void
write_valid_double_sample(bbtc_eb_environment_sample_double_t* out_sample)
{
    *out_sample = (bbtc_eb_environment_sample_double_t)
    {
        .air_velocity_world_m_per_s = {.x = -8.0, .y = 1.5, .z = 0.25},
        .temperature_k = 288.15,
        .absolute_pressure_pa = 101325.0,
        .air_density_kg_per_m3 = 1.225,
        .speed_of_sound_m_per_s = 340.29
    };
}

/**
 * @brief Verifies exact callback inputs and writes a valid double sample.
 */
static bbtc_status_e
tracking_double_provider(
    const void* context_pointer,
    const bbtc_eb_vec3_double_t* position_world_m,
    double time_s,
    bbtc_eb_environment_sample_double_t* out_sample
)
{
    const tracking_context_t* const context = context_pointer;

    if (context == NULL
        || context->expected_context != context_pointer
        || !double_sample_is_zero(out_sample)
        || position_world_m->x != context->expected_x
        || position_world_m->y != context->expected_y
        || position_world_m->z != context->expected_z
        || time_s != context->expected_time_s)
    {
        return BBTC_STATUS_INTERNAL_INVARIANT_FAILURE;
    }

    ++*context->call_count;
    write_valid_double_sample(out_sample);
    return BBTC_STATUS_SUCCESS;
}

/**
 * @brief Accepts only a null context and writes a valid double sample.
 */
static bbtc_status_e
null_context_double_provider(
    const void* context,
    const bbtc_eb_vec3_double_t* position_world_m,
    double time_s,
    bbtc_eb_environment_sample_double_t* out_sample
)
{
    (void)position_world_m;
    (void)time_s;

    if (context != NULL)
        return BBTC_STATUS_INTERNAL_INVARIANT_FAILURE;

    write_valid_double_sample(out_sample);
    return BBTC_STATUS_SUCCESS;
}

/**
 * @brief Deliberately writes output and then returns a provider failure.
 */
static bbtc_status_e
failing_double_provider(
    const void* context,
    const bbtc_eb_vec3_double_t* position_world_m,
    double time_s,
    bbtc_eb_environment_sample_double_t* out_sample
)
{
    (void)context;
    (void)position_world_m;
    (void)time_s;

    write_valid_double_sample(out_sample);
    return BBTC_STATUS_UNSUPPORTED_MODEL_OR_OPTION;
}

/**
 * @brief Returns success with a deliberately invalid B4 environment sample.
 */
static bbtc_status_e
invalid_output_double_provider(
    const void* context,
    const bbtc_eb_vec3_double_t* position_world_m,
    double time_s,
    bbtc_eb_environment_sample_double_t* out_sample
)
{
    (void)context;
    (void)position_world_m;
    (void)time_s;

    write_valid_double_sample(out_sample);
    out_sample->speed_of_sound_m_per_s = 0.0;
    return BBTC_STATUS_SUCCESS;
}

/**
 * @brief Produces a pure position/time-derived sample for order tests.
 */
static bbtc_status_e
functional_double_provider(
    const void* context,
    const bbtc_eb_vec3_double_t* position_world_m,
    double time_s,
    bbtc_eb_environment_sample_double_t* out_sample
)
{
    (void)context;

    write_valid_double_sample(out_sample);
    out_sample->air_velocity_world_m_per_s.x = position_world_m->x + time_s;
    out_sample->air_velocity_world_m_per_s.y = position_world_m->y - time_s;
    out_sample->air_velocity_world_m_per_s.z = position_world_m->z;
    return BBTC_STATUS_SUCCESS;
}

/**
 * @brief Produces one valid native-float sample for dispatch coverage.
 */
static bbtc_status_e
float_provider(
    const void* context,
    const bbtc_eb_vec3_float_t* position_world_m,
    float time_s,
    bbtc_eb_environment_sample_float_t* out_sample
)
{
    (void)context;
    (void)position_world_m;
    (void)time_s;

    *out_sample = (bbtc_eb_environment_sample_float_t)
    {
        .air_velocity_world_m_per_s = {.x = 1.0f, .y = 0.0f, .z = -1.0f},
        .temperature_k = 300.0f,
        .absolute_pressure_pa = 100000.0f,
        .air_density_kg_per_m3 = 1.0f,
        .speed_of_sound_m_per_s = 340.0f
    };

    return BBTC_STATUS_SUCCESS;
}

/**
 * @brief Produces one valid native-long-double vacuum sample.
 */
static bbtc_status_e
long_double_provider(
    const void* context,
    const bbtc_eb_vec3_long_double_t* position_world_m,
    long double time_s,
    bbtc_eb_environment_sample_long_double_t* out_sample
)
{
    (void)context;
    (void)position_world_m;
    (void)time_s;

    *out_sample = (bbtc_eb_environment_sample_long_double_t)
    {
        .air_velocity_world_m_per_s = {.x = 0.0L, .y = 0.0L, .z = 0.0L},
        .temperature_k = 1.0L,
        .absolute_pressure_pa = 0.0L,
        .air_density_kg_per_m3 = 0.0L,
        .speed_of_sound_m_per_s = 0.0L
    };

    return BBTC_STATUS_SUCCESS;
}

/**
 * @brief Verifies valid dispatch, exact inputs, null context, and pre-zeroing.
 */
static int
test_valid_dispatch(void)
{
    unsigned calls = 0U;
    tracking_context_t context =
    {
        .call_count = &calls,
        .expected_context = NULL,
        .expected_x = 10.0,
        .expected_y = -20.0,
        .expected_z = 30.0,
        .expected_time_s = 4.5
    };
    context.expected_context = &context;

    const bbtc_eb_environment_provider_double_t provider =
    {
        .sample = tracking_double_provider,
        .context = &context
    };
    const bbtc_eb_vec3_double_t position = {.x = 10.0, .y = -20.0, .z = 30.0};
    bbtc_eb_environment_sample_double_t sample;
    write_valid_double_sample(&sample);

    /*
     * The tracking callback requires the output to be all-zero on entry. By
     * deliberately passing stale nonzero storage, this proves dispatch clears
     * the destination before callback invocation, not merely after failures.
     */
    if (bbtc_eb_environment_provider_sample_double(
            &provider,
            &position,
            4.5,
            &sample
        ) != BBTC_STATUS_SUCCESS
        || calls != 1U
        || sample.temperature_k != 288.15
        || sample.air_velocity_world_m_per_s.x != -8.0)
    {
        return EXIT_FAILURE;
    }

    const bbtc_eb_environment_provider_double_t null_context_provider =
    {
        .sample = null_context_double_provider,
        .context = NULL
    };

    return bbtc_eb_environment_provider_sample_double(
               &null_context_provider,
               &position,
               0.0,
               &sample
           ) == BBTC_STATUS_SUCCESS
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}

/**
 * @brief Verifies required-pointer rejection and fail-closed output state.
 */
static int
test_null_arguments(void)
{
    const bbtc_eb_vec3_double_t position = {.x = 1.0, .y = 2.0, .z = 3.0};
    const bbtc_eb_environment_provider_double_t valid_provider =
    {
        .sample = null_context_double_provider,
        .context = NULL
    };
    const bbtc_eb_environment_provider_double_t null_callback_provider =
    {
        .sample = NULL,
        .context = NULL
    };
    bbtc_eb_environment_sample_double_t sample;

    write_valid_double_sample(&sample);

    if (bbtc_eb_environment_provider_sample_double(
            NULL,
            &position,
            0.0,
            &sample
        ) != BBTC_STATUS_INVALID_ARGUMENT
        || !double_sample_is_zero(&sample))
    {
        return EXIT_FAILURE;
    }

    write_valid_double_sample(&sample);

    if (bbtc_eb_environment_provider_sample_double(
            &null_callback_provider,
            &position,
            0.0,
            &sample
        ) != BBTC_STATUS_INVALID_ARGUMENT
        || !double_sample_is_zero(&sample))
    {
        return EXIT_FAILURE;
    }

    write_valid_double_sample(&sample);

    if (bbtc_eb_environment_provider_sample_double(
            &valid_provider,
            NULL,
            0.0,
            &sample
        ) != BBTC_STATUS_INVALID_ARGUMENT
        || !double_sample_is_zero(&sample))
    {
        return EXIT_FAILURE;
    }

    return bbtc_eb_environment_provider_sample_double(
               &valid_provider,
               &position,
               0.0,
               NULL
           ) == BBTC_STATUS_INVALID_ARGUMENT
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}

/**
 * @brief Verifies NaN, infinity, and negative-time validation precedence.
 */
static int
test_request_validation(void)
{
    unsigned calls = 0U;
    tracking_context_t context =
    {
        .call_count = &calls,
        .expected_context = NULL,
        .expected_x = 0.0,
        .expected_y = 0.0,
        .expected_z = 0.0,
        .expected_time_s = 0.0
    };
    context.expected_context = &context;

    const bbtc_eb_environment_provider_double_t provider =
    {
        .sample = tracking_double_provider,
        .context = &context
    };

    bbtc_eb_vec3_double_t position = {.x = NAN, .y = 0.0, .z = 0.0};
    bbtc_eb_environment_sample_double_t sample;

    write_valid_double_sample(&sample);

    if (bbtc_eb_environment_provider_sample_double(
            &provider,
            &position,
            INFINITY,
            &sample
        ) != BBTC_STATUS_NAN_INPUT
        || calls != 0U
        || !double_sample_is_zero(&sample))
    {
        return EXIT_FAILURE;
    }

    position.x = 0.0;
    write_valid_double_sample(&sample);

    if (bbtc_eb_environment_provider_sample_double(
            &provider,
            &position,
            NAN,
            &sample
        ) != BBTC_STATUS_NAN_INPUT
        || calls != 0U
        || !double_sample_is_zero(&sample))
    {
        return EXIT_FAILURE;
    }

    position.x = INFINITY;
    write_valid_double_sample(&sample);

    if (bbtc_eb_environment_provider_sample_double(
            &provider,
            &position,
            0.0,
            &sample
        ) != BBTC_STATUS_NONFINITE_INPUT
        || calls != 0U
        || !double_sample_is_zero(&sample))
    {
        return EXIT_FAILURE;
    }

    position.x = 0.0;
    write_valid_double_sample(&sample);

    if (bbtc_eb_environment_provider_sample_double(
            &provider,
            &position,
            INFINITY,
            &sample
        ) != BBTC_STATUS_NONFINITE_INPUT
        || calls != 0U
        || !double_sample_is_zero(&sample))
    {
        return EXIT_FAILURE;
    }

    write_valid_double_sample(&sample);

    if (bbtc_eb_environment_provider_sample_double(
            &provider,
            &position,
            -1.0,
            &sample
        ) != BBTC_STATUS_OUTSIDE_DOMAIN
        || calls != 0U
        || !double_sample_is_zero(&sample))
    {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

/**
 * @brief Verifies provider-status propagation and B4 output revalidation.
 */
static int
test_callback_and_output_failures(void)
{
    const bbtc_eb_vec3_double_t position = {.x = 0.0, .y = 0.0, .z = 0.0};
    bbtc_eb_environment_sample_double_t sample;

    const bbtc_eb_environment_provider_double_t failing_provider =
    {
        .sample = failing_double_provider,
        .context = NULL
    };

    if (bbtc_eb_environment_provider_sample_double(
            &failing_provider,
            &position,
            0.0,
            &sample
        ) != BBTC_STATUS_UNSUPPORTED_MODEL_OR_OPTION
        || !double_sample_is_zero(&sample))
    {
        return EXIT_FAILURE;
    }

    const bbtc_eb_environment_provider_double_t invalid_provider =
    {
        .sample = invalid_output_double_provider,
        .context = NULL
    };

    return bbtc_eb_environment_provider_sample_double(
               &invalid_provider,
               &position,
               0.0,
               &sample
           ) == BBTC_STATUS_OUTSIDE_DOMAIN
            && double_sample_is_zero(&sample)
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}

/**
 * @brief Verifies repeated out-of-order sampling has no hidden history.
 */
static int
test_out_of_order_sampling(void)
{
    const bbtc_eb_environment_provider_double_t provider =
    {
        .sample = functional_double_provider,
        .context = NULL
    };
    const bbtc_eb_vec3_double_t position = {.x = 10.0, .y = 20.0, .z = 30.0};
    bbtc_eb_environment_sample_double_t sample = {0};

    const double times[] = {2.0, 0.0, 1.0};
    const double expected_x[] = {12.0, 10.0, 11.0};
    const double expected_y[] = {18.0, 20.0, 19.0};

    for (size_t i = 0; i < sizeof(times) / sizeof(times[0]); ++i)
    {
        if (bbtc_eb_environment_provider_sample_double(
                &provider,
                &position,
                times[i],
                &sample
            ) != BBTC_STATUS_SUCCESS
            || sample.air_velocity_world_m_per_s.x != expected_x[i]
            || sample.air_velocity_world_m_per_s.y != expected_y[i]
            || sample.air_velocity_world_m_per_s.z != 30.0)
        {
            return EXIT_FAILURE;
        }
    }

    return EXIT_SUCCESS;
}

/**
 * @brief Exercises native float and long-double provider families.
 */
static int
test_other_precision_families(void)
{
    const bbtc_eb_environment_provider_float_t float_descriptor =
    {
        .sample = float_provider,
        .context = NULL
    };
    const bbtc_eb_vec3_float_t float_position = {.x = 1.0f, .y = 2.0f, .z = 3.0f};
    bbtc_eb_environment_sample_float_t float_sample = {0};

    const bbtc_eb_environment_provider_long_double_t long_double_descriptor =
    {
        .sample = long_double_provider,
        .context = NULL
    };
    const bbtc_eb_vec3_long_double_t long_double_position =
    {
        .x = 1.0L,
        .y = 2.0L,
        .z = 3.0L
    };
    bbtc_eb_environment_sample_long_double_t long_double_sample = {0};

    return bbtc_eb_environment_provider_sample_float(
               &float_descriptor,
               &float_position,
               0.5f,
               &float_sample
           ) == BBTC_STATUS_SUCCESS
        && float_sample.temperature_k == 300.0f
        && bbtc_eb_environment_provider_sample_long_double(
               &long_double_descriptor,
               &long_double_position,
               0.5L,
               &long_double_sample
           ) == BBTC_STATUS_SUCCESS
        && long_double_sample.air_density_kg_per_m3 == 0.0L
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}

/**
 * @brief Runs the EB0.1-B5 provider-contract test suite.
 */
int
main(void)
{
    if (test_valid_dispatch() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_null_arguments() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_request_validation() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_callback_and_output_failures() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_out_of_order_sampling() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    return test_other_precision_families();
}
