/**
 * @file
 * @brief Tests EB0.1-B7 reference-frame provider representation and dispatch.
 *
 * The suite freezes callback and descriptor representation, native precision,
 * exact request forwarding, deterministic validation precedence, zero-before-
 * callback behavior, callback-status propagation, B6 output revalidation,
 * successful zero-sample behavior, fail-closed output restoration, repeated
 * out-of-order sampling, and the rule that returned status—not zero-valued B6
 * storage—is authoritative.
 */

#include <bbtc/bbtc.h>

#include <math.h>
#include <stddef.h>
#include <stdlib.h>

static_assert(_Generic(
    ((bbtc_eb_reference_frame_provider_float_t*)0)->sample,
    bbtc_eb_reference_frame_provider_float_fn: 1,
    default: 0
));
static_assert(_Generic(
    ((bbtc_eb_reference_frame_provider_double_t*)0)->sample,
    bbtc_eb_reference_frame_provider_double_fn: 1,
    default: 0
));
static_assert(_Generic(
    ((bbtc_eb_reference_frame_provider_long_double_t*)0)->sample,
    bbtc_eb_reference_frame_provider_long_double_fn: 1,
    default: 0
));
static_assert(_Generic(
    ((bbtc_eb_reference_frame_provider_double_t*)0)->context,
    const void*: 1,
    default: 0
));

#define BBTC_EB_ASSERT_REFERENCE_FRAME_PROVIDER_ORDER(TYPE)                \
    static_assert(offsetof(TYPE, sample) < offsetof(TYPE, context))

BBTC_EB_ASSERT_REFERENCE_FRAME_PROVIDER_ORDER(
    bbtc_eb_reference_frame_provider_float_t
);
BBTC_EB_ASSERT_REFERENCE_FRAME_PROVIDER_ORDER(
    bbtc_eb_reference_frame_provider_double_t
);
BBTC_EB_ASSERT_REFERENCE_FRAME_PROVIDER_ORDER(
    bbtc_eb_reference_frame_provider_long_double_t
);

#undef BBTC_EB_ASSERT_REFERENCE_FRAME_PROVIDER_ORDER

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
 * @brief Reports whether every named field of a native-double B6 sample is zero.
 */
static int
double_sample_is_zero(
    const bbtc_eb_reference_frame_sample_double_t* sample
)
{
    return sample->gravitational_acceleration_reference_m_per_s2.x == 0.0
        && sample->gravitational_acceleration_reference_m_per_s2.y == 0.0
        && sample->gravitational_acceleration_reference_m_per_s2.z == 0.0
        && sample
               ->reference_frame_origin_acceleration_reference_m_per_s2.x
            == 0.0
        && sample
               ->reference_frame_origin_acceleration_reference_m_per_s2.y
            == 0.0
        && sample
               ->reference_frame_origin_acceleration_reference_m_per_s2.z
            == 0.0
        && sample
               ->reference_frame_angular_velocity_reference_rad_per_s.x
            == 0.0
        && sample
               ->reference_frame_angular_velocity_reference_rad_per_s.y
            == 0.0
        && sample
               ->reference_frame_angular_velocity_reference_rad_per_s.z
            == 0.0
        && sample
               ->reference_frame_angular_acceleration_reference_rad_per_s2.x
            == 0.0
        && sample
               ->reference_frame_angular_acceleration_reference_rad_per_s2.y
            == 0.0
        && sample
               ->reference_frame_angular_acceleration_reference_rad_per_s2.z
            == 0.0;
}

/**
 * @brief Writes one representative valid nonzero native-double B6 sample.
 */
static void
write_valid_double_sample(
    bbtc_eb_reference_frame_sample_double_t* out_sample
)
{
    *out_sample = (bbtc_eb_reference_frame_sample_double_t)
    {
        .gravitational_acceleration_reference_m_per_s2 =
        {
            .x = 0.125,
            .y = -0.250,
            .z = 9.750
        },
        .reference_frame_origin_acceleration_reference_m_per_s2 =
        {
            .x = -0.015,
            .y = 0.020,
            .z = 0.005
        },
        .reference_frame_angular_velocity_reference_rad_per_s =
        {
            .x = 0.0,
            .y = 7.0e-5,
            .z = -2.0e-5
        },
        .reference_frame_angular_acceleration_reference_rad_per_s2 =
        {
            .x = 1.0e-7,
            .y = -2.0e-7,
            .z = 3.0e-7
        }
    };
}

/**
 * @brief Verifies exact callback inputs and zero-before-callback behavior.
 */
static bbtc_status_e
tracking_double_provider(
    const void* context_pointer,
    const bbtc_eb_vec3_double_t* position_reference_m,
    double time_s,
    bbtc_eb_reference_frame_sample_double_t* out_sample
)
{
    const tracking_context_t* const context = context_pointer;

    if (context == NULL
        || context->expected_context != context_pointer
        || !double_sample_is_zero(out_sample)
        || position_reference_m->x != context->expected_x
        || position_reference_m->y != context->expected_y
        || position_reference_m->z != context->expected_z
        || time_s != context->expected_time_s)
    {
        return BBTC_STATUS_INTERNAL_INVARIANT_FAILURE;
    }

    ++*context->call_count;
    write_valid_double_sample(out_sample);
    return BBTC_STATUS_SUCCESS;
}

/**
 * @brief Accepts only a null context and writes a valid nonzero B6 sample.
 */
static bbtc_status_e
null_context_double_provider(
    const void* context,
    const bbtc_eb_vec3_double_t* position_reference_m,
    double time_s,
    bbtc_eb_reference_frame_sample_double_t* out_sample
)
{
    (void)position_reference_m;
    (void)time_s;

    if (context != NULL)
        return BBTC_STATUS_INTERNAL_INVARIANT_FAILURE;

    write_valid_double_sample(out_sample);
    return BBTC_STATUS_SUCCESS;
}

/**
 * @brief Writes nonzero output and deliberately returns provider failure.
 */
static bbtc_status_e
failing_double_provider(
    const void* context,
    const bbtc_eb_vec3_double_t* position_reference_m,
    double time_s,
    bbtc_eb_reference_frame_sample_double_t* out_sample
)
{
    (void)context;
    (void)position_reference_m;
    (void)time_s;

    write_valid_double_sample(out_sample);
    return BBTC_STATUS_UNSUPPORTED_MODEL_OR_OPTION;
}

/**
 * @brief Returns success with a deliberately B6-invalid NaN sample.
 */
static bbtc_status_e
invalid_output_double_provider(
    const void* context,
    const bbtc_eb_vec3_double_t* position_reference_m,
    double time_s,
    bbtc_eb_reference_frame_sample_double_t* out_sample
)
{
    (void)context;
    (void)position_reference_m;
    (void)time_s;

    write_valid_double_sample(out_sample);
    out_sample->gravitational_acceleration_reference_m_per_s2.x = NAN;
    return BBTC_STATUS_SUCCESS;
}

/**
 * @brief Returns success while deliberately preserving the exact zero sample.
 *
 * This verifies the B7 distinction between a successful inertial zero-gravity
 * sample and the identical storage state used after dispatch failure. Only the
 * returned status distinguishes the two outcomes.
 */
static bbtc_status_e
zero_success_double_provider(
    const void* context,
    const bbtc_eb_vec3_double_t* position_reference_m,
    double time_s,
    bbtc_eb_reference_frame_sample_double_t* out_sample
)
{
    (void)context;
    (void)position_reference_m;
    (void)time_s;

    *out_sample = (bbtc_eb_reference_frame_sample_double_t){0};
    return BBTC_STATUS_SUCCESS;
}

/**
 * @brief Produces a pure position/time-derived B6 sample for order tests.
 */
static bbtc_status_e
functional_double_provider(
    const void* context,
    const bbtc_eb_vec3_double_t* position_reference_m,
    double time_s,
    bbtc_eb_reference_frame_sample_double_t* out_sample
)
{
    (void)context;

    *out_sample = (bbtc_eb_reference_frame_sample_double_t){0};

    out_sample->gravitational_acceleration_reference_m_per_s2.x =
        position_reference_m->x + time_s;
    out_sample->gravitational_acceleration_reference_m_per_s2.y =
        position_reference_m->y - time_s;
    out_sample->gravitational_acceleration_reference_m_per_s2.z =
        position_reference_m->z;
    out_sample->reference_frame_origin_acceleration_reference_m_per_s2.x =
        time_s;
    out_sample->reference_frame_angular_velocity_reference_rad_per_s.y =
        position_reference_m->x;
    out_sample->reference_frame_angular_acceleration_reference_rad_per_s2.z =
        -time_s;

    return BBTC_STATUS_SUCCESS;
}

/**
 * @brief Produces one valid native-float B6 sample for dispatch coverage.
 */
static bbtc_status_e
float_provider(
    const void* context,
    const bbtc_eb_vec3_float_t* position_reference_m,
    float time_s,
    bbtc_eb_reference_frame_sample_float_t* out_sample
)
{
    (void)context;
    (void)position_reference_m;

    *out_sample = (bbtc_eb_reference_frame_sample_float_t){0};
    out_sample->gravitational_acceleration_reference_m_per_s2.z = -9.0f;
    out_sample->reference_frame_angular_velocity_reference_rad_per_s.x =
        time_s;

    return BBTC_STATUS_SUCCESS;
}

/**
 * @brief Produces one valid native-long-double B6 sample for dispatch coverage.
 */
static bbtc_status_e
long_double_provider(
    const void* context,
    const bbtc_eb_vec3_long_double_t* position_reference_m,
    long double time_s,
    bbtc_eb_reference_frame_sample_long_double_t* out_sample
)
{
    (void)context;

    *out_sample = (bbtc_eb_reference_frame_sample_long_double_t){0};
    out_sample->gravitational_acceleration_reference_m_per_s2.x =
        position_reference_m->x;
    out_sample->reference_frame_origin_acceleration_reference_m_per_s2.y =
        position_reference_m->y;
    out_sample->reference_frame_angular_acceleration_reference_rad_per_s2.z =
        time_s;

    return BBTC_STATUS_SUCCESS;
}

/**
 * @brief Verifies exact forwarding, one callback, and valid output preservation.
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

    const bbtc_eb_reference_frame_provider_double_t provider =
    {
        .sample = tracking_double_provider,
        .context = &context
    };
    const bbtc_eb_vec3_double_t position =
    {
        .x = context.expected_x,
        .y = context.expected_y,
        .z = context.expected_z
    };
    bbtc_eb_reference_frame_sample_double_t sample;

    write_valid_double_sample(&sample);

    if (bbtc_eb_reference_frame_provider_sample_double(
            &provider,
            &position,
            context.expected_time_s,
            &sample
        ) != BBTC_STATUS_SUCCESS
        || calls != 1U)
    {
        return EXIT_FAILURE;
    }

    if (sample.gravitational_acceleration_reference_m_per_s2.x != 0.125
        || sample.gravitational_acceleration_reference_m_per_s2.y != -0.250
        || sample.gravitational_acceleration_reference_m_per_s2.z != 9.750
        || sample
               .reference_frame_origin_acceleration_reference_m_per_s2.x
            != -0.015
        || sample.reference_frame_angular_velocity_reference_rad_per_s.y
            != 7.0e-5
        || sample
               .reference_frame_angular_acceleration_reference_rad_per_s2.z
            != 3.0e-7)
    {
        return EXIT_FAILURE;
    }

    const bbtc_eb_reference_frame_provider_double_t null_context_provider =
    {
        .sample = null_context_double_provider,
        .context = NULL
    };

    return bbtc_eb_reference_frame_provider_sample_double(
               &null_context_provider,
               &position,
               0.0,
               &sample
           ) == BBTC_STATUS_SUCCESS
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}

/**
 * @brief Verifies null-pointer precedence and fail-closed non-null output.
 */
static int
test_null_arguments(void)
{
    const bbtc_eb_vec3_double_t position = {0};
    const bbtc_eb_reference_frame_provider_double_t null_callback_provider =
    {
        .sample = NULL,
        .context = NULL
    };
    const bbtc_eb_reference_frame_provider_double_t valid_provider =
    {
        .sample = null_context_double_provider,
        .context = NULL
    };
    bbtc_eb_reference_frame_sample_double_t sample;

    write_valid_double_sample(&sample);

    if (bbtc_eb_reference_frame_provider_sample_double(
            NULL,
            &position,
            NAN,
            &sample
        ) != BBTC_STATUS_INVALID_ARGUMENT
        || !double_sample_is_zero(&sample))
    {
        return EXIT_FAILURE;
    }

    write_valid_double_sample(&sample);

    if (bbtc_eb_reference_frame_provider_sample_double(
            &null_callback_provider,
            &position,
            INFINITY,
            &sample
        ) != BBTC_STATUS_INVALID_ARGUMENT
        || !double_sample_is_zero(&sample))
    {
        return EXIT_FAILURE;
    }

    write_valid_double_sample(&sample);

    if (bbtc_eb_reference_frame_provider_sample_double(
            &valid_provider,
            NULL,
            NAN,
            &sample
        ) != BBTC_STATUS_INVALID_ARGUMENT
        || !double_sample_is_zero(&sample))
    {
        return EXIT_FAILURE;
    }

    return bbtc_eb_reference_frame_provider_sample_double(
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

    const bbtc_eb_reference_frame_provider_double_t provider =
    {
        .sample = tracking_double_provider,
        .context = &context
    };

    bbtc_eb_vec3_double_t position = {.x = NAN, .y = 0.0, .z = 0.0};
    bbtc_eb_reference_frame_sample_double_t sample;

    write_valid_double_sample(&sample);

    if (bbtc_eb_reference_frame_provider_sample_double(
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

    if (bbtc_eb_reference_frame_provider_sample_double(
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

    if (bbtc_eb_reference_frame_provider_sample_double(
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

    position.x = -INFINITY;
    write_valid_double_sample(&sample);

    if (bbtc_eb_reference_frame_provider_sample_double(
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

    if (bbtc_eb_reference_frame_provider_sample_double(
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

    if (bbtc_eb_reference_frame_provider_sample_double(
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
 * @brief Verifies callback failure, B6 revalidation, and zero restoration.
 */
static int
test_callback_and_output_failures(void)
{
    const bbtc_eb_vec3_double_t position = {0};
    bbtc_eb_reference_frame_sample_double_t sample;

    const bbtc_eb_reference_frame_provider_double_t failing_provider =
    {
        .sample = failing_double_provider,
        .context = NULL
    };

    write_valid_double_sample(&sample);

    const bbtc_status_e provider_status =
        bbtc_eb_reference_frame_provider_sample_double(
            &failing_provider,
            &position,
            0.0,
            &sample
        );

    if (provider_status != BBTC_STATUS_UNSUPPORTED_MODEL_OR_OPTION
        || !double_sample_is_zero(&sample)
        || bbtc_eb_reference_frame_sample_validate_double(&sample)
            != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    const bbtc_eb_reference_frame_provider_double_t invalid_provider =
    {
        .sample = invalid_output_double_provider,
        .context = NULL
    };

    write_valid_double_sample(&sample);

    return bbtc_eb_reference_frame_provider_sample_double(
               &invalid_provider,
               &position,
               0.0,
               &sample
           ) == BBTC_STATUS_NAN_INPUT
            && double_sample_is_zero(&sample)
            && bbtc_eb_reference_frame_sample_validate_double(&sample)
                == BBTC_STATUS_SUCCESS
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}

/**
 * @brief Verifies an exact zero B6 sample may be returned successfully.
 */
static int
test_successful_zero_sample(void)
{
    const bbtc_eb_reference_frame_provider_double_t provider =
    {
        .sample = zero_success_double_provider,
        .context = NULL
    };
    const bbtc_eb_vec3_double_t position = {0};
    bbtc_eb_reference_frame_sample_double_t sample;

    write_valid_double_sample(&sample);

    return bbtc_eb_reference_frame_provider_sample_double(
               &provider,
               &position,
               0.0,
               &sample
           ) == BBTC_STATUS_SUCCESS
            && double_sample_is_zero(&sample)
            && bbtc_eb_reference_frame_sample_validate_double(&sample)
                == BBTC_STATUS_SUCCESS
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}

/**
 * @brief Verifies repeated out-of-order sampling has no hidden history.
 */
static int
test_out_of_order_sampling(void)
{
    const bbtc_eb_reference_frame_provider_double_t provider =
    {
        .sample = functional_double_provider,
        .context = NULL
    };
    const bbtc_eb_vec3_double_t position = {.x = 10.0, .y = 20.0, .z = 30.0};
    bbtc_eb_reference_frame_sample_double_t sample = {0};

    const double times[] = {2.0, 0.0, 1.0};
    const double expected_x[] = {12.0, 10.0, 11.0};
    const double expected_y[] = {18.0, 20.0, 19.0};

    for (size_t i = 0; i < sizeof(times) / sizeof(times[0]); ++i)
    {
        if (bbtc_eb_reference_frame_provider_sample_double(
                &provider,
                &position,
                times[i],
                &sample
            ) != BBTC_STATUS_SUCCESS
            || sample.gravitational_acceleration_reference_m_per_s2.x
                != expected_x[i]
            || sample.gravitational_acceleration_reference_m_per_s2.y
                != expected_y[i]
            || sample.gravitational_acceleration_reference_m_per_s2.z != 30.0
            || sample
                   .reference_frame_origin_acceleration_reference_m_per_s2.x
                != times[i]
            || sample.reference_frame_angular_velocity_reference_rad_per_s.y
                != 10.0
            || sample
                   .reference_frame_angular_acceleration_reference_rad_per_s2.z
                != -times[i])
        {
            return EXIT_FAILURE;
        }
    }

    return EXIT_SUCCESS;
}

/**
 * @brief Exercises native float and long-double B7 provider families.
 */
static int
test_other_precision_families(void)
{
    const bbtc_eb_reference_frame_provider_float_t float_descriptor =
    {
        .sample = float_provider,
        .context = NULL
    };
    const bbtc_eb_vec3_float_t float_position =
    {
        .x = 1.0f,
        .y = 2.0f,
        .z = 3.0f
    };
    bbtc_eb_reference_frame_sample_float_t float_sample = {0};

    const bbtc_eb_reference_frame_provider_long_double_t long_double_descriptor =
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
    bbtc_eb_reference_frame_sample_long_double_t long_double_sample = {0};

    return bbtc_eb_reference_frame_provider_sample_float(
               &float_descriptor,
               &float_position,
               0.5f,
               &float_sample
           ) == BBTC_STATUS_SUCCESS
        && float_sample.gravitational_acceleration_reference_m_per_s2.z
            == -9.0f
        && float_sample.reference_frame_angular_velocity_reference_rad_per_s.x
            == 0.5f
        && bbtc_eb_reference_frame_provider_sample_long_double(
               &long_double_descriptor,
               &long_double_position,
               0.5L,
               &long_double_sample
           ) == BBTC_STATUS_SUCCESS
        && long_double_sample.gravitational_acceleration_reference_m_per_s2.x
            == 1.0L
        && long_double_sample
               .reference_frame_origin_acceleration_reference_m_per_s2.y
            == 2.0L
        && long_double_sample
               .reference_frame_angular_acceleration_reference_rad_per_s2.z
            == 0.5L
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}

/**
 * @brief Runs the complete EB0.1-B7 provider-contract test suite.
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

    if (test_successful_zero_sample() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_out_of_order_sampling() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    return test_other_precision_families();
}
