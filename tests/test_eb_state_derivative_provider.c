/**
 * @file
 * @brief Tests the EB0.1-B10 state-derivative provider contract.
 */

#include <bbtc/bbtc.h>

#include <math.h>
#include <stddef.h>
#include <stdlib.h>

static_assert(_Generic(
    ((bbtc_eb_state_derivative_provider_float_t*)0)->evaluate,
    bbtc_eb_state_derivative_provider_float_fn: 1,
    default: 0
));
static_assert(_Generic(
    ((bbtc_eb_state_derivative_provider_double_t*)0)->evaluate,
    bbtc_eb_state_derivative_provider_double_fn: 1,
    default: 0
));
static_assert(_Generic(
    ((bbtc_eb_state_derivative_provider_long_double_t*)0)->evaluate,
    bbtc_eb_state_derivative_provider_long_double_fn: 1,
    default: 0
));

static_assert(_Generic(
    ((bbtc_eb_state_derivative_provider_float_t*)0)->context,
    const void*: 1,
    default: 0
));
static_assert(_Generic(
    ((bbtc_eb_state_derivative_provider_double_t*)0)->context,
    const void*: 1,
    default: 0
));
static_assert(_Generic(
    ((bbtc_eb_state_derivative_provider_long_double_t*)0)->context,
    const void*: 1,
    default: 0
));

static_assert(
    offsetof(bbtc_eb_state_derivative_provider_float_t, evaluate)
        < offsetof(bbtc_eb_state_derivative_provider_float_t, context)
);
static_assert(
    offsetof(bbtc_eb_state_derivative_provider_double_t, evaluate)
        < offsetof(bbtc_eb_state_derivative_provider_double_t, context)
);
static_assert(
    offsetof(bbtc_eb_state_derivative_provider_long_double_t, evaluate)
        < offsetof(bbtc_eb_state_derivative_provider_long_double_t, context)
);

typedef enum test_mode_e
{
    TEST_NONZERO_SUCCESS = 0,
    TEST_ZERO_SUCCESS,
    TEST_FAILURE_WITH_NAN,
    TEST_NAN_SUCCESS,
    TEST_INF_SUCCESS
}
test_mode_e;

typedef struct test_context_t
{
    int calls;
    int saw_zero;
    int context_ok;
    const void* expected_context;
    test_mode_e mode;
    double times[8];
    size_t time_count;
}
test_context_t;

static bbtc_eb_rigid_body_state_double_t
valid_state(double time_s)
{
    return (bbtc_eb_rigid_body_state_double_t)
    {
        .orientation_world_from_body = {.w = 1.0},
        .position_m = {.x = 10.0, .y = 20.0, .z = 30.0},
        .velocity_m_per_s = {.x = 700.0, .y = -2.0, .z = 1.0},
        .angular_velocity_body_rad_per_s = {.x = 100.0, .y = -3.0, .z = 4.0},
        .time_s = time_s
    };
}

static int
is_zero(const bbtc_eb_rigid_body_state_derivative_double_t* d)
{
    return d->orientation_world_from_body_derivative_per_s.x == 0.0
        && d->orientation_world_from_body_derivative_per_s.y == 0.0
        && d->orientation_world_from_body_derivative_per_s.z == 0.0
        && d->orientation_world_from_body_derivative_per_s.w == 0.0
        && d->position_derivative_reference_m_per_s.x == 0.0
        && d->position_derivative_reference_m_per_s.y == 0.0
        && d->position_derivative_reference_m_per_s.z == 0.0
        && d->velocity_derivative_reference_m_per_s2.x == 0.0
        && d->velocity_derivative_reference_m_per_s2.y == 0.0
        && d->velocity_derivative_reference_m_per_s2.z == 0.0
        && d->angular_velocity_derivative_body_rad_per_s2.x == 0.0
        && d->angular_velocity_derivative_body_rad_per_s2.y == 0.0
        && d->angular_velocity_derivative_body_rad_per_s2.z == 0.0;
}

static bbtc_status_e
double_callback(
    const void* context,
    const bbtc_eb_rigid_body_state_double_t* state,
    bbtc_eb_rigid_body_state_derivative_double_t* out
)
{
    test_context_t* c = (test_context_t*)context;

    if (c != NULL)
    {
        ++c->calls;
        c->context_ok = context == c->expected_context;
        c->saw_zero = is_zero(out);

        if (c->time_count < sizeof(c->times) / sizeof(c->times[0]))
            c->times[c->time_count++] = state->time_s;
    }

    const test_mode_e mode = c != NULL ? c->mode : TEST_ZERO_SUCCESS;

    if (mode == TEST_ZERO_SUCCESS)
        return BBTC_STATUS_SUCCESS;

    out->orientation_world_from_body_derivative_per_s.x = 2.0;
    out->orientation_world_from_body_derivative_per_s.w = -5.0;
    out->position_derivative_reference_m_per_s.x = 700.0;
    out->velocity_derivative_reference_m_per_s2.z = -9.81;
    out->angular_velocity_derivative_body_rad_per_s2.y = 4.0;

    if (mode == TEST_FAILURE_WITH_NAN)
    {
        out->velocity_derivative_reference_m_per_s2.x = NAN;
        return BBTC_STATUS_OUTSIDE_DOMAIN;
    }

    if (mode == TEST_NAN_SUCCESS)
        out->velocity_derivative_reference_m_per_s2.x = NAN;

    if (mode == TEST_INF_SUCCESS)
        out->angular_velocity_derivative_body_rad_per_s2.z = INFINITY;

    return BBTC_STATUS_SUCCESS;
}

static bbtc_status_e
float_callback(
    const void* context,
    const bbtc_eb_rigid_body_state_float_t* state,
    bbtc_eb_rigid_body_state_derivative_float_t* out
)
{
    (void)context;
    out->position_derivative_reference_m_per_s.x = state->velocity_m_per_s.x;
    return BBTC_STATUS_SUCCESS;
}

static bbtc_status_e
long_double_callback(
    const void* context,
    const bbtc_eb_rigid_body_state_long_double_t* state,
    bbtc_eb_rigid_body_state_derivative_long_double_t* out
)
{
    (void)context;
    out->position_derivative_reference_m_per_s.x = state->velocity_m_per_s.x;
    return BBTC_STATUS_SUCCESS;
}

/* These typed assignments freeze the callback signatures: no time parameter. */
static const bbtc_eb_state_derivative_provider_float_fn float_fn =
    float_callback;
static const bbtc_eb_state_derivative_provider_double_fn double_fn =
    double_callback;
static const bbtc_eb_state_derivative_provider_long_double_fn long_double_fn =
    long_double_callback;

static int
test_invalid_request_and_state(void)
{
    bbtc_eb_rigid_body_state_double_t state = valid_state(1.0);
    test_context_t c = {.mode = TEST_NONZERO_SUCCESS};
    c.expected_context = &c;

    const bbtc_eb_state_derivative_provider_double_t provider =
    {
        .evaluate = double_fn,
        .context = &c
    };

    bbtc_eb_rigid_body_state_derivative_double_t out =
    {
        .position_derivative_reference_m_per_s = {.x = 123.0}
    };

    if (bbtc_eb_state_derivative_provider_evaluate_double(
            &provider, &state, NULL
        ) != BBTC_STATUS_INVALID_ARGUMENT
        || c.calls != 0)
    {
        return EXIT_FAILURE;
    }

    if (bbtc_eb_state_derivative_provider_evaluate_double(
            NULL, &state, &out
        ) != BBTC_STATUS_INVALID_ARGUMENT
        || !is_zero(&out))
    {
        return EXIT_FAILURE;
    }

    const bbtc_eb_state_derivative_provider_double_t no_callback =
    {
        .evaluate = NULL,
        .context = &c
    };
    out.position_derivative_reference_m_per_s.x = 123.0;

    if (bbtc_eb_state_derivative_provider_evaluate_double(
            &no_callback, &state, &out
        ) != BBTC_STATUS_INVALID_ARGUMENT
        || !is_zero(&out))
    {
        return EXIT_FAILURE;
    }

    out.position_derivative_reference_m_per_s.x = 123.0;
    if (bbtc_eb_state_derivative_provider_evaluate_double(
            &provider, NULL, &out
        ) != BBTC_STATUS_INVALID_ARGUMENT
        || !is_zero(&out))
    {
        return EXIT_FAILURE;
    }

    state = valid_state(1.0);
    state.velocity_m_per_s.x = NAN;
    if (bbtc_eb_state_derivative_provider_evaluate_double(
            &provider, &state, &out
        ) != BBTC_STATUS_NAN_INPUT
        || c.calls != 0
        || !is_zero(&out))
    {
        return EXIT_FAILURE;
    }

    state = valid_state(1.0);
    state.position_m.z = INFINITY;
    if (bbtc_eb_state_derivative_provider_evaluate_double(
            &provider, &state, &out
        ) != BBTC_STATUS_NONFINITE_INPUT
        || c.calls != 0
        || !is_zero(&out))
    {
        return EXIT_FAILURE;
    }

    state = valid_state(-1.0);
    if (bbtc_eb_state_derivative_provider_evaluate_double(
            &provider, &state, &out
        ) != BBTC_STATUS_OUTSIDE_DOMAIN
        || c.calls != 0
        || !is_zero(&out))
    {
        return EXIT_FAILURE;
    }

    state = valid_state(1.0);
    state.orientation_world_from_body = (bbtc_eb_quat_double_t){0};
    if (bbtc_eb_state_derivative_provider_evaluate_double(
            &provider, &state, &out
        ) != BBTC_STATUS_OUTSIDE_DOMAIN
        || c.calls != 0
        || !is_zero(&out))
    {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

static int
test_success_semantics(void)
{
    bbtc_eb_rigid_body_state_double_t state = valid_state(2.5);
    test_context_t c = {.mode = TEST_NONZERO_SUCCESS};
    c.expected_context = &c;

    const bbtc_eb_state_derivative_provider_double_t provider =
    {
        .evaluate = double_fn,
        .context = &c
    };

    bbtc_eb_rigid_body_state_derivative_double_t out =
    {
        .position_derivative_reference_m_per_s = {.x = 999.0}
    };

    if (bbtc_eb_state_derivative_provider_evaluate_double(
            &provider, &state, &out
        ) != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (c.calls != 1
        || !c.saw_zero
        || !c.context_ok
        || c.time_count != 1
        || c.times[0] != 2.5
        || out.position_derivative_reference_m_per_s.x != 700.0
        || out.velocity_derivative_reference_m_per_s2.z != -9.81)
    {
        return EXIT_FAILURE;
    }

    const bbtc_eb_state_derivative_provider_double_t null_context_provider =
    {
        .evaluate = double_fn,
        .context = NULL
    };

    state = valid_state(-0.0);
    out.position_derivative_reference_m_per_s.x = 456.0;

    return bbtc_eb_state_derivative_provider_evaluate_double(
               &null_context_provider, &state, &out
           ) == BBTC_STATUS_SUCCESS
            && is_zero(&out)
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}

static int
test_failure_and_output_validation(void)
{
    const bbtc_eb_rigid_body_state_double_t state = valid_state(3.0);
    test_context_t c = {.mode = TEST_FAILURE_WITH_NAN};
    c.expected_context = &c;

    bbtc_eb_state_derivative_provider_double_t provider =
    {
        .evaluate = double_fn,
        .context = &c
    };
    bbtc_eb_rigid_body_state_derivative_double_t out = {0};

    if (bbtc_eb_state_derivative_provider_evaluate_double(
            &provider, &state, &out
        ) != BBTC_STATUS_OUTSIDE_DOMAIN
        || !is_zero(&out))
    {
        return EXIT_FAILURE;
    }

    c.mode = TEST_NAN_SUCCESS;
    if (bbtc_eb_state_derivative_provider_evaluate_double(
            &provider, &state, &out
        ) != BBTC_STATUS_NAN_INPUT
        || !is_zero(&out))
    {
        return EXIT_FAILURE;
    }

    c.mode = TEST_INF_SUCCESS;
    return bbtc_eb_state_derivative_provider_evaluate_double(
               &provider, &state, &out
           ) == BBTC_STATUS_NONFINITE_INPUT
            && is_zero(&out)
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}

static int
test_order_independence(void)
{
    test_context_t c = {.mode = TEST_ZERO_SUCCESS};
    c.expected_context = &c;

    const bbtc_eb_state_derivative_provider_double_t provider =
    {
        .evaluate = double_fn,
        .context = &c
    };

    const double times[] = {5.0, 5.0, 4.5, 7.0, 6.0};
    bbtc_eb_rigid_body_state_derivative_double_t out = {0};

    for (size_t i = 0; i < sizeof(times) / sizeof(times[0]); ++i)
    {
        bbtc_eb_rigid_body_state_double_t state = valid_state(times[i]);
        state.position_m.x = (double)i;

        if (bbtc_eb_state_derivative_provider_evaluate_double(
                &provider, &state, &out
            ) != BBTC_STATUS_SUCCESS
            || !is_zero(&out))
        {
            return EXIT_FAILURE;
        }
    }

    if (c.calls != 5 || c.time_count != 5)
        return EXIT_FAILURE;

    for (size_t i = 0; i < 5; ++i)
    {
        if (c.times[i] != times[i])
            return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

static int
test_other_precisions(void)
{
    const bbtc_eb_state_derivative_provider_float_t fp =
    {
        .evaluate = float_fn,
        .context = NULL
    };
    const bbtc_eb_state_derivative_provider_long_double_t lp =
    {
        .evaluate = long_double_fn,
        .context = NULL
    };

    const bbtc_eb_rigid_body_state_float_t fs =
    {
        .orientation_world_from_body = {.w = 1.0f},
        .velocity_m_per_s = {.x = 12.5f},
        .time_s = 1.0f
    };
    const bbtc_eb_rigid_body_state_long_double_t ls =
    {
        .orientation_world_from_body = {.w = 1.0L},
        .velocity_m_per_s = {.x = 25.0L},
        .time_s = 2.0L
    };

    bbtc_eb_rigid_body_state_derivative_float_t fo = {0};
    bbtc_eb_rigid_body_state_derivative_long_double_t lo = {0};

    return bbtc_eb_state_derivative_provider_evaluate_float(&fp, &fs, &fo)
            == BBTC_STATUS_SUCCESS
            && fo.position_derivative_reference_m_per_s.x == 12.5f
            && bbtc_eb_state_derivative_provider_evaluate_long_double(
                   &lp, &ls, &lo
               ) == BBTC_STATUS_SUCCESS
            && lo.position_derivative_reference_m_per_s.x == 25.0L
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}

int
main(void)
{
    if (test_invalid_request_and_state() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_success_semantics() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_failure_and_output_validation() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_order_independence() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    return test_other_precisions();
}
