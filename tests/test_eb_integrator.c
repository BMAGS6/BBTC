/**
 * @file
 * @brief Tests EB0.1-B11 transactional fixed-step RK4 integration.
 *
 * These tests exercise the numerical integrator independently of EB1 physical
 * equations. Synthetic providers expose exact RK4 stage states and failure
 * boundaries, including the historical K4 full-step regression oracle.
 */

#include <bbtc/bbtc.h>

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

typedef bbtc_status_e
(*rk4_float_signature_t)(
    const bbtc_eb_state_derivative_provider_float_t*,
    float,
    bbtc_eb_rigid_body_state_float_t*
);

typedef bbtc_status_e
(*rk4_double_signature_t)(
    const bbtc_eb_state_derivative_provider_double_t*,
    double,
    bbtc_eb_rigid_body_state_double_t*
);

typedef bbtc_status_e
(*rk4_long_double_signature_t)(
    const bbtc_eb_state_derivative_provider_long_double_t*,
    long double,
    bbtc_eb_rigid_body_state_long_double_t*
);

static const rk4_float_signature_t rk4_float_signature = bbtc_eb_rk4_step_float;
static const rk4_double_signature_t rk4_double_signature = bbtc_eb_rk4_step_double;
static const rk4_long_double_signature_t rk4_long_double_signature =
    bbtc_eb_rk4_step_long_double;

static bbtc_eb_rigid_body_state_double_t
valid_double_state(double time_s)
{
    return (bbtc_eb_rigid_body_state_double_t)
    {
        .orientation_world_from_body = {.w = 1.0},
        .position_m = {.x = 1.0, .y = 2.0, .z = 3.0},
        .velocity_m_per_s = {.x = 10.0, .y = 20.0, .z = 30.0},
        .angular_velocity_body_rad_per_s = {.x = 4.0, .y = 5.0, .z = 6.0},
        .time_s = time_s
    };
}

static int
state_bytes_equal(
    const bbtc_eb_rigid_body_state_double_t* state,
    const unsigned char* bytes
)
{
    return memcmp(state, bytes, sizeof(*state)) == 0;
}

typedef struct zero_context_t
{
    int calls;
    const bbtc_eb_rigid_body_state_double_t* caller_state;
    int all_pointers_local;
    int k1_exact;
}
zero_context_t;

static bbtc_status_e
zero_double_provider(
    const void* context,
    const bbtc_eb_rigid_body_state_double_t* state,
    bbtc_eb_rigid_body_state_derivative_double_t* out
)
{
    zero_context_t* c = (zero_context_t*)context;

    if (c != NULL)
    {
        ++c->calls;

        if (state == c->caller_state)
            c->all_pointers_local = 0;

        if (c->calls == 1)
        {
            c->k1_exact =
                state->orientation_world_from_body.x
                    == c->caller_state->orientation_world_from_body.x
                && state->orientation_world_from_body.y
                    == c->caller_state->orientation_world_from_body.y
                && state->orientation_world_from_body.z
                    == c->caller_state->orientation_world_from_body.z
                && state->orientation_world_from_body.w
                    == c->caller_state->orientation_world_from_body.w
                && state->position_m.x == c->caller_state->position_m.x
                && state->position_m.y == c->caller_state->position_m.y
                && state->position_m.z == c->caller_state->position_m.z
                && state->velocity_m_per_s.x
                    == c->caller_state->velocity_m_per_s.x
                && state->angular_velocity_body_rad_per_s.z
                    == c->caller_state->angular_velocity_body_rad_per_s.z
                && state->time_s == c->caller_state->time_s;
        }
    }

    (void)out;
    return BBTC_STATUS_SUCCESS;
}

static bbtc_status_e
zero_float_provider(
    const void* context,
    const bbtc_eb_rigid_body_state_float_t* state,
    bbtc_eb_rigid_body_state_derivative_float_t* out
)
{
    (void)context;
    (void)state;
    (void)out;
    return BBTC_STATUS_SUCCESS;
}

static bbtc_status_e
zero_long_double_provider(
    const void* context,
    const bbtc_eb_rigid_body_state_long_double_t* state,
    bbtc_eb_rigid_body_state_derivative_long_double_t* out
)
{
    (void)context;
    (void)state;
    (void)out;
    return BBTC_STATUS_SUCCESS;
}

static int
test_request_validation(void)
{
    bbtc_eb_rigid_body_state_double_t state = valid_double_state(0.0);
    zero_context_t context =
    {
        .calls = 0,
        .caller_state = &state,
        .all_pointers_local = 1,
        .k1_exact = 0
    };
    const bbtc_eb_state_derivative_provider_double_t provider =
    {
        .evaluate = zero_double_provider,
        .context = &context
    };
    const bbtc_eb_state_derivative_provider_double_t no_callback =
    {
        .evaluate = NULL,
        .context = NULL
    };

    if (rk4_double_signature(&provider, 1.0, NULL)
        != BBTC_STATUS_INVALID_ARGUMENT)
    {
        return EXIT_FAILURE;
    }

    if (rk4_double_signature(NULL, 1.0, &state)
        != BBTC_STATUS_INVALID_ARGUMENT)
    {
        return EXIT_FAILURE;
    }

    if (rk4_double_signature(&no_callback, 1.0, &state)
        != BBTC_STATUS_INVALID_ARGUMENT)
    {
        return EXIT_FAILURE;
    }

    state = valid_double_state(0.0);
    state.orientation_world_from_body = (bbtc_eb_quat_double_t){0};

    if (rk4_double_signature(&provider, NAN, &state)
            != BBTC_STATUS_NAN_INPUT
        || context.calls != 0)
    {
        return EXIT_FAILURE;
    }

    if (rk4_double_signature(&provider, INFINITY, &state)
            != BBTC_STATUS_NONFINITE_INPUT
        || context.calls != 0)
    {
        return EXIT_FAILURE;
    }

    state = valid_double_state(0.0);
    state.velocity_m_per_s.x = NAN;

    if (rk4_double_signature(&provider, INFINITY, &state)
            != BBTC_STATUS_NAN_INPUT
        || context.calls != 0)
    {
        return EXIT_FAILURE;
    }

    state = valid_double_state(0.0);
    state.position_m.y = INFINITY;

    if (rk4_double_signature(&provider, -1.0, &state)
            != BBTC_STATUS_NONFINITE_INPUT
        || context.calls != 0)
    {
        return EXIT_FAILURE;
    }

    state = valid_double_state(0.0);
    state.orientation_world_from_body = (bbtc_eb_quat_double_t){0};

    if (rk4_double_signature(&provider, 1.0, &state)
            != BBTC_STATUS_OUTSIDE_DOMAIN
        || context.calls != 0)
    {
        return EXIT_FAILURE;
    }

    state = valid_double_state(0.0);

    if (rk4_double_signature(&provider, 0.0, &state)
            != BBTC_STATUS_OUTSIDE_DOMAIN
        || rk4_double_signature(&provider, -0.0, &state)
            != BBTC_STATUS_OUTSIDE_DOMAIN
        || rk4_double_signature(&provider, -1.0, &state)
            != BBTC_STATUS_OUTSIDE_DOMAIN
        || context.calls != 0)
    {
        return EXIT_FAILURE;
    }

    if (rk4_double_signature(&provider, INFINITY, &state)
            != BBTC_STATUS_NONFINITE_INPUT
        || rk4_double_signature(&provider, -INFINITY, &state)
            != BBTC_STATUS_NONFINITE_INPUT
        || context.calls != 0)
    {
        return EXIT_FAILURE;
    }

    state = valid_double_state(DBL_MAX / 2.0);

    if (rk4_double_signature(&provider, DBL_MIN, &state)
            != BBTC_STATUS_NUMERICAL_FAILURE
        || context.calls != 0)
    {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

static int
test_request_failure_transaction(void)
{
    bbtc_eb_rigid_body_state_double_t state = valid_double_state(0.0);
    unsigned char before[sizeof(state)];

    zero_context_t context =
    {
        .calls = 0,
        .caller_state = &state,
        .all_pointers_local = 1,
        .k1_exact = 0
    };

    const bbtc_eb_state_derivative_provider_double_t provider =
    {
        .evaluate = zero_double_provider,
        .context = &context
    };

    /*
     * Caller-input failures occur before any provider evaluation and must
     * leave every byte of the caller-owned state untouched.
     */
    memcpy(before, &state, sizeof(state));

    if (rk4_double_signature(NULL, 1.0, &state)
            != BBTC_STATUS_INVALID_ARGUMENT
        || !state_bytes_equal(&state, before))
    {
        return EXIT_FAILURE;
    }

    state = valid_double_state(0.0);
    memcpy(before, &state, sizeof(state));

    if (rk4_double_signature(&provider, NAN, &state)
            != BBTC_STATUS_NAN_INPUT
        || context.calls != 0
        || !state_bytes_equal(&state, before))
    {
        return EXIT_FAILURE;
    }

    state = valid_double_state(0.0);
    state.orientation_world_from_body = (bbtc_eb_quat_double_t){0};
    memcpy(before, &state, sizeof(state));

    if (rk4_double_signature(&provider, 1.0, &state)
            != BBTC_STATUS_OUTSIDE_DOMAIN
        || context.calls != 0
        || !state_bytes_equal(&state, before))
    {
        return EXIT_FAILURE;
    }

    state = valid_double_state(DBL_MAX / 2.0);
    memcpy(before, &state, sizeof(state));

    if (rk4_double_signature(&provider, DBL_MIN, &state)
            != BBTC_STATUS_NUMERICAL_FAILURE
        || context.calls != 0
        || !state_bytes_equal(&state, before))
    {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}


static int
test_zero_derivative_and_transaction_snapshot(void)
{
    bbtc_eb_rigid_body_state_double_t state = valid_double_state(2.0);

    /*
     * Deliberately use a slightly non-unit but B2-valid initial quaternion.
     * K1 must see this exact value; generated stages/final state are projected.
     */
    state.orientation_world_from_body.w = 1.0000001;

    if (bbtc_eb_rigid_body_state_validate_double(&state)
        != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    const bbtc_eb_rigid_body_state_double_t initial = state;

    zero_context_t context =
    {
        .calls = 0,
        .caller_state = &state,
        .all_pointers_local = 1,
        .k1_exact = 0
    };
    const bbtc_eb_state_derivative_provider_double_t provider =
    {
        .evaluate = zero_double_provider,
        .context = &context
    };

    if (rk4_double_signature(&provider, 0.5, &state) != BBTC_STATUS_SUCCESS)
        return EXIT_FAILURE;

    if (context.calls != 4
        || !context.all_pointers_local
        || !context.k1_exact
        || state.time_s != 2.5
        || state.position_m.x != initial.position_m.x
        || state.position_m.y != initial.position_m.y
        || state.position_m.z != initial.position_m.z
        || state.velocity_m_per_s.x != initial.velocity_m_per_s.x
        || state.velocity_m_per_s.y != initial.velocity_m_per_s.y
        || state.velocity_m_per_s.z != initial.velocity_m_per_s.z
        || state.angular_velocity_body_rad_per_s.x
            != initial.angular_velocity_body_rad_per_s.x
        || fabs(state.orientation_world_from_body.w - 1.0) > 8.0 * DBL_EPSILON
        || bbtc_eb_rigid_body_state_validate_double(&state)
            != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

typedef struct exponential_context_t
{
    int calls;
    double x[4];
    double time[4];
    const bbtc_eb_rigid_body_state_double_t* caller_state;
    int all_pointers_local;
}
exponential_context_t;

static bbtc_status_e
exponential_provider(
    const void* context,
    const bbtc_eb_rigid_body_state_double_t* state,
    bbtc_eb_rigid_body_state_derivative_double_t* out
)
{
    exponential_context_t* c = (exponential_context_t*)context;

    if (c->calls < 4)
    {
        c->x[c->calls] = state->position_m.x;
        c->time[c->calls] = state->time_s;
    }

    if (state == c->caller_state)
        c->all_pointers_local = 0;

    ++c->calls;
    out->position_derivative_reference_m_per_s.x = state->position_m.x;
    return BBTC_STATUS_SUCCESS;
}

static int
test_classical_rk4_and_full_step_k4(void)
{
    bbtc_eb_rigid_body_state_double_t state = valid_double_state(0.0);
    state.position_m.x = 1.0;
    state.position_m.y = 0.0;
    state.position_m.z = 0.0;
    state.velocity_m_per_s = (bbtc_eb_vec3_double_t){0};
    state.angular_velocity_body_rad_per_s = (bbtc_eb_vec3_double_t){0};

    exponential_context_t context =
    {
        .calls = 0,
        .x = {0},
        .time = {0},
        .caller_state = &state,
        .all_pointers_local = 1
    };
    const bbtc_eb_state_derivative_provider_double_t provider =
    {
        .evaluate = exponential_provider,
        .context = &context
    };

    if (rk4_double_signature(&provider, 1.0, &state) != BBTC_STATUS_SUCCESS)
        return EXIT_FAILURE;

    const double expected = 65.0 / 24.0;
    const double tolerance = 16.0 * DBL_EPSILON * expected;

    return context.calls == 4
            && context.all_pointers_local
            && context.x[0] == 1.0
            && context.x[1] == 1.5
            && context.x[2] == 1.75
            && context.x[3] == 2.75
            && context.time[0] == 0.0
            && context.time[1] == 0.5
            && context.time[2] == 0.5
            && context.time[3] == 1.0
            && fabs(state.position_m.x - expected) <= tolerance
            && state.time_s == 1.0
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}

typedef struct projection_context_t
{
    int calls;
    int all_states_valid;
}
projection_context_t;

static bbtc_status_e
projection_provider(
    const void* context,
    const bbtc_eb_rigid_body_state_double_t* state,
    bbtc_eb_rigid_body_state_derivative_double_t* out
)
{
    projection_context_t* c = (projection_context_t*)context;
    ++c->calls;

    if (bbtc_eb_rigid_body_state_validate_double(state) != BBTC_STATUS_SUCCESS)
        c->all_states_valid = 0;

    out->orientation_world_from_body_derivative_per_s.x = 0.2;
    return BBTC_STATUS_SUCCESS;
}

static int
test_quaternion_projection(void)
{
    bbtc_eb_rigid_body_state_double_t state = valid_double_state(0.0);
    projection_context_t context =
    {
        .calls = 0,
        .all_states_valid = 1
    };
    const bbtc_eb_state_derivative_provider_double_t provider =
    {
        .evaluate = projection_provider,
        .context = &context
    };

    if (rk4_double_signature(&provider, 1.0, &state) != BBTC_STATUS_SUCCESS)
        return EXIT_FAILURE;

    const double q = state.orientation_world_from_body.x
                   * state.orientation_world_from_body.x
                   + state.orientation_world_from_body.y
                   * state.orientation_world_from_body.y
                   + state.orientation_world_from_body.z
                   * state.orientation_world_from_body.z
                   + state.orientation_world_from_body.w
                   * state.orientation_world_from_body.w;

    return context.calls == 4
            && context.all_states_valid
            && fabs(q - 1.0) <= 8.0 * DBL_EPSILON
            && bbtc_eb_rigid_body_state_validate_double(&state)
                == BBTC_STATUS_SUCCESS
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}

typedef enum generated_failure_mode_e
{
    GENERATED_ZERO_QUATERNION = 0,
    GENERATED_NONFINITE_QUATERNION,
    GENERATED_NONFINITE_TRANSLATION,
    GENERATED_NONFINITE_ANGULAR
}
generated_failure_mode_e;

typedef struct generated_failure_context_t
{
    int calls;
    generated_failure_mode_e mode;
}
generated_failure_context_t;

static bbtc_status_e
generated_failure_provider(
    const void* context,
    const bbtc_eb_rigid_body_state_double_t* state,
    bbtc_eb_rigid_body_state_derivative_double_t* out
)
{
    generated_failure_context_t* c = (generated_failure_context_t*)context;
    ++c->calls;
    (void)state;

    if (c->mode == GENERATED_ZERO_QUATERNION)
    {
        out->orientation_world_from_body_derivative_per_s.w = -2.0;
    }
    else if (c->mode == GENERATED_NONFINITE_QUATERNION)
    {
        out->orientation_world_from_body_derivative_per_s.x = DBL_MAX;
    }
    else if (c->mode == GENERATED_NONFINITE_TRANSLATION)
    {
        out->position_derivative_reference_m_per_s.x = DBL_MAX;
    }
    else
    {
        out->angular_velocity_derivative_body_rad_per_s2.x = DBL_MAX;
    }

    return BBTC_STATUS_SUCCESS;
}

static int
test_generated_state_numerical_failures(void)
{
    bbtc_eb_rigid_body_state_double_t state = valid_double_state(0.0);
    unsigned char before[sizeof(state)];
    memcpy(before, &state, sizeof(state));

    generated_failure_context_t context =
    {
        .calls = 0,
        .mode = GENERATED_ZERO_QUATERNION
    };
    bbtc_eb_state_derivative_provider_double_t provider =
    {
        .evaluate = generated_failure_provider,
        .context = &context
    };

    if (rk4_double_signature(&provider, 1.0, &state)
            != BBTC_STATUS_NUMERICAL_FAILURE
        || context.calls != 1
        || !state_bytes_equal(&state, before))
    {
        return EXIT_FAILURE;
    }

    state = valid_double_state(0.0);
    memcpy(before, &state, sizeof(state));
    context.calls = 0;
    context.mode = GENERATED_NONFINITE_QUATERNION;

    if (rk4_double_signature(&provider, 4.0, &state)
            != BBTC_STATUS_NUMERICAL_FAILURE
        || context.calls != 1
        || !state_bytes_equal(&state, before))
    {
        return EXIT_FAILURE;
    }

    state = valid_double_state(0.0);
    memcpy(before, &state, sizeof(state));
    context.calls = 0;
    context.mode = GENERATED_NONFINITE_TRANSLATION;

    if (rk4_double_signature(&provider, 4.0, &state)
            != BBTC_STATUS_NUMERICAL_FAILURE
        || context.calls != 1
        || !state_bytes_equal(&state, before))
    {
        return EXIT_FAILURE;
    }

    state = valid_double_state(0.0);
    memcpy(before, &state, sizeof(state));
    context.calls = 0;
    context.mode = GENERATED_NONFINITE_ANGULAR;

    if (rk4_double_signature(&provider, 4.0, &state)
            != BBTC_STATUS_NUMERICAL_FAILURE
        || context.calls != 1
        || !state_bytes_equal(&state, before))
    {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

typedef struct provider_failure_context_t
{
    int calls;
    int fail_on;
}
provider_failure_context_t;

static bbtc_status_e
provider_failure_callback(
    const void* context,
    const bbtc_eb_rigid_body_state_double_t* state,
    bbtc_eb_rigid_body_state_derivative_double_t* out
)
{
    provider_failure_context_t* c = (provider_failure_context_t*)context;
    ++c->calls;
    (void)state;
    (void)out;

    return c->calls == c->fail_on
        ? BBTC_STATUS_UNSUPPORTED_MODEL_OR_OPTION
        : BBTC_STATUS_SUCCESS;
}

static int
test_provider_failure_propagation_and_transaction(void)
{
    for (int fail_on = 1; fail_on <= 4; ++fail_on)
    {
        bbtc_eb_rigid_body_state_double_t state = valid_double_state(0.0);
        unsigned char before[sizeof(state)];
        memcpy(before, &state, sizeof(state));

        provider_failure_context_t context =
        {
            .calls = 0,
            .fail_on = fail_on
        };
        const bbtc_eb_state_derivative_provider_double_t provider =
        {
            .evaluate = provider_failure_callback,
            .context = &context
        };

        if (rk4_double_signature(&provider, 1.0, &state)
                != BBTC_STATUS_UNSUPPORTED_MODEL_OR_OPTION
            || context.calls != fail_on
            || !state_bytes_equal(&state, before))
        {
            return EXIT_FAILURE;
        }
    }

    return EXIT_SUCCESS;
}

static int
test_other_precision_families(void)
{
    const bbtc_eb_state_derivative_provider_float_t float_provider =
    {
        .evaluate = zero_float_provider,
        .context = NULL
    };
    const bbtc_eb_state_derivative_provider_long_double_t long_double_provider =
    {
        .evaluate = zero_long_double_provider,
        .context = NULL
    };

    bbtc_eb_rigid_body_state_float_t float_state =
    {
        .orientation_world_from_body = {.w = 1.0f},
        .time_s = 1.0f
    };
    bbtc_eb_rigid_body_state_long_double_t long_double_state =
    {
        .orientation_world_from_body = {.w = 1.0L},
        .time_s = 2.0L
    };

    return rk4_float_signature(&float_provider, 0.25f, &float_state)
            == BBTC_STATUS_SUCCESS
            && float_state.time_s == 1.25f
            && rk4_long_double_signature(
                   &long_double_provider,
                   0.5L,
                   &long_double_state
               ) == BBTC_STATUS_SUCCESS
            && long_double_state.time_s == 2.5L
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}

int
main(void)
{
    if (test_request_validation() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_request_failure_transaction() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_zero_derivative_and_transaction_snapshot() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_classical_rk4_and_full_step_k4() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_quaternion_projection() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_generated_state_numerical_failures() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_provider_failure_propagation_and_transaction() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    return test_other_precision_families();
}
