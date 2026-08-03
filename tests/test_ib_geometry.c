#include <bbtc/bbtc.h>

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <float.h>
#include <math.h>

#if __STDC_VERSION__ < 202311L
#   include <stdbool.h>
#endif


#define REQUIRE_STATUS(label, expression, expected)       \
    do                                                    \
    {                                                     \
        const bbtc_status_e actual_status = (expression); \
        if (actual_status != (expected))                  \
        {                                                 \
            fprintf(stderr,                               \
                "%s returned %u; expected %u\n",          \
                (label),                                  \
                (unsigned int)actual_status,              \
                (unsigned int)(expected)                  \
            );                                            \
            return EXIT_FAILURE;                          \
        }                                                 \
    } while (false)


#define REQUIRE_MUTATION_STATUS(type, validator, original, field, value, expected) \
    do                                                                             \
    {                                                                              \
        type mutated_geometry  = (original);                                       \
        mutated_geometry.field = (value);                                          \
        REQUIRE_STATUS(#validator ": " #field,                                     \
            validator(&mutated_geometry),                                          \
            (expected)                                                             \
        );                                                                         \
    } while (false)


static_assert(
    _Generic(
        ((bbtc_ib_geometry_float_t*)0)->initial_behind_projectile_volume_m3,
        float:   1,
        default: 0
    )
);
static_assert(
    _Generic(
        ((bbtc_ib_geometry_double_t*)0)->initial_behind_projectile_volume_m3,
        double:  1,
        default: 0
    )
);
static_assert(
    _Generic(
        ((bbtc_ib_geometry_long_double_t*)0)->initial_behind_projectile_volume_m3,
        long double: 1,
        default:     0
    )
);

#define RUN_FIELD_MUTATIONS(                                        \
    type, validator, original, field, zero, negative, nan_value,    \
    positive_infinity, negative_infinity)                           \
    do                                                              \
    {                                                               \
        REQUIRE_MUTATION_STATUS(                                    \
            type, validator, original, field, zero,                 \
            BBTC_STATUS_OUTSIDE_DOMAIN                              \
        );                                                          \
        REQUIRE_MUTATION_STATUS(                                    \
            type, validator, original, field, negative,             \
            BBTC_STATUS_OUTSIDE_DOMAIN                              \
        );                                                          \
        REQUIRE_MUTATION_STATUS(                                    \
            type, validator, original, field, nan_value,            \
            BBTC_STATUS_NONFINITE_INPUT                             \
        );                                                          \
        REQUIRE_MUTATION_STATUS(                                    \
            type, validator, original, field, positive_infinity,    \
            BBTC_STATUS_NONFINITE_INPUT                             \
        );                                                          \
        REQUIRE_MUTATION_STATUS(                                    \
            type, validator, original, field, negative_infinity,    \
            BBTC_STATUS_NONFINITE_INPUT                             \
        );                                                          \
    } while (false)

static int
test_float_geometry(void)
{
    const bbtc_ib_geometry_float_t valid =
    {
        .initial_behind_projectile_volume_m3 = 1.0f,
        .bore_cross_sectional_area_m2        = 2.0f,
        .projectile_effective_base_area_m2   = 3.0f,
        .projectile_travel_to_muzzle_m       = 4.0f
    };

    bbtc_ib_geometry_float_t candidate = valid;

    REQUIRE_STATUS("float NULL",
                   bbtc_ib_geometry_validate_float(NULL),
                   BBTC_STATUS_INVALID_ARGUMENT
    );

    REQUIRE_STATUS("float valid",
                   bbtc_ib_geometry_validate_float(&candidate),
                   BBTC_STATUS_SUCCESS
    );

    if (candidate.initial_behind_projectile_volume_m3
            != valid.initial_behind_projectile_volume_m3
        ||
        candidate.bore_cross_sectional_area_m2
            != valid.bore_cross_sectional_area_m2
        ||
        candidate.projectile_effective_base_area_m2
            != valid.projectile_effective_base_area_m2
        ||
        candidate.projectile_travel_to_muzzle_m
            != valid.projectile_travel_to_muzzle_m
    )
    {
        fprintf(stderr, "float validation modified its input record\n");

        return EXIT_FAILURE;
    }

    RUN_FIELD_MUTATIONS(bbtc_ib_geometry_float_t,
                        bbtc_ib_geometry_validate_float,
                        valid,
                        initial_behind_projectile_volume_m3,
                        0.0f, -1.0f, NAN, INFINITY, -INFINITY);

    RUN_FIELD_MUTATIONS(bbtc_ib_geometry_float_t,
                        bbtc_ib_geometry_validate_float,
                        valid,
                        bore_cross_sectional_area_m2,
                        0.0f, -1.0f, NAN, INFINITY, -INFINITY);

    RUN_FIELD_MUTATIONS(bbtc_ib_geometry_float_t,
                        bbtc_ib_geometry_validate_float,
                        valid,
                        projectile_effective_base_area_m2,
                        0.0f, -1.0f, NAN, INFINITY, -INFINITY);

    RUN_FIELD_MUTATIONS(bbtc_ib_geometry_float_t,
                        bbtc_ib_geometry_validate_float,
                        valid,
                        projectile_travel_to_muzzle_m,
                        0.0f, -1.0f, NAN, INFINITY, -INFINITY);


    candidate = (bbtc_ib_geometry_float_t)
    {
        FLT_TRUE_MIN,
        FLT_TRUE_MIN,
        FLT_TRUE_MIN,
        FLT_TRUE_MIN
    };

    REQUIRE_STATUS("float positive subnormal",
                   bbtc_ib_geometry_validate_float(&candidate),
                   BBTC_STATUS_SUCCESS);

    candidate = (bbtc_ib_geometry_float_t)
    {
        FLT_MAX,
        FLT_MAX,
        FLT_MAX,
        FLT_MAX
    };

    REQUIRE_STATUS("float finite maximum",
                   bbtc_ib_geometry_validate_float(&candidate),
                   BBTC_STATUS_SUCCESS);

    return EXIT_SUCCESS;
}

static int
test_double_geometry(void)
{
    const bbtc_ib_geometry_double_t valid =
    {
        .initial_behind_projectile_volume_m3 = 1.0,
        .bore_cross_sectional_area_m2 = 2.0,
        .projectile_effective_base_area_m2 = 3.0,
        .projectile_travel_to_muzzle_m = 4.0
    };

    bbtc_ib_geometry_double_t candidate = valid;

    REQUIRE_STATUS("double NULL",
                   bbtc_ib_geometry_validate_double(NULL),
                   BBTC_STATUS_INVALID_ARGUMENT);

    REQUIRE_STATUS("double valid unequal areas",
                   bbtc_ib_geometry_validate_double(&candidate),
                   BBTC_STATUS_SUCCESS);


    RUN_FIELD_MUTATIONS(bbtc_ib_geometry_double_t,
                        bbtc_ib_geometry_validate_double,
                        valid,
                        initial_behind_projectile_volume_m3,
                        0.0, -1.0, NAN, INFINITY, -INFINITY);

    RUN_FIELD_MUTATIONS(bbtc_ib_geometry_double_t,
                        bbtc_ib_geometry_validate_double,
                        valid,
                        bore_cross_sectional_area_m2,
                        0.0, -1.0, NAN, INFINITY, -INFINITY);

    RUN_FIELD_MUTATIONS(bbtc_ib_geometry_double_t,
                        bbtc_ib_geometry_validate_double,
                        valid,
                        projectile_effective_base_area_m2,
                        0.0, -1.0, NAN, INFINITY, -INFINITY);

    RUN_FIELD_MUTATIONS(bbtc_ib_geometry_double_t,
                        bbtc_ib_geometry_validate_double,
                        valid,
                        projectile_travel_to_muzzle_m,
                        0.0, -1.0, NAN, INFINITY, -INFINITY);


    candidate = (bbtc_ib_geometry_double_t)
    {
        DBL_TRUE_MIN,
        DBL_TRUE_MIN,
        DBL_TRUE_MIN,
        DBL_TRUE_MIN
    };

    REQUIRE_STATUS("double positive subnormal",
                   bbtc_ib_geometry_validate_double(&candidate),
                   BBTC_STATUS_SUCCESS);

    candidate = (bbtc_ib_geometry_double_t)
    {
        DBL_MAX,
        DBL_MAX,
        DBL_MAX,
        DBL_MAX
    };

    REQUIRE_STATUS("double finite maximum",
                   bbtc_ib_geometry_validate_double(&candidate),
                   BBTC_STATUS_SUCCESS);

    return EXIT_SUCCESS;
}

static int
test_long_double_geometry(void)
{
    const bbtc_ib_geometry_long_double_t valid =
    {
        .initial_behind_projectile_volume_m3 = 1.0L,
        .bore_cross_sectional_area_m2 = 2.0L,
        .projectile_effective_base_area_m2 = 3.0L,
        .projectile_travel_to_muzzle_m = 4.0L
    };

    bbtc_ib_geometry_long_double_t candidate = valid;

    REQUIRE_STATUS("long double NULL",
                   bbtc_ib_geometry_validate_long_double(NULL),
                   BBTC_STATUS_INVALID_ARGUMENT);

    REQUIRE_STATUS("long double valid unequal areas",
                   bbtc_ib_geometry_validate_long_double(&candidate),
                   BBTC_STATUS_SUCCESS);

    RUN_FIELD_MUTATIONS(bbtc_ib_geometry_long_double_t,
                        bbtc_ib_geometry_validate_long_double,
                        valid,
                        initial_behind_projectile_volume_m3,
                        0.0L, -1.0L, NAN, INFINITY, -INFINITY);

    RUN_FIELD_MUTATIONS(bbtc_ib_geometry_long_double_t,
                        bbtc_ib_geometry_validate_long_double,
                        valid,
                        bore_cross_sectional_area_m2,
                        0.0L, -1.0L, NAN, INFINITY, -INFINITY);

    RUN_FIELD_MUTATIONS(bbtc_ib_geometry_long_double_t,
                        bbtc_ib_geometry_validate_long_double,
                        valid,
                        projectile_effective_base_area_m2,
                        0.0L, -1.0L, NAN, INFINITY, -INFINITY);

    RUN_FIELD_MUTATIONS(bbtc_ib_geometry_long_double_t,
                        bbtc_ib_geometry_validate_long_double,
                        valid,
                        projectile_travel_to_muzzle_m,
                        0.0L, -1.0L, NAN, INFINITY, -INFINITY);


    candidate = (bbtc_ib_geometry_long_double_t)
    {
        LDBL_TRUE_MIN,
        LDBL_TRUE_MIN,
        LDBL_TRUE_MIN,
        LDBL_TRUE_MIN
    };

    REQUIRE_STATUS("long double positive subnormal",
                   bbtc_ib_geometry_validate_long_double(&candidate),
                   BBTC_STATUS_SUCCESS
    );

    candidate = (bbtc_ib_geometry_long_double_t)
    {
        LDBL_MAX,
        LDBL_MAX,
        LDBL_MAX,
        LDBL_MAX
    };

    REQUIRE_STATUS("long double finite maximum",
                   bbtc_ib_geometry_validate_long_double(&candidate),
                   BBTC_STATUS_SUCCESS);

    return EXIT_SUCCESS;
}

int
main(void)
{
    if (test_float_geometry() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_double_geometry() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    return test_long_double_geometry();
}
