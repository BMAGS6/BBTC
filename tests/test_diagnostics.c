#include <bbtc/bbtc.h>

#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define ARRAY_COUNT(array) (sizeof(array) / sizeof((array)[0]))
#define IS_SINGLE_BIT(value) \
    ( \
        ((uint64_t)(value) != UINT64_C(0)) \
        && ( \
            ((uint64_t)(value) & ((uint64_t)(value) - UINT64_C(1))) \
            == UINT64_C(0) \
        ) \
    )

typedef struct diagnostic_string_case
{
    uint64_t value;
    const char* expected;
} diagnostic_string_case_t;

static_assert(sizeof(bbtc_ib_termination_e) == sizeof(uint32_t));
static_assert(BBTC_IB_TERMINATION_NOT_RUN == 0);
static_assert(BBTC_IB_TERMINATION_MUZZLE_EXIT == 1);
static_assert(BBTC_IB_TERMINATION_NO_IGNITION == 2);
static_assert(BBTC_IB_TERMINATION_PROJECTILE_NOT_STARTED == 3);
static_assert(
    BBTC_IB_TERMINATION_PROJECTILE_STOPPED_BEFORE_MUZZLE_EXIT == 4
);
static_assert(BBTC_IB_TERMINATION_TIME_GUARD_REACHED == 5);
static_assert(BBTC_IB_TERMINATION_PRESSURE_GUARD_REACHED == 6);
static_assert(BBTC_IB_TERMINATION_STEP_GUARD_REACHED == 7);
static_assert(BBTC_IB_TERMINATION_NUMERICAL_FAILURE == 8);

static_assert(sizeof(bbtc_warning_flags_t) == sizeof(uint64_t));
static_assert(sizeof(bbtc_warning_flag_e) == sizeof(uint64_t));
static_assert(BBTC_WARNING_NONE == UINT64_C(0));
static_assert(BBTC_WARNING_HISTORY_TRUNCATED == (UINT64_C(1) << 0));
static_assert(
    BBTC_WARNING_ENERGY_RESIDUAL_EXCEEDED == (UINT64_C(1) << 1)
);
static_assert(
    BBTC_WARNING_INCOMPLETE_BURN_AT_MUZZLE_EXIT == (UINT64_C(1) << 2)
);
static_assert(
    BBTC_WARNING_FALLBACK_APPROXIMATION_USED == (UINT64_C(1) << 3)
);
static_assert(
    BBTC_WARNING_REDUCED_EVENT_LOCATION_ACCURACY == (UINT64_C(1) << 4)
);
static_assert(BBTC_WARNING_DATA_EXTRAPOLATED == (UINT64_C(1) << 5));

static_assert(IS_SINGLE_BIT(BBTC_WARNING_HISTORY_TRUNCATED));
static_assert(IS_SINGLE_BIT(BBTC_WARNING_ENERGY_RESIDUAL_EXCEEDED));
static_assert(IS_SINGLE_BIT(BBTC_WARNING_INCOMPLETE_BURN_AT_MUZZLE_EXIT));
static_assert(IS_SINGLE_BIT(BBTC_WARNING_FALLBACK_APPROXIMATION_USED));
static_assert(IS_SINGLE_BIT(BBTC_WARNING_REDUCED_EVENT_LOCATION_ACCURACY));
static_assert(IS_SINGLE_BIT(BBTC_WARNING_DATA_EXTRAPOLATED));

static_assert(sizeof(bbtc_applicability_flags_t) == sizeof(uint64_t));
static_assert(sizeof(bbtc_applicability_flag_e) == sizeof(uint64_t));
static_assert(BBTC_APPLICABILITY_NONE_REPORTED == UINT64_C(0));
static_assert(
    BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN == (UINT64_C(1) << 0)
);
static_assert(
    BBTC_APPLICABILITY_MODEL_COMBINATION_UNVALIDATED
    == (UINT64_C(1) << 1)
);
static_assert(
    BBTC_APPLICABILITY_ASSUMPTIONS_MATERIALLY_STRESSED
    == (UINT64_C(1) << 2)
);
static_assert(
    BBTC_APPLICABILITY_DATA_PROVENANCE_UNKNOWN == (UINT64_C(1) << 3)
);
static_assert(
    BBTC_APPLICABILITY_REQUESTED_EFFECT_APPROXIMATED
    == (UINT64_C(1) << 4)
);

static_assert(
    IS_SINGLE_BIT(BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN)
);
static_assert(
    IS_SINGLE_BIT(BBTC_APPLICABILITY_MODEL_COMBINATION_UNVALIDATED)
);
static_assert(
    IS_SINGLE_BIT(BBTC_APPLICABILITY_ASSUMPTIONS_MATERIALLY_STRESSED)
);
static_assert(
    IS_SINGLE_BIT(BBTC_APPLICABILITY_DATA_PROVENANCE_UNKNOWN)
);
static_assert(
    IS_SINGLE_BIT(BBTC_APPLICABILITY_REQUESTED_EFFECT_APPROXIMATED)
);

static_assert(
    _Generic(
        bbtc_ib_termination_string(BBTC_IB_TERMINATION_NOT_RUN),
        const char*: 1,
        default: 0
    )
);

static_assert(
    _Generic(
        bbtc_warning_flag_string(BBTC_WARNING_NONE),
        const char*: 1,
        default: 0
    )
);

static_assert(
    _Generic(
        bbtc_applicability_flag_string(
            BBTC_APPLICABILITY_NONE_REPORTED
        ),
        const char*: 1,
        default: 0
    )
);

static int
check_string(
    const char* const label,
    const uint64_t value,
    const char* const actual,
    const char* const expected
)
{
    if (actual == NULL)
    {
        fprintf(
            stderr,
            "%s(%" PRIu64 ") returned null\n",
            label,
            value
        );
        return EXIT_FAILURE;
    }

    if (strcmp(actual, expected) != 0)
    {
        fprintf(
            stderr,
            "%s(%" PRIu64 ") returned \"%s\"; expected \"%s\"\n",
            label,
            value,
            actual,
            expected
        );
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

static int
check_termination_strings(void)
{
    static const diagnostic_string_case_t cases[] =
    {
        {BBTC_IB_TERMINATION_NOT_RUN, "simulation not run"},
        {
            BBTC_IB_TERMINATION_MUZZLE_EXIT,
            "projectile reached muzzle exit"
        },
        {BBTC_IB_TERMINATION_NO_IGNITION, "ignition did not occur"},
        {
            BBTC_IB_TERMINATION_PROJECTILE_NOT_STARTED,
            "projectile did not begin moving"
        },
        {
            BBTC_IB_TERMINATION_PROJECTILE_STOPPED_BEFORE_MUZZLE_EXIT,
            "projectile stopped before muzzle exit"
        },
        {
            BBTC_IB_TERMINATION_TIME_GUARD_REACHED,
            "caller time guard reached"
        },
        {
            BBTC_IB_TERMINATION_PRESSURE_GUARD_REACHED,
            "caller pressure guard reached"
        },
        {
            BBTC_IB_TERMINATION_STEP_GUARD_REACHED,
            "caller step guard reached"
        },
        {
            BBTC_IB_TERMINATION_NUMERICAL_FAILURE,
            "numerical failure"
        }
    };

    for (size_t i = 0; i < ARRAY_COUNT(cases); ++i)
    {
        const bbtc_ib_termination_e termination =
            (bbtc_ib_termination_e)cases[i].value;

        if (
            check_string(
                "bbtc_ib_termination_string",
                cases[i].value,
                bbtc_ib_termination_string(termination),
                cases[i].expected
            ) != EXIT_SUCCESS
        )
        {
            return EXIT_FAILURE;
        }
    }

    if (
        check_string(
            "bbtc_ib_termination_string",
            UINT64_C(9),
            bbtc_ib_termination_string((bbtc_ib_termination_e)UINT32_C(9)),
            "unknown BBTC internal-ballistics termination"
        ) != EXIT_SUCCESS
    )
    {
        return EXIT_FAILURE;
    }

    return check_string(
        "bbtc_ib_termination_string",
        UINT32_MAX,
        bbtc_ib_termination_string((bbtc_ib_termination_e)UINT32_MAX),
        "unknown BBTC internal-ballistics termination"
    );
}

static int
check_warning_strings(void)
{
    static const diagnostic_string_case_t cases[] =
    {
        {BBTC_WARNING_NONE, "no warning reported"},
        {
            BBTC_WARNING_HISTORY_TRUNCATED,
            "requested history was truncated"
        },
        {
            BBTC_WARNING_ENERGY_RESIDUAL_EXCEEDED,
            "energy-accounting residual tolerance exceeded"
        },
        {
            BBTC_WARNING_INCOMPLETE_BURN_AT_MUZZLE_EXIT,
            "propellant burn incomplete at muzzle exit"
        },
        {
            BBTC_WARNING_FALLBACK_APPROXIMATION_USED,
            "fallback approximation used"
        },
        {
            BBTC_WARNING_REDUCED_EVENT_LOCATION_ACCURACY,
            "event located with reduced accuracy"
        },
        {
            BBTC_WARNING_DATA_EXTRAPOLATED,
            "data record extrapolated"
        }
    };

    for (size_t i = 0; i < ARRAY_COUNT(cases); ++i)
    {
        const bbtc_warning_flag_e flag =
            (bbtc_warning_flag_e)cases[i].value;

        if (
            check_string(
                "bbtc_warning_flag_string",
                cases[i].value,
                bbtc_warning_flag_string(flag),
                cases[i].expected
            ) != EXIT_SUCCESS
        )
        {
            return EXIT_FAILURE;
        }
    }

    if (
        check_string(
            "bbtc_warning_flag_string",
            UINT64_C(3),
            bbtc_warning_flag_string((bbtc_warning_flag_e)UINT64_C(3)),
            "unknown or combined BBTC warning flag"
        ) != EXIT_SUCCESS
    )
    {
        return EXIT_FAILURE;
    }

    return check_string(
        "bbtc_warning_flag_string",
        UINT64_C(1) << 63,
        bbtc_warning_flag_string(
            (bbtc_warning_flag_e)(UINT64_C(1) << 63)
        ),
        "unknown or combined BBTC warning flag"
    );
}

static int
check_applicability_strings(void)
{
    static const diagnostic_string_case_t cases[] =
    {
        {
            BBTC_APPLICABILITY_NONE_REPORTED,
            "no applicability limitation reported"
        },
        {
            BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN,
            "outside documented calibration domain"
        },
        {
            BBTC_APPLICABILITY_MODEL_COMBINATION_UNVALIDATED,
            "model combination lacks experimental validation"
        },
        {
            BBTC_APPLICABILITY_ASSUMPTIONS_MATERIALLY_STRESSED,
            "model assumptions materially stressed"
        },
        {
            BBTC_APPLICABILITY_DATA_PROVENANCE_UNKNOWN,
            "data provenance unknown"
        },
        {
            BBTC_APPLICABILITY_REQUESTED_EFFECT_APPROXIMATED,
            "requested physical effect approximated"
        }
    };

    for (size_t i = 0; i < ARRAY_COUNT(cases); ++i)
    {
        const bbtc_applicability_flag_e flag =
            (bbtc_applicability_flag_e)cases[i].value;

        if (
            check_string(
                "bbtc_applicability_flag_string",
                cases[i].value,
                bbtc_applicability_flag_string(flag),
                cases[i].expected
            ) != EXIT_SUCCESS
        )
        {
            return EXIT_FAILURE;
        }
    }

    if (
        check_string(
            "bbtc_applicability_flag_string",
            UINT64_C(3),
            bbtc_applicability_flag_string(
                (bbtc_applicability_flag_e)UINT64_C(3)
            ),
            "unknown or combined BBTC applicability flag"
        ) != EXIT_SUCCESS
    )
    {
        return EXIT_FAILURE;
    }

    return check_string(
        "bbtc_applicability_flag_string",
        UINT64_C(1) << 63,
        bbtc_applicability_flag_string(
            (bbtc_applicability_flag_e)(UINT64_C(1) << 63)
        ),
        "unknown or combined BBTC applicability flag"
    );
}

int
main(void)
{
    if (check_termination_strings() != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (check_warning_strings() != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    return check_applicability_strings();
}
