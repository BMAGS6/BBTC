#include <bbtc/bbtc.h>

#include <cstdint>
#include <cstring>
#include <type_traits>

static_assert(
    std::is_same<
        std::underlying_type<bbtc_status_e>::type,
        std::uint32_t
    >::value,
    "bbtc_status_e must have uint32_t representation"
);

static_assert(
    BBTC_STATUS_SUCCESS == 0,
    "BBTC status value zero must mean success"
);

static_assert(
    std::is_same<
        std::underlying_type<bbtc_ib_termination_e>::type,
        std::uint32_t
    >::value,
    "bbtc_ib_termination_e must have uint32_t representation"
);

static_assert(
    std::is_same<
        std::underlying_type<bbtc_warning_flag_e>::type,
        std::uint64_t
    >::value,
    "bbtc_warning_flag_e must have uint64_t representation"
);

static_assert(
    std::is_same<
        std::underlying_type<bbtc_applicability_flag_e>::type,
        std::uint64_t
    >::value,
    "bbtc_applicability_flag_e must have uint64_t representation"
);

static_assert(
    std::is_same<bbtc_warning_flags_t, std::uint64_t>::value,
    "bbtc_warning_flags_t must be uint64_t"
);

static_assert(
    std::is_same<bbtc_applicability_flags_t, std::uint64_t>::value,
    "bbtc_applicability_flags_t must be uint64_t"
);

static_assert(
    (
        BBTC_WARNING_HISTORY_TRUNCATED
        | BBTC_WARNING_DATA_EXTRAPOLATED
    ) == UINT64_C(0x21),
    "warning bits must combine into a uint64_t mask"
);

static_assert(
    (
        BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN
        | BBTC_APPLICABILITY_DATA_PROVENANCE_UNKNOWN
    ) == UINT64_C(0x09),
    "applicability bits must combine into a uint64_t mask"
);

int
main()
{
    if (
        std::strcmp(
            bbtc_status_string(BBTC_STATUS_SUCCESS),
            "success"
        ) != 0
    )
    {
        return 1;
    }

    if (
        std::strcmp(
            bbtc_ib_termination_string(BBTC_IB_TERMINATION_NOT_RUN),
            "simulation not run"
        ) != 0
    )
    {
        return 1;
    }

    if (
        std::strcmp(
            bbtc_warning_flag_string(BBTC_WARNING_NONE),
            "no warning reported"
        ) != 0
    )
    {
        return 1;
    }

    return std::strcmp(
        bbtc_applicability_flag_string(
            BBTC_APPLICABILITY_NONE_REPORTED
        ),
        "no applicability limitation reported"
    );
}
