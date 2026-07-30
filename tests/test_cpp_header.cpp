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

int
main()
{
    return std::strcmp(
        bbtc_status_string(BBTC_STATUS_SUCCESS),
        "success"
    );
}
