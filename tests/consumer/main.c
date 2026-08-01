/**
 * @file
 * @brief Verifies BBTC use from an independent CMake consumer project.
 */

#include <bbtc/bbtc.h>

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/**
 * @brief Compares one BBTC diagnostic string against its stable text.
 *
 * @param actual String returned by the BBTC public API.
 * @param expected Stable text required by the public contract.
 *
 * @return `EXIT_SUCCESS` when the strings match; otherwise `EXIT_FAILURE`.
 */
static int
check_string(
    const char* const actual,
    const char* const expected
)
{
    if (actual == NULL)
    {
        return EXIT_FAILURE;
    }

    if (strcmp(actual, expected) != 0)
    {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

/**
 * @brief Verifies public precision metadata through the consumer target.
 *
 * @return `EXIT_SUCCESS` when precision metadata is coherent; otherwise
 *         `EXIT_FAILURE`.
 */
static int
check_precision_contract(void)
{
    bbtc_precision_info_t info = {0};

    if (
        bbtc_precision_info(BBTC_PRECISION_DOUBLE, &info)
        != BBTC_STATUS_SUCCESS
    )
    {
        return EXIT_FAILURE;
    }

    if (
        info.precision != BBTC_PRECISION_DOUBLE
        || info.radix < UINT32_C(2)
        || info.storage_bytes != (uint32_t)sizeof(double)
    )
    {
        return EXIT_FAILURE;
    }

    return check_string(
        bbtc_precision_string(BBTC_PRECISION_DOUBLE),
        "double"
    );
}

/**
 * @brief Exercises public BBTC headers and linked diagnostic symbols.
 *
 * @return `EXIT_SUCCESS` when the external consumer contract works; otherwise
 *         `EXIT_FAILURE`.
 */
int
main(void)
{
    if (
        check_string(
            bbtc_status_string(BBTC_STATUS_SUCCESS),
            "success"
        ) != EXIT_SUCCESS
    )
    {
        return EXIT_FAILURE;
    }

    if (check_precision_contract() != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (
        check_string(
            bbtc_ib_termination_string(
                BBTC_IB_TERMINATION_MUZZLE_EXIT
            ),
            "projectile reached muzzle exit"
        ) != EXIT_SUCCESS
    )
    {
        return EXIT_FAILURE;
    }

    if (
        check_string(
            bbtc_warning_flag_string(
                BBTC_WARNING_DATA_EXTRAPOLATED
            ),
            "data record extrapolated"
        ) != EXIT_SUCCESS
    )
    {
        return EXIT_FAILURE;
    }

    return check_string(
        bbtc_applicability_flag_string(
            BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN
        ),
        "outside documented calibration domain"
    );
}
