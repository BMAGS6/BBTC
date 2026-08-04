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
check_string(const char* const actual,
             const char* const expected)
{
    if (actual == NULL)
        return EXIT_FAILURE;

    return strcmp(actual, expected);
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

    if (bbtc_precision_info(BBTC_PRECISION_DOUBLE, &info) != BBTC_STATUS_SUCCESS)
        return EXIT_FAILURE;

    if (info.precision     != BBTC_PRECISION_DOUBLE     ||
        info.radix         < UINT32_C(2)                ||
        info.storage_bytes != (uint32_t)sizeof(double))
    {
        return EXIT_FAILURE;
    }

    return check_string(bbtc_precision_string(BBTC_PRECISION_DOUBLE),
                        "double");
}

/**
 * @brief Verifies precision-qualified geometry through the consumer target.
 *
 * @return `EXIT_SUCCESS` when the geometry contract works; otherwise
 *         `EXIT_FAILURE`.
 */
static uint8_t
check_geometry_contract(void)
{
    const bbtc_ib_geometry_double_t geometry =
    {
        .initial_behind_projectile_volume_m3 = 4.0e-6,
        .bore_cross_sectional_area_m2        = 5.0e-5,
        .projectile_effective_base_area_m2   = 4.8e-5,
        .projectile_travel_to_muzzle_m       = 0.6
    };

    return bbtc_ib_geometry_validate_double(&geometry) == BBTC_STATUS_SUCCESS
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Verifies precision-qualified projectile data through the consumer.
 *
 * @return `EXIT_SUCCESS` when the projectile contract works; otherwise
 *         `EXIT_FAILURE`.
 */
static int
check_projectile_contract(void)
{
    const bbtc_ib_projectile_double_t projectile =
    {
        .mass_kg = 0.01134
    };

    return bbtc_ib_projectile_validate_double(&projectile) == BBTC_STATUS_SUCCESS
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Verifies precision-qualified propellant-charge data through the consumer.
 *
 * @return `EXIT_SUCCESS` when the propellant-charge contract works; otherwise
 *         `EXIT_FAILURE`.
 */
static int
check_propellant_charge_contract(void)
{
    const bbtc_ib_propellant_charge_double_t charge =
    {
        .charge_mass_kg                    = 0.0030,
        .condensed_phase_density_kg_per_m3 = 1600.0
    };

    return bbtc_ib_propellant_charge_validate_double(&charge) == BBTC_STATUS_SUCCESS
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}

/**
 * @brief Exercises public BBTC headers and linked diagnostic symbols.
 *
 * @return `EXIT_SUCCESS` when the external consumer contract works; otherwise
 *         `EXIT_FAILURE`.
 */
int main(void)
{
    if (check_string(bbtc_status_string(BBTC_STATUS_SUCCESS),
                     "success")
        != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (check_precision_contract() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (check_geometry_contract() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (check_projectile_contract() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (check_propellant_charge_contract() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (check_string(bbtc_ib_termination_string(BBTC_IB_TERMINATION_MUZZLE_EXIT),
                     "projectile reached muzzle exit")
        != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    if (check_string(bbtc_warning_flag_string(BBTC_WARNING_DATA_EXTRAPOLATED),
                     "data record extrapolated")
        != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    return check_string(bbtc_applicability_flag_string(BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN),
                        "outside documented calibration domain");
}
