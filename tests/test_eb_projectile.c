/**
 * @file
 * @brief Tests EB0.1-B3 projectile physical properties and validation.
 *
 * These tests freeze native precision-family representation, declaration order,
 * center-of-mass principal-inertia semantics, independent aerodynamic reference
 * quantities, deterministic validation precedence, and fail-closed behavior.
 */

#include <bbtc/bbtc.h>

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>

/* Principal-inertia fields remain native in every precision family. */
static_assert(_Generic(((bbtc_eb_principal_inertia_float_t*)0)
                           ->moment_about_x_kg_m2,
                       float: 1,
                       default: 0));
static_assert(_Generic(((bbtc_eb_principal_inertia_double_t*)0)
                           ->moment_about_y_kg_m2,
                       double: 1,
                       default: 0));
static_assert(_Generic(((bbtc_eb_principal_inertia_long_double_t*)0)
                           ->moment_about_z_kg_m2,
                       long double: 1,
                       default: 0));

/* Projectile properties preserve their native scalar and nested record types. */
static_assert(_Generic(((bbtc_eb_projectile_properties_float_t*)0)
                           ->principal_inertia,
                       bbtc_eb_principal_inertia_float_t: 1,
                       default: 0));
static_assert(_Generic(((bbtc_eb_projectile_properties_double_t*)0)
                           ->principal_inertia,
                       bbtc_eb_principal_inertia_double_t: 1,
                       default: 0));
static_assert(_Generic(((bbtc_eb_projectile_properties_long_double_t*)0)
                           ->principal_inertia,
                       bbtc_eb_principal_inertia_long_double_t: 1,
                       default: 0));

static_assert(_Generic(((bbtc_eb_projectile_properties_float_t*)0)->mass_kg,
                       float: 1,
                       default: 0));
static_assert(_Generic(((bbtc_eb_projectile_properties_double_t*)0)
                           ->reference_diameter_m,
                       double: 1,
                       default: 0));
static_assert(_Generic(((bbtc_eb_projectile_properties_long_double_t*)0)
                           ->reference_area_m2,
                       long double: 1,
                       default: 0));

/* C guarantees increasing addresses for later non-bit-field members. */
#define BBTC_EB_ASSERT_INERTIA_ORDER(TYPE)                                \
    static_assert(                                                        \
        offsetof(TYPE, moment_about_x_kg_m2)                              \
            < offsetof(TYPE, moment_about_y_kg_m2)                        \
    );                                                                    \
    static_assert(                                                        \
        offsetof(TYPE, moment_about_y_kg_m2)                              \
            < offsetof(TYPE, moment_about_z_kg_m2)                        \
    )

BBTC_EB_ASSERT_INERTIA_ORDER(bbtc_eb_principal_inertia_float_t);
BBTC_EB_ASSERT_INERTIA_ORDER(bbtc_eb_principal_inertia_double_t);
BBTC_EB_ASSERT_INERTIA_ORDER(bbtc_eb_principal_inertia_long_double_t);

#undef BBTC_EB_ASSERT_INERTIA_ORDER

#define BBTC_EB_ASSERT_PROJECTILE_ORDER(TYPE)                             \
    static_assert(                                                        \
        offsetof(TYPE, principal_inertia) < offsetof(TYPE, mass_kg)       \
    );                                                                    \
    static_assert(                                                        \
        offsetof(TYPE, mass_kg) < offsetof(TYPE, reference_diameter_m)    \
    );                                                                    \
    static_assert(                                                        \
        offsetof(TYPE, reference_diameter_m)                              \
            < offsetof(TYPE, reference_area_m2)                           \
    )

BBTC_EB_ASSERT_PROJECTILE_ORDER(bbtc_eb_projectile_properties_float_t);
BBTC_EB_ASSERT_PROJECTILE_ORDER(bbtc_eb_projectile_properties_double_t);
BBTC_EB_ASSERT_PROJECTILE_ORDER(bbtc_eb_projectile_properties_long_double_t);

#undef BBTC_EB_ASSERT_PROJECTILE_ORDER


/**
 * @brief Returns representative valid native-double projectile properties.
 */
static bbtc_eb_projectile_properties_double_t
valid_double_properties(void)
{
    return (bbtc_eb_projectile_properties_double_t)
    {
        .principal_inertia =
        {
            .moment_about_x_kg_m2 = 1.0,
            .moment_about_y_kg_m2 = 2.0,
            .moment_about_z_kg_m2 = 2.0
        },
        .mass_kg = 1.0,
        .reference_diameter_m = 2.0,
        .reference_area_m2 = 1.0
    };
}


/**
 * @brief Verifies ordinary valid, non-axisymmetric, and independent-reference
 *        geometry cases.
 */
static int
test_valid_double_properties(void)
{
    bbtc_eb_projectile_properties_double_t properties =
        valid_double_properties();

    if (bbtc_eb_projectile_properties_validate_double(NULL)
        != BBTC_STATUS_INVALID_ARGUMENT)
    {
        return EXIT_FAILURE;
    }

    if (bbtc_eb_projectile_properties_validate_double(&properties)
        != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    /*
     * Deliberately non-axisymmetric diagonal inertia. B3 must not force the two
     * transverse moments to equality.
     */
    properties.principal_inertia.moment_about_x_kg_m2 = 2.0;
    properties.principal_inertia.moment_about_y_kg_m2 = 3.0;
    properties.principal_inertia.moment_about_z_kg_m2 = 4.0;

    if (bbtc_eb_projectile_properties_validate_double(&properties)
        != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    /*
     * Diameter 2 and area 1 deliberately do not satisfy pi*d^2/4. Validation
     * must treat them as independent positive aerodynamic reference quantities.
     */
    return properties.reference_diameter_m == 2.0
            && properties.reference_area_m2 == 1.0
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Verifies positivity requirements and zero-initialized invalid storage.
 */
static int
test_positive_domain_requirements(void)
{
    const bbtc_eb_projectile_properties_double_t zero = {0};

    if (bbtc_eb_projectile_properties_validate_double(&zero)
        != BBTC_STATUS_OUTSIDE_DOMAIN)
    {
        return EXIT_FAILURE;
    }

    bbtc_eb_projectile_properties_double_t properties =
        valid_double_properties();

#define BBTC_EXPECT_DOMAIN_FAILURE(MEMBER, VALUE)                         \
    do                                                                    \
    {                                                                     \
        properties = valid_double_properties();                           \
        properties.MEMBER = (VALUE);                                     \
        if (bbtc_eb_projectile_properties_validate_double(&properties)   \
            != BBTC_STATUS_OUTSIDE_DOMAIN)                               \
        {                                                                 \
            return EXIT_FAILURE;                                          \
        }                                                                 \
    } while (0)

    BBTC_EXPECT_DOMAIN_FAILURE(mass_kg, 0.0);
    BBTC_EXPECT_DOMAIN_FAILURE(mass_kg, -1.0);
    BBTC_EXPECT_DOMAIN_FAILURE(
        principal_inertia.moment_about_x_kg_m2,
        0.0
    );
    BBTC_EXPECT_DOMAIN_FAILURE(
        principal_inertia.moment_about_x_kg_m2,
        -1.0
    );
    BBTC_EXPECT_DOMAIN_FAILURE(
        principal_inertia.moment_about_y_kg_m2,
        0.0
    );
    BBTC_EXPECT_DOMAIN_FAILURE(
        principal_inertia.moment_about_y_kg_m2,
        -1.0
    );
    BBTC_EXPECT_DOMAIN_FAILURE(
        principal_inertia.moment_about_z_kg_m2,
        0.0
    );
    BBTC_EXPECT_DOMAIN_FAILURE(
        principal_inertia.moment_about_z_kg_m2,
        -1.0
    );
    BBTC_EXPECT_DOMAIN_FAILURE(reference_diameter_m, 0.0);
    BBTC_EXPECT_DOMAIN_FAILURE(reference_diameter_m, -1.0);
    BBTC_EXPECT_DOMAIN_FAILURE(reference_area_m2, 0.0);
    BBTC_EXPECT_DOMAIN_FAILURE(reference_area_m2, -1.0);

#undef BBTC_EXPECT_DOMAIN_FAILURE

    return EXIT_SUCCESS;
}


/**
 * @brief Verifies all inertia-triangle boundaries and violations.
 */
static int
test_inertia_triangle_requirements(void)
{
    bbtc_eb_projectile_properties_double_t properties =
        valid_double_properties();

    /* Equality is explicitly valid: Izz == Ixx + Iyy. */
    properties.principal_inertia.moment_about_x_kg_m2 = 1.0;
    properties.principal_inertia.moment_about_y_kg_m2 = 2.0;
    properties.principal_inertia.moment_about_z_kg_m2 = 3.0;

    if (bbtc_eb_projectile_properties_validate_double(&properties)
        != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    properties = valid_double_properties();
    properties.principal_inertia.moment_about_x_kg_m2 = 5.0;
    properties.principal_inertia.moment_about_y_kg_m2 = 2.0;
    properties.principal_inertia.moment_about_z_kg_m2 = 2.0;

    if (bbtc_eb_projectile_properties_validate_double(&properties)
        != BBTC_STATUS_OUTSIDE_DOMAIN)
    {
        return EXIT_FAILURE;
    }

    properties = valid_double_properties();
    properties.principal_inertia.moment_about_x_kg_m2 = 2.0;
    properties.principal_inertia.moment_about_y_kg_m2 = 5.0;
    properties.principal_inertia.moment_about_z_kg_m2 = 2.0;

    if (bbtc_eb_projectile_properties_validate_double(&properties)
        != BBTC_STATUS_OUTSIDE_DOMAIN)
    {
        return EXIT_FAILURE;
    }

    properties = valid_double_properties();
    properties.principal_inertia.moment_about_x_kg_m2 = 2.0;
    properties.principal_inertia.moment_about_y_kg_m2 = 2.0;
    properties.principal_inertia.moment_about_z_kg_m2 = 5.0;

    if (bbtc_eb_projectile_properties_validate_double(&properties)
        != BBTC_STATUS_OUTSIDE_DOMAIN)
    {
        return EXIT_FAILURE;
    }

    /*
     * All three values are finite and valid. Pairwise sums would overflow
     * double, but no finite moment can exceed either real pairwise sum.
     */
    properties = valid_double_properties();
    properties.principal_inertia.moment_about_x_kg_m2 = DBL_MAX;
    properties.principal_inertia.moment_about_y_kg_m2 = DBL_MAX;
    properties.principal_inertia.moment_about_z_kg_m2 = DBL_MAX;

    return bbtc_eb_projectile_properties_validate_double(&properties)
            == BBTC_STATUS_SUCCESS
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Verifies complete-record NaN/infinity/domain precedence.
 */
static int
test_validation_precedence(void)
{
    bbtc_eb_projectile_properties_double_t properties =
        valid_double_properties();

    properties.mass_kg = NAN;
    properties.reference_area_m2 = INFINITY;
    properties.principal_inertia.moment_about_x_kg_m2 = -1.0;

    if (bbtc_eb_projectile_properties_validate_double(&properties)
        != BBTC_STATUS_NAN_INPUT)
    {
        return EXIT_FAILURE;
    }

    properties.mass_kg = 1.0;

    if (bbtc_eb_projectile_properties_validate_double(&properties)
        != BBTC_STATUS_NONFINITE_INPUT)
    {
        return EXIT_FAILURE;
    }

    properties.reference_area_m2 = 1.0;

    return bbtc_eb_projectile_properties_validate_double(&properties)
            == BBTC_STATUS_OUTSIDE_DOMAIN
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Verifies validation does not mutate caller-owned properties.
 */
static int
test_validation_is_read_only(void)
{
    bbtc_eb_projectile_properties_double_t properties =
        valid_double_properties();
    const bbtc_eb_projectile_properties_double_t before = properties;

    if (bbtc_eb_projectile_properties_validate_double(&properties)
        != BBTC_STATUS_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    return properties.principal_inertia.moment_about_x_kg_m2
                == before.principal_inertia.moment_about_x_kg_m2
        && properties.principal_inertia.moment_about_y_kg_m2
                == before.principal_inertia.moment_about_y_kg_m2
        && properties.principal_inertia.moment_about_z_kg_m2
                == before.principal_inertia.moment_about_z_kg_m2
        && properties.mass_kg == before.mass_kg
        && properties.reference_diameter_m == before.reference_diameter_m
        && properties.reference_area_m2 == before.reference_area_m2
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


/**
 * @brief Exercises native float and long-double validation entry points.
 */
static int
test_other_precision_families(void)
{
    const bbtc_eb_projectile_properties_float_t float_properties =
    {
        .principal_inertia =
        {
            .moment_about_x_kg_m2 = 1.0f,
            .moment_about_y_kg_m2 = 2.0f,
            .moment_about_z_kg_m2 = 2.0f
        },
        .mass_kg = 1.0f,
        .reference_diameter_m = 2.0f,
        .reference_area_m2 = 1.0f
    };

    const bbtc_eb_projectile_properties_long_double_t long_double_properties =
    {
        .principal_inertia =
        {
            .moment_about_x_kg_m2 = 1.0L,
            .moment_about_y_kg_m2 = 2.0L,
            .moment_about_z_kg_m2 = 2.0L
        },
        .mass_kg = 1.0L,
        .reference_diameter_m = 2.0L,
        .reference_area_m2 = 1.0L
    };

    return bbtc_eb_projectile_properties_validate_float(&float_properties)
                == BBTC_STATUS_SUCCESS
        && bbtc_eb_projectile_properties_validate_long_double(
            &long_double_properties
        ) == BBTC_STATUS_SUCCESS
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}


int
main(void)
{
    if (test_valid_double_properties() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_positive_domain_requirements() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_inertia_triangle_requirements() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_validation_precedence() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_validation_is_read_only() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    return test_other_precision_families();
}
