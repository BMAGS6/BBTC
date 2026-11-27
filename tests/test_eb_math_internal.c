/**
 * @file
 * @brief White-box tests for the private EB0.1-B2 mathematical foundation.
 *
 * These tests intentionally include the private math header. Ordinary BBTC
 * consumers do not. The purpose is to verify the exact operations that later
 * rigid-body dynamics will depend on before aerodynamic or integration code is
 * allowed to use them.
 */

#include "external/math_internal.h"

#include <math.h>
#include <stdlib.h>

static_assert(
    _Generic(
        bbtc_eb_internal_vec3_dot_float(
            (bbtc_eb_vec3_float_t){0},
            (bbtc_eb_vec3_float_t){0}
        ),
        float: 1,
        default: 0
    )
);

static_assert(
    _Generic(
        bbtc_eb_internal_vec3_norm_double((bbtc_eb_vec3_double_t){0}),
        double: 1,
        default: 0
    )
);

static_assert(
    _Generic(
        bbtc_eb_internal_quat_norm_long_double(
            (bbtc_eb_quat_long_double_t){0}
        ),
        long double: 1,
        default: 0
    )
);

/**
 * @brief Returns whether two doubles are close enough for convention tests.
 */
static int
near_double(double a, double b)
{
    return fabs(a - b) <= 1.0e-12;
}

/**
 * @brief Verifies the right-handed vector basis and elementary operations.
 */
static int
test_vector_algebra_double(void)
{
    const bbtc_eb_vec3_double_t x = {1.0, 0.0, 0.0};
    const bbtc_eb_vec3_double_t y = {0.0, 1.0, 0.0};
    const bbtc_eb_vec3_double_t z = {0.0, 0.0, 1.0};

    const bbtc_eb_vec3_double_t x_cross_y =
        bbtc_eb_internal_vec3_cross_double(x, y);
    const bbtc_eb_vec3_double_t y_cross_z =
        bbtc_eb_internal_vec3_cross_double(y, z);
    const bbtc_eb_vec3_double_t z_cross_x =
        bbtc_eb_internal_vec3_cross_double(z, x);
    const bbtc_eb_vec3_double_t y_cross_x =
        bbtc_eb_internal_vec3_cross_double(y, x);

    if (x_cross_y.x != 0.0 || x_cross_y.y != 0.0 || x_cross_y.z != 1.0
        || y_cross_z.x != 1.0 || y_cross_z.y != 0.0 || y_cross_z.z != 0.0
        || z_cross_x.x != 0.0 || z_cross_x.y != 1.0 || z_cross_x.z != 0.0
        || y_cross_x.x != 0.0 || y_cross_x.y != 0.0 || y_cross_x.z != -1.0)
    {
        return EXIT_FAILURE;
    }

    const bbtc_eb_vec3_double_t a = {1.0, 2.0, 3.0};
    const bbtc_eb_vec3_double_t b = {4.0, 5.0, 6.0};
    const bbtc_eb_vec3_double_t sum =
        bbtc_eb_internal_vec3_add_double(a, b);
    const bbtc_eb_vec3_double_t difference =
        bbtc_eb_internal_vec3_subtract_double(b, a);
    const bbtc_eb_vec3_double_t scaled =
        bbtc_eb_internal_vec3_scale_double(a, 2.0);

    return sum.x == 5.0 && sum.y == 7.0 && sum.z == 9.0
        && difference.x == 3.0 && difference.y == 3.0
        && difference.z == 3.0
        && scaled.x == 2.0 && scaled.y == 4.0 && scaled.z == 6.0
        && bbtc_eb_internal_vec3_dot_double(a, b) == 32.0
        && bbtc_eb_internal_vec3_norm_squared_double(a) == 14.0
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}

/**
 * @brief Verifies vector normalization success and fail-closed behavior.
 */
static int
test_vector_normalization(void)
{
    const bbtc_eb_vec3_double_t input = {3.0, 4.0, 0.0};
    bbtc_eb_vec3_double_t output = {9.0, 9.0, 9.0};

    if (bbtc_eb_internal_vec3_normalize_double(&input, &output)
            != BBTC_STATUS_SUCCESS
        || !near_double(output.x, 0.6)
        || !near_double(output.y, 0.8)
        || !near_double(output.z, 0.0))
    {
        return EXIT_FAILURE;
    }

    const bbtc_eb_vec3_double_t zero = {0};
    output = (bbtc_eb_vec3_double_t){9.0, 9.0, 9.0};
    if (bbtc_eb_internal_vec3_normalize_double(&zero, &output)
            != BBTC_STATUS_OUTSIDE_DOMAIN
        || output.x != 0.0 || output.y != 0.0 || output.z != 0.0)
    {
        return EXIT_FAILURE;
    }

    const bbtc_eb_vec3_double_t nan_value = {NAN, INFINITY, 0.0};
    output = (bbtc_eb_vec3_double_t){9.0, 9.0, 9.0};
    if (bbtc_eb_internal_vec3_normalize_double(&nan_value, &output)
            != BBTC_STATUS_NAN_INPUT
        || output.x != 0.0 || output.y != 0.0 || output.z != 0.0)
    {
        return EXIT_FAILURE;
    }

    const bbtc_eb_vec3_double_t infinite = {INFINITY, 0.0, 0.0};
    output = (bbtc_eb_vec3_double_t){9.0, 9.0, 9.0};
    if (bbtc_eb_internal_vec3_normalize_double(&infinite, &output)
            != BBTC_STATUS_NONFINITE_INPUT
        || output.x != 0.0 || output.y != 0.0 || output.z != 0.0)
    {
        return EXIT_FAILURE;
    }

    output = (bbtc_eb_vec3_double_t){9.0, 9.0, 9.0};
    if (bbtc_eb_internal_vec3_normalize_double(NULL, &output)
            != BBTC_STATUS_INVALID_ARGUMENT
        || output.x != 0.0 || output.y != 0.0 || output.z != 0.0
        || bbtc_eb_internal_vec3_normalize_double(&input, NULL)
            != BBTC_STATUS_INVALID_ARGUMENT)
    {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

/**
 * @brief Constructs the explicit q*(0,v)*conjugate(q) reference rotation.
 */
static bbtc_eb_vec3_double_t
reference_rotate_double(
    bbtc_eb_quat_double_t q,
    bbtc_eb_vec3_double_t v
)
{
    const bbtc_eb_quat_double_t pure = {v.x, v.y, v.z, 0.0};
    const bbtc_eb_quat_double_t rotated =
        bbtc_eb_internal_quat_multiply_double(
            bbtc_eb_internal_quat_multiply_double(q, pure),
            bbtc_eb_internal_quat_conjugate_double(q)
        );

    return (bbtc_eb_vec3_double_t){rotated.x, rotated.y, rotated.z};
}

/**
 * @brief Verifies Hamilton composition, active rotation, and q/-q equivalence.
 */
static int
test_quaternion_conventions_double(void)
{
    const double s = sqrt(0.5);
    const bbtc_eb_quat_double_t identity =
        bbtc_eb_internal_quat_identity_double();
    const bbtc_eb_quat_double_t qx = {s, 0.0, 0.0, s};
    const bbtc_eb_quat_double_t qy = {0.0, s, 0.0, s};
    const bbtc_eb_quat_double_t qz = {0.0, 0.0, s, s};
    const bbtc_eb_vec3_double_t x = {1.0, 0.0, 0.0};
    const bbtc_eb_vec3_double_t y = {0.0, 1.0, 0.0};
    const bbtc_eb_vec3_double_t z = {0.0, 0.0, 1.0};

    const bbtc_eb_vec3_double_t identity_x =
        bbtc_eb_internal_quat_rotate_vec3_double(identity, x);
    const bbtc_eb_vec3_double_t rx_y =
        bbtc_eb_internal_quat_rotate_vec3_double(qx, y);
    const bbtc_eb_vec3_double_t ry_z =
        bbtc_eb_internal_quat_rotate_vec3_double(qy, z);
    const bbtc_eb_vec3_double_t rz_x =
        bbtc_eb_internal_quat_rotate_vec3_double(qz, x);

    if (!near_double(identity_x.x, 1.0)
        || !near_double(identity_x.y, 0.0)
        || !near_double(identity_x.z, 0.0)
        || !near_double(rx_y.x, 0.0)
        || !near_double(rx_y.y, 0.0)
        || !near_double(rx_y.z, 1.0)
        || !near_double(ry_z.x, 1.0)
        || !near_double(ry_z.y, 0.0)
        || !near_double(ry_z.z, 0.0)
        || !near_double(rz_x.x, 0.0)
        || !near_double(rz_x.y, 1.0)
        || !near_double(rz_x.z, 0.0))
    {
        return EXIT_FAILURE;
    }

    const bbtc_eb_quat_double_t xy =
        bbtc_eb_internal_quat_multiply_double(qx, qy);
    const bbtc_eb_quat_double_t yx =
        bbtc_eb_internal_quat_multiply_double(qy, qx);
    const bbtc_eb_vec3_double_t xy_z =
        bbtc_eb_internal_quat_rotate_vec3_double(xy, z);
    const bbtc_eb_vec3_double_t yx_z =
        bbtc_eb_internal_quat_rotate_vec3_double(yx, z);

    if (near_double(xy_z.x, yx_z.x)
        && near_double(xy_z.y, yx_z.y)
        && near_double(xy_z.z, yx_z.z))
    {
        return EXIT_FAILURE;
    }

    const bbtc_eb_vec3_double_t arbitrary = {0.25, -0.5, 0.75};
    const bbtc_eb_vec3_double_t optimized =
        bbtc_eb_internal_quat_rotate_vec3_double(qz, arbitrary);
    const bbtc_eb_vec3_double_t reference =
        reference_rotate_double(qz, arbitrary);

    if (!near_double(optimized.x, reference.x)
        || !near_double(optimized.y, reference.y)
        || !near_double(optimized.z, reference.z))
    {
        return EXIT_FAILURE;
    }

    const bbtc_eb_quat_double_t negative_qz =
        {-qz.x, -qz.y, -qz.z, -qz.w};
    const bbtc_eb_vec3_double_t negative_rotation =
        bbtc_eb_internal_quat_rotate_vec3_double(negative_qz, arbitrary);

    return near_double(optimized.x, negative_rotation.x)
        && near_double(optimized.y, negative_rotation.y)
        && near_double(optimized.z, negative_rotation.z)
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}

/**
 * @brief Verifies quaternion normalization and failure-output clearing.
 */
static int
test_quaternion_normalization(void)
{
    const bbtc_eb_quat_double_t input = {0.0, 0.0, 0.0, 2.0};
    bbtc_eb_quat_double_t output = {9.0, 9.0, 9.0, 9.0};

    if (bbtc_eb_internal_quat_normalize_double(&input, &output)
            != BBTC_STATUS_SUCCESS
        || output.x != 0.0 || output.y != 0.0 || output.z != 0.0
        || output.w != 1.0)
    {
        return EXIT_FAILURE;
    }

    const bbtc_eb_quat_double_t zero = {0};
    output = (bbtc_eb_quat_double_t){9.0, 9.0, 9.0, 9.0};
    if (bbtc_eb_internal_quat_normalize_double(&zero, &output)
            != BBTC_STATUS_OUTSIDE_DOMAIN
        || output.x != 0.0 || output.y != 0.0 || output.z != 0.0
        || output.w != 0.0)
    {
        return EXIT_FAILURE;
    }

    const bbtc_eb_quat_double_t nan_value = {NAN, 0.0, INFINITY, 1.0};
    output = (bbtc_eb_quat_double_t){9.0, 9.0, 9.0, 9.0};
    if (bbtc_eb_internal_quat_normalize_double(&nan_value, &output)
            != BBTC_STATUS_NAN_INPUT
        || output.x != 0.0 || output.y != 0.0 || output.z != 0.0
        || output.w != 0.0)
    {
        return EXIT_FAILURE;
    }

    const bbtc_eb_quat_double_t infinite = {INFINITY, 0.0, 0.0, 1.0};
    output = (bbtc_eb_quat_double_t){9.0, 9.0, 9.0, 9.0};
    return bbtc_eb_internal_quat_normalize_double(&infinite, &output)
            == BBTC_STATUS_NONFINITE_INPUT
        && output.x == 0.0 && output.y == 0.0
        && output.z == 0.0 && output.w == 0.0
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}

/**
 * @brief Locks down the BBTC-body to Cubes-of-Honor-local basis rotation.
 */
static int
test_coh_basis_reference(void)
{
    const bbtc_eb_quat_double_t q_cb = {0.5, 0.5, -0.5, 0.5};
    const bbtc_eb_vec3_double_t x = {1.0, 0.0, 0.0};
    const bbtc_eb_vec3_double_t y = {0.0, 1.0, 0.0};
    const bbtc_eb_vec3_double_t z = {0.0, 0.0, 1.0};

    const bbtc_eb_vec3_double_t mapped_x =
        bbtc_eb_internal_quat_rotate_vec3_double(q_cb, x);
    const bbtc_eb_vec3_double_t mapped_y =
        bbtc_eb_internal_quat_rotate_vec3_double(q_cb, y);
    const bbtc_eb_vec3_double_t mapped_z =
        bbtc_eb_internal_quat_rotate_vec3_double(q_cb, z);

    return near_double(mapped_x.x, 0.0)
        && near_double(mapped_x.y, 0.0)
        && near_double(mapped_x.z, -1.0)
        && near_double(mapped_y.x, 1.0)
        && near_double(mapped_y.y, 0.0)
        && near_double(mapped_y.z, 0.0)
        && near_double(mapped_z.x, 0.0)
        && near_double(mapped_z.y, -1.0)
        && near_double(mapped_z.z, 0.0)
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}

/**
 * @brief Exercises native float and long-double implementations at runtime.
 */
static int
test_other_precision_families(void)
{
    const bbtc_eb_vec3_float_t vf = {3.0f, 4.0f, 0.0f};
    bbtc_eb_vec3_float_t nf = {0};
    const bbtc_eb_quat_long_double_t ql =
        {0.0L, 0.0L, 0.0L, 2.0L};
    bbtc_eb_quat_long_double_t nl = {0};

    if (bbtc_eb_internal_vec3_normalize_float(&vf, &nf)
            != BBTC_STATUS_SUCCESS
        || fabsf(nf.x - 0.6f) > 8.0f * 1.19209290e-7f
        || fabsf(nf.y - 0.8f) > 8.0f * 1.19209290e-7f)
    {
        return EXIT_FAILURE;
    }

    if (bbtc_eb_internal_quat_normalize_long_double(&ql, &nl)
            != BBTC_STATUS_SUCCESS
        || nl.x != 0.0L || nl.y != 0.0L || nl.z != 0.0L || nl.w != 1.0L)
    {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

int
main(void)
{
    if (test_vector_algebra_double() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_vector_normalization() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_quaternion_conventions_double() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_quaternion_normalization() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    if (test_coh_basis_reference() != EXIT_SUCCESS)
        return EXIT_FAILURE;

    return test_other_precision_families();
}
