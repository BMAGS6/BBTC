#include "bbtc/precision.h"

#include <stddef.h>
#include <float.h>

const char*
bbtc_precision_string(const bbtc_precision_e precision)
{
    switch (precision)
    {
        case BBTC_PRECISION_FLOAT:
            return "float";

        case BBTC_PRECISION_DOUBLE:
            return "double";

        case BBTC_PRECISION_LONG_DOUBLE:
            return "long double";

        default:
            return "unknown BBTC precision";
    }
}

bbtc_status_e
bbtc_precision_info(
    const bbtc_precision_e precision,
    bbtc_precision_info_t* const out_info
)
{
    if (out_info == NULL)
    {
        return BBTC_STATUS_INVALID_ARGUMENT;
    }

    *out_info = (bbtc_precision_info_t){0};

    switch (precision)
    {
        case BBTC_PRECISION_FLOAT:
            *out_info = (bbtc_precision_info_t)
            {
                .precision = BBTC_PRECISION_FLOAT,
                .radix = (uint32_t)FLT_RADIX,
                .mantissa_digits = (uint32_t)FLT_MANT_DIG,
                .minimum_normal_exponent = (int32_t)FLT_MIN_EXP,
                .maximum_finite_exponent = (int32_t)FLT_MAX_EXP,
                .decimal_digits = (uint32_t)FLT_DECIMAL_DIG,
                .storage_bytes = (uint32_t)sizeof(float)
            };
            return BBTC_STATUS_SUCCESS;

        case BBTC_PRECISION_DOUBLE:
            *out_info = (bbtc_precision_info_t)
            {
                .precision = BBTC_PRECISION_DOUBLE,
                .radix = (uint32_t)FLT_RADIX,
                .mantissa_digits = (uint32_t)DBL_MANT_DIG,
                .minimum_normal_exponent = (int32_t)DBL_MIN_EXP,
                .maximum_finite_exponent = (int32_t)DBL_MAX_EXP,
                .decimal_digits = (uint32_t)DBL_DECIMAL_DIG,
                .storage_bytes = (uint32_t)sizeof(double)
            };
            return BBTC_STATUS_SUCCESS;

        case BBTC_PRECISION_LONG_DOUBLE:
            *out_info = (bbtc_precision_info_t)
            {
                .precision = BBTC_PRECISION_LONG_DOUBLE,
                .radix = (uint32_t)FLT_RADIX,
                .mantissa_digits = (uint32_t)LDBL_MANT_DIG,
                .minimum_normal_exponent = (int32_t)LDBL_MIN_EXP,
                .maximum_finite_exponent = (int32_t)LDBL_MAX_EXP,
                .decimal_digits = (uint32_t)LDBL_DECIMAL_DIG,
                .storage_bytes = (uint32_t)sizeof(long double)
            };
            return BBTC_STATUS_SUCCESS;

        default:
            return BBTC_STATUS_INVALID_ARGUMENT;
    }
}
