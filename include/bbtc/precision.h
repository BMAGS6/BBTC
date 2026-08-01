/**
 * @file
 * @brief Public scalar-precision identity and platform metadata.
 */
#ifndef BBTC_PRECISION_H
#define BBTC_PRECISION_H

#include <stdint.h>

#include <bbtc/status.h>

#ifdef __cplusplus
    extern "C" {
#endif

/**
 * @brief Identifies one first-class BBTC continuous-scalar family.
 *
 * The one-byte underlying representation and numeric values are part of the
 * public interface. Zero and every unrecognized value are invalid and do not
 * select a scalar family. This enumeration identifies metadata and reporting;
 * it does not replace precision-qualified problem, option, result, or function
 * types.
 */
typedef enum bbtc_precision_e : uint8_t
{
    /** The ISO C `float` scalar family. */
    BBTC_PRECISION_FLOAT = 1,

    /** The ISO C `double` scalar family. */
    BBTC_PRECISION_DOUBLE = 2,

    /** The ISO C `long double` scalar family. */
    BBTC_PRECISION_LONG_DOUBLE = 3
}
bbtc_precision_e;

/**
 * @brief Describes the host implementation of one BBTC scalar family.
 *
 * Every field reports a property of the compiler and target used to build the
 * linked BBTC library. Storage size does not by itself establish precision or a
 * particular IEEE 754 interchange format.
 */
typedef struct bbtc_precision_info_t
{
    /** Floating-point radix reported by `FLT_RADIX`. */
    uint32_t radix;

    /** Significand digits reported by the matching `*_MANT_DIG` macro. */
    uint32_t mantissa_digits;

    /** Normalized minimum exponent reported by the matching `*_MIN_EXP`. */
    int32_t minimum_normal_exponent;

    /** Finite maximum exponent reported by the matching `*_MAX_EXP`. */
    int32_t maximum_finite_exponent;

    /** Round-trip decimal digits reported by the matching `*_DECIMAL_DIG`. */
    uint32_t decimal_digits;

    /** Object-representation size reported by `sizeof`, in bytes. */
    uint32_t storage_bytes;

    /** Scalar family described by this record. */
    bbtc_precision_e precision;

    uint8_t _unused_padding[3];
}
bbtc_precision_info_t;

/**
 * @brief Returns immutable, nonlocalized text for a precision value.
 *
 * The returned pointer is never NULL. It refers to immutable static storage,
 * may be read concurrently, and must not be modified or freed. Unrecognized
 * values return `"unknown BBTC precision"`.
 *
 * @param precision Precision value to describe.
 *
 * @return A NUL-terminated string in immutable static storage.
 */
const char*
bbtc_precision_string(bbtc_precision_e precision);

/**
 * @brief Queries host metadata for one supported scalar family.
 *
 * On success, `out_info` is populated from `<float.h>` and `sizeof` properties
 * of the linked library build. When `precision` is zero or unrecognized, the
 * output record is cleared and `BBTC_STATUS_INVALID_ARGUMENT` is returned. A
 * NULL output pointer also returns `BBTC_STATUS_INVALID_ARGUMENT`.
 *
 * @param precision Scalar family to query.
 * @param out_info  Caller-owned output record to populate.
 *
 * @return `BBTC_STATUS_SUCCESS` on success; otherwise a nonzero status.
 */
bbtc_status_e
bbtc_precision_info(bbtc_precision_e       precision,
                    bbtc_precision_info_t* out_info);

#ifdef __cplusplus
}   /* extern "C" */
#endif
#endif /* BBTC_PRECISION_H */
