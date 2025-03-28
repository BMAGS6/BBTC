/**
 * @file compiler_macros.h
 * @brief Contains macros for cross-compatible compiler optimizations and compatibility.
 */

#ifndef COMPILER_MACROS_H
#define COMPILER_MACROS_H

/** 
 * @defgroup CompilerFlags Compiler Detection Macros
 * @brief Macros that detect the compiler being used.
 *
 * These macros allow conditional compilation based on the detected compiler.
 * They normally do not have to be manually set by the user.
 */

/** @def COMPILER_CLANG
 *  @ingroup CompilerFlags
 *  @brief Set to 1 if compiled with Clang, otherwise 0.
 */
#define COMPILER_CLANG 0

/** @def COMPILER_GCC
 *  @ingroup CompilerFlags
 *  @brief Set to 1 if compiled with GCC, otherwise 0.
 */
#define COMPILER_GCC 0

/** @def COMPILER_MSVC
 *  @ingroup CompilerFlags
 *  @brief Set to 1 if compiled with Microsoft Visual C++, otherwise 0.
 */
#define COMPILER_MSVC 0

/** @def COMPILER_UNKNOWN
 *  @ingroup CompilerFlags
 *  @brief Set to 1 if the compiler is unknown or unsupported, otherwise 0.
 */
#define COMPILER_UNKNOWN 0

#if defined (__clang__)
  #undef COMPILER_CLANG
  #define COMPILER_CLANG 1
#elif defined(__GNUC__) || defined(__GNUG__)
  #undef COMPILER_GCC
  #define COMPILER_GCC 1
#elif defined(_MSC_VER)
  #undef COMPILER_MSVC
  #define COMPILER_MSVC 1
#else
  #undef COMPILER_UNKNOWN
  #define COMPILER_UNKNOWN 1
#endif

/** 
 * @defgroup STDCVersion C Standard Version Macros
 * @brief Macros for detecting the C standard version used.
 *
 * These macros set C_VERSION to the detected C standard:
 * - 23 for C23
 * - 17 for C17
 * - 11 for C11
 * - 99 for C99
 * - 90 for C90
 */
#ifdef __STDC_VERSION__
  #if __STDC_VERSION__ >= 202311L
    #define C_VERSION 23
  #elif __STDC_VERSION__ >= 201710L
    #define C_VERSION 17
  #elif __STDC_VERSION__ >= 201112L
    #define C_VERSION 11
  #elif __STDC_VERSION__ >= 199901L
    #define C_VERSION 99
  #else
    #define C_VERSION 90
  #endif
#else
  #define C_VERSION 90
#endif

#ifdef __cplusplus
  #undef restrict
  #define restrict /* Disable restrict in C++ */
#endif

/**
 * @defgroup OptimizationMacros Optimization and Performance Macros
 * @brief Macros for compiler optimizations and function visibility.
 */

/// @def ALIGN(n)
/// @ingroup OptimizationMacros
/// @brief Aligns a variable or struct to \p n bytes.
///
/// Example usage:
/// @code
/// typedef struct ALIGN(64) {
///     int a;
///     double b;
/// } AlignedStruct;
/// @endcode
#ifdef DOXYGEN
  #define ALIGN(n)
#elif COMPILER_GCC || COMPILER_CLANG
  #define ALIGN(n) __attribute__((aligned(n)))
#elif COMPILER_MSVC
  #define ALIGN(n) __declspec(align(n))
#else
  #define ALIGN(n)
#endif

/// @def HAS_BUILTIN(x)
/// @ingroup OptimizationMacros
/// @brief Checks if the compiler supports a particular builtin.
#ifdef DOXYGEN
  #define HAS_BUILTIN(x) 0
#elif defined(__has_builtin)
  #define HAS_BUILTIN(x) __has_builtin(x)
#else
  #define HAS_BUILTIN(x) 0
#endif

/// @def PREFETCH(addr)
/// @ingroup OptimizationMacros
/// @brief Hints to the compiler to prefetch memory at the given address.
#ifdef DOXYGEN
  #define PREFETCH(addr)
#elif HAS_BUILTIN(__builtin_prefetch)
  #define PREFETCH(addr) __builtin_prefetch(addr)
#else
  #define PREFETCH(addr)
#endif

/// @def LIKELY(x)
/// @ingroup OptimizationMacros
/// @brief Optimizes branch prediction by marking an expression as likely true.
///
/// Compilers may use this to optimize hot paths.
#ifdef DOXYGEN
  #define LIKELY(x) (x)
  #define UNLIKELY(x) (x)
#elif COMPILER_GCC || COMPILER_CLANG
  #define LIKELY(x)   __builtin_expect(!!(x), 1)
  #define UNLIKELY(x) __builtin_expect(!!(x), 0)
#else
  #define LIKELY(x)   (x)
  #define UNLIKELY(x) (x)
#endif

/// @def HOT
/// @ingroup OptimizationMacros
/// @brief Marks a function as frequently called, hinting for better cache locality.
#ifdef DOXYGEN
  #define HOT
#elif COMPILER_GCC || COMPILER_CLANG
  #define HOT __attribute__((hot))
#else
  #define HOT
#endif

/// @def EXPORT_SYMBOL
/// @ingroup OptimizationMacros
/// @brief Controls symbol visibility in shared libraries.
#ifdef DOXYGEN
  #define EXPORT_SYMBOL
#elif COMPILER_GCC || COMPILER_CLANG
  #define EXPORT_SYMBOL __attribute__((visibility("default")))
#elif COMPILER_MSVC
  #define EXPORT_SYMBOL __declspec(dllexport)
#else
  #define EXPORT_SYMBOL
#endif

#endif // COMPILER_MACROS_H
