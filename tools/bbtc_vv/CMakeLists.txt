# Independent VVUQ-only library. No dependency on the production BBTC target.
add_library(bbtc_vv_exact_dyadic STATIC exact_dyadic.c exact_dyadic.h)

target_include_directories(bbtc_vv_exact_dyadic PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_features(bbtc_vv_exact_dyadic PUBLIC c_std_23)
set_target_properties(bbtc_vv_exact_dyadic PROPERTIES
    C_STANDARD 23 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)

# The proof assumes contraction-free arithmetic and correct observation of fenv.
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(bbtc_vv_exact_dyadic PRIVATE
        -frounding-math -ffp-contract=off -fno-fast-math)
endif()

if(UNIX)
    target_link_libraries(bbtc_vv_exact_dyadic PUBLIC m)
endif()

if(BBTC_VVUQ_BUILD_TESTS)
    enable_testing()
    add_subdirectory("${PROJECT_SOURCE_DIR}/tests/validation" "${PROJECT_BINARY_DIR}/vv-validation-tests")
endif()
