add_executable(
    bbtc_test_status
    test_status.c
)

target_link_libraries(
    bbtc_test_status
    PRIVATE
        bbtc::bbtc
)

set_target_properties(
    bbtc_test_status
    PROPERTIES
        C_EXTENSIONS OFF
        C_STANDARD 23
        C_STANDARD_REQUIRED ON
)

add_test(
    NAME bbtc.status
    COMMAND bbtc_test_status
)

add_executable(
    bbtc_test_diagnostics
    test_diagnostics.c
)

target_link_libraries(
    bbtc_test_diagnostics
    PRIVATE
        bbtc::bbtc
)

set_target_properties(
    bbtc_test_diagnostics
    PROPERTIES
        C_EXTENSIONS OFF
        C_STANDARD 23
        C_STANDARD_REQUIRED ON
)

add_test(
    NAME bbtc.diagnostics
    COMMAND bbtc_test_diagnostics
)

set(
    BBTC_CONSUMER_FIXTURE_SOURCE_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/consumer"
)

set(
    BBTC_CONSUMER_FIXTURE_BINARY_DIR
    "${CMAKE_CURRENT_BINARY_DIR}/consumer"
)

configure_file(
    consumer/run_consumer_fixture.cmake.in
    run_consumer_fixture.cmake
    @ONLY
)

add_test(
    NAME bbtc.consumer
    COMMAND
        "${CMAKE_COMMAND}"
        -P
        "${CMAKE_CURRENT_BINARY_DIR}/run_consumer_fixture.cmake"
)

# C++ is optional and is used only to verify public-header interoperability.
if(NOT CMAKE_CXX_COMPILER_LOADED)
    include(CheckLanguage)
    check_language(CXX)

    if(CMAKE_CXX_COMPILER)
        enable_language(CXX)
    endif()
endif()

if(CMAKE_CXX_COMPILER_LOADED)
    add_executable(
        bbtc_test_cpp_header
        test_cpp_header.cpp
    )

    target_link_libraries(
        bbtc_test_cpp_header
        PRIVATE
            bbtc::bbtc
    )

    target_compile_features(
        bbtc_test_cpp_header
        PRIVATE
            cxx_std_11
    )

    set_target_properties(
        bbtc_test_cpp_header
        PROPERTIES
            CXX_EXTENSIONS OFF
    )

    add_test(
        NAME bbtc.cpp_header
        COMMAND bbtc_test_cpp_header
    )
endif()
