cmake_minimum_required(VERSION 3.22)

project(
    BBTCConsumerFixture
    LANGUAGES C
)

if(NOT DEFINED BBTC_SOURCE_DIR)
    message(
        FATAL_ERROR
        "BBTC_SOURCE_DIR must identify the BBTC source tree"
    )
endif()

set(
    BBTC_BUILD_TESTS
    OFF
    CACHE BOOL
    "Disable BBTC's own tests inside the consumer fixture"
    FORCE
)

add_subdirectory(
    "${BBTC_SOURCE_DIR}"
    bbtc
    EXCLUDE_FROM_ALL
)

add_executable(
    bbtc_consumer_fixture
    main.c
)

set_target_properties(
    bbtc_consumer_fixture
    PROPERTIES
        C_EXTENSIONS OFF
)

target_link_libraries(
    bbtc_consumer_fixture
    PRIVATE
        bbtc::bbtc
)

# Intentionally do not add BBTC include paths or request C23 here. Both
# requirements must propagate through the public bbtc::bbtc target.
enable_testing()

add_test(
    NAME bbtc.consumer.runtime
    COMMAND bbtc_consumer_fixture
)
