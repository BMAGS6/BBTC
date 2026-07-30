add_executable(
    bbtc_test_link_smoke
    test_link.c
)

target_link_libraries(
    bbtc_test_link_smoke
    PRIVATE
        bbtc::bbtc
)

set_target_properties(
    bbtc_test_link_smoke
    PROPERTIES
        C_EXTENSIONS OFF
        C_STANDARD 23
        C_STANDARD_REQUIRED ON
)

add_test(
    NAME bbtc.link_smoke
    COMMAND bbtc_test_link_smoke
)
