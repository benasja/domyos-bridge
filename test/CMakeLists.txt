# Host unit tests for the IDF-free modules. Plain CMake, no ESP-IDF needed:
#   cmake -S test -B build-test && cmake --build build-test && ctest --test-dir build-test --output-on-failure
cmake_minimum_required(VERSION 3.16)
project(domyos_bridge_host_tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(MAIN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../main)

add_executable(host_tests
    test_main.cpp
    test_ftms_parser.cpp
    test_crank_synth.cpp
    test_bridge.cpp
    test_power_filter.cpp
    ${MAIN_DIR}/ftms_parser.cpp
    ${MAIN_DIR}/crank_synth.cpp
    ${MAIN_DIR}/shared_state.cpp
    ${MAIN_DIR}/power_filter.cpp
)
target_include_directories(host_tests PRIVATE ${MAIN_DIR})
# Same constraints as the firmware: no exceptions, no RTTI.
target_compile_options(host_tests PRIVATE -Wall -Wextra -Werror -fno-exceptions -fno-rtti
    -fsanitize=address,undefined)
target_link_options(host_tests PRIVATE -fsanitize=address,undefined)

enable_testing()
add_test(NAME host_tests COMMAND host_tests)
