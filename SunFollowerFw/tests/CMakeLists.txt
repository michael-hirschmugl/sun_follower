cmake_minimum_required(VERSION 3.10)
project(ScpiParserTests)

# GoogleTest einbinden
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
add_subdirectory(../external/googletest googletest-build)

set(SOURCES
    scpi_parser_tests.cpp
    ../Core/Src/scpi_parser.cpp
    ../Core/Src/command.cpp
)

add_executable(scpi_parser_tests ${SOURCES})

target_include_directories(scpi_parser_tests PRIVATE
    ../Core/Inc
)

target_link_libraries(scpi_parser_tests gtest gtest_main)

enable_testing()
add_test(NAME ScpiParserTests COMMAND scpi_parser_tests)
