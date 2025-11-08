cmake_minimum_required(VERSION 3.18)

# create the project
project(pinalyzer)

set(CMAKE_BUILD_TYPE Release)

add_subdirectory("lib/argtable3")

file(GLOB SOURCES "src/*.c")

add_executable(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC lib)
target_link_libraries(${PROJECT_NAME} argtable3 m)
target_compile_options(${PROJECT_NAME} PUBLIC -Wall -Wextra -Wpedantic -Wdouble-promotion)

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 20)
