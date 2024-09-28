cmake_minimum_required(VERSION 3.2)

project(altitude_estimator)

message("altitude estimator list directory is ${CMAKE_CURRENT_LIST_DIR}")

set(CNPY_PATH "${CMAKE_CURRENT_SOURCE_DIR}/src/cnpy")

include_directories(${CNPY_PATH})

add_subdirectory(src/cnpy)
add_subdirectory(src/ukf)

target_compile_definitions(UKF PRIVATE DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data/")

