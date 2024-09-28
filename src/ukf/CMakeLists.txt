cmake_minimum_required(VERSION 3.0)

project(UKF)

find_package(Eigen3 REQUIRED)

add_executable(${PROJECT_NAME} unscented_kalman_filter.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC cnpy Eigen3::Eigen matplot)