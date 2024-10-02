cmake_minimum_required(VERSION 3.0)

project(UKF)

find_package(Eigen3 REQUIRED)

add_executable(${PROJECT_NAME} sensor_fusion.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC cnpy Eigen3::Eigen matplot)