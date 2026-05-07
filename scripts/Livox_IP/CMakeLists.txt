cmake_minimum_required(VERSION 3.10)
project(Livox_IP)

find_package(livox_lidar_sdk_shared REQUIRED)

add_executable(Livox_IP set_mid360_ip.cpp)
target_link_libraries(Livox_IP livox_lidar_sdk_shared)
