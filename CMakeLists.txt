cmake_minimum_required(VERSION 3.8)
project(pilot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# === Try to find serial (optional) ===
find_package(serial QUIET)

# === CLIENT TELEOP NODE (only if serial is found) ===
if(serial_FOUND)
  add_executable(client_teleop src/client_teleop_serial.cpp)
  target_include_directories(client_teleop PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_compile_features(client_teleop PUBLIC c_std_99 cxx_std_17)
  ament_target_dependencies(client_teleop rclcpp geometry_msgs serial)

  install(TARGETS client_teleop DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING "serial not found — skipping client_teleop_serial.cpp")
endif()

# === HOST TELEOP KEYBOARD NODE (always built) ===
add_executable(host_teleop src/host_teleop.cpp)
target_include_directories(host_teleop PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(host_teleop PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(host_teleop rclcpp geometry_msgs)

install(TARGETS host_teleop DESTINATION lib/${PROJECT_NAME})

# === Lint/Tests ===
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
