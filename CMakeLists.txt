cmake_minimum_required(VERSION 3.8)
project(pilot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(odrive_can REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

# Find SDL2 for teleop
find_package(SDL2 QUIET)

# === DIFFERENTIAL DRIVE CONTROLLER ===
add_executable(diff_drive_controller src/diff_drive_controller.cpp)
ament_target_dependencies(diff_drive_controller rclcpp geometry_msgs nav_msgs tf2_ros odrive_can)
install(TARGETS diff_drive_controller DESTINATION lib/${PROJECT_NAME})

# === PCD PROCESSOR (laptop-side map processing) ===
add_executable(pcd_processor src/pcd_processor.cpp)
ament_target_dependencies(pcd_processor rclcpp sensor_msgs std_srvs pcl_conversions)
target_include_directories(pcd_processor PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(pcd_processor ${PCL_LIBRARIES})
add_definitions(${PCL_DEFINITIONS})
install(TARGETS pcd_processor DESTINATION lib/${PROJECT_NAME})

# === RAW MAP SAVER (robot-side raw map saving) ===
add_executable(raw_map_saver src/raw_map_saver.cpp)
ament_target_dependencies(raw_map_saver rclcpp sensor_msgs std_srvs pcl_conversions)
target_include_directories(raw_map_saver PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(raw_map_saver ${PCL_LIBRARIES})
add_definitions(${PCL_DEFINITIONS})
install(TARGETS raw_map_saver DESTINATION lib/${PROJECT_NAME})

# === LASER MAP ROTATOR ===
add_executable(laser_map_rotator src/laser_map_rotator.cpp)
ament_target_dependencies(laser_map_rotator rclcpp sensor_msgs pcl_conversions tf2 tf2_geometry_msgs tf2_sensor_msgs)
target_include_directories(laser_map_rotator PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(laser_map_rotator ${PCL_LIBRARIES})
install(TARGETS laser_map_rotator DESTINATION lib/${PROJECT_NAME})

# === GPR MOTION FOLLOWER (mirrors forward distance to ODrive micro) ===
add_executable(gpr_motion_follower src/gpr_motion_follower.cpp)
ament_target_dependencies(gpr_motion_follower rclcpp geometry_msgs nav_msgs odrive_can)
install(TARGETS gpr_motion_follower DESTINATION lib/${PROJECT_NAME})


# === SHUTDOWN SERVICE ===
add_executable(shutdown_service src/shutdown_service.cpp)
ament_target_dependencies(shutdown_service rclcpp std_srvs)
install(TARGETS shutdown_service DESTINATION lib/${PROJECT_NAME})

# === HOST TELEOP (SDL2-based) ===
if(SDL2_FOUND)
  add_executable(host_teleop src/host_teleop.cpp)
  target_include_directories(host_teleop PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_compile_features(host_teleop PUBLIC c_std_99 cxx_std_17)
  ament_target_dependencies(host_teleop rclcpp geometry_msgs std_msgs odrive_can std_srvs)
  target_link_libraries(host_teleop SDL2)

  install(TARGETS host_teleop DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING "SDL2 not found — cannot build host_teleop (SDL2 version)")
endif()

# Install launch and config files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config directory so YAMLs are available at runtime
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(
  PROGRAMS scripts/gpr_serial_bridge.py
  scripts/path_to_foot.py
  DESTINATION lib/${PROJECT_NAME}
)

# === Lint/Tests ===
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
