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
find_package(Eigen3 REQUIRED)

# GStreamer for video_streamer
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
  pkg_check_modules(GSTREAMER QUIET gstreamer-1.0)
  pkg_check_modules(GSTREAMER_APP QUIET gstreamer-app-1.0)
  pkg_check_modules(GLIB2 QUIET glib-2.0)
endif()

# Find SDL2 for teleop
find_package(SDL2 QUIET)

# OpenCV for video_recorder and unified_data_collector
find_package(OpenCV QUIET COMPONENTS core imgproc videoio imgcodecs highgui)

# ==================== JPEG XL Detection Function ====================
function(setup_jpegxl TARGET_NAME)
  # Try multiple possible locations for JPEG XL
  set(JXL_POSSIBLE_PATHS 
    "/home/${USER}/libjxl"
    "/home/roofus/libjxl" 
    "/home/avenblake/libjxl"
    "/usr/local/libjxl"
    "/opt/libjxl")
  
  set(JXL_ROOT_DIR "")
  foreach(PATH ${JXL_POSSIBLE_PATHS})
    if(EXISTS "${PATH}/build/lib/include/jxl/encode.h" AND EXISTS "${PATH}/build/lib/libjxl.so")
      set(JXL_ROOT_DIR "${PATH}")
      break()
    endif()
  endforeach()
  
  if(JXL_ROOT_DIR)
    set(JXL_INCLUDE_DIR "${JXL_ROOT_DIR}/build/lib/include")
    set(JXL_LIB_DIR "${JXL_ROOT_DIR}/build/lib")
    message(STATUS "Found JPEG XL at: ${JXL_ROOT_DIR}")
    
    target_include_directories(${TARGET_NAME} PUBLIC ${JXL_INCLUDE_DIR})
    target_link_libraries(${TARGET_NAME} 
      ${JXL_LIB_DIR}/libjxl.so
      ${JXL_LIB_DIR}/libjxl_threads.so
      ${JXL_LIB_DIR}/libjxl_cms.so)
    target_compile_definitions(${TARGET_NAME} PRIVATE HAVE_JPEGXL=1)
    message(STATUS "JPEG XL support enabled for ${TARGET_NAME}")
  else()
    message(WARNING "JPEG XL library not found — ${TARGET_NAME} will use fallback")
    target_compile_definitions(${TARGET_NAME} PRIVATE HAVE_JPEGXL=0)
  endif()
endfunction()

# === DIFFERENTIAL DRIVE CONTROLLER ===
add_executable(diff_drive_controller src/diff_drive_controller.cpp)
ament_target_dependencies(diff_drive_controller rclcpp geometry_msgs nav_msgs tf2_ros odrive_can sensor_msgs std_msgs)
target_link_libraries(diff_drive_controller Eigen3::Eigen)
install(TARGETS diff_drive_controller DESTINATION lib/${PROJECT_NAME})

# === POSE CONTROLLER ===
add_executable(pose_controller src/pose_controller.cpp)
target_include_directories(pose_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pose_controller PUBLIC cxx_std_17)
ament_target_dependencies(pose_controller rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs std_srvs odrive_can)
target_link_libraries(pose_controller Eigen3::Eigen)
install(TARGETS pose_controller DESTINATION lib/${PROJECT_NAME})

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
target_link_libraries(raw_map_saver ${PCL_LIBRARIES} Eigen3::Eigen)
add_definitions(${PCL_DEFINITIONS})
install(TARGETS raw_map_saver DESTINATION lib/${PROJECT_NAME})

# === LASER MAP ROTATOR ===
add_executable(laser_map_rotator src/laser_map_rotator.cpp)
ament_target_dependencies(laser_map_rotator rclcpp sensor_msgs pcl_conversions tf2 tf2_geometry_msgs tf2_sensor_msgs)
target_include_directories(laser_map_rotator PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(laser_map_rotator ${PCL_LIBRARIES})
install(TARGETS laser_map_rotator DESTINATION lib/${PROJECT_NAME})

# === POINT CLOUD DOWNSAMPLER ===
add_executable(point_cloud_downsampler src/point_cloud_downsampler.cpp)
ament_target_dependencies(point_cloud_downsampler rclcpp sensor_msgs pcl_conversions)
target_include_directories(point_cloud_downsampler PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(point_cloud_downsampler ${PCL_LIBRARIES})
install(TARGETS point_cloud_downsampler DESTINATION lib/${PROJECT_NAME})

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

## Video recorder removed per user request
if(OpenCV_FOUND)
  add_executable(video_recorder src/video_recorder.cpp)
  target_compile_features(video_recorder PUBLIC cxx_std_17)
  ament_target_dependencies(video_recorder rclcpp std_srvs)
  target_include_directories(video_recorder PUBLIC ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(video_recorder ${OpenCV_LIBS})
  install(TARGETS video_recorder DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING "OpenCV not found — video_recorder will not be built")
endif()

# === VIDEO STREAMER (GStreamer tee: record + stream) ===
if(PkgConfig_FOUND AND GSTREAMER_FOUND AND GLIB2_FOUND)
  add_executable(video_streamer_gst src/video_streamer_gst.cpp)
  target_compile_features(video_streamer_gst PUBLIC cxx_std_17)
  ament_target_dependencies(video_streamer_gst rclcpp std_srvs)
  target_include_directories(video_streamer_gst PUBLIC ${GSTREAMER_INCLUDE_DIRS} ${GLIB2_INCLUDE_DIRS})
  target_link_libraries(video_streamer_gst ${GSTREAMER_LIBRARIES} ${GLIB2_LIBRARIES})
  install(TARGETS video_streamer_gst DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING "GStreamer not found — video_streamer_gst will not be built")
endif()

# === UNIFIED DATA COLLECTOR (Thermal + Dual Cameras + Odometry) ===
if(PkgConfig_FOUND AND GSTREAMER_FOUND AND GLIB2_FOUND AND OpenCV_FOUND)
  # Find JPEG XL library
  pkg_check_modules(LIBJXL QUIET libjxl)
  
  add_executable(unified_data_collector src/unified_data_collector.cpp)
  target_compile_features(unified_data_collector PUBLIC cxx_std_17)
  ament_target_dependencies(unified_data_collector rclcpp nav_msgs std_srvs)
  target_include_directories(unified_data_collector PUBLIC 
    ${GSTREAMER_INCLUDE_DIRS} ${GLIB2_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(unified_data_collector 
    ${GSTREAMER_LIBRARIES} ${GSTREAMER_APP_LIBRARIES} ${GLIB2_LIBRARIES} ${OpenCV_LIBS}
    seekcamera)
  
  # Add JPEG XL support using custom-built library
  setup_jpegxl(unified_data_collector)
  
  install(TARGETS unified_data_collector DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING "GStreamer, OpenCV, or SeekCamera not found — unified_data_collector will not be built")
endif()

# === SIMPLE CAMERA CAPTURE ===
if(OpenCV_FOUND)
  add_executable(simple_camera_capture src/simple_camera_capture.cpp)
  target_compile_features(simple_camera_capture PUBLIC cxx_std_17)
  target_include_directories(simple_camera_capture PUBLIC ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(simple_camera_capture ${OpenCV_LIBS})
  
  # Add JPEG XL support using custom-built library
  setup_jpegxl(simple_camera_capture)
  
  install(TARGETS simple_camera_capture DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING "OpenCV not found — simple_camera_capture will not be built")
endif()

# === JPEG XL TEST SIMPLE ===
if(OpenCV_FOUND)
  add_executable(jpegxl_test_simple src/jpegxl_test_simple.cpp)
  target_compile_features(jpegxl_test_simple PUBLIC cxx_std_17)
  target_include_directories(jpegxl_test_simple PUBLIC ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(jpegxl_test_simple ${OpenCV_LIBS})
  
  # Add JPEG XL support using custom-built library
  setup_jpegxl(jpegxl_test_simple)
  
  install(TARGETS jpegxl_test_simple DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING "OpenCV not found — jpegxl_test_simple will not be built")
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
  scripts/gpr_travel_distance.py
  scripts/drive_travel_distance.py
  scripts/left_wheel_distance.py
  scripts/bot_travel_distance.py
  scripts/test_gpr_position.py
  scripts/gpr_scan_controller.py
  scripts/pose_controller.py
  scripts/mpc_autonomous_controller.py
  scripts/mpc_accel_autonomous_controller.py
  scripts/odom_gap_monitor.py
  scripts/setup_data_folders.py
  scripts/F2C/f2c_check.py
  scripts/F2C/pointcloud_to_f2c_field.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install odom_tilt_corrector without .py suffix so it can be launched as a node name
install(
  PROGRAMS scripts/odom_tilt_corrector.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME odom_tilt_corrector
)

# === Lint/Tests ===
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
