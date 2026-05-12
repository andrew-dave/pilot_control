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
find_package(diagnostic_msgs REQUIRED)

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
  set(JXL_INCLUDE_DIR "")
  set(JXL_LIBRARIES "")

  # Prefer a standard system-wide install so both robots use the same layout.
  find_path(JXL_SYSTEM_INCLUDE_DIR
    NAMES jxl/encode.h
    PATHS /usr/local/include /usr/include
    NO_DEFAULT_PATH
  )
  find_library(JXL_SYSTEM_LIBRARY
    NAMES jxl libjxl
    PATHS /usr/local/lib /usr/lib /usr/lib/x86_64-linux-gnu
    NO_DEFAULT_PATH
  )
  find_library(JXL_SYSTEM_THREADS_LIBRARY
    NAMES jxl_threads libjxl_threads
    PATHS /usr/local/lib /usr/lib /usr/lib/x86_64-linux-gnu
    NO_DEFAULT_PATH
  )
  find_library(JXL_SYSTEM_CMS_LIBRARY
    NAMES jxl_cms libjxl_cms
    PATHS /usr/local/lib /usr/lib /usr/lib/x86_64-linux-gnu
    NO_DEFAULT_PATH
  )

  if(JXL_SYSTEM_INCLUDE_DIR AND JXL_SYSTEM_LIBRARY AND JXL_SYSTEM_THREADS_LIBRARY AND JXL_SYSTEM_CMS_LIBRARY)
    set(JXL_INCLUDE_DIR "${JXL_SYSTEM_INCLUDE_DIR}")
    set(JXL_LIBRARIES
      "${JXL_SYSTEM_LIBRARY}"
      "${JXL_SYSTEM_THREADS_LIBRARY}"
      "${JXL_SYSTEM_CMS_LIBRARY}")
    message(STATUS "Found JPEG XL via system install: ${JXL_SYSTEM_LIBRARY}")
  else()
    # Backward-compatible fallback for existing local source builds.
    set(JXL_POSSIBLE_PATHS
      "/home/${USER}/libjxl"
      "/home/roofus/libjxl"
      "/home/avenblake/libjxl"
      "/usr/local/libjxl"
      "/opt/libjxl")

    foreach(PATH ${JXL_POSSIBLE_PATHS})
      set(CANDIDATE_INCLUDE_DIR "${PATH}/build/lib/include")
      set(CANDIDATE_LIB_DIR "${PATH}/build/lib")

      if(EXISTS "${CANDIDATE_INCLUDE_DIR}/jxl/encode.h"
        AND EXISTS "${CANDIDATE_LIB_DIR}/libjxl.so"
        AND EXISTS "${CANDIDATE_LIB_DIR}/libjxl_threads.so"
        AND EXISTS "${CANDIDATE_LIB_DIR}/libjxl_cms.so")
        set(JXL_INCLUDE_DIR "${CANDIDATE_INCLUDE_DIR}")
        set(JXL_LIBRARIES
          "${CANDIDATE_LIB_DIR}/libjxl.so"
          "${CANDIDATE_LIB_DIR}/libjxl_threads.so"
          "${CANDIDATE_LIB_DIR}/libjxl_cms.so")
        message(STATUS "Found JPEG XL via local source build: ${PATH}")
        break()
      endif()
    endforeach()
  endif()

  if(JXL_INCLUDE_DIR AND JXL_LIBRARIES)
    target_include_directories(${TARGET_NAME} PUBLIC ${JXL_INCLUDE_DIR})
    target_link_libraries(${TARGET_NAME} ${JXL_LIBRARIES})
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

# === LOCAL NAV GRID PUBLISHER (robot-side compact exploration map) ===
add_executable(local_nav_grid_publisher src/local_nav_grid_publisher.cpp)
ament_target_dependencies(local_nav_grid_publisher rclcpp nav_msgs sensor_msgs std_msgs)
target_link_libraries(local_nav_grid_publisher Eigen3::Eigen)
install(TARGETS local_nav_grid_publisher DESTINATION lib/${PROJECT_NAME})

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

# === GPS DRIVER (u-blox ZED-F9P) ===
add_executable(gps_driver src/gps_driver.cpp)
target_compile_features(gps_driver PUBLIC cxx_std_17)
ament_target_dependencies(gps_driver rclcpp sensor_msgs geometry_msgs diagnostic_msgs std_msgs std_srvs)
install(TARGETS gps_driver DESTINATION lib/${PROJECT_NAME})

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
  add_executable(unified_data_collector src/unified_data_collector.cpp)
  target_compile_features(unified_data_collector PUBLIC cxx_std_17)
  ament_target_dependencies(unified_data_collector rclcpp nav_msgs std_srvs sensor_msgs std_msgs)
  target_include_directories(unified_data_collector PUBLIC 
    ${GSTREAMER_INCLUDE_DIRS} ${GLIB2_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(unified_data_collector 
    ${GSTREAMER_LIBRARIES} ${GSTREAMER_APP_LIBRARIES} ${GLIB2_LIBRARIES} ${OpenCV_LIBS}
    seekcamera)
  
  # Add JPEG XL support when available.
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
  
  # Add JPEG XL support when available.
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
  
  # Add JPEG XL support when available.
  setup_jpegxl(jpegxl_test_simple)
  
  install(TARGETS jpegxl_test_simple DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING "OpenCV not found — jpegxl_test_simple will not be built")
endif()

# === THERMAL CHECK (headless Seek SDK frame check for preflight) ===
if(OpenCV_FOUND)
  # Check if seekcamera header exists
  find_path(SEEKCAMERA_INCLUDE_DIR seekcamera/seekcamera.h
    PATHS /usr/include /usr/local/include)
  find_library(SEEKCAMERA_LIB seekcamera)
  if(SEEKCAMERA_INCLUDE_DIR AND SEEKCAMERA_LIB)
    add_executable(thermal_check src/thermal_check.cpp)
    target_compile_features(thermal_check PUBLIC cxx_std_17)
    target_include_directories(thermal_check PUBLIC
      ${OpenCV_INCLUDE_DIRS} ${SEEKCAMERA_INCLUDE_DIR})
    target_link_libraries(thermal_check ${OpenCV_LIBS} ${SEEKCAMERA_LIB})
    install(TARGETS thermal_check DESTINATION lib/${PROJECT_NAME})
    message(STATUS "thermal_check will be built (SeekCamera + OpenCV found)")
  else()
    message(WARNING "SeekCamera SDK not found — thermal_check will not be built")
  endif()
else()
  message(WARNING "OpenCV not found — thermal_check will not be built")
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

# Install udev rules so the operator can run install_seek_udev_rule.sh
# to grant `roofus` write access to per-device sysfs `authorized` (used
# by seek_usb_reset.py to programmatically replug the Seek thermal
# camera when the SDK wedges).  See udev/99-bdr-seek-thermal.rules.
install(
  DIRECTORY udev
  DESTINATION share/${PROJECT_NAME}
)

# Install coverage planner robot registry for packaged/runtime lookup.
install(
  FILES scripts/F2C/cpp/config/robots.json
  DESTINATION share/${PROJECT_NAME}/config
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
  scripts/data_collection_coordinator.py
  scripts/udc_supervisor.py
  scripts/seek_usb_reset.py
  scripts/install_seek_udev_rule.sh
  scripts/finalize_mission_local.py
  scripts/F2C/f2c_check.py
  scripts/F2C/pointcloud_to_f2c_field.py
  scripts/measure_actuation_delay.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install startup_preflight script
install(
  PROGRAMS scripts/startup_preflight.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME startup_preflight
)

# Install odom_tilt_corrector without .py suffix so it can be launched as a node name
install(
  PROGRAMS scripts/odom_tilt_corrector.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME odom_tilt_corrector
)

# Install tilt_calibration script
install(
  PROGRAMS scripts/tilt_calibration.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME tilt_calibration
)

# Install robot auth CLI (used by Coverage Planner over SSH)
install(
  PROGRAMS scripts/robot_auth_cli.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pilot_control_auth
)

# Install robot mission CLI (robot-enforced mission start)
install(
  PROGRAMS scripts/robot_mission_cli.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pilot_control_mission
)

# === Lint/Tests ===
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
