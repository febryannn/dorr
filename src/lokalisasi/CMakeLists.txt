cmake_minimum_required(VERSION 3.8)
project(lokalisasi)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(udp_bot REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

# TODO: marvelmind_nav is not available in ROS2 yet.
# Uncomment when a ROS2 port becomes available.
# find_package(marvelmind_nav REQUIRED)

set(COMMON_DEPS
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  udp_bot
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# --- fusion_gps_odom ---
# TODO: fusion_gps_odom depends on marvelmind_nav which is not available in ROS2.
# Uncomment the following when marvelmind_nav is ported to ROS2.
# add_executable(fusion_gps_odom src/fusion_gps_odom.cpp)
# ament_target_dependencies(fusion_gps_odom ${COMMON_DEPS} marvelmind_nav)

# --- odom ---
add_executable(odom src/odom.cpp)
ament_target_dependencies(odom ${COMMON_DEPS})

# --- rgbd_cam_loc ---
add_executable(rgbd_cam_loc src/rgbd_cam_loc.cpp)
ament_target_dependencies(rgbd_cam_loc ${COMMON_DEPS})
target_link_libraries(rgbd_cam_loc ${PCL_LIBRARIES})

# --- slam_rgbd_cam ---
add_executable(slam_rgbd_cam src/slam_rgbd_cam.cpp)
ament_target_dependencies(slam_rgbd_cam ${COMMON_DEPS})
target_link_libraries(slam_rgbd_cam ${PCL_LIBRARIES})

# Install executables
install(TARGETS
  odom
  rgbd_cam_loc
  slam_rgbd_cam
  # fusion_gps_odom  # TODO: uncomment when marvelmind_nav is available
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
