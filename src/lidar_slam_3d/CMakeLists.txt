cmake_minimum_required(VERSION 3.8)
project(lidar_slam_3d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE "Release")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(std_srvs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(G2O REQUIRED)
find_package(Cholmod REQUIRED)
find_package(PCL REQUIRED)

set(G2O_LIBS
  g2o_core g2o_stuff g2o_types_sba g2o_types_slam3d
)

set(ROS_DEPS
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  laser_geometry
  pcl_conversions
  std_srvs
)

# lidar_slam_3d executable
add_executable(lidar_slam_3d
  src/floor_filter.cpp
  src/map_builder.cpp
  src/lidar_slam_3d_ros.cpp
  src/node.cpp
)

target_include_directories(lidar_slam_3d PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/lidar_slam_3d>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIR}
  ${CHOLMOD_INCLUDE_DIR}
)

ament_target_dependencies(lidar_slam_3d ${ROS_DEPS})

target_link_libraries(lidar_slam_3d
  ${PCL_LIBRARIES}
  ${CHOLMOD_LIBRARIES}
  ${G2O_LIBS}
)

install(TARGETS
  lidar_slam_3d
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
