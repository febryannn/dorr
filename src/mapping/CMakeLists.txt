cmake_minimum_required(VERSION 3.8)
project(mapping)

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
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(PCL REQUIRED)

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/input_pos.srv"
)

# simple_mapping executable
add_executable(simple_mapping src/simple_mapping.cpp)
target_include_directories(simple_mapping PUBLIC
  ${PCL_INCLUDE_DIRS}
)
target_link_directories(simple_mapping PUBLIC
  ${PCL_LIBRARY_DIRS}
)
target_link_libraries(simple_mapping
  ${PCL_LIBRARIES}
)
ament_target_dependencies(simple_mapping
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(simple_mapping "${cpp_typesupport_target}")

# manual_mapping executable
add_executable(manual_mapping src/manual_mapping.cpp)
target_include_directories(manual_mapping PUBLIC
  ${PCL_INCLUDE_DIRS}
)
target_link_directories(manual_mapping PUBLIC
  ${PCL_LIBRARY_DIRS}
)
target_link_libraries(manual_mapping
  ${PCL_LIBRARIES}
)
ament_target_dependencies(manual_mapping
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
)
target_link_libraries(manual_mapping "${cpp_typesupport_target}")

# Install targets
install(TARGETS
  simple_mapping
  manual_mapping
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
