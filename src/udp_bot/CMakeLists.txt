cmake_minimum_required(VERSION 3.8)
project(udp_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/kirim_kecepatan_udp.msg"
  "msg/kirim_offset_udp.msg"
  "msg/terima_udp.msg"
)

# Build executable
add_executable(udp_bot src/udp_bot.cpp)
target_include_directories(udp_bot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(udp_bot
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

# Link generated message typesupport to the executable
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(udp_bot "${cpp_typesupport_target}")

# Install executable
install(TARGETS udp_bot
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install Python scripts
install(PROGRAMS
  scripts/teleop_robot.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
