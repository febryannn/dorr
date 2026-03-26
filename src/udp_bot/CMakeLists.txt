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
find_package(udp_bot_msgs REQUIRED)

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
  udp_bot_msgs
)

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
