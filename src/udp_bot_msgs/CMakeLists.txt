cmake_minimum_required(VERSION 3.8)
project(udp_bot_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/kirim_kecepatan_udp.msg"
  "msg/kirim_offset_udp.msg"
  "msg/terima_udp.msg"
)

ament_package()
