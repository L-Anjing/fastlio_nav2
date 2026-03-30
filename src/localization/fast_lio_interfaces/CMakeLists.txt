cmake_minimum_required(VERSION 3.16)
project(fast_lio_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Pose6D.msg"
)

ament_export_dependencies(rosidl_default_runtime)

install(DIRECTORY
  msg/
  DESTINATION share/${PROJECT_NAME}/msg
)

ament_package()
