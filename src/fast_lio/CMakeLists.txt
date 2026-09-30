cmake_minimum_required(VERSION 3.16)
project(fast_lio)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-O3 -fexceptions)
add_compile_definitions(ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/")

message(STATUS "Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  include(ProcessorCount)
  ProcessorCount(PROCESSOR_COUNT)
  if(PROCESSOR_COUNT GREATER 4)
    add_compile_definitions(MP_EN MP_PROC_NUM=3)
  elseif(PROCESSOR_COUNT GREATER 3)
    add_compile_definitions(MP_EN MP_PROC_NUM=2)
  else()
    add_compile_definitions(MP_PROC_NUM=1)
  endif()
else()
  add_compile_definitions(MP_PROC_NUM=1)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Threads REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

add_executable(fastlio_mapping
  src/laserMapping.cpp
  src/preprocess.cpp
  include/ikd-Tree/ikd_Tree.cpp
)

target_include_directories(fastlio_mapping PRIVATE
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(fastlio_mapping
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  pcl_conversions
  livox_ros_driver2
  urdf
  std_msgs
)

target_link_libraries(fastlio_mapping
  ${PCL_LIBRARIES}
  OpenMP::OpenMP_CXX
  Threads::Threads
)

# 先验 PCD 地图 -> 2D 占据栅格
add_executable(pcd_to_occupancy_map src/pcd_to_occupancy_map.cpp)
target_include_directories(pcd_to_occupancy_map PRIVATE
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(pcd_to_occupancy_map
  rclcpp
  nav_msgs
  sensor_msgs
)
target_link_libraries(pcd_to_occupancy_map
  ${PCL_LIBRARIES}
)

# 基于 PCL ICP 的重定位，输出给 reloc 接口
add_executable(icp_relocalizer src/icp_relocalizer.cpp)
target_include_directories(icp_relocalizer PRIVATE
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(icp_relocalizer
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  pcl_conversions
)
target_link_libraries(icp_relocalizer
  ${PCL_LIBRARIES}
)

install(TARGETS fastlio_mapping pcd_to_occupancy_map icp_relocalizer
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY PCD DESTINATION share/${PROJECT_NAME})
install(FILES
  rviz_cfg/mapping_mid360.rviz
  rviz_cfg/reloc_mid360.rviz
  DESTINATION share/${PROJECT_NAME}/rviz_cfg)

ament_package()
