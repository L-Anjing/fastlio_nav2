cmake_minimum_required(VERSION 3.8)
project(navigation)

# colcon 默认不设置 CMAKE_BUILD_TYPE。空值既没有 -O 优化也没有 -DNDEBUG，Eigen 表达式
# 模板会退化成逐函数调用：实测同一段 MINCO 规划在这种配置下 6.3 s，-O3 下 0.06 s。
# 这里给默认值；显式传 -DCMAKE_BUILD_TYPE=Debug 仍然有效。
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    Debug Release RelWithDebInfo MinSizeRel)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_EXPORT_COMPILE_COMMANDS)
  set(CMAKE_EXPORT_COMPILE_COMMANDS ${CMAKE_EXPORT_COMPILE_COMMANDS})
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PlanMeta.msg"
  DEPENDENCIES std_msgs)

add_library(navigation_terrain_minco_planner SHARED
  src/terrain_minco_planner.cpp
  src/terrain_core/common/environment/nav_map.cpp
  src/terrain_core/common/trajectory/minco_trajectory.cpp
  src/terrain_core/path_planner/numerics/banded_system.cpp
  src/terrain_core/path_planner/numerics/lbfgs_minimizer.cpp
  src/terrain_core/path_planner/search/grid_utils.cpp
  src/terrain_core/path_planner/search/spatial_grid_astar.cpp
  src/terrain_core/path_planner/trajectory/minco_minjerk.cpp
  src/terrain_core/path_planner/trajectory/minco_optimizer.cpp
)
target_include_directories(navigation_terrain_minco_planner PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(navigation_terrain_minco_planner
  Eigen3::Eigen
  ${OpenCV_LIBS})
ament_target_dependencies(navigation_terrain_minco_planner
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2_ros)
rosidl_get_typesupport_target(navigation_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(navigation_terrain_minco_planner
  ${navigation_typesupport_target})

pluginlib_export_plugin_description_file(
  nav2_core terrain_minco_planner_plugin.xml)

install(TARGETS navigation_terrain_minco_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY LICENSES
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_clearance_oracle test/test_clearance_oracle.cpp)
  target_include_directories(test_clearance_oracle PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(test_clearance_oracle Eigen3::Eigen ${OpenCV_LIBS})
  ament_target_dependencies(test_clearance_oracle nav_msgs)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  params
  behavior_trees
  rviz
  launch)
