cmake_minimum_required(VERSION 3.8)
project(kelo_bt_nodes)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav_msgs REQUIRED)

add_library(kelo_is_person_blocking_bt_node SHARED src/is_person_blocking.cpp)
target_compile_definitions(kelo_is_person_blocking_bt_node PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(kelo_is_person_blocking_bt_node rclcpp std_msgs behaviortree_cpp nav2_behavior_tree)

add_library(kelo_skip_path_start_bt_node SHARED src/skip_path_start.cpp)
target_compile_definitions(kelo_skip_path_start_bt_node PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(kelo_skip_path_start_bt_node behaviortree_cpp nav_msgs nav2_behavior_tree)

install(TARGETS kelo_is_person_blocking_bt_node kelo_skip_path_start_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
