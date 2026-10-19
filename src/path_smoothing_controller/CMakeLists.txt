cmake_minimum_required(VERSION 3.8)
project(path_smoothing_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Trajectory.msg"
  "msg/TrajectoryPoint.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

add_library(path_processing src/path_processing.cpp)
set_target_properties(path_processing PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(path_processing PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_options(path_processing PRIVATE -Wall -Wextra -Wpedantic)

add_executable(trajectory_controller src/trajectory_controller_node.cpp)
target_include_directories(trajectory_controller PRIVATE include)
ament_target_dependencies(trajectory_controller
  builtin_interfaces geometry_msgs nav2_msgs nav_msgs rclcpp rclcpp_action
  std_msgs tf2 tf2_geometry_msgs tf2_ros visualization_msgs
)
target_link_libraries(trajectory_controller path_processing)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(trajectory_controller "${cpp_typesupport_target}")

add_library(trajectory_controller_plugin SHARED src/nav2_trajectory_controller.cpp)
target_include_directories(trajectory_controller_plugin PUBLIC include)
ament_target_dependencies(trajectory_controller_plugin
  builtin_interfaces geometry_msgs nav2_core nav2_costmap_2d nav2_msgs nav_msgs
  pluginlib rclcpp rclcpp_lifecycle tf2 tf2_geometry_msgs tf2_ros
)
target_link_libraries(trajectory_controller_plugin path_processing "${cpp_typesupport_target}")
pluginlib_export_plugin_description_file(nav2_core path_smoothing_controller_plugins.xml)

install(TARGETS path_processing trajectory_controller trajectory_controller_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY msg DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/plot_trajectory_bag.py DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(path_processing_test test/path_processing_test.cpp)
  target_include_directories(path_processing_test PRIVATE include)
  target_link_libraries(path_processing_test path_processing)

  ament_add_gtest(nav2_plugin_load_test test/nav2_plugin_load_test.cpp)
  ament_target_dependencies(nav2_plugin_load_test nav2_core pluginlib)
endif()

ament_export_include_directories(include)
ament_export_dependencies(
  builtin_interfaces geometry_msgs nav2_core nav2_costmap_2d nav2_msgs nav_msgs
  pluginlib rclcpp rclcpp_lifecycle rosidl_default_runtime std_msgs tf2
  tf2_geometry_msgs tf2_ros
)
ament_package()
