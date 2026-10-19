cmake_minimum_required(VERSION 3.8)
project(robot_simulation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch maps meshes models rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
