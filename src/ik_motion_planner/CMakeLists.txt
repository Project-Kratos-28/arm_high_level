cmake_minimum_required(VERSION 3.8)
project(ik_motion_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)

add_executable(ik_solver_node src/ik_solver_node.cpp)
ament_target_dependencies(ik_solver_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  moveit_core
  moveit_ros_planning
)

add_executable(trac_ik_benchmark src/trac_ik_benchmark.cpp)
ament_target_dependencies(trac_ik_benchmark
  rclcpp
  geometry_msgs
  moveit_core
  moveit_ros_planning
)

install(TARGETS
  ik_solver_node
  trac_ik_benchmark
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
