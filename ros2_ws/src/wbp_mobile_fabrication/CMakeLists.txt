cmake_minimum_required(VERSION 3.8)
project(wbp_mobile_fabrication)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(wbp_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(wbp_sim_interfaces REQUIRED)

add_executable(world_motion_coordinator
  src/world_motion_coordinator.cpp
)

ament_target_dependencies(world_motion_coordinator
  rclcpp
  geometry_msgs
  nav_msgs
  wbp_interfaces
)

add_executable(
    world_trajectory_generator
    src/world_trajectory_generator.cpp
)

ament_target_dependencies(
    world_trajectory_generator
    rclcpp
    geometry_msgs
)

add_executable(
    fabrication_monitor
    src/fabrication_monitor.cpp
)

ament_target_dependencies(
  fabrication_monitor
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  wbp_interfaces
  wbp_sim_interfaces
)

add_executable(gcode_world_executor
    src/gcode_world_executor.cpp)

ament_target_dependencies(gcode_world_executor
    rclcpp
    geometry_msgs)

    add_executable(
  world_motion_controller
  src/world_motion_controller.cpp
)

ament_target_dependencies(
  world_motion_controller
  wbp_interfaces
  rclcpp
  geometry_msgs
  nav_msgs
  wbp_sim_interfaces
)

install(TARGETS
world_motion_controller
  world_motion_coordinator
  world_trajectory_generator
  fabrication_monitor
  gcode_world_executor
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
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

ament_package()
