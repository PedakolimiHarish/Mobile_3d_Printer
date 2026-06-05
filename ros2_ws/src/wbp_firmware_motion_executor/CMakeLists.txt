cmake_minimum_required(VERSION 3.8)
project(wbp_firmware_motion_executor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(wbp_interfaces REQUIRED)

add_executable(motion_executor
  src/motion_executor.cpp
  src/main.cpp
)

target_include_directories(motion_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(motion_executor
  rclcpp
  sensor_msgs
  wbp_interfaces
)

install(TARGETS
  motion_executor
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(motion_monitor 
src/motion_monitor_main.cpp
src/motion_monitor.cpp)

target_include_directories(motion_monitor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  
ament_target_dependencies(motion_monitor 
rclcpp 
sensor_msgs 
wbp_interfaces
)

install(TARGETS
  motion_monitor
  DESTINATION lib/${PROJECT_NAME})


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
