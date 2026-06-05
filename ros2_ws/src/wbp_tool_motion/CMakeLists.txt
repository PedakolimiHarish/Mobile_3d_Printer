cmake_minimum_required(VERSION 3.8)
project(wbp_tool_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)



add_executable(tool_target_controller
  src/tool_target_controller.cpp
)

ament_target_dependencies(
  tool_target_controller
  rclcpp
  geometry_msgs
)

install(
  TARGETS
    tool_target_controller
  DESTINATION lib/${PROJECT_NAME}
)

###############################################

add_executable(tool_disturbance_injector
    src/tool_disturbance_injector.cpp)

ament_target_dependencies(tool_disturbance_injector
    rclcpp
    nav_msgs
    geometry_msgs)

install(TARGETS
    tool_disturbance_injector
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
