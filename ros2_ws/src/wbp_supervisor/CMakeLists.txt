cmake_minimum_required(VERSION 3.8)
project(wbp_supervisor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(wbp_interfaces REQUIRED)

find_package(std_srvs REQUIRED)
add_executable(supervisor_node
  src/supervisor_node.cpp
  src/ipc_job_writer.cpp
)


target_sources(supervisor_node
  PRIVATE
    src/supervisor_node.cpp
    src/ipc_reader.cpp
    src/ipc_command_writer.cpp
    src/command_policy.cpp
)

target_include_directories(supervisor_node
  PRIVATE
    include
    ${wbp_interfaces_INCLUDE_DIRS}
)

ament_target_dependencies(supervisor_node
  rclcpp
  wbp_interfaces
  std_srvs
)

install(
  TARGETS supervisor_node
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
