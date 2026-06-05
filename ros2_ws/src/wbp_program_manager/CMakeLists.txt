cmake_minimum_required(VERSION 3.8)
project(wbp_program_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =========================
# Dependencies
# =========================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(wbp_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)

# =========================
# Executable
# =========================
add_executable(program_manager
  src/program_manager.cpp
)

ament_target_dependencies(program_manager
  rclcpp
  wbp_interfaces
)

target_link_libraries(program_manager
  yaml-cpp
)

# =========================
# Install
# =========================
install(TARGETS
  program_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# =========================
# Testing (Optional)
# =========================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
