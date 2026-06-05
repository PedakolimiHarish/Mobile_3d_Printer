cmake_minimum_required(VERSION 3.8)
project(wbp_ui)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(std_srvs REQUIRED)
find_package(wbp_interfaces REQUIRED)
find_package(wbp_sim_interfaces REQUIRED)

add_executable(wbp_ui
  src/main.cpp
  src/ui_node.cpp
  src/main_window.cpp
  src/ui_policy.cpp
  include/wbp_ui/main_window.hpp   # ← REQUIRED for MOC
)

target_include_directories(wbp_ui
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  wbp_ui
  rclcpp
  wbp_interfaces
  wbp_sim_interfaces
  std_srvs
)

target_link_libraries(wbp_ui
  Qt5::Widgets
)

install(TARGETS wbp_ui
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
