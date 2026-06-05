cmake_minimum_required(VERSION 3.10)
project(wbp_firmware LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(ENABLE_ROS_SIM "Enable ROS-SIM motion backend" ON)

find_package(wbp_interfaces REQUIRED)

if(ENABLE_ROS_SIM)
  find_package(rclcpp REQUIRED)
  find_package(wbp_sim_interfaces REQUIRED)
endif()


add_executable(wbp_firmware
    src/main.cpp
    src/state_machine.cpp
    src/command_executor.cpp
    src/safe_travel_rules.cpp
    src/command_channel.cpp      
    src/ipc/state_publisher.cpp
    src/execution_loop.cpp
    src/execution_orchestrator.cpp
    src/persistence/persistent_state_io.cpp
    src/motion/motion_controller_stub.cpp
    src/safety/safety_monitor.cpp
    src/extruder/extruder_controller_stub.cpp
    src/motion/ros_sim_motion_controller.cpp
    src/job_ingress.cpp
    src/job_decoder.cpp
    src/job_validator.cpp
    src/job_id_allocator.cpp
    src/job_hash.cpp
    src/job_store.cpp
    src/ipc/job_ingress_ipc.cpp
    src/job_ingress_state.cpp
    src/execution_gate.cpp
    src/job_cleanup.cpp
    src/invariants.cpp
    src/anchor_state.cpp
    src/segment_runner.cpp
    src/printed_volume.cpp
    src/persistence/persistent_execution_snapshot.cpp
    
  )

if(ENABLE_ROS_SIM)
  target_link_libraries(wbp_firmware
    rclcpp::rclcpp
    #wbp_interfaces::wbp_interfaces
    #wbp_sim_interfaces::wbp_sim_interfaces
  )

  target_include_directories(wbp_firmware 
  PRIVATE
    ${rclcpp_INCLUDE_DIRS}
    ${wbp_interfaces_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${wbp_sim_interfaces_INCLUDE_DIRS}
  PUBLIC 
    include
  )

  ament_target_dependencies(wbp_firmware
    rclcpp
    wbp_sim_interfaces
    wbp_interfaces

  )
endif()

target_compile_options(wbp_firmware PRIVATE
    -Wall -Wextra -Wpedantic
)
