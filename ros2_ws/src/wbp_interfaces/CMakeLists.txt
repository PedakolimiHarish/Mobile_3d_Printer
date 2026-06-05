cmake_minimum_required(VERSION 3.8)
project(wbp_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(wbp_interfaces_headers INTERFACE)

target_include_directories(wbp_interfaces_headers INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/MachineState.msg
  msg/Anchor.msg
  msg/AnchorArray.msg
  msg/MotionSegment.msg
  msg/SegmentStatus.msg
  msg/ActuatorCommand.msg
  msg/MotionRuntimeState.msg
  msg/BaseState.msg
  msg/BaseVelocityCommand.msg
  msg/BaseMotionCommand.msg
  msg/BaseMotionResult.msg
  msg/AnchorCorrection.msg
  msg/ToolCompensation.msg
  
  srv/LifecycleCommand.srv
  srv/JobClear.srv
  srv/JobSubmit.srv
  srv/StartJob.srv

    DEPENDENCIES
    builtin_interfaces
    geometry_msgs

)

install(
  TARGETS wbp_interfaces_headers
  EXPORT wbp_interfaces_headers
)

ament_export_targets(wbp_interfaces_headers)
ament_export_dependencies(rosidl_default_runtime)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
