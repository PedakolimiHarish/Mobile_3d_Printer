cmake_minimum_required(VERSION 3.8)
project(wbp_sim_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotionCommand.msg"
  "msg/MotionResult.msg"
  "msg/AlignmentStatus.msg"
  "msg/MotionStatus.msg"
  "msg/MotionTarget.msg"
)

ament_package()
