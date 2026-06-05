cmake_minimum_required(VERSION 3.8)
project(wbp_sim_motion)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

find_package(wbp_sim_interfaces REQUIRED)
find_package(wbp_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(sim_motion_controller
  src/sim_motion_controller.cpp
)

ament_target_dependencies(sim_motion_controller
  rclcpp
  wbp_sim_interfaces
  geometry_msgs
  wbp_interfaces
)

add_executable(execution_tf_publisher
  src/execution_tf_publisher.cpp
)
ament_target_dependencies(execution_tf_publisher
  rclcpp
  tf2_ros
  geometry_msgs
  wbp_interfaces
)

add_executable(execution_joint_publisher
  src/execution_joint_publisher.cpp
  )
ament_target_dependencies(execution_joint_publisher
  rclcpp
  sensor_msgs
  trajectory_msgs
  wbp_interfaces
)

add_executable(actuator_bridge
  src/actuator_bridge.cpp
)
ament_target_dependencies(actuator_bridge
  rclcpp
  std_msgs
  wbp_interfaces
)

install(TARGETS
  sim_motion_controller
  execution_tf_publisher
  execution_joint_publisher
  actuator_bridge
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(motion_result_adapter
  src/motion_result_adapter.cpp
)

ament_target_dependencies(
  motion_result_adapter
  rclcpp
  std_msgs
  wbp_interfaces
  wbp_sim_interfaces
)

install(
  TARGETS motion_result_adapter
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
