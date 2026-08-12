to build firmware

cd ~/wbp_controller/firmware/build
cmake ..
make

to run firmware

./wbp_firmware


source ~/wbp_controller/ros2_ws/install/setup.bash
cd ~/wbp_controller/firmware/build
cmake ..
make

pkill -f ros2
pkill -f gazebo

source install/setup.bash

rm /var/lib/wbp/persistent_execution_snapshot.bin
source ~/wbp_controller/ros2_ws/install/setup.bash

./wbp_firmware
ros2 launch wbp_bringup wbp_full_stack.launch.py

ros2 run wbp_supervisor supervisor_node
ros2 run wbp_firmware_bridge firmware_supervisor_node


ros2 service call /firmware/job_submit wbp_interfaces/srv/JobSubmit "{ job_blob: [1], source: 'ros_test' }"

ros2 service call /firmware/job_submit wbp_interfaces/srv/JobSubmit "{ job_blob: [1], source: 'test' }"

ros2 service call /firmware/start_job wbp_interfaces/srv/StartJob "{}"
ros2 service call /firmware/pause std_srvs/srv/Trigger "{}"
ros2 service call /firmware/resume std_srvs/srv/Trigger "{}"
ros2 service call /firmware/abort std_srvs/srv/Trigger "{}"
ros2 service call /firmware/clear_job std_srvs/srv/Trigger "{}"


ros2 topic echo /machine_state --once
ros2 topic echo /firmware/state --once

ros2 run joint_state_publisher_gui joint_state_publisher_gui

ros2 launch wbp_sim_gazebo full_sim.launch.py

ros2 launch wbp_description rviz.launch.py
ros2 launch wbp_bringup wbp_full_stack.launch.py
ros2 launch wbp_sim_gazebo gazebo_with_robot.launch.py
ros2 run wbp_sim_motion execution_joint_publisher --ros-args -p layer_height:=0.2
./wbp_firmware
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock



ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock


ros2 topic pub --once /motion_segment wbp_interfaces/msg/MotionSegment "{
  segment_id: 1,
  joint_names: ['base_to_mast_joint','carriage_to_tool_joint','mast_to_carriage_joint','tool_mount_joint'],
  target_positions: [0.30, 0.30, 0.30, 0.30],
  max_velocity: 0.1,
  max_acceleration: 0.1,
  position_tolerance: 0.01
}"

ros2 run wbp_program_manager program_manager --ros-args -p yaml_file:=/home/guru/wbp_controller/ros2_ws/src/wbp_program_manager/config/motion_test.yaml



ros2 topic pub /forward_position_controller/commands \
std_msgs/msg/Float64MultiArray \
"{data: [0.0, 0.0, 0.0, 0.0]}"

ros2 run wbp_firmware_motion_executor motion_executor
ros2 run wbp_firmware_motion_executor motion_monitor
ros2 run wbp_sim_motion motion_result_adapter
ros2 run wbp_sim_motion sim_motion_controller

ros2 topic echo /actuator_command
ros2 topic echo /forward_position_controller/commands

ros2 topic pub /motion_reset std_msgs/msg/Empty "{}" --once
ros2 topic pub /machine_estop std_msgs/msg/Bool "data: true" --once
ros2 topic pub /motion_override std_msgs/msg/Float64 "data: 5" --once

ros2 service call /firmware/pause std_srvs/srv/Trigger --once


ros2 launch wbp_sim_gazebo full_sim.launch.py


ros2 topic pub /base/motion_command wbp_interfaces/msg/BaseMotionCommand "{sequence_id: 1, target_x: 1.0, target_y: 1.0, target_theta: 0.1}" --once


ros2 topic pub /base/motion_command \
wbp_interfaces/msg/BaseMotionCommand \
"{sequence_id: 5,
target_x: 0.0,
target_y: 0.0,
target_theta: 01.0,
linear_velocity: 0.10,
angular_velocity: 0.10}" --once

ros2 topic pub /base/motion_command \
wbp_interfaces/msg/BaseMotionCommand \
"{sequence_id: 2,
target_x: 0.0,
target_y: 0.0,
target_theta: 0.0,
linear_velocity: 0.10,
angular_velocity: 0.0}" --once


ros2 topic pub /base/motion_command \
wbp_interfaces/msg/BaseMotionCommand \
"{sequence_id: 2, target_x: 4.0, target_y: 0.0, target_theta: 0.0, linear_velocity: 0.5, angular_velocity: 0.0}" --once

ros2 topic pub /anchor_correction wbp_interfaces/msg/AnchorCorrection "{x: 2.0, y: 1.0, theta: 0.0, valid: true}" --once

ros2 run wbp_localization anchor_localizer


ros2 topic pub /world_tool_target \
geometry_msgs/msg/PoseStamped \
"{
header: {
  frame_id: 'map'
},
pose: {
  position: {
    x: 10.0,
    y: 0.0,
    z: 0.14
  },
  orientation: {
    x: 0.0,
    y: 0.0,
    z: 0.0,
    w: 1.0
  }
}}" --once
