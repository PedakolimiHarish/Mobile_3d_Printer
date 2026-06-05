# start of file: ros2_ws/src/wbp_bringup/launch/wbp_full_stack.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # Supervisor Node
        Node(
            package='wbp_supervisor',
            executable='supervisor_node',
            name='supervisor',
            output='screen'
        ),

        # Firmware Bridge
        Node(
            package='wbp_firmware_bridge',
            executable='firmware_supervisor_node',
            name='firmware_bridge',
            output='screen'
        ),

        # UI Node
        Node(
            package='wbp_ui',
            executable='wbp_ui',
            name='wbp_ui',
            output='screen'
        ),
        
        Node(
            package='wbp_program_manager',
            executable='program_manager',
            name='program_manager',
            output='screen'
        ),
    ])

# end of file: ros2_ws/src/wbp_bringup/launch/wbp_full_stack.launch.py