from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():

    return LaunchDescription([

        # =========================================
        # Base Motion
        # =========================================

        Node(
            package="wbp_base_motion",
            executable="base_motion_controller",
            output="screen"
        ),

        # =========================================
        # Localization
        # =========================================

        Node(
            package="wbp_localization",
            executable="anchor_localizer",
            output="screen"
        ),

        # =========================================
        # World Pose Coordinator
        # =========================================

        Node(
            package="wbp_coordination",
            executable="world_pose_coordinator",
            output="screen"
        ),

        # =========================================
        # World Motion Coordinator
        # =========================================

        Node(
            package="wbp_mobile_fabrication",
            executable="world_motion_coordinator",
            output="screen"
        ),

        # =========================================
        # Compensation Controller
        # =========================================

        Node(
            package="wbp_compensation",
            executable="world_compensation_controller",
            output="screen"
        ),
        
        Node(
            package="wbp_tool_control",
            executable="tool_compensation_applier",
            output="screen"
        ),

        Node(
            package="wbp_tool_motion",
            executable="tool_target_controller",
            output="screen"
        ),
        
        Node(
            package="wbp_tool_control",
            executable="tool_servo_simulator",
            output="screen"
        ),
        
        Node(
            package="wbp_tool_control",
            executable="tool_tracking_monitor",
            output="screen"
        ),
        
        Node(
            package="wbp_tool_motion",
            executable="tool_disturbance_injector",
            output="screen"
        ),
    ])