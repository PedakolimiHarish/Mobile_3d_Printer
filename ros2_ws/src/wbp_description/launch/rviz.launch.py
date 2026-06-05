import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    pkg_wbp_description = get_package_share_directory('wbp_description')

    ARGUMENTS = [
    DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz'
    ),

    DeclareLaunchArgument(
        'rviz_config', default_value='rviz.rviz',
        description='RViz config file'
    ),

   DeclareLaunchArgument(
        'model', default_value='wbp_printer_arm.urdf',
        description='Name of the URDF description to load'
    ),

    DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Flag to enable use_sim_time'
    ),
    ]
    
    # Define the path to your URDF or Xacro file
    urdf_file_path = PathJoinSubstitution([
        pkg_wbp_description,  # Replace with your package name
        "urdf",
        LaunchConfiguration('model')  # Replace with your URDF or Xacro file
    ])

    # Launch rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_wbp_description, 'rviz', LaunchConfiguration('rviz_config')])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )
    
    

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': ParameterValue(
            Command(['cat ', urdf_file_path]),
            value_type=str),
             'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )
    
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"
        ],
        output="screen"
    )
    
    #launchDescriptionObject = LaunchDescription()
    launchDescriptionObject = LaunchDescription(ARGUMENTS)
    launchDescriptionObject.add_action(rviz_node)
    launchDescriptionObject.add_action(joint_state_publisher_node)
    launchDescriptionObject.add_action(robot_state_publisher_node)
    launchDescriptionObject.add_action(ros_gz_bridge)
    return launchDescriptionObject