from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    pkg_wbp_description = get_package_share_directory('wbp_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_wbp_sim_gazebo = get_package_share_directory('wbp_sim_gazebo')

    ARGUMENTS = [
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz'),
        DeclareLaunchArgument('rviz_config', default_value='rviz.rviz', description='RViz config file'),
        DeclareLaunchArgument('model', default_value='wbp_printer_arm.urdf', description='URDF file name'),
        DeclareLaunchArgument('use_sim_time', default_value='True', description='Enable sim time'),
    ]

    urdf_file_path = PathJoinSubstitution([pkg_wbp_description, "urdf", LaunchConfiguration('model')])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(Command(['cat ', urdf_file_path]), value_type=str),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
    )

    # Gazebo launch
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': '-r empty.sdf', 'on_exit_shutdown': 'true'}.items()
    )

    # Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'wbp_printer_arm', '-topic', '/robot_description', '-z', '0.2'],
        output='screen'
    )

    # ROS ↔ Gazebo bridge – corrected directions: '[' means GZ -> ROS for odom and tf
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",           # bidirectional
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",            # GZ → ROS (note '[')
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",               # GZ → ROS (note '[')
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",   # GZ → ROS
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
        ],
        output="screen"
    )

    # Controller YAML path
    controller_yaml = os.path.join(pkg_wbp_sim_gazebo, 'config', 'robot_controllers.yaml')

    # Spawners connect to the controller_manager that runs inside Gazebo (no extra ros2_control_node!)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_wbp_description, 'rviz', LaunchConfiguration('rviz_config')])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gz_sim)
    ld.add_action(spawn_robot)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(ros_gz_bridge)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(arm_controller_spawner)
    ld.add_action(rviz_node)
    return ld