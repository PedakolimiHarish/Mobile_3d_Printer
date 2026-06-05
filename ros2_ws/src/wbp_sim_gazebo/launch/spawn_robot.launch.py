# start of file ros2_ws/src/wbp_sim_gazebo/launch/spawn_robot.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'wbp_robot',
            '-topic', '/robot_description'
        ],
        output='screen'
    )

    """ joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    ) """

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robot_controller"],
        output="screen",
    )

    return LaunchDescription([
        spawn_robot,
        joint_state_broadcaster_spawner,
        robot_controller_spawner
    ])


# end of file ros2_ws/src/wbp_sim_gazebo/launch/spawn_robot.launch.py