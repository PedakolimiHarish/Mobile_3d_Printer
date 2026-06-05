# start of file ros2_ws/src/wbp_sim_gazebo/launch/gazebo_with_robot.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    controllers_file = PathJoinSubstitution([
        FindPackageShare("wbp_sim_gazebo"),
        "config",
        "robot_controllers.yaml"
    ])

    gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('wbp_sim_gazebo'),
                'launch',
                'gazebo_world.launch.py'
            ])
        )
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('wbp_sim_gazebo'),
                'launch',
                'spawn_robot.launch.py'
            ])
        )
    )

    return LaunchDescription([
        gazebo_world,
        spawn_robot
    ])


# end of file ros2_ws/src/wbp_sim_gazebo/launch/gazebo_with_robot.launch.py