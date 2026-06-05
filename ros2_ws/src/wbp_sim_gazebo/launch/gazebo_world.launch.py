# start of file: ros2_ws/src/wbp_sim_gazebo/launch/gazebo_world.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import  LaunchConfiguration, PathJoinSubstitution, TextSubstitution


def generate_launch_description():

    world_arg = DeclareLaunchArgument(
        'world', default_value='empty_site.sdf',
        description='Name of the Gazebo world file to load'
    )

    pkg_wbp_sim_gazebo = get_package_share_directory('wbp_sim_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Add your own gazebo library path here
    gazebo_models = "gazebo_models"

    gazebo_models_path = os.path.join(
        pkg_wbp_sim_gazebo,
        gazebo_models
    )

    os.environ.setdefault("GZ_SIM_RESOURCE_PATH", "")
    os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments={'gz_args': [PathJoinSubstitution([
            pkg_wbp_sim_gazebo,
            'worlds',
            LaunchConfiguration('world')
        ]),
        #TextSubstitution(text=' -r -v -v1 --render-engine ogre --render-engine-gui-api-backend opengl')],
        TextSubstitution(text=' -r -v -v1')],
        'on_exit_shutdown': 'true'}.items()
    )

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(world_arg)
    launchDescriptionObject.add_action(gazebo_launch)

    return launchDescriptionObject

# end of file: ros2_ws/src/wbp_sim_gazebo/launch/gazebo_world.launch.py