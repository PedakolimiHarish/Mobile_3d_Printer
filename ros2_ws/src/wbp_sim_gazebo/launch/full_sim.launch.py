# start of launch file: ros2_ws/src/wbp_sim_gazebo/launch/full_sim.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import TimerAction

def generate_launch_description():

    pkg_sim = FindPackageShare('wbp_sim_gazebo')
    pkg_desc = FindPackageShare('wbp_description')
    urdf_file = PathJoinSubstitution([
        pkg_desc,
        'urdf',
        'wbp_printer_arm.urdf'
    ])

    # robot_state_publisher (CRITICAL)
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue( Command(['cat ', urdf_file]), value_type=str),
            'use_sim_time': True
        }],
    )
        
    # Clock bridge
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/default/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        remappings=[
            ('/world/default/clock', '/clock')
        ],
        output='screen'
    )

    # Gazebo world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_sim,
                'launch',
                'gazebo_world.launch.py'
            ])
        )
    )

    # Spawn robot into Gazebo
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'wbp_robot',
            '-topic', '/robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.01'
        ],
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
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model"   # GZ → ROS
        
        ],
        output="screen"
    )
    
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
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_desc, 'rviz', 'rviz.rviz'])],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    actuator_bridge = Node(
        package="wbp_sim_motion",
        executable="actuator_bridge",
        output="screen"
    )

    base_motion = Node(
        package="wbp_base_motion",
        executable="base_motion_controller",
        output="screen"
    )
    
    gcode_world_executor = Node(
        package="wbp_mobile_fabrication",
        executable="gcode_world_executor",
        parameters=[{'use_sim_time': True}],
        output="screen"
    )

    anchor_localizer = Node(
        package="wbp_localization",
        executable="anchor_localizer",
        parameters=[{'use_sim_time': True}],
        output="screen"
    )

    fake_anchor_solver = Node(
            package="wbp_localization",
            executable="fake_anchor_solver",
            parameters=[{'use_sim_time': True}],
            output="screen"
        )
    world_pose_coordinator = Node(
        package="wbp_coordination",
        executable="world_pose_coordinator",
        parameters=[{'use_sim_time': True}],
        output="screen"
    )

    world_trajectory_generator = Node(
        package='wbp_mobile_fabrication',
        executable='world_trajectory_generator',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    world_motion_coordinator = Node(
        package="wbp_mobile_fabrication",
        executable="world_motion_coordinator",
        parameters=[{'use_sim_time': True}],
        output="screen"
    )
    
    world_motion_controller = Node(
    package="wbp_mobile_fabrication",
    executable="world_motion_controller",
    parameters=[{'use_sim_time': True}],
    output="screen"
)
    
    world_compensation_controller = Node(
        package="wbp_compensation",
        executable="world_compensation_controller",
        parameters=[{'use_sim_time': True}],
        output="screen"
    )

    tool_compensation_applier = Node(
        package="wbp_tool_control",
        executable="tool_compensation_applier",
        parameters=[{'use_sim_time': True}],
        output="screen"
    )

    """ tool_target_controller = Node(
        package="wbp_tool_motion",
        executable="tool_target_controller",
        parameters=[{'use_sim_time': True}],
        output="screen"
    )

    tool_servo_simulator = Node(
        package="wbp_tool_control",
        executable="tool_servo_simulator",
        parameters=[{'use_sim_time': True}],
        output="screen"
    ) 

    tool_tracking_monitor = Node(
        package="wbp_tool_control",
        executable="tool_tracking_monitor",
        parameters=[{'use_sim_time': True}],
        output="screen"
    ) 

    tool_disturbance_injector = Node(
        package="wbp_tool_motion",
        executable="tool_disturbance_injector",
        parameters=[{'use_sim_time': True}],
        output="screen"
    ) """
    
    physical_compensation_controller = Node(
        package="wbp_tool_control",
        executable="physical_compensation_controller",
        parameters=[{'use_sim_time': True}],
        output="screen"
    )
    
    delayed_mobile_stack = TimerAction(
        period=10.0,
        actions=[

                world_pose_coordinator,

                #gcode_world_executor,
                
                world_motion_controller,

                world_motion_coordinator,

                #tool_target_controller,

                #tool_servo_simulator,

                #tool_tracking_monitor,

                world_compensation_controller,
                
                physical_compensation_controller,

                #tool_compensation_applier,

                # tool_disturbance_injector,
            ]
    )

    return LaunchDescription([
        clock_bridge,
        rsp,
        gazebo,
        spawn,
        ros_gz_bridge,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        actuator_bridge,
        base_motion,
        anchor_localizer,
        fake_anchor_solver,
        delayed_mobile_stack,
       
        rviz
    ])
    
    # end of launch file: ros2_ws/src/wbp_sim_gazebo/launch/full_sim.launch.py