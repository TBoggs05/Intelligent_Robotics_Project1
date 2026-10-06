"""
Project 1 Primary Launch File.

Brings up the Gazebo Harmonic simulation environment with custom test world,
spawns the TurtleBot 4, bridges RPLiDAR sensor data, starts scan relaying,
launches online async SLAM mapping, and initializes all reactive navigation
subsumption nodes (collision detection, obstacle detection, random turn, and
robot brain).
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for the complete Project 1 system."""
    pkg_reactive_nav = get_package_share_directory('reactive_navigation')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    tb4_launch_dir = os.path.join(
        get_package_share_directory('turtlebot4_gz_bringup'), 'launch'
    )
    turtlebot4_node_params = os.path.join(
        pkg_reactive_nav,
        'config',
        'turtlebot4_node.yaml',
    )

    launch_args = [
        DeclareLaunchArgument(
            'world', default_value='test_world',
            description='Simulation World name'),
        DeclareLaunchArgument(
            'headless', default_value='false', choices=['true', 'false'],
            description='Run Gazebo in headless mode (server only)'),
        DeclareLaunchArgument(
            'nav2', default_value='false', choices=['true', 'false'],
            description='Run the Nav2 navigation stack'),
        DeclareLaunchArgument(
            'rviz', default_value='false', choices=['true', 'false'],
            description='Start RViz'),
        DeclareLaunchArgument(
            'mapping', default_value='true', choices=['true', 'false'],
            description='Run SLAM for background occupancy grid mapping'),
        DeclareLaunchArgument(
            'undock_on_start', default_value='false', choices=['true', 'false'],
            description='Request undocking before robot control starts'),
    ]

    # Ensure Gazebo can locate our custom worlds
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([
            os.path.join(pkg_reactive_nav, 'worlds'),
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
            '/opt/ros/jazzy/share',
        ])
    )

    # Force Intel GPU / DRI_PRIME=1 to prevent Nouveau OpenGL driver crashes
    dri_prime = SetEnvironmentVariable(
        name='DRI_PRIME',
        value='1',
    )

    # Gazebo with GUI
    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb4_launch_dir, 'sim.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'model': 'standard',
        }.items(),
        condition=UnlessCondition(LaunchConfiguration('headless')),
    )

    # Gazebo headless (server only)
    gazebo_headless = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': [
                    LaunchConfiguration('world'),
                    '.sdf',
                    ' -s',
                    ' -r',
                    ' -v 4',
                ],
                'on_exit_shutdown': 'true',
            }.items(),
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            output='screen',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        ),
    ], condition=IfCondition(LaunchConfiguration('headless')))

    # Spawn TurtleBot 4 and dock
    tb4_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb4_launch_dir, 'turtlebot4_spawn.launch.py')),
        launch_arguments={
            # Start in center of inner room
            'x': '1.524',
            'y': '3.810',
            'z': '0.0',
            # Face east toward doorway
            'yaw': '0.0',
            'rviz': LaunchConfiguration('rviz'),
            'slam': 'false',
            'nav2': LaunchConfiguration('nav2'),
            'model': 'standard',
            'use_sim_time': 'true',
            'param_file': turtlebot4_node_params,
        }.items()
    )

    # Asynchronous SLAM mapping (used when slam:=true) to prevent sync message dropping
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py',
            )
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': os.path.join(
                get_package_share_directory('turtlebot4_navigation'),
                'config',
                'slam.yaml',
            ),
        }.items(),
        condition=IfCondition(LaunchConfiguration('mapping')),
    )

    robot_brain = Node(
        package='reactive_navigation',
        executable='robot_brain',
        name='robot_brain',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'undock_on_start': LaunchConfiguration('undock_on_start'),
        }],
    )

    collision_detection = Node(
        package='reactive_navigation',
        executable='collision_detection',
        name='collision_detection',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    obstacle_detection = Node(
        package='reactive_navigation',
        executable='obstacle_detection_node',
        name='obstacle_detection_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    random_turn = Node(
        package='reactive_navigation',
        executable='random_turn_node',
        name='random_turn_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Explicit RPLiDAR bridge for custom Gazebo simulation world.
    # The default bringup launch does not forward custom world names to
    # the bridge, causing scans to be bridged on the wrong world topic.
    rplidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='rplidar_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            [
                '/world/',
                LaunchConfiguration('world'),
                '/model/turtlebot4/link/rplidar_link/sensor/rplidar/scan'
                '@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            ]
        ],
        remappings=[
            (
                [
                    '/world/',
                    LaunchConfiguration('world'),
                    '/model/turtlebot4/link/rplidar_link/sensor/rplidar/scan',
                ],
                '/scan_raw',
            )
        ],
    )

    # Relay LiDAR scan with canonical URDF frame_id ('rplidar_link')
    # This guarantees SLAM and obstacle detection always have valid TF to base_link
    scan_relay = Node(
        package='reactive_navigation',
        executable='scan_frame_relay',
        name='scan_frame_relay',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'input_topic': [
                '/world/',
                LaunchConfiguration('world'),
                '/model/turtlebot4/link/rplidar_link/sensor/rplidar/scan',
            ],
            'output_topic': '/scan_mapping',
            'frame_id': 'rplidar_link',
        }],
    )

    return LaunchDescription(launch_args + [
        gz_resource_path,
        dri_prime,
        gazebo_gui,
        gazebo_headless,
        tb4_spawn,
        rplidar_bridge,
        scan_relay,
        slam,
        collision_detection,
        obstacle_detection,
        random_turn,
        TimerAction(period=5.0, actions=[robot_brain]),
    ])
