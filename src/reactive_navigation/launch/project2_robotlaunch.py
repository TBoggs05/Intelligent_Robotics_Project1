import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for the physical TurtleBot 4."""
    pkg_reactive_nav = get_package_share_directory('reactive_navigation')
    tb4_bringup_launch_dir = os.path.join(
        get_package_share_directory('turtlebot4_bringup'), 'launch'
    )
    tb4_navigation_launch_dir = os.path.join(
        get_package_share_directory('turtlebot4_navigation'), 'launch'
    )
    turtlebot4_node_params = os.path.join(
        pkg_reactive_nav,
        'config',
        'turtlebot4_node.yaml',
    )

    launch_args = [
        DeclareLaunchArgument(
            'nav2', default_value='false', choices=['true', 'false'],
            description='Run the Nav2 navigation stack'),
        DeclareLaunchArgument(
            'mapping', default_value='true', choices=['true', 'false'],
            description='Run SLAM for background occupancy grid mapping'),
        DeclareLaunchArgument(
            'undock_on_start', default_value='false', choices=['true', 'false'],
            description='Request undocking before robot control starts'),
    ]

    turtlebot4 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb4_bringup_launch_dir, 'standard.launch.py')
        ),
        launch_arguments={
            'param_file': turtlebot4_node_params,
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb4_navigation_launch_dir, 'nav2.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('nav2')),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py',
            )
        ),
        launch_arguments={
            'use_sim_time': 'false',
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
            'use_sim_time': False,
            'undock_on_start': LaunchConfiguration('undock_on_start'),
        }],
    )

    collision_detection = Node(
        package='reactive_navigation',
        executable='collision_detection',
        name='collision_detection',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    obstacle_detection = Node(
        package='reactive_navigation',
        executable='obstacle_detection_node',
        name='obstacle_detection_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    random_turn = Node(
        package='reactive_navigation',
        executable='random_turn_node',
        name='random_turn_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    return LaunchDescription(launch_args + [
        turtlebot4,
        nav2,
        slam,
        collision_detection,
        obstacle_detection,
        random_turn,
        TimerAction(period=5.0, actions=[robot_brain]),
    ])
