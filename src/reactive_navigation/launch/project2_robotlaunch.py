import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
    Shutdown,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    AndSubstitution,
    EqualsSubstitution,
    LaunchConfiguration,
    NotEqualsSubstitution,
    OrSubstitution,
)
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue


def configure_map_mode(context):
    """Validate command ownership and resolve the saved map path."""
    map_path = LaunchConfiguration('map').perform(context)
    mapping = IfCondition(LaunchConfiguration('mapping')).evaluate(context)
    nav2 = IfCondition(LaunchConfiguration('nav2')).evaluate(context)
    if map_path and mapping:
        raise ValueError('A saved map requires mapping:=false; do not run SLAM with AMCL.')
    if nav2 and not map_path:
        raise ValueError('Nav2 requires a saved map; provide map:=/path/to/map.yaml.')
    if not map_path:
        return []
    map_path = os.path.abspath(os.path.expanduser(map_path))
    if not os.path.isfile(map_path):
        raise ValueError(f'Map YAML does not exist: {map_path}')
    return [SetLaunchConfiguration('map', map_path)]


def generate_launch_description():
    """Generate launch description for the physical TurtleBot 4."""
    tb4_navigation_launch_dir = os.path.join(
        get_package_share_directory('turtlebot4_navigation'), 'launch'
    )

    launch_args = [
        DeclareLaunchArgument(
            'map', default_value='',
            description='Saved map YAML; enables localization, Nav2 and RViz by default'),
        DeclareLaunchArgument(
            'nav2', default_value=NotEqualsSubstitution(LaunchConfiguration('map'), ''),
            choices=['true', 'false'],
            description='Run the Nav2 navigation stack'),
        DeclareLaunchArgument(
            'mapping', default_value=EqualsSubstitution(LaunchConfiguration('map'), ''),
            choices=['true', 'false'],
            description='Run SLAM for background occupancy grid mapping'),
        DeclareLaunchArgument(
            'reactive', default_value=AndSubstitution(
                EqualsSubstitution(LaunchConfiguration('map'), ''),
                EqualsSubstitution(LaunchConfiguration('nav2'), 'false'),
            ),
            choices=['true', 'false'],
            description='Run autonomous reactive driving instead of Nav2'),
        DeclareLaunchArgument(
            'rviz', default_value=NotEqualsSubstitution(LaunchConfiguration('map'), ''),
            choices=['true', 'false'],
            description='Open the Nav2 navigation view in RViz'),
        DeclareLaunchArgument(
            'undock_on_start', default_value='false', choices=['true', 'false'],
            description='Request undocking before robot control starts'),
        DeclareLaunchArgument(
            'dds_udp_only', default_value='true', choices=['true', 'false'],
            description='Disable Fast DDS shared memory for local lifecycle service traffic'),
    ]

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb4_navigation_launch_dir, 'localization.launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': 'false',
            'use_composition': 'False',
        }.items(),
        condition=IfCondition(NotEqualsSubstitution(LaunchConfiguration('map'), '')),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d',
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'rviz', 'nav2_default_view.rviz',
            ),
        ],
        parameters=[{'use_sim_time': False}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    nav2 = GroupAction(
        condition=IfCondition(LaunchConfiguration('nav2')),
        actions=[
            SetRemap(src='cmd_vel', dst='/cmd_vel_nav'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(tb4_navigation_launch_dir, 'nav2.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'false',
                }.items(),
            ),
        ],
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
            'autonomous_enabled': LaunchConfiguration('reactive'),
            'require_localization': ParameterValue(
                NotEqualsSubstitution(LaunchConfiguration('map'), ''),
                value_type=bool,
            ),
        }],
        on_exit=Shutdown(reason='robot_brain exited'),
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
        OpaqueFunction(function=configure_map_mode),
        SetEnvironmentVariable('ROS_SUPER_CLIENT', 'False'),
        SetEnvironmentVariable(
            'FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4',
            condition=IfCondition(LaunchConfiguration('dds_udp_only')),
        ),
        localization,
        nav2,
        slam,
        rviz,
        GroupAction(
            condition=IfCondition(OrSubstitution(
                LaunchConfiguration('reactive'),
                NotEqualsSubstitution(LaunchConfiguration('map'), ''),
            )),
            actions=[
                collision_detection,
                obstacle_detection,
                random_turn,
                TimerAction(period=5.0, actions=[robot_brain]),
            ],
        ),
    ])
