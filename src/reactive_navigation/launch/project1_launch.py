import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node  # Added import for Node


def generate_launch_description():
    world_name = 'test_world'

    tb4_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(
                    'turtlebot4_gz_bringup'
                ),
                'launch',
                'turtlebot4_gz.launch.py'
            )
        ),

        launch_arguments={
            'world': world_name,
            
            # Start in center of inner room
            'x': '1.524',
            'y': '3.810',
            'z': '0.0',
            
            # Face east toward doorway
            'yaw': '0.0',
            
            'rviz': 'false',
        }.items()
    )

    # Added ros_gz_bridge Node
    rplidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='rplidar_bridge',
        output='screen',
        arguments=[
            '/world/test_world/model/turtlebot4/link/rplidar_link/sensor/rplidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
        ],
        remappings=[
            ('/world/test_world/model/turtlebot4/link/rplidar_link/sensor/rplidar/scan', '/scan_raw')
        ],
        parameters=[{'use_sim_time': True}]
    )
    scan_relay = Node(
        package='reactive_navigation',
        executable='scan_frame_relay',
        name='scan_frame_relay',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'input_topic': '/scan_raw',
            'output_topic': '/scan_mapping',
            'frame_id': 'rplidar_link',
        }],
    )
    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('reactive_navigation'),
            'launch',
            'mapping_launch.py',
        )),
    )
    #Add robot brain node
    robot_brain = Node(
        package='reactive_navigation',
        executable='robot_brain',
        name='robot_brain',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    obstacle_detection = Node(
            package='reactive_navigation',
            executable='obstacle_detection',
            name='obstacle_detection',
            output='screen',
            parameters=[{'use_sim_time': True}]
    )
    collision_detection= Node(
                package='reactive_navigation',
                executable='collision_detection',
                name='collision_detection',
                output='screen',
                parameters=[{'use_sim_time': True}]
    )
    keyboard_controller = Node(
            package = 'reactive_navigation',
            executable='keyboard_controller',
            name ='keyboard_controller',
            output='screen',
            parameters=[{'use_sim_time': True}],
            prefix='gnome-terminal --'
    )
    random_turn = Node(
            package = 'reactive_navigation',
            executable='random_turn',
            name ='random_turn',
            output='screen',
            parameters=[{'use_sim_time': True}]
    )
    return LaunchDescription([
        rplidar_bridge,
        tb4_sim,
        scan_relay,
        mapping,
        collision_detection,
        obstacle_detection,
        keyboard_controller,
        random_turn,
        TimerAction(period=5.0, actions=[robot_brain]),
        
    ])