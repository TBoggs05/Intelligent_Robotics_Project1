import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


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

    teleop_topic = '/robot_brain/teleop_cmd'

    robot_brain_node = Node(
        package='reactive_navigation',
        executable='robot_brain',
        name='robot_brain',
        output='screen',
        parameters=[
            {
                'teleop_topic': teleop_topic,
                'teleop_timeout_sec': 0.35,
            }
        ],
    )

    keyboard_controller_node = Node(
        package='reactive_navigation',
        executable='keyboard_controller',
        name='keyboard_controller',
        output='screen',
        parameters=[
            {
                'output_topic': teleop_topic,
            }
        ],
    )

    random_turn_node = Node(
        package='reactive_navigation',
        executable='random_turn',
        name='random_turn',
        output='screen',
    )

    return LaunchDescription([
        tb4_sim,
        robot_brain_node,
        keyboard_controller_node,
        random_turn_node,
    ])
