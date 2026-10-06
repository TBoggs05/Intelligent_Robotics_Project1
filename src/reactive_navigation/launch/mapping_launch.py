"""Start SLAM after project1_launch.py has Gazebo and the scan relay running."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    """Generate launch description for standalone online SLAM."""
    slam_params = RewrittenYaml(
        source_file=os.path.join(
            get_package_share_directory('turtlebot4_navigation'),
            'config', 'slam.yaml'),
        param_rewrites={'scan_topic': '/scan'},
        convert_types=True,
    )

    # Async SLAM skips stale scans instead of stalling when the sim runs slowly.
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_toolbox'),
            'launch', 'online_async_launch.py')),
        launch_arguments={
            'slam_params_file': slam_params,
            'use_sim_time': 'true',
        }.items()
    )

    return LaunchDescription([slam])