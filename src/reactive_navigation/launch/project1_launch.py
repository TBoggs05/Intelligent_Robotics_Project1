import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    pkg_dir = get_package_share_directory('reactive_navigation')

    #world_file = 'os.path.join(pkg_dir, 'worlds', 'test_world')'
    set__gz_resource_path = AppendEnvironmentVariable(
        name = 'GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_dir, 'worlds')
    )

    tb4_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('turtlebot4_gz_bringup'),
                'launch',
                'turtlebot4_gz.launch.py'
            )
        ]),

        launch_arguments={
            'world': world_file,

            # Start in center of inner room
            'x': '1.524',
            'y': '3.810',
            'z': '0.0',

            # Face east toward doorway
            'yaw': '0.0',

            'rviz': 'false',
        }.items()
    )

    return LaunchDescription([
        tb4_sim
    ])

