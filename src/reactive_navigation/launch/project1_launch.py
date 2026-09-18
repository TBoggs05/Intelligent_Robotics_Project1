import os 

from ament_index_python.packages import get_package_share_directory 

from launch import LaunchDescription 

from launch.actions import IncludeLaunchDescription 

from launch.launch_description_sources import PythonLaunchDescriptionSource 

 

def generate_launch_description(): 

    pkg_dir = get_package_share_directory('reactive_navigation') 

    world_file = os.path.join(pkg_dir, 'worlds', 'test_world.sdf') 

 

    # TurtleBot 4 simulation launch 

    tb4_sim = IncludeLaunchDescription( 

        PythonLaunchDescriptionSource([ 

            os.path.join(get_package_share_directory('turtlebot4_gz_bringup'), 'launch', 'turtlebot4_gz.launch.py') 

        ]), 

        launch_arguments={ 

            'world': world_file, 

            'slam': 'true', # Satisfies the background mapping requirement 

        }.items() 

    ) 

 

    return LaunchDescription([ 

        tb4_sim 

    ]) 

 
