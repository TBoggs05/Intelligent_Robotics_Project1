ROS Version: ROS Jazzy
Gazebo Sim Version: 8.15.0




How to load gazebo simulation
1) Source the ROS distro (i.e. underlay)
source /opt/ros/jazzy/setup.bash
2) Source the workspace (i.e. overlay)
source ~/project1_ws/install/setup.bash
3) Build the project using colcon
colcon build --symlink-install
4) Launch the simulation (ros2 launch <package_name> <launch_file_name>)
ros2 launch reactive_navigation project1_launch.py


#BRIDGE COMMAND FOR /SCAN#
ros2 run ros_gz_bridge parameter_bridge \
  '/world/empty/model/turtlebot4/link/rplidar_link/sensor/rplidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan' \
  --ros-args \
  -r '/world/empty/model/turtlebot4/link/rplidar_link/sensor/rplidar/scan:=/scan'




KNOWN ISSUES (sometimes, doesn't impact functionality):
On open: bridge crash occurs
On close ruby3.2 stops unexpectedly error




#PROJECT1 - Intelligent Robotics
#Team - Trace Boggs, Luis Cardenas, Jackson Dunlap

See architecture for setup (still needs to include mapping node)