ROS Version: ROS Jazzy
Gazebo Sim Version: 8.15.0




How to load gazebo simulation
1) Source the ROS distro (i.e. underlay):
source /opt/ros/jazzy/setup.bash
2) Source the workspace (i.e. overlay):
source ~/project1_ws/install/setup.bash
3) Build the project using colcon:
colcon build --symlink-install
4) Launch the simulation (ros2 launch <package_name> <launch_file_name>):
colcon build --symlink-install
5) Launch keyboard controller (separate terminal, same environment):
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -r cmd_vel:=/keyboard_input
6) To download map (AFTER RUNNING FOR ~30 SECONDS):
ros2 run nav2_map_server map_saver_cli -f ~/project1_ws/my_map

Connecting/Running the turtlebot
Terminal 1 (SSH into the robot, only used to check it is up):
1) ssh student@<robotname>.cs.nor.ou.edu   (password: student)
2) ros2 topic list   (expect /scan, /tf, /odom)
3) If topics don't show, run: turtlebot4-daemon-restart
4) Check topics again. If still missing, run: ros2 launch turtlebot4_bringup robot.launch.py

Terminal 2 (desktop, NOT ssh; runs the project):
Every new desktop terminal needs steps 1-4 below:
1) robot-setup.sh
2) Enter the robot name, then run each command it prints (unset/export lines, then ros2 daemon stop; ros2 daemon start)
3) ros2 topic list   (confirm the robot's topics are visible)
4) source /opt/ros/jazzy/setup.bash
   cd ~/project1_ws && colcon build --symlink-install
   source install/setup.bash
5) ros2 launch reactive_navigation project2_robotlaunch.py

Terminal 3 (desktop; keyboard control, repeat steps 1-4 above in it):
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -r cmd_vel:=/keyboard_input
Use low speeds (e.g. speed=0.1, turn=0.2).

Terminal 4 (desktop; save the map while the launch is still running, repeat steps 1-3 above):
source /opt/ros/jazzy/setup.bash
ros2 run nav2_map_server map_saver_cli -f ~/project1_ws/my_map
(Or press Ctrl-C in Terminal 3 and run it there.)

When finished: Ctrl-C the launch and return the robot to its dock.

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