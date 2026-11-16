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

## Simplified Robot Startup

For saved-map navigation, you only need one terminal running the robot launch
and a second terminal for waypoint commands. The SSH terminal is optional; use
it only to check or restart the robot-side services. Build again only after
changing package code or launch files.

### Optional Shell Shortcuts

Add these functions to `~/.bashrc`, then open a new terminal or run
`source ~/.bashrc`. They save the repeated ROS, workspace, and local Nav2
overlay setup; they do not choose a robot or configure its network.

```bash
pjws() {
  cd "$HOME/project1_ws" || return
  source /opt/ros/jazzy/setup.bash
  source install/setup.bash
  source setup_local_nav2.bash
  export ROS_SUPER_CLIENT=False
}

pjmap() {
  pjws || return
  local map_path="$HOME/project1_ws/my_map.yaml"
  ros2 launch reactive_navigation project2_robotlaunch.py \
    "map:=$map_path" reactive:=true
}

pjwaypoint() {
  pjws || return
  local map_path="$HOME/project1_ws/my_map.yaml"
  ros2 run reactive_navigation tour_guide_node --map "$map_path" "$@"
}

pjteleop() {
  pjws || return
  ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -p stamped:=true -r cmd_vel:=/keyboard_input
}
```

Keep robot selection separate: in **each desktop terminal**, run
`robot-setup.sh`, select the same robot, and execute the environment and ROS
daemon commands it prints. After `ros2 daemon stop; ros2 daemon start`, run
the commands as printed, including `ROS_SUPER_CLIENT=True`. The setup script
uses it for the ROS CLI daemon. The project launch explicitly sets it to
`False` for its application nodes, preventing them from inheriting the CLI
setting. The shortcuts cannot replace robot selection because the required DDS
domain and discovery server depend on which robot you selected.
Do not also apply an `rc`, `rcb`, or `rcs` shortcut unless you have confirmed it
targets that same robot; mixing profiles can connect the terminal to the wrong
ROS domain.

### Robot Workflow

1. In desktop Terminal 1, run `robot-setup.sh`, select the robot, and apply the
   printed setup commands. Then launch the saved map:

   ```bash
   pjmap
   ```

   Without the optional shortcut, use:

   ```bash
   source /opt/ros/jazzy/setup.bash
   cd ~/project1_ws
   source install/setup.bash
   source setup_local_nav2.bash
   ros2 launch reactive_navigation project2_robotlaunch.py map:=$HOME/project1_ws/my_map.yaml
   ```

2. In desktop Terminal 2, repeat `robot-setup.sh` for the same robot and apply
  its printed setup commands. Run `pjwaypoint list` to see saved locations,
   `pjwaypoint save entrance` to save the robot's current localized position,
   or `pjwaypoint go entrance` to navigate there. Add a location name after
   `save` or `go`. This terminal runs a command and returns when navigation
   finishes; the launch in Terminal 1 stays running.

3. In desktop Terminal 3, repeat `robot-setup.sh` for the same robot and apply
  its printed setup commands, then run `pjteleop` when you want keyboard
  control. In RViz, set **2D Pose Estimate** after startup. The robot stays
  stopped until localization is ready, then `pjmap` enables reactive roaming
  between Nav2 goals. Wait for `/bt_navigator` to be active and
  `/navigate_to_pose` to have one action server before sending goals.

  Teleop commands temporarily override Nav2 and reactive commands. If a
  `pjwaypoint go` command is still running, Ctrl-C it in Terminal 2 to cancel
  that goal before taking over manually; otherwise its goal resumes after
  keyboard input times out. Teleop must publish to `/keyboard_input`, not
  directly to `/cmd_vel`.

For a robot with a configured shortcut in `~/.bashrc`, the current command notes
map `rc` to `matamata`, `rcb` to `leatherback`, and `rcs` to `softshell`. A profile
only configures this terminal's ROS network; it does not launch the robot or
navigate it. Prefer `robot-setup.sh` if the hostname or DDS settings differ from
those notes.

## Navigate Using a Saved Map

Stop the previous project launch first. Complete the same robot network setup
above, then build, source, and launch from the desktop:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/project1_ws
colcon build --symlink-install
source install/setup.bash
source setup_local_nav2.bash
ros2 launch reactive_navigation project2_robotlaunch.py map:=$HOME/project1_ws/my_map.yaml
```

Pass the YAML file, not the PGM. Keep `my_map.pgm` beside `my_map.yaml`,
since the YAML references that image using a relative path. A quoted
`"map:=~/project1_ws/my_map.yaml"` argument is also supported.

Providing `map` starts the map server, AMCL, Nav2, RViz, and the reactive
behavior nodes. The robot brain arbitrates bumper stops, teleop, reactive
obstacle avoidance, Nav2 goal commands, and optional exploration before
publishing the project command to the base. Nav2's smoothed, collision-monitored
command is routed through the brain. No goal or undock request is sent
automatically. By default, reactive roaming is off in map mode; use
`reactive:=true` to enable it after localization and between Nav2 goals. Without
`map`, the existing reactive exploration behavior remains the default. Do not
run another reactive launch at the same time.

Teleop must send stamped velocity to the brain's input topic. Run this in another
terminal, or use `pjteleop` from the simplified workflow:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -r cmd_vel:=/keyboard_input
```

Do not run a separate teleop publisher directly on `/cmd_vel`; it bypasses the
brain's command arbitration.

In RViz, use **2D Pose Estimate** to place the robot at its actual position and
heading on the saved map. Confirm the laser scan aligns with the map and the
robot pose remains stable before sending a **Nav2 Goal**. Undock the robot before
testing navigation. Map loading alone does not determine the robot's pose.

Useful checks from a terminal with the same robot network setup and sourced workspace:

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 lifecycle get /bt_navigator
ros2 topic echo /amcl_pose --once
ros2 run tf2_ros tf2_echo map base_link
ros2 action list
ros2 action info /navigate_to_pose
```

Expect active map-server/AMCL/navigation nodes, a pose after initialization, a valid
`map` to `base_link` transform, and one action server for `/navigate_to_pose`.
Use `rviz:=false` for a headless launch, or `nav2:=false` to inspect localization
without the navigation stack.

### No-Administrator Nav2 Setup

The system lifecycle manager has a `diagnostic_updater` ABI mismatch, so it
cannot activate the map server or AMCL. The resulting missing map/TF causes
RViz to discard laser scans. Increasing the scan queue will not fix this.

A compatible lifecycle-manager binary has been extracted into your home
directory without changing system packages. Source `setup_local_nav2.bash`
after the ROS and workspace setup in each launch terminal. This selects the
local executable and library; no administrator access is required.

To recreate that local overlay on this workstation:

```bash
mkdir -p "$HOME/.local/share/project1_ros_overlay/downloads"
cd "$HOME/.local/share/project1_ros_overlay/downloads"
apt download ros-jazzy-nav2-lifecycle-manager=1.3.13-1noble.20260921.025008
dpkg-deb -x ros-jazzy-nav2-lifecycle-manager_1.3.13-1noble.20260921.025008_amd64.deb ..
cd ~/project1_ws
source setup_local_nav2.bash
```

That version was selected for the installed Jazzy libraries. If it is no longer
available in the package repository, a replacement must be checked for matching
dependencies before use. To stop using the overlay, open a fresh terminal and
omit `source setup_local_nav2.bash`.

Automatic map-server/AMCL activation, saved-map publication, and RViz startup
have been verified with this overlay on an isolated ROS domain. Physical
localization and navigation goals still need verification on the robot.

After the map appears, use **2D Pose Estimate**. Scan-drop messages can continue
until AMCL has an initial pose and the robot publishes its odometry and sensor
transforms. The robot intentionally remains stationary until you send a goal;
saved-map mode does not start reactive exploration or automatically undock.

### Navigation Action Unavailable

If RViz reports that `/navigate_to_pose` is unavailable even after AMCL accepts
the initial pose, check `/bt_navigator`: it must be `active`, not `unconfigured`.
One observed startup stalled after the planner's lifecycle response timed out.

The launch now defaults to Fast DDS UDP-only transport to avoid the shared-memory
path implicated in that lost response. Discovery-server and domain settings are
preserved. Stop the previous launch with Ctrl-C before relaunching; already-running
processes will not pick up the changed transport. Do not start a second copy.
Use `dds_udp_only:=false` to retain the inherited transport settings for comparison.

With UDP-only transport, an isolated-domain test using synthetic TF, odometry,
and scans verified that all managed Nav2 nodes became active and the
`/navigate_to_pose` action server was available. No navigation goal was sent.
This verifies stack startup, not physical localization, navigation, or that the
transport change resolves every discovery-server timeout on the robot network.

## Save and Visit Named Locations

Named locations are separate from the occupancy-map image. The tour-guide command
saves them in `my_map.waypoints.yaml` beside `my_map.yaml`, including the robot's
map-frame position and heading. It does not modify the map image or map YAML.

Build once after adding the command:

```bash
cd ~/project1_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select reactive_navigation
source install/setup.bash
```

Keep the saved-map launch running. In another terminal, complete the same robot
network setup and source ROS, the workspace, and `setup_local_nav2.bash`.
Navigate the robot to a location using RViz, wait for it to arrive, and point it
in the heading you want recorded. Then run from `~/project1_ws`:

```bash
ros2 run reactive_navigation tour_guide_node save entrance
ros2 run reactive_navigation tour_guide_node save desk
ros2 run reactive_navigation tour_guide_node list
ros2 run reactive_navigation tour_guide_node go entrance
```

Run each `save` while the robot is physically at that location. Saving records
the robot's current localized pose, not the last point clicked in RViz. The
command waits for a fresh `map` to `base_link` transform; AMCL must be initialized.

The default map is `./my_map.yaml`. For a different map or working directory,
place `--map` before the command:

```bash
ros2 run reactive_navigation tour_guide_node --map "$HOME/project1_ws/my_map.yaml" go desk
ros2 run reactive_navigation tour_guide_node save desk --overwrite
```

`go` waits for Nav2 to report arrival and returns a nonzero exit status if the
goal is rejected or navigation fails. Ctrl-C requests cancellation of that goal;
check that cancellation is confirmed. Do not send competing goals from RViz
while this command is navigating. Keep the robot undocked and localized on the
same saved map. If you regenerate the map or change its coordinate frame,
re-record the locations. Waypoint coordinates are meters and heading is radians
in the YAML file; `list` displays heading in degrees and works without the robot.

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