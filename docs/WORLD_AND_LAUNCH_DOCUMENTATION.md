# World and Launch File Documentation (Section 5.2)

**Course:** CS 4023/5023 - Intro to Intelligent Robotics, Fall 2026  
**Assignment:** Project 1: Simulated Mobile Robot Reactive Navigation  
**Team Members:** Trace Boggs, Luis Cardenas, Jackson Dunlap  

---

## 1. Introduction and Overview for a Lay Audience

This document provides a non-technical, descriptive explanation of the simulation world and the system launch configuration developed for Project 1. The goal of the project is to simulate an autonomous mobile robot (a TurtleBot 4 standard model with an iRobot Create 3 base and an RPLiDAR sensor) navigating through an indoor environment consisting of an enclosed room and an adjacent hallway. 

Rather than relying on pre-programmed floor plans or global path planning, the robot operates purely through reactive behaviors: sensing its immediate surroundings and responding in real time to walls and obstacles. To evaluate this behavior accurately and safely, the physical environment, sensors, motors, and coordinate transforms must be modeled faithfully within the Gazebo simulation engine and interconnected through the Robot Operating System (ROS 2 Jazzy).

---

## 2. Environment Geometry and World File (`test_world.sdf`)

The simulation world is defined in Simulation Description Format (SDF), an XML-based specification used by Gazebo to describe physical entities, collision boundaries, visual appearances, lighting, and physics parameters.

### 2.1 Coordinate System and Units
Gazebo operates in the standard international metric system (meters, kilograms, seconds, and radians). Because the project specification defines the environment in imperial units (feet), all dimensions are converted using the exact standard conversion factor:
$$\text{1 foot} = 0.3048\text{ meters}$$

### 2.2 Room and Hallway Layout
The simulation world represents an enclosed floor plan consisting of an inner rectangular room and an outer hallway that wraps around two adjacent sides in a backwards "L" shape.

1. **Outer Boundary Walls:**
   - **Total Footprint:** The entire structure occupies an outer footprint of 15 feet wide (4.572 meters along the X axis) by 20 feet long (6.096 meters along the Y axis).
   - **Outer West Wall:** A solid vertical wall running from $Y = 0\text{ m}$ to $Y = 6.096\text{ m}$ at $X = 0\text{ m}$. It has a thickness of 0.1 m and a height of 1.0 m.
   - **Outer North Wall:** A solid horizontal wall at $Y = 6.096\text{ m}$ running from $X = 0\text{ m}$ to $X = 4.572\text{ m}$.
   - **Outer East Wall:** A solid vertical wall at $X = 4.572\text{ m}$ running from $Y = 0\text{ m}$ to $Y = 6.096\text{ m}$.
   - **Outer South Wall:** A solid horizontal wall at $Y = 0\text{ m}$ running from $X = 0\text{ m}$ to $X = 4.572\text{ m}$.
   - Together, these four walls completely enclose the simulation arena, guaranteeing that the robot remains inside the test area and cannot accidentally escape into the unbounded void of the physics engine.

2. **Inner Room:**
   - The room is located in the upper-left quadrant of the footprint.
   - It measures approximately 10 feet wide (3.048 meters along X) by 15 feet long (4.572 meters along Y).
   - The west and north sides of the room are bounded by the outer west and outer north walls.
   - **Inner South Wall:** Located at $Y = 1.524\text{ m}$ (5 feet from the south boundary), extending from $X = 0\text{ m}$ to $X = 3.048\text{ m}$. This separates the room from the bottom hallway branch.
   - **Inner East Wall & Center Doorway:** Located at $X = 3.048\text{ m}$ (10 feet from the west wall), running from $Y = 1.524\text{ m}$ to $Y = 6.096\text{ m}$ (a total span of 15 feet). In the exact center of this 15-foot wall, a 5-foot (1.524 m) opening is left open to serve as the doorway connecting the room to the hallway. Structurally, this is modeled as two separate 5-foot wall segments:
     - *Bottom Segment:* $Y = 1.524\text{ m}$ to $Y = 3.048\text{ m}$ (center pose at $Y = 2.286\text{ m}$).
     - *Doorway Gap:* $Y = 3.048\text{ m}$ to $Y = 4.572\text{ m}$ (free passage of 1.524 m).
     - *Top Segment:* $Y = 4.572\text{ m}$ to $Y = 6.096\text{ m}$ (center pose at $Y = 5.334\text{ m}$).

3. **Backwards L-Shaped Hallway:**
   - The hallway wraps around the south and east sides of the inner room with a uniform width of 5 feet (1.524 meters).
   - **Shorter Branch (South):** Runs along the bottom from $X = 0\text{ m}$ to $X = 4.572\text{ m}$ between $Y = 0\text{ m}$ and $Y = 1.524\text{ m}$. Its total length is 15 feet (4.572 m). Both ends are closed by the outer west and outer east walls.
   - **Longer Branch (East):** Runs along the right side from $Y = 0\text{ m}$ to $Y = 6.096\text{ m}$ between $X = 3.048\text{ m}$ and $X = 4.572\text{ m}$. Its total length is 20 feet (6.096 m). It terminates at the closed outer north wall.
   - From the perspective of the longer branch of the hallway, the 5-foot doorway into the room begins exactly 5 feet from the closed northern end of the hallway ($Y = 4.572\text{ m}$ to $Y = 3.048\text{ m}$), matching the specification precisely.

4. **Physics, Ground Plane, and Lighting:**
   - **Ground Plane:** Modeled with an expansive 100 m $\times$ 100 m flat collision box and grey visual surface with appropriate friction parameters to ensure smooth, slip-free wheel traction.
   - **Physics Engine:** Runs Gazebo's physics system with a maximum step size of 0.001 s (1 ms) and real-time update rate of 1000 Hz, with contact and scene broadcasting plugins enabled.
   - **Lighting:** A simulated directional sun light source is positioned at $Z = 10\text{ m}$ with soft diffuse lighting and specular reflections to render walls clearly in the graphical view.

---

## 3. ROS 2 Launch Architecture (`project1_launch.py`)

Launching a modern robotics simulation requires orchestrating multiple processes concurrently: physics simulation, sensor bridging, coordinate frame broadcasting, hardware abstraction nodes, behavior controllers, and mapping services. The launch file `project1_launch.py` coordinates all of these components into a single command.

### 3.1 Key Launch Components
1. **Gazebo Simulation Engine:**
   - By default, Gazebo starts with the full 3D Graphical User Interface (`headless:=false`).
   - For automated testing, CI, or resource-constrained environments, a server-only headless mode is supported (`headless:=true`), which runs `gz sim -s -r` with zero graphics overhead.
   - To resolve a known driver bug between Nouveau/Mesa OpenGL and Ogre2 under Linux/Xwayland, the launch file automatically sets `DRI_PRIME=1`, directing 3D rendering to the integrated Intel UHD graphics card to prevent crashes.

2. **Robot and Dock Spawning (`turtlebot4_spawn.launch.py`):**
   - The robot is spawned in the exact physical center of the inner room at coordinate $(X=1.524\text{ m}, Y=3.810\text{ m}, Z=0.0\text{ m})$ with an initial yaw orientation of $0.0\text{ radians}$ (facing east, directly aimed at the 5-foot doorway).
   - All standard robot state publishers, joint state publishers, and differential drive controllers are initialized.

3. **LiDAR Sensor Bridge and Frame Relays:**
   - The TurtleBot 4 RPLiDAR sensor emits 2D laser scans in Gazebo. To make these scans accessible to standard ROS 2 nodes, a dedicated `ros_gz_bridge` instance maps the simulation topic `/world/test_world/model/turtlebot4/link/rplidar_link/sensor/rplidar/scan` directly to the standard ROS 2 topic `/scan` at ~62 Hz.
   - A static transform publisher (`rplidar_colons_stf`) links the scoped Gazebo sensor frame (`turtlebot4::rplidar_link::rplidar`) to the standard robot URDF frame (`rplidar_link`), allowing both reactive navigation and SLAM to transform laser ranges into the robot's base coordinate system without dropping scans.

4. **Reactive Navigation Nodes:**
   - `collision_detection`: Monitors Create 3 bumper contact hazards and publishes `/collision_detected`.
   - `obstacle_detection_node`: Subscribes to `/scan` and `/odom`, detects obstacles within 1 ft, and publishes escape/avoid commands on `/obstacle_twist` and state on `/obstacle_state`.
   - `random_turn_node`: Subscribes to `/odom` and `/robot_state`, tracks displacement, and signals random turns every 1 ft of forward driving on `/random_turn/active` and `/random_turn/cmd`.
   - `robot_brain`: Starts after a brief stabilization delay (5 seconds), functioning as the central priority arbiter that publishes final stamped velocity commands to `/cmd_vel`.

5. **Background Mapping (`slam_toolbox`):**
   - Controlled by the `mapping` launch argument (enabled by default with `mapping:=true`).
   - Launches `slam_toolbox` in asynchronous mode (`online_async_launch.py`), utilizing Ceres Solver optimization.
   - Because it operates asynchronously in a dedicated worker thread, incoming laser scans and odometry transformations are processed without blocking the reactive control loop, continuously generating an occupancy grid published to `/map`.

---

## 4. Instructions for Running on CSN Linux Machines

Follow these step-by-step instructions to build, launch, and operate the project on the University of Oklahoma CSN Linux lab computers.

### Step 1: Open Terminal and Set Up the Workspace
Open a bash shell terminal and source the system ROS 2 Jazzy underlay:
```bash
source /opt/ros/jazzy/setup.bash
cd ~/project1_ws/Intelligent_Robotics_Project1
```

### Step 2: Build the ROS 2 Package
Build the package using `colcon` with symlink installation:
```bash
colcon build --symlink-install --packages-select reactive_navigation
```
Verify that the output shows `Summary: 1 package finished` with 0 failures.

### Step 3: Source the Built Workspace Overlay
```bash
source install/setup.bash
```

### Step 4: Run the Verification Test Suite
Run the automated test suite to verify all 6 reactive behaviors and style compliance:
```bash
colcon test --packages-select reactive_navigation
colcon test-result --all --verbose
```
All 9 tests (behaviors, flake8, pep257) should report PASSED with 0 errors and 0 failures.

### Step 5: Launch the Simulation
Launch the complete simulation with the Gazebo graphical user interface:
```bash
ros2 launch reactive_navigation project1_launch.py
```
*(Optional Launch Arguments):*
- Headless execution (no GUI window): `ros2 launch reactive_navigation project1_launch.py headless:=true`
- Disable background mapping: `ros2 launch reactive_navigation project1_launch.py mapping:=false`
- Launch RViz visualization: `ros2 launch reactive_navigation project1_launch.py rviz:=true`

### Step 6: Teleoperation (Keyboard Control)
To test Priority 2 (keyboard teleoperation) while the robot is running:
1. Open a **second terminal window**.
2. Source the environment:
   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/project1_ws/Intelligent_Robotics_Project1/install/setup.bash
   ```
3. Run the interactive keyboard controller:
   ```bash
   ros2 run reactive_navigation keyboard_controller
   ```
4. Control keys:
   - `w`: Drive forward (0.3 m/s)
   - `s`: Drive backward (-0.3 m/s)
   - `a`: Rotate left (0.8 rad/s)
   - `d`: Rotate right (-0.8 rad/s)
   - `x` or `[Space]`: Stop (0.0 m/s)
   - `Ctrl+C`: Exit keyboard controller and return control to autonomous behaviors.

### Step 7: Saving the Generated Occupancy Grid Map
Once the robot has explored the room and hallway, open another terminal and save the map:
```bash
source /opt/ros/jazzy/setup.bash
ros2 run nav2_map_server map_saver_cli -f ~/project1_ws/project1_map --ros-args -p use_sim_time:=true
```
This writes `project1_map.yaml` and `project1_map.pgm` to disk.

