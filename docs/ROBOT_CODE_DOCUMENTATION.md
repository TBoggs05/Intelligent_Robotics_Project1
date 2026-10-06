# Robot Code Architecture Documentation (Section 5.4)

**Course:** CS 4023/5023 - Intro to Intelligent Robotics, Fall 2026  
**Assignment:** Project 1: Simulated Mobile Robot Reactive Navigation  
**Team Members:** Trace Boggs, Luis Cardenas, Jackson Dunlap  

---

## 1. Selected Reactive Architecture: Subsumption Architecture

For Project 1, our team implemented a **Subsumption Architecture**, directly derived from the foundational reactive robotics principles pioneered by Rodney Brooks (1986). In contrast to classical deliberative architectures (Sense-Plan-Act), which construct centralized global world models and compute trajectories offline, the subsumption architecture decomposes the robot's control system into a set of distinct, autonomous behavioral layers organized in a strict priority hierarchy.

### 1.1 Why We Chose the Subsumption Architecture
We selected the subsumption architecture for several compelling design reasons:
1. **Direct Coupling of Perception to Action:** Each behavior couples its own perceptual processing directly to motor commands without requiring intermediate world modeling or graph searching. This guarantees minimal computational latency and instantaneous real-time reflexes (crucial for collision avoidance and bumper halts).
2. **Natural Alignment with Assignment Priorities:** The project specifications explicitly dictate an ordered set of six behaviors where higher-priority actions must supersede lower-priority actions. Brooks' concept of **suppression** (subsuming the inputs or outputs of lower-level layers when higher-level layers are triggered) maps directly to this requirement.
3. **Robustness and Graceful Degradation:** Lower-level competency (e.g., driving forward or turning randomly) operates independently of higher-level competencies (e.g., obstacle escape or human teleoperation). If a higher-level sensor fails or ceases to publish, the robot never freezes; it gracefully falls back to its lower-level competency layers.
4. **Modularity and Clean Object-Oriented Decomposition:** In our ROS 2 implementation, each behavioral competency is encapsulated within its own dedicated, object-oriented node class (`CollisionDetection`, `KeyboardController`, `ObstacleDetectionNode`, `RandomTurn`, and `RobotBrain`). This architecture facilitates clean unit testing, individual parameter tuning, and high modularity.

---

## 2. Core Architectural Characteristics and How the Code Embodies Them

Brooks' subsumption architecture relies on several fundamental mechanisms that are explicitly represented in our codebase:

### 2.1 Behavioral Decomposition into Competency Layers
The software is organized into six discrete layers, indexed by priority from highest (Priority 1) to lowest (Priority 6):
- **Layer 1 (Priority 1) - Collision Halt:** Monitored by `collision_detection.py`. When bumper contacts occur on the Create 3 base, all motor output is immediately zeroed, halting the robot and triggering bumper clearance recovery.
- **Layer 2 (Priority 2) - Human Teleoperation:** Managed by `keyboard_controller.py`. Human keyboard commands override all autonomous navigation, allowing immediate manual steering.
- **Layer 3 (Priority 3) - Symmetric Obstacle Escape:** Handled by `obstacle_detection_node.py`. When an obstacle directly in front of the robot is roughly equidistant on both sides within 1 foot, the robot initiates a fixed turnaround of $180^\circ \pm 30^\circ$.
- **Layer 4 (Priority 4) - Asymmetric Obstacle Avoidance:** Handled by `obstacle_detection_node.py`. When an obstacle is detected within 1 foot closer on one side, the robot reflexively steers away from the closer side.
- **Layer 5 (Priority 5) - Random Turn Exploration:** Handled by `random_turn.py`. After accumulating 1 foot of forward motion, the robot turns by a uniformly sampled angle in the range $[-15^\circ, +15^\circ]$ to explore the environment.
- **Layer 6 (Priority 6) - Drive Forward:** The default basal competency implemented directly inside `robot_brain.py`, commanding constant forward velocity ($v_x = 0.25\text{ m/s}$).

### 2.2 Output Suppression via Central Priority Arbiter (`RobotBrain`)
In classical hardware subsumption, wires connect modules, with higher-level wires physically inhibiting or suppressing lower-level signal lines. In our software implementation, this suppression mechanism is embodied in the `update()` control loop of `RobotBrain` (`robot_brain.py`):

```python
def update(self):
    # 1. Collision Halt (Priority 1)
    collision_cmd = self.halt_on_collision()
    if collision_cmd is not None:
        self.publish_state()
        self.publish_twist(collision_cmd)
        return  # Suppresses all lower layers

    # 2. Human Teleoperation (Priority 2)
    keyboard_cmd = self.handle_keyboard_input()
    if keyboard_cmd is not None:
        self.publish_state()
        self.publish_twist(keyboard_cmd)
        return  # Suppresses layers 3, 4, 5, 6

    # 3 & 4. Obstacle Escape & Avoidance (Priorities 3 & 4)
    obstacle_cmd = self.handle_obstacles()
    if obstacle_cmd is not None:
        self.publish_state()
        self.publish_twist(obstacle_cmd)
        return  # Suppresses layers 5 and 6

    # 5. Random Turn Exploration (Priority 5)
    random_cmd = self.handle_random_turn()
    if random_cmd is not None:
        self.publish_state()
        self.publish_twist(random_cmd)
        return  # Suppresses layer 6

    # 6. Default Competency: Drive Forward (Priority 6)
    self.state = State.DRIVE_FORWARD
    self.publish_state()
    default_msg = Twist()
    default_msg.linear.x = 0.25
    self.publish_twist(default_msg)
```

At every iteration (running at 20 Hz), the arbiter checks each layer in strict descending order of priority. As soon as a higher-priority layer produces an active command, it is published to `/cmd_vel`, and the loop immediately returns, effectively **suppressing** all lower layers.

### 2.3 Reflex vs. Fixed Action Pattern Distinction
The assignment specifications draw an important ethological distinction between reflexes and fixed action patterns, both of which are modeled accurately:
- **Asymmetric Avoidance as a Reflex (Layer 4):** A reflex persists only as long as the triggering stimulus is present. In `ObstacleDetectionNode`, when an asymmetric obstacle is detected within 1 foot, the node emits an angular velocity command directed away from the closer side. The instant the robot turns sufficiently far that the obstacle leaves the 1-foot front detection cone, the stimulus ceases, and the node immediately transitions state back to `State.DRIVE_FORWARD`. Control instantly falls back through the arbiter to basal forward driving.
- **Symmetric Escape as a Fixed Action Pattern (Layer 3):** A fixed action pattern is an instinctive behavioral sequence that, once triggered by a sign stimulus, runs to completion even if the stimulus disappears during execution. In `ObstacleDetectionNode`, when a symmetric obstacle is detected within 1 foot, the node transitions to `state = 'ESCAPE'`, samples a target turnaround angle $\theta_{\text{target}} \sim \mathcal{U}(150^\circ, 210^\circ)$ ($180^\circ \pm 30^\circ$), and tracks accumulated rotation using incremental odometry. Even after the robot turns $30^\circ$ and the wall is no longer in front of the LiDAR, the node continues to execute the turn until $\Delta \theta_{\text{accumulated}} \ge \theta_{\text{target}}$, fulfilling the fixed action pattern requirement.

---

## 3. Data Structures and Algorithms

### 3.1 State Representation (`robot_state.py`)
To represent behavioral states cleanly and avoid magic numbers, we define a standard Python `Enum`:
```python
class State(Enum):
    COLLIDING = 1
    HUMAN_CONTROLLING = 2
    ESCAPE_SYMMETRIC = 3
    AVOID_ASYMMETRIC = 4
    TURN_RANDOMLY = 5
    DRIVE_FORWARD = 6
```
The numerical values of the enum correspond exactly to the priority ranking (1 through 6), making state transitions readily verifiable in logs and telemetry via the `/robot_state` topic.

### 3.2 Obstacle Detection Algorithms (`obstacle_detection_node.py`)
1. **Front Sector Partitioning:**
   The node filters the 360° LiDAR scan array to a front sector spanning $\pm 30^\circ$ relative to the robot's heading ($[-\pi/6, +\pi/6]$ radians). Ray angles are transformed into the robot's base coordinate frame using the transform between `base_link` and `rplidar_link`. Rays with angles $\in [1^\circ, 30^\circ]$ are classified into the left sub-sector; rays with angles $\in [-30^\circ, -1^\circ]$ are classified into the right sub-sector; and rays within $[-1^\circ, +1^\circ]$ (dead-center) are registered into both partitions to prevent central obstacles from being falsely classified as purely lateral.

2. **Distance Threshold Calibration:**
   The specification dictates triggering when obstacles are within 1 foot of the robot's body. Because the RPLiDAR is mounted at an offset ($X = -0.04\text{ m}$) from the robot center and the TurtleBot 4 chassis has a radius of $0.164\text{ m}$, the distance from the LiDAR sensor to the front bumper edge is:
   $$d_{\text{offset}} = 0.164\text{ m} - (-0.04\text{ m}) = 0.204\text{ m}$$
   The triggering distance threshold is therefore calibrated as:
   $$d_{\text{threshold}} = d_{\text{offset}} + 1\text{ ft} = 0.204\text{ m} + 0.3048\text{ m} = 0.5088\text{ m}$$

3. **Symmetry Classification:**
   An obstacle condition is classified as symmetric if both left and right partitions detect an obstacle within $d_{\text{threshold}}$ and their minimum ranges satisfy:
   $$|d_{\text{left}} - d_{\text{right}}| \le \text{tolerance} \quad (\text{tolerance} = 0.15 \times 1\text{ ft} \approx 0.0457\text{ m})$$
   Otherwise, if either condition is violated, the obstacle is classified as asymmetric, and the turning direction is chosen away from $\min(d_{\text{left}}, d_{\text{right}})$.

4. **Continuous Odometry Tracking and Wraparound Prevention:**
   To track yaw rotation accurately across the $[-\pi, +\pi]$ boundary during escape and random turns, the node tracks incremental differences between consecutive odometry messages:
   $$\Delta \theta_k = \text{atan2}(\sin(\theta_k - \theta_{k-1}), \cos(\theta_k - \theta_{k-1}))$$
   $$\theta_{\text{accumulated}} = \sum |\Delta \theta_k|$$
   This incremental accumulation completely avoids singularity glitches or infinite spin loops caused by angle normalization wraparounds.

### 3.3 Random Turn Algorithm (`random_turn.py`)
1. **Forward Displacement Accumulation:**
   The Euclidean displacement between successive odometry positions is computed:
   $$\Delta s_k = \sqrt{(x_k - x_{k-1})^2 + (y_k - y_{k-1})^2}$$
   To guarantee that turns only occur after forward driving (and not during turns or obstacle avoidance), $\Delta s_k$ is only accumulated when the current state reported by `RobotBrain` is strictly `State.DRIVE_FORWARD`:
   $$s_{\text{accumulated}} \leftarrow s_{\text{accumulated}} + \Delta s_k \quad \text{if } \text{state} == \text{State.DRIVE_FORWARD}$$

2. **Triggering and Uniform Sampling:**
   When $s_{\text{accumulated}} \ge 0.3048\text{ m}$ (1 foot), the node samples a turn angle:
   $$\alpha \sim \mathcal{U}(-15^\circ, +15^\circ)$$
   The turning direction is set to $\text{sgn}(\alpha)$ and speed to $0.6\text{ rad/s}$. Once accumulated rotation satisfies $|\Delta \theta| \ge |\alpha|$, the turn completes, and $s_{\text{accumulated}}$ resets to zero.

3. **State Synchronization and Preemption:**
   If a higher-priority behavior takes over (e.g., bumper contact or obstacle detection), `random_turn.py` intercepts the state change on `/robot_state`, immediately cancels any pending random turn, and resets $s_{\text{accumulated}} = 0.0$, ensuring that the robot always drives a full 1 foot forward after recovering from an obstacle before triggering its next random turn.

### 3.4 Bumper Collision Detection (`collision_detection.py`)
The Create 3 base publishes a `HazardDetectionVector` on `/hazard_detection`. Each hazard entry contains a numeric `type`. The node inspects the vector and sets `/collision_detected` to `True` if and only if an entry matches `HazardDetection.BUMP` (`type == 1`). Optical cliff sensors and wheel drop detections are filtered out, ensuring the robot halts strictly on mechanical bumper impact.

---

## 4. Coding Standards and Style Compliance

All source files are written in object-oriented Python 3 adhering to the ROS 2 Python Style Guide. The code passes 100% of automated tests for:
- **`ament_flake8`**: PEP 8 compliance, formatting, line length, and variable scoping.
- **`ament_pep257`**: Complete docstring documentation for all modules, classes, and methods.
- **`test_behaviors.py`**: Comprehensive pytest unit test suite verifying each individual behavior and priority arbitration independently.
- Clean shutdown handling with `ExternalShutdownException` to ensure zero hanging processes or memory leaks upon exit.

