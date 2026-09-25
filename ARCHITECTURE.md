# Project 1 Architecture

## Node Architecture

### 1. Keyboard Controller Node
**File:** `keyboard_controller.py`
- **Responsibility:** Capture keyboard input from the user
- **Publishes:** `/keyboard_input` (Twist messages)
- **Subscribes:** None
- **Behavior:** Listens for key presses (w/a/s/d) and publishes velocity commands

### 2. Collision Detection Node
**File:** `collision_detection_node.py`
- **Responsibility:** Monitor bumper sensors and detect collisions
- **Publishes:** `/collision_detected` (Bool messages)
- **Subscribes:** TurtleBot 4 bumper/contact sensor topics
- **Behavior:** Reads bumper state and publishes True/False collision status

### 3. Obstacle Sensing Node
**File:** `obstacle_detection_node.py`
- **Responsibility:** Process lidar/sensor data to detect obstacles
- **Publishes:** `/obstacles` (obstacle location/distance data)
- **Subscribes:** TurtleBot 4 lidar/sensor topics
- **Behavior:** Analyzes sensor data and publishes obstacle information

### 4. Random Turn Node
**File:** `random_turn_node.py`
- **Responsibility:** Track movement distance and trigger random turns
- **Publishes:** `/turn_signal` (Bool or signal when 1 ft reached)
- **Subscribes:** `/odom` (odometry data from TurtleBot 4)
- **Behavior:** Accumulates distance traveled; signals Robot Brain Node every 1 foot

### 5. Robot Brain Node (Main Controller)
**File:** `robot_brain.py`
**Class:** `Robot(Node)`
- **Responsibility:** Central decision-making and behavior arbitration
- **Publishes:** `/cmd_vel` (Twist commands to TurtleBot 4)
- **Subscribes:**
  - `/keyboard_input` (from Keyboard Controller)
  - `/collision_detected` (from Collision Detection)
  - `/obstacles` (from Obstacle Sensing)
  - `/turn_signal` (from Random Turn)
- **Behavior:** Runs priority-based control loop

## Robot Brain Node Control Loop

The Robot Brain Node runs a priority-based finite state machine with 6 behaviors in order of priority:

```python
def control_loop(self):
    # Check behaviors in priority order (lowest enum value = highest priority)
    if self.halt_on_collision():          # State.COLLIDING = 1
        return
    if self.handle_keyboard_input():      # State.HUMAN_CONTROLLING = 2
        return
    if self.escape_symmetric_obstacles(): # State.ESCAPE_SYMMETRIC = 3
        return
    if self.avoid_asymmetric_obstacles(): # State.AVOID_ASYMMETRIC = 4
        return
    if self.turn_randomly():              # State.TURN_RANDOMLY = 5
        return
    if self.drive_forward():              # State.DRIVE_FORWARD = 6
        return
```

## State Enumeration

```python
class State(Enum):
    COLLIDING = 1
    HUMAN_CONTROLLING = 2
    ESCAPE_SYMMETRIC = 3
    AVOID_ASYMMETRIC = 4
    TURN_RANDOMLY = 5
    DRIVE_FORWARD = 6
```

## ROS 2 Topic Specifications

| Topic | Message Type | Publisher | Subscriber | Frequency |
|-------|--------------|-----------|------------|-----------|
| `/cmd_vel` | Twist | Robot Brain Node | TurtleBot 4 | 10 Hz |
| `/keyboard_input` | Twist | Keyboard Controller | Robot Brain Node | On input |
| `/collision_detected` | Bool | Collision Detection | Robot Brain Node | 10 Hz |
| `/obstacles` | (custom) | Obstacle Sensing | Robot Brain Node | 10 Hz |
| `/turn_signal` | Bool/Int32 | Random Turn | Robot Brain Node | Every 1 ft |
| `/odom` | Odometry | TurtleBot 4 | Random Turn | 50 Hz |

## Behavior Descriptions

1. **Halt on Collision:** Publishes zero velocity when bumper sensors detect contact
2. **Handle Keyboard Input:** Forwards user keyboard commands directly to TurtleBot 4
3. **Escape Symmetric Obstacles:** Turns 180°±30° when encountering symmetric obstacles within 1 ft
4. **Avoid Asymmetric Obstacles:** Reflexively turns away from the closest obstacle
5. **Turn Randomly:** Executes ±15° random turn every 1 foot of forward movement
6. **Drive Forward:** Default behavior; moves robot forward at constant velocity

