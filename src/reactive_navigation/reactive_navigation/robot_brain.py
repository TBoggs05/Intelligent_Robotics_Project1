"""
Central Robot Brain arbitration node implementing Brooks Subsumption.

This node implements the behavioral coordinator for the reactive navigation
system. Following Rodney Brooks' Subsumption Architecture, behaviors are
organized into distinct layers where higher-priority behavioral layers
suppress (inhibit) lower-priority behavioral layers:

Priority Hierarchy (Level 1 is highest priority, Level 6 is lowest):
    Level 1: Collision Halt (Bumper contact hazard -> Stop immediately)
    Level 2: Human Teleoperation (Keyboard override -> Manual user control)
    Level 3: Symmetric Obstacle Escape (LiDAR head-on wall -> 180 deg turn)
    Level 4: Asymmetric Obstacle Avoidance (LiDAR side hazard -> Steer away)
    Level 5: Random Turn Exploration (Every 1 ft forward -> +-15 deg turn)
    Level 6: Drive Forward (Default basal drive -> Constant 0.25 m/s)

In addition to the 6 primary layers, an initial undocking sequence manages
the Create 3 charging dock separation before normal autonomous navigation
begins.
"""

import math

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist, TwistStamped
from irobot_create_msgs.action import Undock
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from reactive_navigation.robot_state import State
from std_msgs.msg import Bool, Int32


class RobotBrain(Node):
    """
    Central robot brain arbitrating reactive navigation behaviors.

    Coordinates sensor inputs and behavior recommendations according
    to a strict priority hierarchy using Brooks' subsumption principles.
    """

    def __init__(self):
        """Initialize the RobotBrain node, parameters, topics, and state."""
        super().__init__('robot_brain')

        # --------------------------------------------------------------------
        # Behavioral State Tracking
        # --------------------------------------------------------------------
        # Current active behavioral state (defaults to basal drive forward)
        self.state = State.DRIVE_FORWARD

        # Priority 2: Keyboard teleoperation tracking
        self.keyboard_input = None
        self.last_key_time = None

        # Priority 1: Bumper collision tracking & post-collision recovery
        self.collision_detected = False
        self.bump_clear_since = None
        self.collision_recovery_pending = False
        self.collision_recovery_active = False
        self.collision_recovery_last_yaw = None
        self.collision_recovery_progress = 0.0

        # Odometry orientation tracking for turn integration
        self.last_odom_yaw = None
        self.turn_away_last_yaw = None
        self.turn_away_progress = 0.0
        self.turn_away_active = False

        # Priority 3 & 4: Obstacle detection inputs (from LiDAR node)
        self.obstacle_twist = None
        self.obstacle_twist_stamp = None
        self.obstacle_behavior_state = None
        self.obstacle_state_stamp = None
        self.last_obstacle_update_stamp = None

        # Priority 5: Random turn inputs (from random turn node)
        self.random_turn_active = False
        self.random_turn_cmd = Twist()

        # --------------------------------------------------------------------
        # Startup Undock Sequence Configuration
        # --------------------------------------------------------------------
        # Controls whether robot undocks before beginning autonomous control
        self.startup_action_name = None
        self.startup_next_action = 'undock'
        self.startup_action_pending = False
        self.startup_failed = False
        self.undock_rejected = False
        self.undock_client = ActionClient(self, Undock, '/undock')

        self.declare_parameter('undock_on_start', True)
        self.undock_finished = not self.get_parameter(
            'undock_on_start'
        ).value

        # --------------------------------------------------------------------
        # Node Parameters
        # --------------------------------------------------------------------
        self.declare_parameter('teleop_topic', '/keyboard_input')
        self.declare_parameter('teleop_timeout_sec', 0.5)
        self.teleop_topic = self.get_parameter('teleop_topic').value
        self.teleop_timeout_sec = float(
            self.get_parameter('teleop_timeout_sec').value
        )

        self.declare_parameter('obstacle_timeout_sec', 0.5)
        self.obstacle_timeout_sec = float(
            self.get_parameter('obstacle_timeout_sec').value
        )

        # --------------------------------------------------------------------
        # Subscriptions
        # --------------------------------------------------------------------
        # Priority 2: Keyboard teleoperation command input
        self.keyboard_sub = self.create_subscription(
            Twist,
            self.teleop_topic,
            self.keyboard_callback,
            10,
        )

        # Priority 5: Random turn status and command inputs
        self.random_turn_active_sub = self.create_subscription(
            Bool,
            '/random_turn/active',
            self.random_turn_active_callback,
            10,
        )
        self.random_turn_cmd_sub = self.create_subscription(
            Twist,
            '/random_turn/cmd',
            self.random_turn_cmd_callback,
            10,
        )

        # Priority 1: Bumper contact hazard input
        self.collision_detection_sub = self.create_subscription(
            Bool,
            '/collision_detected',
            self.collision_callback,
            10,
        )

        # Odometry subscription for yaw angle integration during recovery
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10,
        )

        # Priority 3 & 4: Obstacle detection command and state inputs
        self.obstacle_twist_sub = self.create_subscription(
            Twist,
            '/obstacle_twist',
            self.obstacle_twist_callback,
            10,
        )
        self.obstacle_state_sub = self.create_subscription(
            Int32,
            '/obstacle_state',
            self.obstacle_state_callback,
            10,
        )

        # --------------------------------------------------------------------
        # Publishers
        # --------------------------------------------------------------------
        # Diagnostic behavior state publisher (Priority 1-6)
        self.state_pub = self.create_publisher(
            Int32,
            '/robot_state',
            10,
        )

        # Final motor command publisher to Create 3 base
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped,
            'cmd_vel',
            10,
        )

    # ------------------------------------------------------------------------
    # KEYBOARD CONTROLLER HANDLERS (Priority 2: Human Teleoperation)
    # ------------------------------------------------------------------------
    def keyboard_callback(self, msg: Twist):
        """
        Store the most recent teleoperation command with a receive timestamp.

        :param msg: geometry_msgs/Twist command from keyboard_controller.
        """
        self.keyboard_input = msg
        self.last_key_time = self.get_clock().now()
        # Immediately mark human controlling state unless physical collision
        if self.state != State.COLLIDING:
            self.state = State.HUMAN_CONTROLLING

    def handle_keyboard_input(self):
        """
        Return the most recent teleoperation command while it is still fresh.

        :return: geometry_msgs/Twist if a fresh keypress exists, else None.
        """
        if self.keyboard_input is None or self.last_key_time is None:
            return None

        age_sec = (
            self.get_clock().now() - self.last_key_time
        ).nanoseconds / 1e9

        if age_sec > self.teleop_timeout_sec:
            # Teleop command expired; clear buffer and release override
            self.keyboard_input = None
            self.last_key_time = None
            return None

        self.state = State.HUMAN_CONTROLLING
        return self.keyboard_input

    # ------------------------------------------------------------------------
    # COLLISION DETECTION HANDLERS (Priority 1: Bumper Contact Hazard)
    # ------------------------------------------------------------------------
    def collision_callback(self, msg: Bool):
        """
        Track current bumper hazard state from collision_detection node.

        :param msg: std_msgs/Bool indicating whether bumper contact exists.
        """
        if msg.data and not self.collision_detected:
            self.get_logger().warning(
                'Collision detected; stopping the robot.'
            )
            # Arm collision recovery sequence once bumper contact clears
            if self.undock_finished:
                self.collision_recovery_pending = True
                self.collision_recovery_active = False
                self.collision_recovery_progress = 0.0

        self.collision_detected = msg.data
        if msg.data:
            self.bump_clear_since = None
        elif self.bump_clear_since is None:
            self.bump_clear_since = self.get_clock().now()

    def odom_callback(self, msg: Odometry):
        """
        Extract robot yaw from odometry quaternion to track rotation angles.

        Tracks angular displacement during post-dock turn away and recovery.

        :param msg: nav_msgs/Odometry message containing robot pose.
        """
        orientation = msg.pose.pose.orientation
        # Convert quaternion (x, y, z, w) to planar yaw angle
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (
                orientation.y * orientation.y + orientation.z * orientation.z
            ),
        )

        # Track turn-away after undocking from dock
        if self.turn_away_active and self.turn_away_last_yaw is not None:
            delta_yaw = math.atan2(
                math.sin(yaw - self.turn_away_last_yaw),
                math.cos(yaw - self.turn_away_last_yaw),
            )
            self.turn_away_progress += abs(delta_yaw)
            if self.turn_away_progress >= math.pi:
                self.turn_away_active = False
                self.undock_finished = True
                self.state = State.DRIVE_FORWARD
                self.publish_state()
                self.publish_twist(Twist())
                self.get_logger().info(
                    'Turn-away completed; enabling programmed control.'
                )

        # Track turn-away during post-collision recovery
        if (
            self.collision_recovery_active
            and self.collision_recovery_last_yaw is not None
        ):
            delta_yaw = math.atan2(
                math.sin(yaw - self.collision_recovery_last_yaw),
                math.cos(yaw - self.collision_recovery_last_yaw),
            )
            self.collision_recovery_progress += abs(delta_yaw)
            if self.collision_recovery_progress >= math.pi:
                self.collision_recovery_active = False
                self.collision_recovery_pending = False
                self.state = State.DRIVE_FORWARD
                self.publish_state()
                self.publish_twist(Twist())
                self.get_logger().info(
                    'Collision recovery turn completed; resuming control.'
                )

        self.last_odom_yaw = yaw
        if self.turn_away_active:
            self.turn_away_last_yaw = yaw
        if self.collision_recovery_active:
            self.collision_recovery_last_yaw = yaw

    def halt_on_collision(self):
        """
        Return a zero-velocity stop command while a collision hazard is present.

        :return: geometry_msgs/Twist stop command if colliding, else None.
        """
        if not self.collision_detected:
            return None
        self.state = State.COLLIDING
        return Twist()

    # ------------------------------------------------------------------------
    # OBSTACLE DETECTION HANDLERS (Priority 3 & 4: Escape & Avoidance)
    # ------------------------------------------------------------------------
    def obstacle_twist_callback(self, msg: Twist):
        """
        Receive velocity command recommended by obstacle detection node.

        :param msg: geometry_msgs/Twist recommended velocity.
        """
        self.obstacle_twist = msg
        self.obstacle_twist_stamp = self.get_clock().now()

    def obstacle_state_callback(self, msg: Int32):
        """
        Receive active obstacle state (ESCAPE_SYMMETRIC or AVOID_ASYMMETRIC).

        :param msg: std_msgs/Int32 representing the obstacle state enum value.
        """
        try:
            state = State(msg.data)
        except ValueError:
            self.get_logger().warning(f'Unknown obstacle state: {msg.data}')
            return

        self.last_obstacle_update_stamp = self.get_clock().now()
        if state in (State.ESCAPE_SYMMETRIC, State.AVOID_ASYMMETRIC):
            self.obstacle_behavior_state = state
            self.obstacle_state_stamp = self.get_clock().now()
        else:
            self.obstacle_behavior_state = None
            self.obstacle_state_stamp = None
            self.obstacle_twist = None
            self.obstacle_twist_stamp = None

    def handle_obstacles(self):
        """
        Return active obstacle avoidance command if valid and not expired.

        :return: geometry_msgs/Twist if obstacle behavior active, else None.
        """
        if (
            self.obstacle_twist is None
            or self.obstacle_twist_stamp is None
            or self.obstacle_behavior_state is None
            or self.obstacle_state_stamp is None
        ):
            return None

        now = self.get_clock().now()
        command_age = (now - self.obstacle_twist_stamp).nanoseconds / 1e9
        state_age = (now - self.obstacle_state_stamp).nanoseconds / 1e9
        if (
            command_age > self.obstacle_timeout_sec
            or state_age > self.obstacle_timeout_sec
        ):
            return None

        self.state = self.obstacle_behavior_state
        return self.obstacle_twist

    # ------------------------------------------------------------------------
    # RANDOM TURN HANDLERS (Priority 5: Exploration Turn)
    # ------------------------------------------------------------------------
    def random_turn_active_callback(self, msg: Bool):
        """
        Receive random turn active status from random_turn node.

        :param msg: std_msgs/Bool indicating if a random turn is in progress.
        """
        self.random_turn_active = msg.data

    def random_turn_cmd_callback(self, msg: Twist):
        """
        Receive random turn velocity command from random_turn node.

        :param msg: geometry_msgs/Twist velocity command for the random turn.
        """
        self.random_turn_cmd = msg

    def handle_random_turn(self):
        """
        Return random turn command if currently active.

        :return: geometry_msgs/Twist if random turn is active, else None.
        """
        if self.random_turn_active:
            return self.random_turn_cmd
        return None

    # ------------------------------------------------------------------------
    # STATE PUBLISHER
    # ------------------------------------------------------------------------
    def publish_state(self):
        """Publish the behavior currently selected by the Robot Brain."""
        msg = Int32()
        msg.data = self.state.value
        self.state_pub.publish(msg)

    # ------------------------------------------------------------------------
    # MAIN CONTROL LOOP (Brooks Subsumption Priority Arbitration)
    # ------------------------------------------------------------------------
    def update(self):
        """
        Apply the priority logic for reactive behaviors.

        In accordance with Rodney Brooks' Subsumption Architecture, each
        behavior layer is evaluated in strict priority order. When a
        higher-priority layer is triggered, it commands the robot and returns
        immediately, thereby inhibiting/suppressing all lower-priority layers.

        Priority order (highest to lowest):
            0. Absolute Prerequisite: Complete startup undocking sequence.
            1. Halt if collision(s) detected by bumper(s).
            2. Accept keyboard movement commands from a human user.
            3. Escape from (roughly) symmetric obstacles within 1ft.
            4. Avoid asymmetric obstacles within 1ft in front of the robot.
            5. Turn randomly (+-15 deg) after every 1ft of forward movement.
            6. Drive forward (basal behavior).
        """
        # --- PREREQUISITE: Initial Undock Sequence ---
        if not self.undock_finished:
            self.handle_startup_sequence()
            return

        # --- PRIORITY 1: Bumper Collision Halt ---
        # Highest safety layer: if physical bumper contact is sensed,
        # publish zero velocity immediately to prevent damage.
        collision_cmd = self.halt_on_collision()
        if collision_cmd is not None:
            self.publish_state()
            self.publish_twist(collision_cmd)
            return

        # --- PRIORITY 2: Human Teleoperation ---
        # Human input overrides all automatic navigation behaviors.
        keyboard_cmd = self.handle_keyboard_input()
        if keyboard_cmd is not None:
            self.collision_recovery_pending = False
            self.collision_recovery_active = False
            self.publish_state()
            self.publish_twist(keyboard_cmd)
            return

        # --- COLLISION RECOVERY: Post-Collision Turn ---
        # If bumper contact has cleared and human has not taken control,
        # execute automatic turn away before resuming forward motion.
        if self.collision_recovery_pending:
            self.handle_collision_recovery()
            return

        # --- PRIORITY 3 & 4: Obstacle Handling ---
        # Symmetric Escape (Priority 3) or Asymmetric Avoidance (Priority 4)
        # as classified and generated by the LiDAR obstacle detection node.
        obstacle_cmd = self.handle_obstacles()
        if obstacle_cmd is not None:
            self.publish_state()
            self.publish_twist(obstacle_cmd)
            return

        # --- PRIORITY 5: Random Turn Exploration ---
        # Triggers a +-15 deg turn every 1 ft of accumulated forward travel.
        random_cmd = self.handle_random_turn()
        if random_cmd is not None:
            self.state = State.TURN_RANDOMLY
            self.publish_state()
            self.publish_twist(random_cmd)
            return

        # --- PRIORITY 6: Basal Drive Forward ---
        # Lowest priority behavior: drives forward continuously at 0.25 m/s.
        self.state = State.DRIVE_FORWARD
        self.publish_state()

        default_msg = Twist()
        default_msg.linear.x = 0.25
        default_msg.angular.z = 0.0
        self.publish_twist(default_msg)

    # ------------------------------------------------------------------------
    # STARTUP & RECOVERY HELPER METHODS
    # ------------------------------------------------------------------------
    def handle_startup_sequence(self):
        """Undock before control, docking once first if the goal is rejected."""
        if self.startup_failed or self.startup_action_pending:
            return

        action_name = self.startup_next_action
        if action_name == 'rotate_away':
            self.handle_turn_away()
            return

        action_client = self.undock_client
        goal = Undock.Goal()

        if not action_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warning(
                f'Waiting for /{action_name} action server.',
                throttle_duration_sec=2.0,
            )
            return

        self.startup_action_name = action_name
        self.startup_action_pending = True
        self.get_logger().info(f'Sending {action_name} goal.')
        goal_future = action_client.send_goal_async(goal)
        goal_future.add_done_callback(self._startup_goal_response_callback)

    def handle_turn_away(self):
        """Rotate away from dock after undock action completes."""
        if self.collision_detected:
            self.bump_clear_since = None
            self.get_logger().warning(
                'Waiting for bumper contact to clear before turning away.',
                throttle_duration_sec=2.0,
            )
            return

        if self.bump_clear_since is None:
            self.bump_clear_since = self.get_clock().now()
            return

        clear_duration = (
            self.get_clock().now() - self.bump_clear_since
        ).nanoseconds / 1e9
        if clear_duration < 0.5:
            return

        if self.last_odom_yaw is None:
            self.get_logger().warning(
                'Waiting for odometry before turning away from the dock.',
                throttle_duration_sec=2.0,
            )
            return

        if not self.turn_away_active:
            self.turn_away_active = True
            self.turn_away_last_yaw = self.last_odom_yaw
            self.turn_away_progress = 0.0
            self.get_logger().info('Turning away from the dock.')

        self.state = State.TURN_RANDOMLY
        self.publish_state()
        command = Twist()
        command.angular.z = 0.6
        self.publish_twist(command)

    def handle_collision_recovery(self):
        """Rotate robot to recover heading after clearing bumper contact."""
        self.state = State.COLLIDING
        self.publish_state()

        if self.bump_clear_since is None:
            self.bump_clear_since = self.get_clock().now()
            self.publish_twist(Twist())
            return

        clear_duration = (
            self.get_clock().now() - self.bump_clear_since
        ).nanoseconds / 1e9
        if clear_duration < 0.5 or self.last_odom_yaw is None:
            self.publish_twist(Twist())
            return

        if not self.collision_recovery_active:
            self.collision_recovery_active = True
            self.collision_recovery_last_yaw = self.last_odom_yaw
            self.collision_recovery_progress = 0.0
            self.get_logger().info(
                'Turning away after bumper clearance.'
            )

        self.state = State.TURN_RANDOMLY
        self.publish_state()
        command = Twist()
        command.angular.z = 0.6
        self.publish_twist(command)

    def _startup_goal_response_callback(self, future):
        """Keep motor control gated unless the startup goal is accepted."""
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._fail_startup_action(
                f'{self.startup_action_name} goal request failed: {exc}'
            )
            return

        if not goal_handle.accepted:
            if self._continue_after_undock_rejection(
                'Undock was rejected; the controller reports the robot is '
                'already undocked.'
            ):
                return
            self._fail_startup_action(
                f'{self.startup_action_name} goal was rejected'
            )
            return

        self.get_logger().info(f'{self.startup_action_name} goal accepted.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._startup_result_callback)

    def _startup_result_callback(self, future):
        """Enable control only after docking/undocking reports success."""
        try:
            wrapped_result = future.result()
        except Exception as exc:
            self._fail_startup_action(
                f'{self.startup_action_name} action failed: {exc}'
            )
            return

        action_name = self.startup_action_name
        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            self._fail_startup_action(
                f'{action_name} action ended with status '
                f'{wrapped_result.status}.'
            )
            return

        self.startup_action_pending = False
        self.startup_action_name = None

        is_docked = wrapped_result.result.is_docked
        if action_name == 'undock' and not is_docked:
            self.get_logger().info(
                'Undock action succeeded; turning away from the dock.'
            )
            self.startup_next_action = 'rotate_away'
            return

        self._fail_startup_action(
            f'{action_name} action completed without reaching expected dock.'
        )

    def _continue_after_undock_rejection(self, reason):
        """Allow operation if robot is already confirmed off dock."""
        if self.startup_action_name != 'undock' or self.undock_rejected:
            return False

        self.get_logger().info(
            f'{reason} Robot is already undocked; enabling programmed '
            'control immediately.'
        )
        self.undock_rejected = True
        self.undock_finished = True
        self.startup_action_name = None
        self.startup_next_action = None
        self.startup_action_pending = False
        return True

    def _fail_startup_action(self, reason):
        """Halt startup actions on unrecoverable error."""
        self.startup_action_pending = False
        self.startup_failed = True
        self.get_logger().error(
            f'{reason} Robot control will remain stopped.'
        )

    def publish_twist(self, msg: Twist):
        """
        Convert internal Twist command to TwistStamped and send it.

        :param msg: geometry_msgs/Twist velocity command.
        """
        stamped_msg = TwistStamped()
        stamped_msg.header.stamp = self.get_clock().now().to_msg()
        stamped_msg.twist = msg
        self.cmd_vel_pub.publish(stamped_msg)


def main(args=None):
    """Run the robot brain node with a 20 Hz update loop."""
    rclpy.init(args=args)
    node = RobotBrain()

    try:
        while rclpy.ok():
            node.update()
            rclpy.spin_once(node, timeout_sec=0.05)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
