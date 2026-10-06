"""
Random turn exploration behavior node (Priority Level 5).

This node implements Behavior 5 of the reactive navigation subsumption
architecture: after every 1 foot of forward movement accumulated while in
the basal DRIVE_FORWARD state, execute a random turn uniformly sampled
within +-15 degrees.

Subscribed Topics:
    /odom (nav_msgs/Odometry): Robot pose for distance and yaw tracking.
    /robot_state (std_msgs/Int32): Current active state from robot_brain.

Published Topics:
    /random_turn/active (std_msgs/Bool): Indicates if a turn is executing.
    /random_turn/cmd (geometry_msgs/Twist): Velocity command for the turn.
"""

import math
import random

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from reactive_navigation.robot_state import State
from std_msgs.msg import Bool, Int32


class RandomTurn(Node):
    """
    Executes a random turn within +-15 deg after 1ft of forward movement.

    Accumulates Euclidean planar distance traveled while the robot is in
    the DRIVE_FORWARD state. Once 1 foot (0.3048 m) has elapsed, begins a
    rotation uniformly sampled from [-15 deg, +15 deg] and signals
    robot_brain to arbitrate this behavior.
    """

    def __init__(self):
        """Initialize the RandomTurn node, tracking variables, and topics."""
        super().__init__('random_turn')

        # --------------------------------------------------------------------
        # Constants
        # --------------------------------------------------------------------
        self.one_foot = 0.3048  # 1 foot in meters per specification
        self.turn_speed = 0.6  # Turning angular velocity in rad/s

        # --------------------------------------------------------------------
        # Behavioral State Tracking
        # --------------------------------------------------------------------
        # Mirror of current robot brain active state
        self.current_state = State.DRIVE_FORWARD

        # --------------------------------------------------------------------
        # Odometry Integration Variables
        # --------------------------------------------------------------------
        self.last_x = None
        self.last_y = None
        self.last_yaw = None
        self.distance_since_turn = 0.0

        # --------------------------------------------------------------------
        # Random Turn Execution Variables
        # --------------------------------------------------------------------
        self.turn_active = False
        self.turn_direction = 1  # +1 for CCW (left), -1 for CW (right)
        self.target_angle = 0.0  # Absolute target angular displacement in rad
        self.angle_turned = 0.0  # Accumulated angular displacement in rad

        # --------------------------------------------------------------------
        # Subscriptions
        # --------------------------------------------------------------------
        # Odometry subscription for distance and heading tracking
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10,
        )

        # Brain state subscription to reset accumulator when interrupted
        self.state_sub = self.create_subscription(
            Int32,
            '/robot_state',
            self.state_callback,
            10,
        )

        # --------------------------------------------------------------------
        # Publishers
        # --------------------------------------------------------------------
        # Active flag publisher indicating turn request to robot_brain
        self.active_pub = self.create_publisher(
            Bool,
            '/random_turn/active',
            10,
        )

        # Velocity command publisher for the random turn maneuver
        self.cmd_pub = self.create_publisher(
            Twist,
            '/random_turn/cmd',
            10,
        )

        # Publish behavior command periodically at 20 Hz
        self.timer = self.create_timer(0.05, self.publish_behavior)

        self.get_logger().info('Random Turn behavior node initialized.')

    def state_callback(self, msg: Int32):
        """
        Receive active behavioral state from robot_brain.

        If a higher-priority behavior (collision, teleoperation, or obstacle
        avoidance) takes control, reset the accumulated forward distance
        and cancel any active random turn to avoid spurious turns.

        :param msg: std_msgs/Int32 representing current State enum value.
        """
        try:
            new_state = State(msg.data)
            # Reset distance if higher-priority behaviors suppress random turn
            if new_state in (
                State.COLLIDING,
                State.HUMAN_CONTROLLING,
                State.ESCAPE_SYMMETRIC,
                State.AVOID_ASYMMETRIC,
            ):
                self.distance_since_turn = 0.0
                if self.turn_active:
                    self.turn_active = False
                    self.angle_turned = 0.0
            self.current_state = new_state
        except ValueError:
            self.get_logger().warning(f'Unknown robot state: {msg.data}')

    def odom_callback(self, msg: Odometry):
        """
        Track planar distance traveled and turn displacement from odometry.

        Accumulates forward displacement during DRIVE_FORWARD and monitors
        heading rotation during TURN_RANDOMLY using shortest angular diff.

        :param msg: nav_msgs/Odometry pose update.
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation

        # Convert quaternion (x, y, z, w) to planar yaw angle
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # Initialize odometry reference on very first callback
        if self.last_x is None:
            self.last_x = x
            self.last_y = y
            self.last_yaw = yaw
            return

        # Calculate planar Euclidean distance increment
        dx = x - self.last_x
        dy = y - self.last_y
        distance_moved = math.hypot(dx, dy)

        # Only accumulate forward distance when DRIVE_FORWARD is active
        if self.current_state == State.DRIVE_FORWARD and not self.turn_active:
            self.distance_since_turn += distance_moved

        # Trigger random turn once 1 foot (0.3048 m) has elapsed
        if self.distance_since_turn >= self.one_foot and not self.turn_active:
            self.start_random_turn()

        # Track turn progress while executing random turn
        if self.turn_active and self.current_state == State.TURN_RANDOMLY:
            yaw_change = yaw - self.last_yaw
            # Normalize yaw angle change to [-pi, +pi] to prevent boundary jumps
            yaw_change = math.atan2(
                math.sin(yaw_change), math.cos(yaw_change)
            )
            self.angle_turned += abs(yaw_change)

            # Check if rotation target has been reached
            if self.angle_turned >= self.target_angle:
                self.turn_active = False
                self.angle_turned = 0.0
                self.get_logger().info('Random turn completed successfully.')

        # Store previous pose for next differential step
        self.last_x = x
        self.last_y = y
        self.last_yaw = yaw

    def start_random_turn(self):
        """Begin a new random turn sampled uniformly from [-15, +15] deg."""
        angle_degrees = random.uniform(-15.0, 15.0)
        angle_radians = math.radians(angle_degrees)

        if angle_degrees >= 0.0:
            self.turn_direction = 1  # CCW / Left
        else:
            self.turn_direction = -1  # CW / Right

        self.target_angle = abs(angle_radians)
        self.angle_turned = 0.0
        self.turn_active = True
        self.distance_since_turn = 0.0

        self.get_logger().info(
            f'Random turn requested: {angle_degrees:.2f} degrees'
        )

    def publish_behavior(self):
        """Publish behavior active flag and recommended Twist to robot_brain."""
        active_msg = Bool()
        active_msg.data = self.turn_active
        self.active_pub.publish(active_msg)

        cmd = Twist()
        if self.turn_active:
            cmd.linear.x = 0.0
            cmd.angular.z = self.turn_direction * self.turn_speed
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)


def main(args=None):
    """Run the random turn node."""
    rclpy.init(args=args)
    node = RandomTurn()
    try:
        rclpy.spin(node)
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
