"""
Obstacle Detection and Avoidance Node (Priorities 3 and 4).

Implements two distinct reactive behaviors based on frontal LiDAR ranges:
- Priority 3 (Symmetric Obstacle Escape): A fixed action pattern triggered
  when obstacles are detected symmetrically within 1ft (0.3048m) in front of
  the robot. Rotates 180 +- 30 degrees using odometry integration.
- Priority 4 (Asymmetric Obstacle Avoidance): A reflexive behavior triggered
  when an obstacle is closer on one side within 1ft. Steers away from the
  closer obstacle until the stimulus is cleared.
"""

import math
import random

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from reactive_navigation.robot_state import State
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32
import tf2_ros

# ============================================================================
# Constants & Geometric Parameters
# ============================================================================

# 1 foot in meters per assignment specifications
ONE_FOOT = 0.3048

# Distance from RPLiDAR center to front chassis edge (0.164m - (-0.04m) = 0.204m)
ROBOT_FRONT_OFFSET = 0.204

# Detection cone: +/- 30 degrees from robot heading (total 60 degree field of view)
FRONT_ANGLE = 30.0

# Detection threshold: 1 ft in front of the physical robot chassis
OBSTACLE_DISTANCE = ONE_FOOT + ROBOT_FRONT_OFFSET

# Maximum left/right range difference considered "roughly symmetric"
SYMMETRY_TOLERANCE = 0.15 * ONE_FOOT

# Angular turn rates (rad/s)
AVOID_TURN_SPEED = 1.0
ESCAPE_TURN_SPEED = 1.0

# Default escape angle (180 degrees)
ESCAPE_ANGLE = math.radians(180.0)


class ObstacleDetectionNode(Node):
    """
    Evaluates 2D laser scans and odometry to execute reactive avoidance.

    Subscribes to /scan and /odom, identifies frontal obstacles within 1ft,
    and publishes recommended motion commands to /obstacle_twist and state
    indicators to /obstacle_state.
    """

    def __init__(self):
        """Initialize publishers, subscriptions, TF listener, and state."""
        super().__init__('obstacle_detection_node')

        # Parameters
        self.declare_parameter('scan_topic', '/scan')
        scan_topic = self.get_parameter('scan_topic').value

        # Publishers to robot_brain arbiter
        self.publisher = self.create_publisher(
            Twist,
            '/obstacle_twist',
            10,
        )
        self.state_publisher = self.create_publisher(
            Int32,
            '/obstacle_state',
            10,
        )

        # Sensor and odometry subscriptions
        self.create_subscription(
            LaserScan,
            scan_topic,
            self.lidar_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10,
        )

        # Coordinate transform listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Behavioral state ('NORMAL' or 'ESCAPE')
        self.state = 'NORMAL'

        # Odometry tracking for fixed action pattern
        self.current_yaw = None
        self.previous_yaw = None
        self.escape_rotation = 0.0
        self.target_escape_angle = ESCAPE_ANGLE
        self.escape_direction = -1.0

        # LiDAR relative orientation cache
        self.cached_lidar_yaw = None
        self.lidar_transform_logged = False
        self.first_scan_logged = False

        self.get_logger().info('Obstacle detection node started.')

    def create_command(self, linear_x=0.0, angular_z=0.0):
        """
        Construct a Twist message with specified velocities.

        :param linear_x: Forward velocity in m/s.
        :param angular_z: Rotational velocity in rad/s.
        :return: Populated Twist message.
        """
        command = Twist()
        command.linear.x = linear_x
        command.angular.z = angular_z
        return command

    def odom_callback(self, message: Odometry):
        """
        Extract yaw orientation and accumulate rotation during escape maneuvers.

        Uses incremental angular differences rather than global coordinates
        to prevent wrap-around discontinuity at +/- pi boundaries.
        """
        q = message.pose.pose.orientation

        # Quaternion to Euler yaw conversion
        sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
        cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(sin_yaw, cos_yaw)

        # First odometry reading initialization
        if self.previous_yaw is None:
            self.previous_yaw = yaw
            self.current_yaw = yaw
            return

        # Normalized incremental change since previous sample
        delta_yaw = self.normalize_angle(yaw - self.previous_yaw)
        self.current_yaw = yaw
        self.previous_yaw = yaw

        # Accumulate total angular displacement during fixed action pattern
        if self.state == 'ESCAPE':
            self.escape_rotation += abs(delta_yaw)
            if self.escape_rotation >= self.target_escape_angle:
                self.perform_escape()

    def lidar_callback(self, message: LaserScan):
        """
        Process laser scan rays to detect obstacles within 1 foot in front.

        Transforms scan angles into robot chassis frame (base_link), segments
        readings into left/right frontal sectors (+/- 30 deg), and triggers
        either symmetric escape (Priority 3) or asymmetric avoidance (Priority 4).
        """
        if not self.first_scan_logged:
            self.get_logger().info(
                f'Received scan with frame {message.header.frame_id}.'
            )
            self.first_scan_logged = True

        # If a fixed action pattern escape is running, continue until complete
        if self.state == 'ESCAPE':
            self.perform_escape()
            return

        # Resolve LiDAR mount orientation relative to base_link via TF
        if self.cached_lidar_yaw is None:
            try:
                frame_to_lookup = message.header.frame_id
                try:
                    transform = self.tf_buffer.lookup_transform(
                        'base_link',
                        frame_to_lookup,
                        rclpy.time.Time(),
                    )
                except Exception:
                    transform = self.tf_buffer.lookup_transform(
                        'base_link',
                        'rplidar_link',
                        rclpy.time.Time(),
                    )

                q = transform.transform.rotation
                sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
                cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                self.cached_lidar_yaw = math.atan2(sin_yaw, cos_yaw)
            except Exception:
                # Default to pi/2 (TurtleBot 4 RPLiDAR physical mount angle)
                self.cached_lidar_yaw = math.pi / 2.0

        lidar_yaw = self.cached_lidar_yaw
        if not self.lidar_transform_logged:
            self.get_logger().info(
                'LiDAR orientation relative to base_link: '
                f'{math.degrees(lidar_yaw):.1f} degrees'
            )
            self.lidar_transform_logged = True

        left_distance = None
        right_distance = None
        front_angle = math.radians(FRONT_ANGLE)

        # Inspect all rays within the front +/- 30 degree cone
        for i, distance in enumerate(message.ranges):
            if not math.isfinite(distance):
                continue
            if distance < message.range_min or distance > message.range_max:
                continue

            lidar_angle = message.angle_min + i * message.angle_increment
            robot_angle = self.normalize_angle(lidar_angle + lidar_yaw)

            # Filter rays outside the +/- 30 degree forward sector
            if abs(robot_angle) > front_angle:
                continue

            # Classify into left sector (> +1 deg)
            if robot_angle > math.radians(1.0):
                if left_distance is None or distance < left_distance:
                    left_distance = distance
            # Classify into right sector (< -1 deg)
            elif robot_angle < math.radians(-1.0):
                if right_distance is None or distance < right_distance:
                    right_distance = distance
            # Center dead-ahead: contributes to both sectors
            else:
                if left_distance is None or distance < left_distance:
                    left_distance = distance
                if right_distance is None or distance < right_distance:
                    right_distance = distance

        # Check proximity threshold (within 1 foot of front edge)
        left_obstacle = (
            left_distance is not None and left_distance <= OBSTACLE_DISTANCE
        )
        right_obstacle = (
            right_distance is not None and right_distance <= OBSTACLE_DISTANCE
        )

        left_str = f'{left_distance:.2f}m' if left_distance is not None else 'None'
        right_str = f'{right_distance:.2f}m' if right_distance is not None else 'None'
        self.get_logger().info(
            f'Front distance: left={left_str}, right={right_str} '
            f'(threshold={OBSTACLE_DISTANCE:.2f}m)',
            throttle_duration_sec=2.0,
        )

        # --------------------------------------------------------------------
        # Priority 3: Symmetric Obstacle Escape (Fixed Action Pattern)
        # --------------------------------------------------------------------
        if (
            left_obstacle
            and right_obstacle
            and abs(left_distance - right_distance) <= SYMMETRY_TOLERANCE
        ):
            self.start_escape(left_distance, right_distance)
            return

        # --------------------------------------------------------------------
        # Priority 4: Asymmetric Obstacle Avoidance (Reflexive Steering)
        # --------------------------------------------------------------------
        if left_obstacle or right_obstacle:
            if left_obstacle and (
                not right_obstacle or left_distance < right_distance
            ):
                # Closer on left -> reflexively steer right
                command = self.create_command(
                    linear_x=0.0,
                    angular_z=-AVOID_TURN_SPEED,
                )
                self.get_logger().info(
                    f'AVOID: LEFT {left_distance:.3f}m -> Turn RIGHT'
                )
            else:
                # Closer on right -> reflexively steer left
                command = self.create_command(
                    linear_x=0.0,
                    angular_z=AVOID_TURN_SPEED,
                )
                self.get_logger().info(
                    f'AVOID: RIGHT {right_distance:.3f}m -> Turn LEFT'
                )

            self.publish_obstacle_state(State.AVOID_ASYMMETRIC)
            self.publisher.publish(command)
            return

        # No obstacle within 1 foot: signal DRIVE_FORWARD
        self.publish_obstacle_state(State.DRIVE_FORWARD)

    def start_escape(self, left_dist=None, right_dist=None):
        """
        Initiate fixed action pattern escape maneuver (180 +- 30 degrees).

        Samples turn angle uniformly in [150, 210] degrees and sets turn
        direction away from slightly closer obstacle if known.
        """
        if self.current_yaw is None:
            self.get_logger().warn(
                'Cannot ESCAPE because odometry is unavailable.'
            )
            self.publish_obstacle_state(State.DRIVE_FORWARD)
            return

        self.state = 'ESCAPE'
        self.target_escape_angle = math.radians(random.uniform(150.0, 210.0))

        if (
            left_dist is not None
            and right_dist is not None
            and abs(left_dist - right_dist) > 1e-4
        ):
            self.escape_direction = -1.0 if left_dist < right_dist else 1.0
        else:
            self.escape_direction = random.choice([-1.0, 1.0])

        self.publish_obstacle_state(State.ESCAPE_SYMMETRIC)
        self.escape_rotation = 0.0
        self.previous_yaw = self.current_yaw

        self.get_logger().info(
            'ESCAPE: symmetric obstacle detected within 1ft. '
            f'Beginning fixed {math.degrees(self.target_escape_angle):.1f} deg turn.'
        )
        self.perform_escape()

    def perform_escape(self):
        """Publish rotation command until accumulated turn reaches target angle."""
        if self.escape_rotation < self.target_escape_angle:
            self.publish_obstacle_state(State.ESCAPE_SYMMETRIC)
            command = self.create_command(
                linear_x=0.0,
                angular_z=self.escape_direction * ESCAPE_TURN_SPEED,
            )
            self.publisher.publish(command)
            return

        # Escape completed: return to NORMAL state
        command = self.create_command(linear_x=0.0, angular_z=0.0)
        self.publisher.publish(command)
        self.state = 'NORMAL'
        self.escape_rotation = 0.0
        self.publish_obstacle_state(State.DRIVE_FORWARD)

        self.get_logger().info(
            f'ESCAPE complete: {math.degrees(self.target_escape_angle):.1f} deg rotated.'
        )

    def publish_obstacle_state(self, state: State):
        """
        Broadcast current obstacle state to the central arbiter.

        :param state: Active State enum value.
        """
        message = Int32()
        message.data = state.value
        self.state_publisher.publish(message)

    @staticmethod
    def normalize_angle(angle: float) -> float:
        """
        Normalize angle to the interval [-pi, +pi].

        :param angle: Raw angle in radians.
        :return: Normalized angle in radians.
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    """Run obstacle detection node."""
    rclpy.init(args=args)
    node = ObstacleDetectionNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        pass
    finally:
        try:
            if rclpy.ok():
                node.publisher.publish(node.create_command(0.0, 0.0))
                node.publish_obstacle_state(State.DRIVE_FORWARD)
        except Exception:
            pass

        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
