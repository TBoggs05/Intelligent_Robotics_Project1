import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf2_ros
from reactive_navigation.robot_state import State

# ============================================================
# Constants
# ============================================================

ONE_FOOT = 0.3048

# Look ±30 degrees from the robot's actual front
FRONT_ANGLE = 35.0

# Obstacles within 1 foot
OBSTACLE_DISTANCE = ONE_FOOT

# Difference between left/right obstacles that is considered
# "roughly symmetric"
SYMMETRY_TOLERANCE = 0.15 * ONE_FOOT

# Normal forward speed
FORWARD_SPEED = 0.80

# Reflexive avoidance turn speed
AVOID_TURN_SPEED = 2.0

# Fixed escape turn speed
ESCAPE_TURN_SPEED = 1.5

# Escape approximately 180 degrees
ESCAPE_ANGLE = math.radians(180.0)


class ObstacleDetectionNode(Node):

    def __init__(self):
        super().__init__('obstacle_detection_node')

        # --------------------------------------------------------
        # Publisher
        # --------------------------------------------------------

        self.publisher = self.create_publisher(
            (TwistStamped),
            '/avoid_obstacles',
            10
        )

        # --------------------------------------------------------
        # Subscribers
        # --------------------------------------------------------

        self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # --------------------------------------------------------
        # TF
        # --------------------------------------------------------

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        # --------------------------------------------------------
        # State
        # --------------------------------------------------------

        # NORMAL or ESCAPE
        self.state = 'NORMAL'

        # Current robot yaw
        self.current_yaw = None

        # Previous odometry yaw
        self.previous_yaw = None

        # Accumulated rotation during ESCAPE
        self.escape_rotation = 0.0

        # Whether we have successfully obtained the LiDAR->base
        # transform yet
        self.lidar_transform_logged = False

        self.get_logger().info(
            'Obstacle detection node started.'
        )

    # ============================================================
    # Create TwistStamped command
    # ============================================================

    def create_command(
        self,
        linear_x=0.0,
        angular_z=0.0
    ):

        command = TwistStamped()

        command.header.stamp = (
            self.get_clock().now().to_msg()
        )

        command.header.frame_id = 'base_link'

        command.twist.linear.x = linear_x
        command.twist.angular.z = angular_z

        return command

    # ============================================================
    # Odometry callback
    # ============================================================

    def odom_callback(self, message):

        q = message.pose.pose.orientation

        # Quaternion -> yaw
        sin_yaw = 2.0 * (
            q.w * q.z +
            q.x * q.y
        )

        cos_yaw = 1.0 - 2.0 * (
            q.y * q.y +
            q.z * q.z
        )

        yaw = math.atan2(
            sin_yaw,
            cos_yaw
        )

        # First odometry reading
        if self.previous_yaw is None:
            self.previous_yaw = yaw
            self.current_yaw = yaw
            return

        # Small change in yaw since previous message.
        # Normalize ONLY the incremental change.
        delta_yaw = self.normalize_angle(
            yaw - self.previous_yaw
        )

        self.current_yaw = yaw
        self.previous_yaw = yaw

        # --------------------------------------------------------
        # During ESCAPE, accumulate rotation.
        #
        # This prevents the ±pi wraparound problem that caused
        # the robot to spin forever.
        # --------------------------------------------------------

        if self.state == 'ESCAPE':
            self.escape_rotation += abs(delta_yaw)

    # ============================================================
    # LiDAR callback
    # ============================================================

    def lidar_callback(self, message):

        # --------------------------------------------------------
        # ESCAPE
        #
        # Once started, ignore the LiDAR stimulus until the
        # approximately 180 degree action is complete.
        # --------------------------------------------------------

        if self.state == 'ESCAPE':
            self.perform_escape()
            return

        # --------------------------------------------------------
        # Determine LiDAR orientation relative to base_link
        # --------------------------------------------------------

        try:

            transform = self.tf_buffer.lookup_transform(
                'base_link',
                message.header.frame_id,
                rclpy.time.Time()
            )

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException
        ):

            self.get_logger().warn(
                'Could not find TF from LiDAR frame '
                f'{message.header.frame_id} to base_link.'
            )

            return

        # --------------------------------------------------------
        # Get yaw of the LiDAR frame relative to base_link
        # --------------------------------------------------------

        q = transform.transform.rotation

        sin_yaw = 2.0 * (
            q.w * q.z +
            q.x * q.y
        )

        cos_yaw = 1.0 - 2.0 * (
            q.y * q.y +
            q.z * q.z
        )

        lidar_yaw = math.atan2(
            sin_yaw,
            cos_yaw
        )

        if not self.lidar_transform_logged:

            self.get_logger().info(
                'LiDAR orientation relative to base_link: '
                f'{math.degrees(lidar_yaw):.1f} degrees'
            )

            self.lidar_transform_logged = True

        # --------------------------------------------------------
        # Find closest obstacle on LEFT and RIGHT of robot
        #
        # IMPORTANT:
        #
        # base_link:
        #     0°   = FRONT
        #   +90°   = LEFT
        #   -90°   = RIGHT
        #
        # The LiDAR can have any orientation. TF converts its
        # scan angles into the base_link coordinate system.
        # --------------------------------------------------------

        left_distance = None
        right_distance = None

        front_angle = math.radians(FRONT_ANGLE)

        for i, distance in enumerate(message.ranges):

            if not math.isfinite(distance):
                continue

            lidar_angle = (
                message.angle_min +
                i * message.angle_increment
            )

            # Convert LiDAR ray angle into base_link angle
            robot_angle = self.normalize_angle(
                lidar_angle + lidar_yaw
            )

            # ----------------------------------------------------
            # Only consider the front ±30°
            # ----------------------------------------------------

            if abs(robot_angle) > front_angle:
                continue

            # ----------------------------------------------------
            # Left side of front
            # ----------------------------------------------------

            if robot_angle > math.radians(1.0):

                if (
                    left_distance is None
                    or distance < left_distance
                ):
                    left_distance = distance

            # ----------------------------------------------------
            # Right side of front
            # ----------------------------------------------------

            elif robot_angle < math.radians(-1.0):

                if (
                    right_distance is None
                    or distance < right_distance
                ):
                    right_distance = distance

            # ----------------------------------------------------
            # Very center of robot.
            #
            # Treat a centered obstacle as relevant to BOTH sides.
            # This prevents a wall/dead-center obstacle from being
            # incorrectly classified as purely left or right.
            # ----------------------------------------------------

            else:

                if (
                    left_distance is None
                    or distance < left_distance
                ):
                    left_distance = distance

                if (
                    right_distance is None
                    or distance < right_distance
                ):
                    right_distance = distance

        # --------------------------------------------------------
        # Determine whether obstacles are within 1 foot
        # --------------------------------------------------------

        left_obstacle = (
            left_distance is not None
            and left_distance <= OBSTACLE_DISTANCE
        )

        right_obstacle = (
            right_distance is not None
            and right_distance <= OBSTACLE_DISTANCE
        )

        # --------------------------------------------------------
        # ESCAPE:
        #
        # Both sides have obstacles and their distances are
        # roughly equal.
        # --------------------------------------------------------

        if (
            left_obstacle
            and right_obstacle
            and abs(left_distance - right_distance)
            <= SYMMETRY_TOLERANCE
        ):

            self.start_escape()
            return

        # --------------------------------------------------------
        # AVOID:
        #
        # Reflexively turn away from the closer obstacle.
        # This is NOT persistent. It will stop as soon as the
        # asymmetric obstacle condition disappears.
        # --------------------------------------------------------

        if left_obstacle or right_obstacle:

            # --------------------------------------------
            # Left obstacle is closer
            # -> Turn RIGHT
            # --------------------------------------------

            if (
                left_obstacle
                and (
                    not right_obstacle
                    or left_distance < right_distance
                )
            ):

                command = self.create_command(
                    linear_x=0.0,
                    angular_z=-AVOID_TURN_SPEED
                )

                self.get_logger().info(
                    f'AVOID: LEFT '
                    f'{left_distance:.3f} m '
                    f'-> RIGHT'
                )

            # --------------------------------------------
            # Right obstacle is closer
            # -> Turn LEFT
            # --------------------------------------------

            else:

                command = self.create_command(
                    linear_x=0.0,
                    angular_z=AVOID_TURN_SPEED
                )

                self.get_logger().info(
                    f'AVOID: RIGHT '
                    f'{right_distance:.3f} m '
                    f'-> LEFT'
                )

            self.publisher.publish(command)
            return

        # --------------------------------------------------------
        # NORMAL:
        # No obstacle within 1 foot.
        # --------------------------------------------------------

        #command = self.create_command(
        #    linear_x=FORWARD_SPEED,
        #   angular_z=0.0
        #)

        #self.publisher.publish(command)

    # ============================================================
    # Start ESCAPE behavior
    # ============================================================

    def start_escape(self):

        if self.current_yaw is None:

            self.get_logger().warn(
                'Cannot ESCAPE because odometry is unavailable.'
            )

            return

        self.state = 'ESCAPE'

        # Reset accumulated rotation
        self.escape_rotation = 0.0

        # Make sure our incremental odometry starts here
        self.previous_yaw = self.current_yaw

        self.get_logger().info(
            'ESCAPE: symmetric obstacle detected. '
            'Beginning fixed 180 degree turn.'
        )

        self.perform_escape()

    # ============================================================
    # Perform ESCAPE behavior
    # ============================================================

    def perform_escape(self):

        # --------------------------------------------------------
        # Continue turning until accumulated rotation reaches
        # approximately 180 degrees.
        # --------------------------------------------------------

        if self.escape_rotation < ESCAPE_ANGLE:

            command = self.create_command(
                linear_x=0.0,
                angular_z=-ESCAPE_TURN_SPEED
            )

            self.publisher.publish(command)

            return

        # --------------------------------------------------------
        # Escape complete
        # --------------------------------------------------------

        command = self.create_command(
            linear_x=0.0,
            angular_z=0.0
        )

        self.publisher.publish(command) #return to default state.

        self.state = 'NORMAL'
        self.escape_rotation = 0.0

        self.get_logger().info(
            'ESCAPE complete: '
            'approximately 180 degrees rotated.'
        )

    # ============================================================
    # Normalize an angle to [-pi, pi]
    # ============================================================

    @staticmethod
    def normalize_angle(angle):

        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle


# ================================================================
# Main
# ================================================================

def main(args=None):

    rclpy.init(args=args)

    node = ObstacleDetectionNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        # Stop the robot
        #stop_command = node.create_command(
        #    linear_x=0.0,
        #    angular_z=0.0
        #)

        #node.publisher.publish(stop_command)

        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()