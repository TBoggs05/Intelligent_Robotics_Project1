import math
import random

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int32

from reactive_navigation.robot_state import State


class RandomTurn(Node):

    def __init__(self):
        super().__init__('random_turn')

        # Constants

        # 1 ft in meters
        self.one_foot = 0.3048

        # Turning speed
        self.turn_speed = 0.6


        # Current Robot Brain state
        self.current_state = State.DRIVE_FORWARD


        # Odometry tracking
        self.last_x = None
        self.last_y = None
        self.last_yaw = None

        self.distance_since_turn = 0.0


        # Random turn tracking
        self.turn_active = False

        self.turn_direction = 1

        self.target_angle = 0.0
        self.angle_turned = 0.0


        # Subscribers

        # Robot position/orientation
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Current behavior selected by robot_brain
        self.state_sub = self.create_subscription(
            Int32,
            '/robot_state',
            self.state_callback,
            10
        )


        # Publishers to robot_brain

        # Does Task 5 currently want control?
        self.active_pub = self.create_publisher(
            Bool,
            '/random_turn/active',
            10
        )

        # What velocity does Task 5 want?
        self.cmd_pub = self.create_publisher(
            Twist,
            '/random_turn/cmd',
            10
        )

        # Publish request 20 times per second
        self.timer = self.create_timer(
            0.05,
            self.publish_behavior
        )

        self.get_logger().info(
            'Random Turn started.'
        )


    # Receive current state from robot_brain

    def state_callback(self, msg):

        try:
            self.current_state = State(msg.data)

        except ValueError:
            self.get_logger().warning(
                f'Unknown robot state: {msg.data}'
            )



    # Receive odometry

    def odom_callback(self, msg):

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        # Convert quaternion to yaw
        siny_cosp = 2.0 * (
            q.w * q.z +
            q.x * q.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            q.y * q.y +
            q.z * q.z
        )

        yaw = math.atan2(
            siny_cosp,
            cosy_cosp
        )

        # First odometry reading
        if self.last_x is None:

            self.last_x = x
            self.last_y = y
            self.last_yaw = yaw

            return


        # Measure forward distance

        dx = x - self.last_x
        dy = y - self.last_y

        distance_moved = math.hypot(
            dx,
            dy
        )

        # Only count movement when Task 6 is actually active
        if (
            self.current_state == State.DRIVE_FORWARD
            and not self.turn_active
        ):

            self.distance_since_turn += distance_moved


        # Start Task 5 after 1 foot

        if (
            self.distance_since_turn >= self.one_foot
            and not self.turn_active
        ):

            self.start_random_turn()


        # Measure the turn

        # Only count rotation when robot_brain has actually
        # given control to Task 5.
        if (
            self.turn_active
            and self.current_state == State.TURN_RANDOMLY
        ):

            yaw_change = yaw - self.last_yaw

            # Normalize angle to [-pi, pi]
            yaw_change = math.atan2(
                math.sin(yaw_change),
                math.cos(yaw_change)
            )

            self.angle_turned += abs(
                yaw_change
            )

            # Turn finished
            if self.angle_turned >= self.target_angle:

                self.turn_active = False

                self.angle_turned = 0.0

                self.get_logger().info(
                    'Random turn complete.'
                )

        # Save current odometry for next callback
        self.last_x = x
        self.last_y = y
        self.last_yaw = yaw



    # Begin a new random turn

    def start_random_turn(self):

        # Uniformly sampled between -15 and +15 degrees
        angle_degrees = random.uniform(
            -15.0,
            15.0
        )

        angle_radians = math.radians(
            angle_degrees
        )

        # Positive = left
        # Negative = right
        if angle_degrees >= 0.0:
            self.turn_direction = 1
        else:
            self.turn_direction = -1

        self.target_angle = abs(
            angle_radians
        )

        self.angle_turned = 0.0
        self.turn_active = True

        # Start measuring the next foot after this turn
        self.distance_since_turn = 0.0

        self.get_logger().info(
            f'Random turn requested: '
            f'{angle_degrees:.2f} degrees'
        )



    # Publish Task 5 request to robot_brain
    
    def publish_behavior(self):

        active_msg = Bool()

        active_msg.data = self.turn_active

        self.active_pub.publish(
            active_msg
        )

        cmd = Twist()

        if self.turn_active:

            # Stop forward movement while turning
            cmd.linear.x = 0.0

            cmd.angular.z = (
                self.turn_direction *
                self.turn_speed
            )

        else:

            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)


def main(args=None):

    rclpy.init(args=args)

    node = RandomTurn()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()