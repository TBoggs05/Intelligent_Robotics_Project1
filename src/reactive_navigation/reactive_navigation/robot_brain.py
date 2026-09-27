import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from reactive_navigation.robot_state import State


class RobotBrain(Node):
    """Central robot brain.

    Keep the keyboard input handling in this file, but leave clear sections for
    the other team members to add their own sensor and navigation logic.
    """

    def __init__(self):
        super().__init__('robot_brain')

        # Default state: robot can drive forward unless a higher-priority behavior takes over.
        self.state = State.DRIVE_FORWARD
        self.keyboard_input = None

        # --------------------------------------------------------------------
        # KEYBOARD CONTROLLER SECTION (SUBSCRIPTIONS AND PUBLISHERS)
        # --------------------------------------------------------------------
        self.keyboard_sub = self.create_subscription(
            Twist,
            'keyboard_input',
            self.keyboard_callback,
            10,
        )

        # --------------------------------------------------------------------
        # FINAL MOTOR OUTPUT (PUBLISHER)
        # --------------------------------------------------------------------
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    # ------------------------------------------------------------------------
    # KEYBOARD CONTROLLER SECTION
    # ------------------------------------------------------------------------
    def keyboard_callback(self, msg: Twist):
        """Store the most recent keyboard command."""
        self.keyboard_input = msg
        self.state = State.HUMAN_CONTROLLING

    def handle_keyboard_control(self):
        """Return the active keyboard command when human control is active."""
        if self.keyboard_input is not None:
            return self.keyboard_input
        return None

    # ------------------------------------------------------------------------
    # COLLISION DETECTION SECTION
    # ------------------------------------------------------------------------
    # TODO: Add collision callback(s) and collision state logic.
    # Expected behavior:
    #   - subscribe to bumper/contact sensor topics
    #   - set self.state = State.COLLIDING when a collision is detected
    #   - publish a zero Twist to stop the robot immediately

    def handle_collision(self):
        """Placeholder for collision logic."""
        # TODO: implement collision detection and stop behavior here.
        return False

    # ------------------------------------------------------------------------
    # OBSTACLE DETECTION SECTION
    # ------------------------------------------------------------------------
    # TODO: Add obstacle detection callback(s) and avoidance logic.
    # Expected behavior:
    #   - subscribe to lidar / obstacle sensor topics
    #   - detect nearby obstacles
    #   - set the robot state to ESCAPE_SYMMETRIC or AVOID_ASYMMETRIC when appropriate

    def handle_obstacles(self):
        """Placeholder for obstacle logic."""
        # TODO: implement obstacle avoidance logic here.
        return False

    # ------------------------------------------------------------------------
    # RANDOM TURN SECTION
    # ------------------------------------------------------------------------
    # TODO: Add odometry-based random turn behavior.
    # Expected behavior:
    #   - track forward travel distance
    #   - trigger a random turn after a distance threshold
    #   - set self.state = State.TURN_RANDOMLY when appropriate

    def handle_random_turn(self):
        """Placeholder for random-turn logic."""
        # TODO: implement random turn logic here.
        return False

    # ------------------------------------------------------------------------
    # MAIN CONTROL LOOP
    # ------------------------------------------------------------------------
    def update(self):
        """Apply the current priority logic for keyboard-driven behavior.

        Priority order should eventually be:
            1. collision stop
            2. human control
            3. symmetric obstacle escape
            4. asymmetric obstacle avoidance
            5. random turn
            6. default forward motion
        """
        # Lower enum value means higher priority.
        if self.state == State.COLLIDING:
            self.publish_twist(Twist())
            return

        keyboard_cmd = self.handle_keyboard_control()
        if keyboard_cmd is not None:
            self.publish_twist(keyboard_cmd)
            return

        # TODO: Add placeholder checks for obstacle handling and random turns here.
        # For now, the robot falls back to a simple forward command.

        default_msg = Twist()
        default_msg.linear.x = 0.5
        self.publish_twist(default_msg)

    def publish_twist(self, msg: Twist):
        """Send the selected Twist to the robot."""
        self.cmd_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotBrain()

    try:
        while rclpy.ok():
            node.update()
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
