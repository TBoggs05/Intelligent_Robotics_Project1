#!/usr/bin/env python3
"""ROS 2 keyboard controller node.

This node reads single-key terminal input and republishes it as a
`geometry_msgs/Twist` message on the `keyboard_input` topic. The robot brain or
other nodes can then consume that topic and convert it into final motor commands.
"""

import select
import sys

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

try:
    import termios
    import tty
except ImportError:
    termios = None
    tty = None


class KeyboardController(Node):
    """Publish Twist commands from terminal key presses to the /keyboard_input topic."""

    def __init__(self):
        # Initialize the ROS node. The node name is used in ROS logs and graph tools.
        super().__init__('keyboard_controller')

        # Keep output topic configurable for easy remapping into robot_brain.
        self.declare_parameter('output_topic', '/robot_brain/teleop_cmd')
        self.output_topic = self.get_parameter('output_topic').value

        # Publish commands as a Twist so any consumer can read linear and angular motion.
        self.publisher_ = self.create_publisher(Twist, self.output_topic, 10)

        # Poll for key presses repeatedly at a fixed interval.
        self.timer_ = self.create_timer(0.05, self.timer_callback)

        # Track terminal configuration so the original terminal state can be restored.
        self._terminal_ready = False
        self._original_terminal_settings = None

        # Map key presses to (linear_x, angular_z) command values.
        # This is the simplest possible teleop mapping for the robot.
        self._key_map = {
            'w': (1.0, 0.0),  # forward
            's': (-1.0, 0.0),  # reverse
            'a': (0.0, 1.0),   # turn left
            'd': (0.0, -1.0),  # turn right
            'x': (0.0, 0.0),   # stop
        }

        # Put the terminal in raw mode so single keystrokes are read immediately.
        self._configure_terminal()

    def _configure_terminal(self):
        # If there is no TTY attached, we cannot read keys from the terminal.
        if not sys.stdin.isatty() or termios is None or tty is None:
            self.get_logger().warn(
                'Keyboard input is not attached to a TTY; keyboard control will not read keys.'
            )
            return

        # Save the current terminal settings so we can restore them when the node exits.
        self._original_terminal_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        self._terminal_ready = True

    def _restore_terminal(self):
        # Restore standard terminal behavior when shutting down.
        if self._terminal_ready and self._original_terminal_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._original_terminal_settings)
            self._terminal_ready = False

    def timer_callback(self):
        # Ignore this callback if the process is not running in an interactive terminal.
        if not sys.stdin.isatty():
            return

        # Non-blocking check to see whether a key is ready.
        if not select.select([sys.stdin], [], [], 0)[0]:
            return

        # Read exactly one character from stdin.
        key = sys.stdin.read(1)

        # Ctrl-C is the usual way to stop the keyboard node.
        if key == '\x03':
            self.get_logger().info('Keyboard interrupt received; shutting down controller.')
            raise KeyboardInterrupt

        # Ignore keys that are not part of our control mapping.
        if key not in self._key_map:
            return

        # Convert the key to a Twist message.
        linear_x, angular_z = self._key_map[key]
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z

        # Publish the command so other nodes can react to it.
        self.publisher_.publish(msg)
        self.get_logger().info(
            f'Published keyboard command: key={key!r}, linear_x={linear_x}, angular_z={angular_z}'
        )

    def destroy_node(self):
        # Always restore the terminal state before shutting down.
        self._restore_terminal()
        super().destroy_node()


def main(args=None):
    # Standard ROS 2 node startup pattern.
    rclpy.init(args=args)
    node = KeyboardController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
