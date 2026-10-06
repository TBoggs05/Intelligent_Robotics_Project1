#!/usr/bin/env python3
"""
Keyboard teleoperation controller node (Priority Level 2).

Captures non-blocking single-keystroke user input from the terminal and
publishes velocity commands to the `/keyboard_input` topic. The RobotBrain
subscribes to this topic and grants it Priority Level 2 human override over
autonomous behaviors.

Controls:
    w: Drive forward (+0.3 m/s)
    s: Drive reverse (-0.3 m/s)
    a: Rotate counter-clockwise / left (+0.8 rad/s)
    d: Rotate clockwise / right (-0.8 rad/s)
    x / Space: Stop (0.0 m/s, 0.0 rad/s)
    Ctrl+C: Clean shutdown and restore terminal settings

Published Topics:
    /keyboard_input (geometry_msgs/Twist): User velocity command.
"""

import select
import sys

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

try:
    import termios
    import tty
except ImportError:
    termios = None
    tty = None


class KeyboardController(Node):
    """
    Publish Twist commands from terminal key presses to /keyboard_input.

    Puts the standard input terminal in cbreak mode to read single keypresses
    immediately without requiring the user to press Enter.
    """

    def __init__(self):
        """Initialize the keyboard controller node, mappings, and timer."""
        super().__init__('keyboard_controller')

        # Publish commands as Twist messages consumed by robot_brain
        self.publisher_ = self.create_publisher(
            Twist,
            '/keyboard_input',
            10,
        )

        # Poll for user key presses at 20 Hz
        self.timer_ = self.create_timer(0.05, self.timer_callback)

        # Terminal configuration tracking for graceful restoration on exit
        self._terminal_ready = False
        self._original_terminal_settings = None

        # Key mapping dictionary: key -> (linear_x m/s, angular_z rad/s)
        self._key_map = {
            'w': (0.3, 0.0),    # Forward
            's': (-0.3, 0.0),   # Reverse
            'a': (0.0, 0.8),    # Turn Left (CCW)
            'd': (0.0, -0.8),   # Turn Right (CW)
            'x': (0.0, 0.0),    # Stop
            ' ': (0.0, 0.0),    # Stop
        }

        # Configure stdin terminal in non-blocking cbreak mode
        self._configure_terminal()

        self.get_logger().info(
            'Keyboard Controller started.\n'
            'Controls: [w] forward, [s] reverse, [a] left, [d] right, '
            '[x/space] stop'
        )

    def _configure_terminal(self):
        """Configure stdin into cbreak mode for instantaneous key capture."""
        if not sys.stdin.isatty() or termios is None or tty is None:
            self.get_logger().warn(
                'Keyboard input is not attached to a TTY; '
                'keyboard control will not read keys.'
            )
            return

        self._original_terminal_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        self._terminal_ready = True

    def _restore_terminal(self):
        """Restore terminal attributes back to their original settings."""
        if self._terminal_ready and self._original_terminal_settings is not None:
            termios.tcsetattr(
                sys.stdin,
                termios.TCSADRAIN,
                self._original_terminal_settings,
            )
            self._terminal_ready = False

    def timer_callback(self):
        """Poll stdin for a keypress and publish corresponding Twist command."""
        if not sys.stdin.isatty():
            return

        # Non-blocking poll for available stdin data
        if not select.select([sys.stdin], [], [], 0)[0]:
            return

        key = sys.stdin.read(1)

        # Handle Ctrl+C gracefully
        if key == '\x03':
            self.get_logger().info(
                'Keyboard interrupt received; shutting down controller.'
            )
            raise KeyboardInterrupt

        if key not in self._key_map:
            return

        linear_x, angular_z = self._key_map[key]
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z

        self.publisher_.publish(msg)
        self.get_logger().info(
            f'Published keyboard command: key={key!r}, '
            f'linear_x={linear_x}, angular_z={angular_z}'
        )

    def destroy_node(self):
        """Restore terminal state before destroying the node."""
        self._restore_terminal()
        super().destroy_node()


def main(args=None):
    """Run the keyboard controller node."""
    rclpy.init(args=args)
    node = KeyboardController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
