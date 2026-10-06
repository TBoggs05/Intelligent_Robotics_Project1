"""
Bumper collision detection node (Priority Level 1).

Monitors the iRobot Create 3 base hazard detection vector and publishes
a boolean collision status. Specifically filters for physical bumper contact
(HazardDetection.BUMP) while ignoring non-collision hazards (e.g. cliff sensors
or wheel drops), providing the safety halt signal to robot_brain.

Subscribed Topics:
    /hazard_detection (irobot_create_msgs/HazardDetectionVector):
        Active hazard vector from Create 3 base.

Published Topics:
    /collision_detected (std_msgs/Bool):
        True if physical bumper contact is active, False otherwise.
"""

from irobot_create_msgs.msg import HazardDetection, HazardDetectionVector
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool


class CollisionDetection(Node):
    """
    Detects bumper collision hazards from the Create 3 base.

    Subscribes to /hazard_detection and checks each reported hazard.
    Publishes True on /collision_detected whenever a BUMP type hazard
    is active.
    """

    def __init__(self):
        """Initialize the CollisionDetection node, subscriptions, and publishers."""
        super().__init__('collision_detection')

        # Subscribe to the Create 3 base hazard detection vector
        self.subscription = self.create_subscription(
            HazardDetectionVector,
            '/hazard_detection',
            self.contact_alert_callback,
            10,
        )

        # Publish binary bumper collision signal consumed by robot_brain
        self.publisher_ = self.create_publisher(
            Bool,
            '/collision_detected',
            10,
        )

        self.get_logger().info('Collision Detection node initialized.')

    def contact_alert_callback(self, msg: HazardDetectionVector):
        """
        Process hazard detections and publish True if a bumper contact exists.

        Filters specifically for HazardDetection.BUMP to isolate physical
        obstacle impacts from other base hazards (cliff, wheel drop, etc.).

        :param msg: irobot_create_msgs/HazardDetectionVector from base.
        """
        collision = Bool()
        collision.data = any(
            detection.type == HazardDetection.BUMP
            for detection in msg.detections
        )
        self.publisher_.publish(collision)


def main(args=None):
    """Run the collision detection node."""
    rclpy.init(args=args)
    node = CollisionDetection()
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
