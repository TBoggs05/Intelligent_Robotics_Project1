from irobot_create_msgs.msg import HazardDetection, HazardDetectionVector
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool


class CollisionDetection(Node):
    """Publish whether the Create 3 reports a physical bumper collision."""

    def __init__(self):
        super().__init__('collision_detection')
        self.subscription = self.create_subscription(
            HazardDetectionVector,
            '/hazard_detection',
            self.contact_alert_callback,
            10,
        )
        self.publisher_ = self.create_publisher(
            Bool,
            '/collision_detected',
            10,
        )

    def contact_alert_callback(self, msg: HazardDetectionVector):
        """Publish true only when the hazard vector contains a BUMP event."""
        collision = Bool()
        collision.data = any(
            detection.type == HazardDetection.BUMP
            for detection in msg.detections
        )
        self.publisher_.publish(collision)


def main(args=None):
    rclpy.init(args=args)
    node = CollisionDetection()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()