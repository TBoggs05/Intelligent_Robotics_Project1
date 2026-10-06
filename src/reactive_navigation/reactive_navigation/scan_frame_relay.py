"""
Republish LiDAR scans in a canonical frame present in the TF tree.

In Gazebo Harmonic simulation, the TurtleBot 4 RPLiDAR sensor plugin
publishes raw scans stamped with 'turtlebot4/rplidar_link/rplidar'. The static
transform publisher bridging this frame to 'rplidar_link' can have timing
races or disconnects with SLAM and obstacle detection listeners.

Because the sensor physical origin coincides with 'rplidar_link' (as defined
in the TurtleBot 4 URDF and robot_state_publisher TF tree), this node relays
the LaserScan data with header.frame_id set to 'rplidar_link' on both
'/scan_mapping' and '/scan'. This ensures reliable TF lookups for
slam_toolbox and obstacle_detection_node.

Subscribed Topics:
    /scan_raw (sensor_msgs/LaserScan): Raw scan from ros_gz_bridge.

Published Topics:
    /scan_mapping (sensor_msgs/LaserScan): Relayed scan with canonical frame.
    /scan (sensor_msgs/LaserScan): Canonical scan on standard topic.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanFrameRelay(Node):
    """
    Relays LaserScan messages with corrected frame_id.

    Re-stamps incoming LaserScan headers with the canonical URDF link
    frame_id (default: 'rplidar_link') and republishes to output topics
    using sensor data QoS.
    """

    def __init__(self):
        """Initialize the ScanFrameRelay node, parameters, and publishers."""
        super().__init__('scan_frame_relay')

        self.declare_parameter('input_topic', '/scan_raw')
        self.declare_parameter('output_topic', '/scan_mapping')
        self.declare_parameter('frame_id', 'rplidar_link')

        self.frame_id = self.get_parameter('frame_id').value
        self.output_topic = self.get_parameter('output_topic').value
        self.first_scan_logged = False

        self.publisher = self.create_publisher(
            LaserScan,
            self.output_topic,
            qos_profile_sensor_data,
        )

        self.scan_publisher = self.create_publisher(
            LaserScan,
            '/scan',
            qos_profile_sensor_data,
        )

        input_topic = self.get_parameter('input_topic').value
        self.create_subscription(
            LaserScan,
            input_topic,
            self.scan_callback,
            qos_profile_sensor_data,
        )
        if input_topic != '/scan_raw':
            self.create_subscription(
                LaserScan,
                '/scan_raw',
                self.scan_callback,
                qos_profile_sensor_data,
            )

    def scan_callback(self, msg: LaserScan):
        """Update frame_id and republish the scan on both output topics."""
        if not self.first_scan_logged:
            self.get_logger().info(
                f'Received LiDAR scan on {msg.header.frame_id}; '
                f'republishing on {self.frame_id} -> '
                f'{self.output_topic} and /scan.'
            )
            self.first_scan_logged = True

        msg.header.frame_id = self.frame_id
        self.publisher.publish(msg)
        self.scan_publisher.publish(msg)


def main(args=None):
    """Run the scan frame relay node."""
    rclpy.init(args=args)
    node = ScanFrameRelay()
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