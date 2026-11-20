# Baseline code was gathered by following this tutorial:
# https://medium.com/@aneja0512/integrating-yolov8-with-the-turtlebot4-camera-for-real-time-object-detection-using-ros-2-jazzy-d7082e2ba0a6

import json

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
from cv_bridge import CvBridge
from ultralytics import YOLO


class ObjectDetection(Node):
    def __init__(self):
        super().__init__('yolo_detector_viz')

        # Convert ROS Image messages into OpenCV images.
        self.bridge = CvBridge()

        # Load YOLOv8 small model.
        self.model = YOLO('yolov8s.pt')
        self.confidence_threshold = 0.65

        # Subscribe to the TurtleBot4 RGB camera feed.
        self.image_sub = self.create_subscription(
            Image,
            '/oakd/rgb/preview/image_raw',
            self.process_image,
            10
        )

        # Publish annotated camera images.
        self.viz_pub = self.create_publisher(
            Image, '/yolo/visualization', 10
        )

        # Publish object detections as a JSON string. A separate context
        # manager can subscribe to this topic and interpret the data.
        self.context_pub = self.create_publisher(
            String, '/robot_visual_context', 10
        )

        self.results = None
        self.get_logger().info('YOLO detector with visualization and detections started')

    def process_image(self, msg):
        # Convert incoming ROS image to OpenCV BGR frame.
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Run YOLO. The 65% cutoff applies to visualization and published data.
        self.results = self.model(
            cv_image, conf=self.confidence_threshold, verbose=False
        )

        # Draw bounding boxes and labels on the frame.
        annotated = self.results[0].plot()

        # Convert annotated OpenCV image back into a ROS Image message.
        msg_out = Image()
        msg_out.header = msg.header
        msg_out.height = annotated.shape[0]
        msg_out.width = annotated.shape[1]
        msg_out.encoding = 'bgr8'
        msg_out.is_bigendian = 0
        msg_out.step = annotated.shape[1] * 3
        msg_out.data = np.ascontiguousarray(annotated, dtype=np.uint8).tobytes()

        self.viz_pub.publish(msg_out)

        # Extract and publish detection data for the context manager.
        self.get_fov()

    def get_fov(self):
        detections = []

        for result in self.results:
            for box in result.boxes:
                class_id = int(box.cls[0])
                label = result.names[class_id]
                confidence = float(box.conf[0])

                if confidence < self.confidence_threshold:
                    continue

                # Bounding box center and size, measured in image pixels.
                x_center, y_center, width, height = box.xywh[0].tolist()

                detections.append({
                    'object': label,
                    'confidence': confidence,
                    'x': x_center,
                    'y': y_center,
                    'width': width,
                    'height': height
                })

        # Serialize the detection list as JSON and publish via ROS 2.
        # This does NOT write a .json file to disk.
        message = String()
        message.data = json.dumps(detections)
        self.context_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetection()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
