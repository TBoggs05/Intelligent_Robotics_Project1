import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import math
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException


class ContextManager(Node):

    def __init__(self):
        super().__init__('context_manager')

        # Receive object detections
        self.detection_sub = self.create_subscription(
            String,
            '/robot_visual_context',
            self.process_detections,
            10
        )

        # Recieve orientation and position from odom
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # Stores robot position on map
        self.robot_pose = None

        # Updates position every 0.1 sec
        self.pose_time = self.create_time(
            0.1,
            self.update_robot_pose
        )

        # Publish interpreted context
        self.context_pub = self.create_publisher(
            String,
            '/interpreted_context',
            10
        )

        self.get_logger().info("Context manager started")


    def get_position(self, x, image_width=250):
        # Determine horizontal object position
        relative_x = x / image_width

        if relative_x < 0.33:
            return "left"

        elif relative_x > 0.67:
            return "right"

        else:
            return "center"


    def process_detections(self, msg):
        # Convert JSON message back into a Python list
        detections = json.loads(msg.data)

        context = {
            "robot_pose":self.robot_pose,
            "objects": [],
            "object_count": len(detections),
            "summary": ""
        }

        # Process each detected object
        for detection in detections:

            name = detection["object"]
            confidence = detection["confidence"]

            x = detection["x"]
            y = detection["y"]

            width = detection["width"]
            height = detection["height"]

            # Determine relative image position
            position = self.get_position(x)

            # Calculate bounding box area
            area = width * height

            interpreted_object = {
                "name": name,
                "confidence": confidence,
                "position": position,
                "pixel_coordinates": [x, y],
                "bounding_box_area": area
            }

            context["objects"].append(interpreted_object)

        # Generate natural language description
        descriptions = []

        for obj in context["objects"]:

            descriptions.append(
                f"{obj['name']} on the {obj['position']}"
            )

        if descriptions:
            context["summary"] = (
                "The robot currently sees: "
                + ", ".join(descriptions)
            )

        else:
            context["summary"] = "No objects currently detected."

        # Publish interpreted context
        output = String()
        output.data = json.dumps(context)

        self.context_pub.publish(output)

    def update_robot_pose(self):
        try:
            # Get robot position relative to SLAM map
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                Time()
            )

            # Get map coords
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z

            # Get orientation quaternion where q=(x,y,z,w)
            q = transform.transform.rotation

            # Convert to radians
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)

            # Convert to degrees
            yaw_degrees = math.degrees(yaw)

            # Store robot map position
            self.robot_pose = {
                "x": round(x,3),
                "y": round(y,3),
                "z": round(z,3),
                "yaw": round(yaw_degrees,2),
                "frame": "map"
            }
        except TransformException:
            self.robot.pose = None



def main(args=None):
    rclpy.init(args=args)
    node = ContextManager()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
