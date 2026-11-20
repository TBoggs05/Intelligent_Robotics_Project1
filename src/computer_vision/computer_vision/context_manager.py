import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json


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

        # Publish interpreted context
        self.context_pub = self.create_publisher(
            String,
            '/robot_context',
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
