import math
import random

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int32, String
from ros_gz_interfaces.msg import Contacts
from reactive_navigation.robot_state import State


class CollisionDetection(Node):

    def __init__(self):
        super().__init__('collision_detection')
        self.get_logger().info('CollisionDetection Node Instantiated!')
        #subscriber to read bumper data from /bumper_contact
        self.subscription = self.create_subscription(
            Contacts,
            '/bumper_contact',
            self.contact_alert_callback,
            10) #10 is QoS history depth => size of msg queue
        self.subscription #prevent unused var

        #Publisher to write a 'bump!' notification to robot_brain
        self.publisher_ = self.create_publisher(String, 'bump_notifier', 10)

    #called to handle bumper contact. Will prime message to be sent to robot brain for handling.
    def contact_alert_callback(self, msg):
          msg = String()
          msg.data = 'bump!'
          self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args = args)
    #create node
    node = CollisionDetection()
    #spin up node 
    try:
            node.get_logger().info('CollisionDetection Node Spun Up!')
            rclpy.spin(node)
    except KeyboardInterrupt:
            pass
    finally:
            node.destroy_node()
            if rclpy.ok(): #if the context is cleaned up by this point, clean it up manually.
                rclpy.shutdown()
   

if __name__ == '__main__':

    main()