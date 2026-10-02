import math
import random

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int32
from sensor_msgs.msg import LaserScan
from reactive_navigation.robot_state import State

#GLOBAL CONVERSION MACROS
ONE_FOOT = 0.3048
FRONT_ANGLE = 30.0 #We split left and right LiDAR by +-30 degrees

class ObstacleDetectionNode(Node):

    def __init__(self):
        super().__init__('osbtacle_detection_node')
        #Sub to /scan to read LiDAR info 
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
    def scan_callback(self, msg):
        # msg.ranges contains the distance measurements
        # msg.angle_min is the angle of ranges[0]
        # msg.angle_increment is the angular spacing

        pass

def main(args=None):
    pass
   

if __name__ == '__main__':

    main()