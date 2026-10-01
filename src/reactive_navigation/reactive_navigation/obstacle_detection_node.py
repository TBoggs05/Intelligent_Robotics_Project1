import math
import random

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int32

from reactive_navigation.robot_state import State


class ObstacleDetectionNode(Node):

    def __init__(self):
        super().__init__('osbtacle_detection_node')
        
def main(args=None):
    pass
   

if __name__ == '__main__':

    main()