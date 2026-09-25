import rclpy
from rclpy import node
from geometry_msgs.msg import Twist
import robot_state
from robot_state import State
#Main functionality for simulated turtlebot4. Should handle movement, input,
#   obstacle avoidance, etc.
#PROJECT1 - Intelligent Robotics

class RobotBrain:
    def __init__(self):
        print('test')

    self.keyboard_input = None
    self.state = State.COLLIDING

    #Basic robot functionality, in order of priority

    def halt_on_collision():
        pass
    def handle_keyboard_input():
        pass
    def escape_symmetric_obstacles():
        pass
    def avoid_asymmetric_obstacles():
        pass
    def turn_randomly():
        pass
    def drive_forward():
        pass
