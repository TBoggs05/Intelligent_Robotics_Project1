from robot_brain import *

def main(args=None):
    rclpy.init(args=args)

    robot_brain = RobotBrain()

    rclpy.spin(robot_brain) #spin up the connection

    #destroy node explicity
    robot_brain.destroy_node()
    rclpy.shutdown()

if __name__ == 'main':
    main()