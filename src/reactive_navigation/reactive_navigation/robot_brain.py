import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String
from nav_msgs.msg import Odometry
from irobot_create_msgs.action import Undock
from rclpy.action import ActionClient
from reactive_navigation.robot_state import State
import math

class RobotBrain(Node):
    """Central robot brain.

    Keep the keyboard input handling in this file, but leave clear sections for
    the other team members to add their own sensor and navigation logic.
    """

    def __init__(self):
        super().__init__('robot_brain')

        # Default state: robot can drive forward unless a higher-priority behavior takes over.
        self.state = State.UNDOCKING
        self.keyboard_input = None
        self.undock_finished = False

        # Odom management. Keeps true current state for undocking
        self.current_x = 0
        self.current_y = 0
        self.current_yaw = 0
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )


        # --------------------------------------------------------------------
        # KEYBOARD CONTROLLER SECTION (SUBSCRIPTIONS AND PUBLISHERS)
        # --------------------------------------------------------------------
        self.keyboard_sub = self.create_subscription(
            Twist,
            'keyboard_input',
            self.keyboard_callback,
            10,
        )
        # --------------------------------------------------------------------
        # RANDOM TURN SECTION
        # --------------------------------------------------------------------
        self.random_turn_active = False
        self.random_turn_cmd = Twist()
        
        self.random_turn_active_sub = self.create_subscription(
            Bool,
            '/random_turn/active',
            self.random_turn_active_callback,
            10
        )
        
        self.random_turn_cmd_sub = self.create_subscription(
            Twist,
            '/random_turn/cmd',
            self.random_turn_cmd_callback,
            10
        )
        # --------------------------------------------------------------------
        # COLLISION DETECTION SECTION
        # --------------------------------------------------------------------
        self.collision_detection_sub = self.create_subscription(
            String,
            'bump_notifier',
            self.handle_collision,
            10
        )
        # --------------------------------------------------------------------
        # OBSTACLE AVOIDANCE SECTION
        # --------------------------------------------------------------------
        self.escape_command = Twist()
        self.obstacle_avoidance_sub = self.create_subscription(
            Twist,
            '/avoid_obstacles',
            self.handle_obstacles,
            10
        )
        self.obstacle_avoidance_end_sub = self.create_subscription(
                    Twist,
                    '/avoid_stop',
                    self.handle_obstacle_stop,
                    10
        )
        self.obstacle_command = None
        # --------------------------------------------------------------------
        # ROBOT STATE PUBLISHER
        #--------------------------------------------------------------------
        self.state_pub = self.create_publisher(
            Int32,
            '/robot_state',
            10
        )
        # --------------------------------------------------------------------
        # FINAL MOTOR OUTPUT (PUBLISHER)
        # --------------------------------------------------------------------
        self.cmd_vel_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)


    


    #--------------------------------
    #FUNCTION FOR EVAULATION TWISTS
    #---------------------------------
    def is_twist_zero(msg, tolerance=1e-6):
        t = msg.twist
        return (
            abs(t.linear.x) < tolerance
            and abs(t.linear.y) < tolerance
            and abs(t.linear.z) < tolerance
            and abs(t.angular.x) < tolerance
            and abs(t.angular.y) < tolerance
            and abs(t.angular.z) < tolerance
        )
    #ODOMETRY CALLBACK TO ASSIGN POSITIONS FOR UNDOCKING ROUTINE
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Extract quaternion orientation
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        # Convert quaternion to yaw
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)

        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
    # ------------------------------------------------------------------------
    # KEYBOARD CONTROLLER SECTION
    # ------------------------------------------------------------------------
    def keyboard_callback(self, msg: Twist):
        """Store the most recent keyboard command."""
        if(self.state.value > State.HUMAN_CONTROLLING.value):
            self.keyboard_input = msg
            self.state = State.HUMAN_CONTROLLING

    def handle_keyboard_control(self):
        """Return the active keyboard command when human control is active."""
        if self.keyboard_input is not None:
            return self.keyboard_input
        return None

    # ------------------------------------------------------------------------
    # COLLISION DETECTION SECTION
    # ------------------------------------------------------------------------
    def handle_collision(self, msg):
       # if(self.state.value >= State.COLLIDING.value): #higher value means low prio, so overtake.
            self.state = State.COLLIDING
            self.get_logger().warn('Collision Detected. Halting Movement and Overriding Program Priority.')
    
    # ------------------------------------------------------------------------
    # OBSTACLE DETECTION SECTION
    # ------------------------------------------------------------------------
    def handle_obstacles(self, msg):
        if(self.state.value > State.ESCAPE_SYMMETRIC.value):
            self.state = State.ESCAPE_SYMMETRIC
            self.obstacle_command = msg

    def handle_obstacle_stop(self, msg):
        if(msg == True and (self.state == State.ESCAPE_SYMMETRIC or self.state == State.AVOID_ASYMMETRIC)):
            self.state = State.DRIVE_FORWARD #return to default state after escape finishes


    # ------------------------------------------------------------------------
    # RANDOM TURN SECTION
    # ------------------------------------------------------------------------
    def random_turn_active_callback(self, msg):
        self.random_turn_active = msg.data
        
    def random_turn_cmd_callback(self, msg):
        self.random_turn_cmd = msg   

    def handle_random_turn(self):
        if self.random_turn_active:
            return self.random_turn_cmd
        
        return None
    
    # ================================================================
    # STATE PUBLISHER
    # ================================================================

    def publish_state(self):
        """
        Publish the behavior currently selected by the Robot Brain.
        """

        msg = Int32()

        msg.data = self.state.value

        self.state_pub.publish(msg)
    # ================================================================
    # HELPER FUNCTIONS
    # ================================================================
    def publish_twist(self, msg: Twist):
            """Convert internal Twist command to TwistStamped and send it"""
    
            stamped_msg = TwistStamped()
    
            stamped_msg.header.stamp = (
                self.get_clock().now().to_msg()
            )
    
            stamped_msg.twist = msg
    
            self.cmd_vel_pub.publish(stamped_msg)
    def normalize_angle(self, angle):
        """
        Normalize angle to [-pi, pi].
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle
    def stop_robot(self):
        msg = TwistStamped()

        msg.header.stamp = self.get_clock().now().to_msg()

        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.0
        self.get_logger().info('Stopping the robot!')
        self.cmd_vel_pub.publish(msg)
    def undock(self):
        if self.undock_finished == False:
            """
            Manually undock the robot.

            Sequence:
                1. Back away from the dock.
                2. Rotate 180 degrees.
                3. Resume normal driving.

            This does not use the /undock action, since that kept breaking...
            Instead, we hard code odom readings to ensure the robot leaves the dock.
            """

            # ============================================================
            # Configuration
            # ============================================================

            BACKUP_DISTANCE = 0.20       # meters
            BACKUP_SPEED = 0.10          # m/s

            TURN_ANGLE = math.pi         # 180 degrees
            TURN_SPEED = 0.8             # rad/s

            # ============================================================
            # Initialize undock sequence
            # ============================================================

            if not hasattr(self, 'manual_undock_started'):
                self.manual_undock_started = False
                self.manual_undock_start_x = None
                self.manual_undock_start_y = None
                self.manual_undock_start_yaw = None
                self.manual_undock_phase = "BACKUP"

            # ------------------------------------------------------------
            # Start sequence
            # ------------------------------------------------------------

            if not self.manual_undock_started:

                self.manual_undock_started = True

                self.manual_undock_phase = "BACKUP"

                # Save starting odometry position
                self.manual_undock_start_x = self.current_x
                self.manual_undock_start_y = self.current_y

                self.manual_undock_start_yaw = self.current_yaw

                self.get_logger().info(
                    "Manual undock started: backing away from dock..."
                )

            # ============================================================
            # BACK UP
            # ============================================================

            if self.manual_undock_phase == "BACKUP":

                dx = self.current_x - self.manual_undock_start_x
                dy = self.current_y - self.manual_undock_start_y

                distance = math.sqrt(dx * dx + dy * dy)

                if distance < BACKUP_DISTANCE:

                    msg = TwistStamped()

                    msg.header.stamp = self.get_clock().now().to_msg()

                    # Move backward
                    msg.twist.linear.x = -BACKUP_SPEED
                    msg.twist.angular.z = 0.0

                    self.cmd_vel_pub.publish(msg)

                    return

                # We have backed up far enough
                self.stop_robot()

                self.get_logger().info(
                    "Backup complete. Beginning 180 degree turn..."
                )

                # Save the yaw at the START of the turn
                self.manual_undock_start_yaw = self.current_yaw

                self.manual_undock_phase = "TURN"

                return

            # ============================================================
            # Turn for 2 seconds
            # ============================================================

            if self.manual_undock_phase == "TURN":

                # Initialize timer the first time we enter TURN
                if not hasattr(self, 'manual_undock_turn_start'):
                    self.manual_undock_turn_start = self.get_clock().now()

                # Calculate elapsed time
                elapsed = (
                    self.get_clock().now() - self.manual_undock_turn_start
                ).nanoseconds / 1e9

                # --------------------------------------------------------
                # Keep turning for 2 seconds
                # --------------------------------------------------------

                if elapsed < 2.0:

                    msg = TwistStamped()

                    msg.header.stamp = self.get_clock().now().to_msg()

                    msg.twist.linear.x = 0.0

                    # Rotate counter-clockwise
                    msg.twist.angular.z = TURN_SPEED

                    self.cmd_vel_pub.publish(msg)

                    return

            # --------------------------------------------------------
            # Finished turning
            # --------------------------------------------------------

            self.stop_robot()

            self.get_logger().info(
                "Manual undock complete. Resuming normal navigation."
            )

            self.undock_finished = True
            self.state = State.DRIVE_FORWARD

            return
    # ------------------------------------------------------------------------
    # MAIN CONTROL LOOP
    # ------------------------------------------------------------------------
    def update(self):
        """Apply the current priority logic for keyboard-driven behavior.

        Priority order should eventually be:
            0. undocking sequence* (should run before main loop)
            1. collision stop
            2. human control
            3. symmetric obstacle escape
            4. asymmetric obstacle avoidance
            5. random turn
            6. default forward motion
            where lower value => higher priority
        """
        self.undock() #undock before beginning true routine; 

        if self.state == State.COLLIDING:
            self.stop_robot()
            return

        elif self.state == State.HUMAN_CONTROLLING:
            keyboard_cmd = self.handle_keyboard_control()
            if keyboard_cmd is not None:
                self.publish_twist(keyboard_cmd)
                return

        elif self.state == State.AVOID_ASYMMETRIC:
          #  self.publish_twist(self.obstacle_command)
            pass
        elif self.state == State.ESCAPE_SYMMETRIC:
           # self.publish_twist(self.obstacle_command)
            pass
        elif self.state == State.TURN_RANDOMLY:
            pass

        # Default behaviour (Drive forward)
        if self.state == State.DRIVE_FORWARD:
            self.publish_state()
            default_msg = Twist()
            default_msg.linear.x = 0.5
            default_msg.angular.z = 0.0
            self.publish_twist(default_msg)

        #Current Random Turn Code (NEEDS TO BE REFACTORED INTO PRIORITY SCHEME)
        random_cmd = self.handle_random_turn() 
        
        if random_cmd is not None: #if a random_command is due:
            self.state = State.TURN_RANDOMLY    #update state
            self.publish_state()                #publish state
            self.publish_twist(                 #send random_turn command       
                random_cmd
            )
            return

        
def main(args=None):
    rclpy.init(args=args)
    node = RobotBrain()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): #fixes shutdown error. add to other nodes if error occurs
            rclpy.shutdown()


if __name__ == '__main__':
    main()
