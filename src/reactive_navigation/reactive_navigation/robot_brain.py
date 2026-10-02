import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String
from irobot_create_msgs.action import Undock
from rclpy.action import ActionClient
from reactive_navigation.robot_state import State


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
        # State management for Undock action
        self.undock_client = ActionClient(self, Undock, '/undock')
        self.undock_goal_sent = False
        self.undock_finished = False

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
        self.escape_command = TwistStamped()
        self.obstacle_avoidance_sub = self.create_subscription(
            TwistStamped,
            '/avoid_obstacles',
            self.handle_obstacles,
            10
        )
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

    # ------------------------------------------------------------------------
    # KEYBOARD CONTROLLER SECTION
    # ------------------------------------------------------------------------
    def keyboard_callback(self, msg: Twist):
        """Store the most recent keyboard command."""
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
    # TODO: Add collision callback(s) and collision state logic.
    # Expected behavior:
    #   - subscribe to bumper/contact sensor topics
    #   - set self.state = State.COLLIDING when a collision is detected
    #   - publish a zero Twist to stop the robot immediately

    def handle_collision(self, msg):
        self.state = State.COLLIDING
        self.get_logger().warn('Collision Detected. Halting Movement and Overriding Program Priority.')
    # ------------------------------------------------------------------------
    # OBSTACLE DETECTION SECTION
    # ------------------------------------------------------------------------
    # TODO: Add obstacle detection callback(s) and avoidance logic.
    # Expected behavior:
    #   - subscribe to lidar / obstacle sensor topics
    #   - detect nearby obstacles
    #   - set the robot state to ESCAPE_SYMMETRIC or AVOID_ASYMMETRIC when appropriate

    def handle_obstacles(self, msg):
        if(self.state.value > State.ESCAPE_SYMMETRIC.value): #if not in higher priority task, then escape
            self.state = State.ESCAPE_SYMMETRIC
            self.get_logger().warn('Obstacle ahead! Taking action.')
            self.publish_twist_stamped(self.escape_command)

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

    # ------------------------------------------------------------------------
    # MAIN CONTROL LOOP
    # ------------------------------------------------------------------------
    def update(self):
        """Apply the current priority logic for keyboard-driven behavior.

        Priority order should eventually be:
            1. collision stop
            2. human control
            3. symmetric obstacle escape
            4. asymmetric obstacle avoidance
            5. random turn
            6. default forward motion
        """
        # --- ABSOLUTE PRIORITY: Initial Undock Sequence ---
        if not self.undock_finished:
            if not self.undock_goal_sent:
                # Wait for the server, then send the undock goal
                if self.undock_client.wait_for_server(timeout_sec=0.1):
                    self.get_logger().info('Undock..')
                    self.undock_goal_sent = True
                    
                    # Send the goal. The robot will autonomously drive off the dock.
                    future = self.undock_client.send_goal_async(Undock.Goal())
                    
                    # Tell the script what to do when the robot finishes undocking
                    def done_callback(fut):
                        self.get_logger().info('Undocking finished!')
                        self.undock_finished = True
                    
                    # We link the callback directly here to keep it simple
                    future.add_done_callback(
                        lambda fut: fut.result().get_result_async().add_done_callback(done_callback)
                    )
                else:
                    self.get_logger().warn('Waiting for /undock action server to become active...', throttle_duration_sec=2.0)
            
            return # Block any other movements from overriding the undock sequence
        # -----------------------------------------------------
        #Program routine
        # Lower enum value means higher priority.
        if self.state == State.COLLIDING:
            halt_msg = Twist()
            halt_msg.linear.x = 0.0
            halt_msg.angular.z = 0.0
            self.publish_twist(halt_msg)
            return

        keyboard_cmd = self.handle_keyboard_control()
        if keyboard_cmd is not None:
            self.publish_twist(keyboard_cmd)
            return

        # TODO: Add placeholder checks for obstacle handling.
        # For now, the robot falls back to a simple forward command.
        if self.state == State.AVOID_ASYMMETRIC or self.state == State.ESCAPE_SYMMETRIC:
            pass


        random_cmd = self.handle_random_turn()
        
        if random_cmd is not None:
            self.state = State.TURN_RANDOMLY
            self.publish_state()
            self.publish_twist(
                random_cmd
            )
            return

        # Default behaviour (Drive forward)
        if self.state == State.DRIVE_FORWARD:
        #self.state = State.DRIVE_FORWARD

            self.publish_state()

            default_msg = Twist()
            default_msg.linear.x = 0.5
            default_msg.angular.z = 0.0

            self.publish_twist(default_msg)

    def publish_twist(self, msg: Twist):
        """Convert internal Twist command to TwistStamped and send it"""

        stamped_msg = TwistStamped()

        stamped_msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        stamped_msg.twist = msg

        self.cmd_vel_pub.publish(stamped_msg)

    def publish_twist_stamped(self, msg: TwistStamped):
        
            stamped_msg = TwistStamped()
        
            self.cmd_vel_pub.publish(stamped_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotBrain()

    try:
        while rclpy.ok():
            node.update()
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): #fixes shutdown error. add to other nodes if error occurs
            rclpy.shutdown()


if __name__ == '__main__':
    main()
