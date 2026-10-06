"""Unit tests for reactive navigation behaviors and priority hierarchy."""

import math

from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import HazardDetection, HazardDetectionVector
from nav_msgs.msg import Odometry
import pytest
import rclpy
from reactive_navigation.collision_detection import CollisionDetection
from reactive_navigation.obstacle_detection_node import ObstacleDetectionNode
from reactive_navigation.random_turn import RandomTurn
from reactive_navigation.robot_brain import RobotBrain
from reactive_navigation.robot_state import State
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


@pytest.fixture(scope='module', autouse=True)
def rclpy_init_shutdown():
    """Initialize rclpy once for the test module."""
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_state_enum_priority_order():
    """Verify that State enum values strictly reflect priority 1 to 6."""
    assert State.COLLIDING.value == 1
    assert State.HUMAN_CONTROLLING.value == 2
    assert State.ESCAPE_SYMMETRIC.value == 3
    assert State.AVOID_ASYMMETRIC.value == 4
    assert State.TURN_RANDOMLY.value == 5
    assert State.DRIVE_FORWARD.value == 6


def test_collision_detection_bump_only():
    """Verify CollisionDetection detects BUMP and ignores non-bumper hazards."""
    node = CollisionDetection()
    try:
        # Cliff detection should NOT trigger bumper collision
        cliff_msg = HazardDetectionVector()
        cliff_hazard = HazardDetection()
        cliff_hazard.type = HazardDetection.CLIFF
        cliff_msg.detections.append(cliff_hazard)

        published = []
        node.publisher_.publish = lambda m: published.append(m.data)
        node.contact_alert_callback(cliff_msg)
        assert published[-1] is False

        # Bumper contact MUST trigger collision
        bump_msg = HazardDetectionVector()
        bump_hazard = HazardDetection()
        bump_hazard.type = HazardDetection.BUMP
        bump_msg.detections.append(bump_hazard)

        node.contact_alert_callback(bump_msg)
        assert published[-1] is True
    finally:
        node.destroy_node()


def test_robot_brain_priority_hierarchy():
    """Verify RobotBrain arbitrates commands according to priority 1-6."""
    brain = RobotBrain()
    brain.undock_finished = True
    try:
        # Default: Drive Forward (Priority 6)
        brain.collision_detected = False
        brain.keyboard_input = None
        brain.obstacle_twist = None
        brain.random_turn_active = False

        brain.update()
        assert brain.state == State.DRIVE_FORWARD

        # Priority 5: Random Turn takes over Drive Forward
        brain.random_turn_active = True
        brain.random_turn_cmd = Twist()
        brain.random_turn_cmd.angular.z = 0.5
        brain.update()
        assert brain.state == State.TURN_RANDOMLY

        # Priority 4: Asymmetric Avoidance takes over Random Turn
        brain.obstacle_twist = Twist()
        brain.obstacle_twist.angular.z = 1.0
        brain.obstacle_twist_stamp = brain.get_clock().now()
        brain.obstacle_behavior_state = State.AVOID_ASYMMETRIC
        brain.obstacle_state_stamp = brain.get_clock().now()
        brain.update()
        assert brain.state == State.AVOID_ASYMMETRIC

        # Priority 3: Symmetric Escape takes over Random Turn
        brain.obstacle_behavior_state = State.ESCAPE_SYMMETRIC
        brain.update()
        assert brain.state == State.ESCAPE_SYMMETRIC

        # Priority 2: Keyboard teleop overrides obstacle behaviors
        key_twist = Twist()
        key_twist.linear.x = 0.3
        brain.keyboard_callback(key_twist)
        brain.update()
        assert brain.state == State.HUMAN_CONTROLLING

        # Priority 1: Bumper collision halts robot and overrides teleop
        bump_msg = Bool()
        bump_msg.data = True
        brain.collision_callback(bump_msg)
        brain.update()
        assert brain.state == State.COLLIDING
    finally:
        brain.destroy_node()


def test_obstacle_detection_symmetric_escape():
    """Verify symmetric obstacles trigger 180 +- 30 deg fixed action pattern escape."""
    node = ObstacleDetectionNode()
    try:
        node.current_yaw = 0.0
        node.previous_yaw = 0.0
        node.cached_lidar_yaw = 0.0

        scan = LaserScan()
        scan.header.frame_id = 'base_link'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.radians(1.0)
        scan.range_min = 0.1
        scan.range_max = 12.0
        num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1

        # Symmetric wall directly ahead: both left and right see 0.40m (< OBSTACLE_DISTANCE)
        dist = 0.40
        ranges = [float('inf')] * num_readings
        for i in range(num_readings):
            angle = scan.angle_min + i * scan.angle_increment
            if abs(angle) <= math.radians(25.0):
                ranges[i] = dist
        scan.ranges = ranges

        commands = []
        states = []
        node.publisher.publish = lambda m: commands.append(m)
        node.state_publisher.publish = lambda m: states.append(m.data)

        node.lidar_callback(scan)

        assert node.state == 'ESCAPE'
        assert states[-1] == State.ESCAPE_SYMMETRIC.value
        # Target angle must be within [150, 210] degrees (180 +- 30 deg)
        deg = math.degrees(node.target_escape_angle)
        assert 150.0 <= deg <= 210.0
    finally:
        node.destroy_node()


def test_obstacle_detection_asymmetric_avoidance():
    """Verify asymmetric obstacles trigger reflexive turn away from closer obstacle."""
    node = ObstacleDetectionNode()
    try:
        node.cached_lidar_yaw = 0.0
        node.current_yaw = 0.0
        node.previous_yaw = 0.0

        scan = LaserScan()
        scan.header.frame_id = 'base_link'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.radians(1.0)
        scan.range_min = 0.1
        scan.range_max = 12.0
        num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1

        # Obstacle on LEFT closer (0.35m) than RIGHT (0.48m)
        ranges = [float('inf')] * num_readings
        for i in range(num_readings):
            angle = scan.angle_min + i * scan.angle_increment
            if math.radians(5.0) <= angle <= math.radians(25.0):
                ranges[i] = 0.35  # left closer
            elif math.radians(-25.0) <= angle <= math.radians(-5.0):
                ranges[i] = 0.48  # right farther
        scan.ranges = ranges

        commands = []
        states = []
        node.publisher.publish = lambda m: commands.append(m)
        node.state_publisher.publish = lambda m: states.append(m.data)

        node.lidar_callback(scan)

        # Must turn RIGHT (negative angular z) away from closer left obstacle
        assert states[-1] == State.AVOID_ASYMMETRIC.value
        assert commands[-1].angular.z < 0.0

        # Obstacle on RIGHT closer (0.35m) than LEFT (0.48m)
        for i in range(num_readings):
            angle = scan.angle_min + i * scan.angle_increment
            if math.radians(5.0) <= angle <= math.radians(25.0):
                ranges[i] = 0.48  # left farther
            elif math.radians(-25.0) <= angle <= math.radians(-5.0):
                ranges[i] = 0.35  # right closer
        scan.ranges = ranges

        node.lidar_callback(scan)
        # Must turn LEFT (positive angular z) away from closer right obstacle
        assert states[-1] == State.AVOID_ASYMMETRIC.value
        assert commands[-1].angular.z > 0.0
    finally:
        node.destroy_node()


def test_random_turn_after_one_foot():
    """Verify RandomTurn triggers uniformly within +-15 deg after 1 ft forward."""
    node = RandomTurn()
    try:
        node.current_state = State.DRIVE_FORWARD

        # Initialize odometry at (0, 0)
        odom = Odometry()
        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.orientation.w = 1.0
        node.odom_callback(odom)

        assert node.distance_since_turn == 0.0
        assert not node.turn_active

        # Drive forward 0.20 m (less than 1 ft = 0.3048 m)
        odom.pose.pose.position.x = 0.20
        node.odom_callback(odom)
        assert not node.turn_active
        assert math.isclose(node.distance_since_turn, 0.20, abs_tol=1e-3)

        # Drive forward to 0.35 m (crosses 1 ft threshold)
        odom.pose.pose.position.x = 0.35
        node.odom_callback(odom)
        assert node.turn_active
        assert 0.0 <= math.degrees(node.target_angle) <= 15.0
    finally:
        node.destroy_node()
