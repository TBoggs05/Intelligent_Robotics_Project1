"""Save named map poses and navigate to them using Nav2."""

import argparse
import math
from pathlib import Path
import sys
import tempfile
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from tf2_ros import Buffer, TransformException, TransformListener
import yaml


class WaypointStore:
    """Keep named locations in a separate YAML file for one saved map."""

    def __init__(self, map_path):
        """Load locations belonging to the selected map."""
        self.map_path = Path(map_path).expanduser().resolve()
        if not self.map_path.is_file():
            raise ValueError(f'Map YAML does not exist: {self.map_path}')
        self.path = self.map_path.with_suffix('.waypoints.yaml')
        self.locations = {}
        if self.path.exists():
            with self.path.open() as stream:
                document = yaml.safe_load(stream)
            if not isinstance(document, dict) or document.get('frame_id') != 'map':
                raise ValueError(f'Invalid waypoint file: {self.path}')
            if document.get('map') != str(self.map_path):
                raise ValueError('Waypoint file belongs to a different map.')
            locations = document.get('locations')
            if not isinstance(locations, dict):
                raise ValueError('Waypoint locations must be a mapping.')
            for name, pose in locations.items():
                self._validate_name(name)
                self.locations[name] = self._validate_pose(pose)

    @staticmethod
    def _validate_name(name):
        """Reject empty names and surrounding whitespace."""
        if not isinstance(name, str) or not name.strip() or name != name.strip():
            raise ValueError('Location names must be nonempty without surrounding whitespace.')

    @staticmethod
    def _validate_pose(pose):
        """Require finite planar position and heading values."""
        if not isinstance(pose, dict) or set(pose) != {'x', 'y', 'yaw'}:
            raise ValueError('Each location must contain x, y, and yaw.')
        for value in pose.values():
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError('Waypoint coordinates must be finite numbers.')
            if not math.isfinite(value):
                raise ValueError('Waypoint coordinates must be finite numbers.')
        return {key: float(value) for key, value in pose.items()}

    def save(self, name, pose, overwrite=False):
        """Atomically persist a location without silently replacing a name."""
        self._validate_name(name)
        if name in self.locations and not overwrite:
            raise ValueError(f'Location {name!r} exists; use --overwrite to replace it.')
        locations = dict(self.locations)
        locations[name] = self._validate_pose(pose)
        document = {
            'map': str(self.map_path),
            'frame_id': 'map',
            'locations': locations,
        }
        temporary_path = None
        try:
            with tempfile.NamedTemporaryFile(
                mode='w', dir=self.path.parent, prefix=self.path.name + '.', delete=False,
            ) as stream:
                temporary_path = Path(stream.name)
                yaml.safe_dump(document, stream, sort_keys=True)
            temporary_path.replace(self.path)
        finally:
            if temporary_path is not None:
                temporary_path.unlink(missing_ok=True)
        self.locations = locations

    def get(self, name):
        """Return a named location or explain how to inspect saved names."""
        if name not in self.locations:
            raise ValueError(f'Unknown location {name!r}; use the list command.')
        return dict(self.locations[name])


def waypoint_goal(pose):
    """Convert a planar saved location into a map-frame navigation pose."""
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.pose.position.x = pose['x']
    goal.pose.position.y = pose['y']
    goal.pose.orientation.z = math.sin(pose['yaw'] / 2.0)
    goal.pose.orientation.w = math.cos(pose['yaw'] / 2.0)
    return goal


class TourGuide(Node):
    """Record localized poses and request named destinations from Nav2."""

    def __init__(self):
        """Create the TF listener and navigation action client."""
        super().__init__('tour_guide')
        self.declare_parameter('base_frame', 'base_link')
        self.base_frame = self.get_parameter('base_frame').value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.navigator = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_future = None
        self.goal_handle = None

    def current_pose(self, timeout):
        """Wait for a fresh map-to-robot transform before recording a pose."""
        deadline = time.monotonic() + timeout
        reason = 'No map-to-robot transform has been received.'
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                transform = self.tf_buffer.lookup_transform('map', self.base_frame, Time())
            except TransformException as error:
                reason = str(error)
                continue
            age = (self.get_clock().now() - Time.from_msg(transform.header.stamp))
            if abs(age.nanoseconds / 1e9) > 2.0:
                reason = 'The map-to-robot transform is stale or the clocks disagree.'
                continue
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            yaw = math.atan2(
                2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
                1.0 - 2.0 * (rotation.y ** 2 + rotation.z ** 2),
            )
            return {'x': translation.x, 'y': translation.y, 'yaw': yaw}
        raise RuntimeError(
            f'Cannot save a localized pose: {reason} '
            'Check AMCL and set 2D Pose Estimate in RViz.'
        )

    def navigate(self, name, pose, timeout):
        """Send one Nav2 goal and wait for its actual completion status."""
        self.current_pose(timeout)
        if not self.navigator.wait_for_server(timeout_sec=timeout):
            raise RuntimeError(
                'Nav2 navigate_to_pose is unavailable; check bt_navigator is active.'
            )
        goal = NavigateToPose.Goal()
        goal.pose = waypoint_goal(pose)
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        self.get_logger().info(f'Navigating to {name!r}. Press Ctrl-C to cancel.')
        self.goal_future = self.navigator.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, self.goal_future)
        if not self.goal_future.done():
            raise RuntimeError('Navigation goal acceptance was interrupted.')
        self.goal_handle = self.goal_future.result()
        if not self.goal_handle.accepted:
            raise RuntimeError(f'Nav2 rejected the goal for {name!r}.')
        result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        if not result_future.done():
            raise RuntimeError('Navigation result was interrupted.')
        status = result_future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f'Navigation to {name!r} did not succeed (status {status}).')
        self.get_logger().info(f'Arrived at {name!r}.')

    def cancel_navigation(self):
        """Request cancellation of this command's goal, never other clients' goals."""
        if self.goal_future is None:
            return
        if self.goal_handle is None:
            rclpy.spin_until_future_complete(self, self.goal_future, timeout_sec=2.0)
            if not self.goal_future.done():
                self.get_logger().error(
                    'Goal acceptance is unknown; check RViz and stop the robot if necessary.'
                )
                return
            self.goal_handle = self.goal_future.result()
        if not self.goal_handle.accepted:
            return
        future = self.goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.done() and future.result().goals_canceling:
            self.get_logger().info('Navigation cancellation accepted.')
        else:
            self.get_logger().error('Cancellation not confirmed; stop navigation from RViz.')


def command_parser():
    """Define the one-shot named-location commands."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', default='my_map.yaml', help='Map YAML (default: ./my_map.yaml)')
    parser.add_argument(
        '--timeout', type=float, default=15.0, help='Connection timeout in seconds',
    )
    commands = parser.add_subparsers(dest='command', required=True)
    save = commands.add_parser('save', help='Save the current localized robot pose')
    save.add_argument('name')
    save.add_argument('--overwrite', action='store_true')
    commands.add_parser('list', help='List saved locations without connecting to the robot')
    go = commands.add_parser('go', help='Navigate to a saved location and wait for arrival')
    go.add_argument('name')
    return parser


def main(args=None):
    """Run one save, list, or navigation command with clear exit status."""
    argv = sys.argv if args is None else ['tour_guide_node', *args]
    parser = command_parser()
    options = parser.parse_args(remove_ros_args(args=argv)[1:])
    if not math.isfinite(options.timeout) or options.timeout <= 0.0:
        parser.error('--timeout must be a positive finite number')
    node = None
    initialized = False
    try:
        store = WaypointStore(options.map)
        if options.command == 'list':
            if not store.locations:
                print(f'No locations saved for {store.map_path}.')
            for name, pose in sorted(store.locations.items()):
                print(f'{name}: x={pose["x"]:.3f}, y={pose["y"]:.3f}, '
                      f'yaw={math.degrees(pose["yaw"]):.1f} deg')
            return 0
        if options.command == 'go':
            pose = store.get(options.name)
        else:
            store._validate_name(options.name)
            if options.name in store.locations and not options.overwrite:
                raise ValueError(
                    f'Location {options.name!r} exists; use --overwrite to replace it.'
                )
        rclpy.init(args=argv, signal_handler_options=SignalHandlerOptions.NO)
        initialized = True
        node = TourGuide()
        if options.command == 'save':
            store.save(options.name, node.current_pose(options.timeout), options.overwrite)
            print(f'Saved {options.name!r} in {store.path}.')
        else:
            node.navigate(options.name, pose, options.timeout)
        return 0
    except KeyboardInterrupt:
        if node is not None and rclpy.ok():
            node.cancel_navigation()
        return 130
    except (OSError, ValueError, RuntimeError, yaml.YAMLError, ExternalShutdownException) as error:
        print(f'Error: {error}', file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if initialized and rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
