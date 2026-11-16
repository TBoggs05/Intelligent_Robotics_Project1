"""Tests for map-specific named waypoints and navigation goals."""

import math
from types import SimpleNamespace
from unittest.mock import Mock

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TransformStamped
import pytest
from rclpy.clock import ClockType
from rclpy.time import Time
from reactive_navigation import tour_guide_node
from reactive_navigation.tour_guide_node import TourGuide, waypoint_goal, WaypointStore
import yaml


@pytest.fixture
def map_path(tmp_path):
    """Provide a saved-map path without modifying the real map."""
    path = tmp_path / 'test_map.yaml'
    path.write_text('image: test_map.pgm\n')
    return path


def test_waypoint_round_trip(map_path):
    """Preserve position, heading, and previous locations across invocations."""
    original_map = map_path.read_text()
    store = WaypointStore(map_path)
    store.save('entrance', {'x': 1.2, 'y': -0.5, 'yaw': math.pi / 2})
    store = WaypointStore(map_path)
    store.save('desk', {'x': 2.0, 'y': 3.0, 'yaw': 0.0})
    reloaded = WaypointStore(map_path)
    assert reloaded.get('entrance') == {'x': 1.2, 'y': -0.5, 'yaw': math.pi / 2}
    assert reloaded.get('desk') == {'x': 2.0, 'y': 3.0, 'yaw': 0.0}
    assert map_path.read_text() == original_map
    assert store.path == map_path.with_suffix('.waypoints.yaml')


def test_waypoint_overwrite_requires_permission(map_path):
    """Prevent accidental replacement of a previously recorded location."""
    store = WaypointStore(map_path)
    pose = {'x': 0.0, 'y': 1.0, 'yaw': 0.0}
    store.save('desk', pose)
    with pytest.raises(ValueError, match='--overwrite'):
        store.save('desk', {'x': 2.0, 'y': 1.0, 'yaw': 0.0})
    assert WaypointStore(map_path).get('desk') == pose
    store.save('desk', {'x': 2.0, 'y': 1.0, 'yaw': 0.0}, overwrite=True)
    assert WaypointStore(map_path).get('desk')['x'] == 2.0


@pytest.mark.parametrize('pose', [
    {'x': float('nan'), 'y': 0.0, 'yaw': 0.0},
    {'x': 0.0, 'y': float('inf'), 'yaw': 0.0},
    {'x': True, 'y': 0.0, 'yaw': 0.0},
    {'x': 0.0, 'y': 0.0},
])
def test_invalid_waypoint_pose(map_path, pose):
    """Reject malformed or nonfinite locations before writing a file."""
    store = WaypointStore(map_path)
    with pytest.raises(ValueError):
        store.save('desk', pose)
    assert not store.path.exists()


def test_wrong_map_is_rejected(map_path):
    """Do not silently use locations recorded in a different map frame."""
    store = WaypointStore(map_path)
    store.path.write_text(yaml.safe_dump({
        'map': '/another/map.yaml', 'frame_id': 'map', 'locations': {},
    }))
    with pytest.raises(ValueError, match='different map'):
        WaypointStore(map_path)


def test_unknown_location(map_path):
    """Fail clearly for a name that has not been saved."""
    with pytest.raises(ValueError, match='Unknown location'):
        WaypointStore(map_path).get('missing')


def test_waypoint_goal_preserves_heading():
    """Use map coordinates and a normalized yaw quaternion for Nav2."""
    goal = waypoint_goal({'x': 1.0, 'y': -2.0, 'yaw': math.pi / 2})
    assert goal.header.frame_id == 'map'
    assert goal.pose.position.x == 1.0
    assert goal.pose.position.y == -2.0
    assert goal.pose.orientation.z == pytest.approx(math.sqrt(0.5))
    assert goal.pose.orientation.w == pytest.approx(math.sqrt(0.5))


@pytest.mark.parametrize('status', [
    GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED,
])
def test_navigation_completion_status(monkeypatch, status):
    """Only report arrival when Nav2 actually reports success."""
    node = Mock()
    monkeypatch.setattr(TourGuide, 'current_pose', Mock(return_value={
        'x': 0.0, 'y': 0.0, 'yaw': 0.0,
    }))
    node.navigator.wait_for_server.return_value = True
    node.navigator.send_goal_async.return_value.done.return_value = True
    node.get_clock.return_value.now.return_value.to_msg.return_value = Time().to_msg()
    handle = node.navigator.send_goal_async.return_value.result.return_value
    handle.accepted = True
    handle.get_result_async.return_value.done.return_value = True
    handle.get_result_async.return_value.result.return_value.status = status
    monkeypatch.setattr(tour_guide_node.rclpy, 'spin_until_future_complete', Mock())
    pose = {'x': 1.0, 'y': 2.0, 'yaw': 0.0}
    if status == GoalStatus.STATUS_SUCCEEDED:
        TourGuide.navigate(node, 'desk', pose, 1.0)
        assert 'Arrived' in node.get_logger.return_value.info.call_args.args[0]
    else:
        with pytest.raises(RuntimeError, match='did not succeed'):
            TourGuide.navigate(node, 'desk', pose, 1.0)
    sent = node.navigator.send_goal_async.call_args.args[0]
    assert sent.pose.header.frame_id == 'map'
    assert sent.pose.pose.position.x == 1.0


def test_navigation_rejection(monkeypatch):
    """Rejected goals must not be reported as successful navigation."""
    node = Mock()
    node.navigator.wait_for_server.return_value = True
    node.get_clock.return_value.now.return_value.to_msg.return_value = Time().to_msg()
    node.navigator.send_goal_async.return_value.done.return_value = True
    node.navigator.send_goal_async.return_value.result.return_value.accepted = False
    monkeypatch.setattr(TourGuide, 'current_pose', Mock(return_value={
        'x': 0.0, 'y': 0.0, 'yaw': 0.0,
    }))
    monkeypatch.setattr(tour_guide_node.rclpy, 'spin_until_future_complete', Mock())
    with pytest.raises(RuntimeError, match='rejected'):
        TourGuide.navigate(node, 'desk', {'x': 1.0, 'y': 2.0, 'yaw': 0.0}, 1.0)


def test_current_pose_and_staleness(monkeypatch):
    """Save the map-frame robot pose, but refuse stale transforms."""
    node = Mock()
    node.base_frame = 'base_link'
    transform = TransformStamped()
    transform.header.stamp = Time(seconds=100).to_msg()
    transform.transform.translation.x = 1.0
    transform.transform.translation.y = -2.0
    transform.transform.rotation.z = math.sin(math.pi / 4)
    transform.transform.rotation.w = math.cos(math.pi / 4)
    node.tf_buffer.lookup_transform.return_value = transform
    node.get_clock.return_value.now.return_value = Time(seconds=101, clock_type=ClockType.ROS_TIME)
    monkeypatch.setattr(tour_guide_node.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(tour_guide_node.rclpy, 'spin_once', Mock())
    pose = TourGuide.current_pose(node, 1.0)
    assert pose == pytest.approx({'x': 1.0, 'y': -2.0, 'yaw': math.pi / 2})
    node.get_clock.return_value.now.return_value = Time(seconds=110, clock_type=ClockType.ROS_TIME)
    ticks = iter([0.0, 0.0, 2.0])
    monkeypatch.setattr(tour_guide_node.time, 'monotonic', lambda: next(ticks))
    with pytest.raises(RuntimeError, match='stale'):
        TourGuide.current_pose(node, 1.0)


def test_cancel_only_own_goal(monkeypatch):
    """Cancel the accepted goal handle rather than all Nav2 goals."""
    node = Mock()
    node.goal_future = Mock()
    node.goal_handle.accepted = True
    future = node.goal_handle.cancel_goal_async.return_value
    future.done.return_value = True
    future.result.return_value = SimpleNamespace(goals_canceling=[object()])
    monkeypatch.setattr(tour_guide_node.rclpy, 'spin_until_future_complete', Mock())
    TourGuide.cancel_navigation(node)
    node.goal_handle.cancel_goal_async.assert_called_once()
    assert 'accepted' in node.get_logger.return_value.info.call_args.args[0]


def test_list_is_offline(map_path, capsys):
    """List saved locations without initializing ROS or requiring Nav2."""
    WaypointStore(map_path).save('desk', {'x': 1.0, 'y': 2.0, 'yaw': 0.0})
    assert tour_guide_node.main(['--map', str(map_path), 'list']) == 0
    assert 'desk: x=1.000, y=2.000' in capsys.readouterr().out
