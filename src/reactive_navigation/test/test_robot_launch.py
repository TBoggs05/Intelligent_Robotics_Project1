"""Tests for saved-map launch defaults and command ownership."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
import pytest


@pytest.fixture
def robot_launch():
    """Load the physical robot launch module from source."""
    launch_path = Path(__file__).parents[1] / 'launch' / 'project2_robotlaunch.py'
    spec = importlib.util.spec_from_file_location('robot_launch', launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def launch_context(robot_launch, **overrides):
    """Resolve launch argument defaults without starting processes."""
    context = LaunchContext()
    context.launch_configurations.update(overrides)
    for action in robot_launch.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    return context


def test_default_reactive_mapping(robot_launch):
    """No map preserves reactive exploration and SLAM."""
    context = launch_context(robot_launch)
    assert context.launch_configurations['mapping'] == 'true'
    assert context.launch_configurations['reactive'] == 'true'
    assert context.launch_configurations['nav2'] == 'false'
    assert context.launch_configurations['rviz'] == 'false'
    assert robot_launch.configure_map_mode(context) == []


def test_saved_map_defaults(robot_launch, tmp_path, monkeypatch):
    """A map enables localization/navigation and expands a quoted home path."""
    map_path = tmp_path / 'my_map.yaml'
    map_path.touch()
    monkeypatch.setenv('HOME', str(tmp_path))
    context = launch_context(robot_launch, map='~/my_map.yaml')
    assert context.launch_configurations['mapping'] == 'false'
    assert context.launch_configurations['reactive'] == 'false'
    assert context.launch_configurations['nav2'] == 'true'
    assert context.launch_configurations['rviz'] == 'true'
    for action in robot_launch.configure_map_mode(context):
        action.execute(context)
    assert context.launch_configurations['map'] == str(map_path)


def test_saved_map_without_nav2_is_stationary(robot_launch, tmp_path):
    """Disabling navigation must not re-enable reactive driving on a map."""
    map_path = tmp_path / 'my_map.yaml'
    map_path.touch()
    context = launch_context(robot_launch, map=str(map_path), nav2='false')
    assert context.launch_configurations['reactive'] == 'false'
    assert context.launch_configurations['mapping'] == 'false'
    robot_launch.configure_map_mode(context)


def test_saved_map_can_enable_reactive_roaming(robot_launch, tmp_path):
    """Allow reactive driving between Nav2 goals when explicitly requested."""
    map_path = tmp_path / 'my_map.yaml'
    map_path.touch()
    context = launch_context(robot_launch, map=str(map_path), reactive='true')
    for action in robot_launch.configure_map_mode(context):
        action.execute(context)
    assert context.launch_configurations['map'] == str(map_path)
    assert context.launch_configurations['reactive'] == 'true'


@pytest.mark.parametrize('enabled', ['true', 'false'])
def test_dds_transport_override(robot_launch, enabled):
    """Use UDP-only transport unless the operator opts out."""
    context = launch_context(robot_launch, dds_udp_only=enabled)
    context.environment['FASTDDS_BUILTIN_TRANSPORTS'] = 'DEFAULT'
    for action in robot_launch.generate_launch_description().entities:
        if isinstance(action, SetEnvironmentVariable):
            action.visit(context)
    expected = 'UDPv4' if enabled == 'true' else 'DEFAULT'
    assert context.environment['FASTDDS_BUILTIN_TRANSPORTS'] == expected


def test_launch_disables_super_client_for_application_nodes(robot_launch):
    """Keep the ROS CLI daemon's super-client setting out of application nodes."""
    context = launch_context(robot_launch)
    context.environment['ROS_SUPER_CLIENT'] = 'True'
    for action in robot_launch.generate_launch_description().entities:
        if isinstance(action, SetEnvironmentVariable):
            action.visit(context)
    assert context.environment['ROS_SUPER_CLIENT'] == 'False'


@pytest.mark.parametrize('overrides, message', [
    ({'map': '/missing/map.yaml'}, 'Map YAML does not exist'),
    ({'map': '/some/map.yaml', 'mapping': 'true'}, 'mapping:=false'),
    ({'nav2': 'true'}, 'requires a saved map'),
])
def test_invalid_map_modes(robot_launch, overrides, message):
    """Reject missing files and conflicting localization or motor control."""
    context = launch_context(robot_launch, **overrides)
    with pytest.raises(ValueError, match=message):
        robot_launch.configure_map_mode(context)
