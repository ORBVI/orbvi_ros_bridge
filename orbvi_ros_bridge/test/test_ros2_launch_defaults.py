#!/usr/bin/env python3
"""Resolve real launch defaults in order without starting hardware clients."""
from pathlib import Path
import unittest

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
USER_LAUNCHES = (
    "orbvi_ros_bridge.launch.py",
    "orbvi_ros_bridge_mid360.launch.py",
    "orbvi_ros_bridge_pano.launch.py",
)


class Ros2LaunchDefaultsTest(unittest.TestCase):
    def test_tf_defaults_resolve_with_and_without_overrides(self):
        cases = (
            ({}, "true", "true"),
            ({"publish_vio_tf": "false"}, "false", "false"),
            ({"publish_sensor_tf": "false"}, "true", "false"),
            ({"publish_vio_tf": "false", "publish_sensor_tf": "true"}, "false", "true"),
        )
        for filename in USER_LAUNCHES:
            for overrides, expected_vio, expected_sensor in cases:
                with self.subTest(launch=filename, overrides=overrides):
                    context = LaunchContext()
                    context.launch_configurations.update(overrides)
                    source = PythonLaunchDescriptionSource(
                        str(PACKAGE_ROOT / "launch" / "ros2" / filename))
                    description = source.get_launch_description(context)
                    # Match launch action execution order. Only evaluate declarations:
                    # no Nodes, process creation, network or board connection.
                    for action in description.entities:
                        if isinstance(action, DeclareLaunchArgument):
                            action.execute(context)
                    self.assertEqual(context.launch_configurations["publish_vio_tf"], expected_vio)
                    self.assertEqual(context.launch_configurations["publish_sensor_tf"], expected_sensor)


if __name__ == "__main__":
    unittest.main()
