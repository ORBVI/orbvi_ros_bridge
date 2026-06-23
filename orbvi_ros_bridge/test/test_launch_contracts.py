#!/usr/bin/env python3

import ast
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]

ROS1_LAUNCH_DEFAULTS = {
    "orbvi_ros_bridge.launch": "raw,rectified,imu,lidar,lidar_imu,disparity,depth,vio",
    "orbvi_ros_bridge_visual.launch": "raw,rectified,imu,disparity,depth,vio",
    "orbvi_ros_bridge_mid360.launch": "raw,rectified,imu,lidar,lidar_imu,disparity,depth,vio",
}

ROS2_LAUNCH_DEFAULTS = {
    "orbvi_ros_bridge.launch.py": "raw,rectified,imu,lidar,lidar_imu,disparity,depth,vio",
    "orbvi_ros_bridge_visual.launch.py": "raw,rectified,imu,disparity,depth,vio",
    "orbvi_ros_bridge_mid360.launch.py": "raw,rectified,imu,lidar,lidar_imu,disparity,depth,vio",
}

PANORAMA_ARGUMENT_DEFAULTS = {
    "pano_profile": "baseline",
    "pano_width": "2048",
    "pano_height": "1024",
    "pano_fov_half_deg": "95.0",
    "pano_seam_blend_px": "32",
    "pano_seam_mode": "fixed",
    "pano_dp_seam_band_px": "96",
    "pano_dp_seam_smoothness": "8.0",
    "pano_seam_avoidance_penalty": "220.0",
    "pano_blend": "multiband",
    "pano_photometric_align": "true",
    "pano_seam_ghost_suppression": "false",
    "pano_seam_ghost_threshold": "80.0",
}


def ros1_arg_defaults(path):
    root = ET.parse(path).getroot()
    return {
        element.attrib["name"]: element.attrib.get("default", "")
        for element in root.findall("arg")
    }


def ros1_param_value(path, name):
    root = ET.parse(path).getroot()
    for node in root.findall("node"):
        for param in node.findall("param"):
            if param.attrib.get("name") == name:
                return param.attrib.get("value", "")
    raise AssertionError(f"{path.name} does not set parameter {name}")


def ros1_rosparam_file(path):
    root = ET.parse(path).getroot()
    for node in root.findall("node"):
        for rosparam in node.findall("rosparam"):
            if rosparam.attrib.get("command") == "load":
                return rosparam.attrib.get("file", "")
    raise AssertionError(f"{path.name} does not load a rosparam file")


def ros2_argument_defaults(path):
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        if any(isinstance(target, ast.Name) and target.id == "ARGUMENT_DEFAULTS"
               for target in node.targets):
            return ast.literal_eval(node.value)
    raise AssertionError(f"{path.name} does not define ARGUMENT_DEFAULTS")


def yaml_scalar_defaults(path):
    values = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        key = key.strip()
        value = value.strip().strip("\"'")
        if not value or key in ("/**", "ros__parameters"):
            continue
        values[key] = value.lower() if value.lower() in ("true", "false") else value
    return values


class LaunchContractTest(unittest.TestCase):
    def test_ros1_launch_files_expose_streams_for_optional_pano(self):
        for launch_name, expected_streams in ROS1_LAUNCH_DEFAULTS.items():
            with self.subTest(launch_name=launch_name):
                path = PACKAGE_ROOT / "launch" / launch_name
                args = ros1_arg_defaults(path)
                self.assertEqual(args["streams"], expected_streams)
                self.assertEqual(ros1_param_value(path, "streams"), "$(arg streams)")
                self.assertEqual(
                    args["pano_config"],
                    "$(find orbvi_ros_bridge)/config/pano_mode2.yaml",
                )
                self.assertEqual(ros1_rosparam_file(path), "$(arg pano_config)")

    def test_ros2_launch_files_declare_pano_stream_and_options(self):
        for launch_name, expected_streams in ROS2_LAUNCH_DEFAULTS.items():
            with self.subTest(launch_name=launch_name):
                path = PACKAGE_ROOT / "launch" / "ros2" / launch_name
                defaults = ros2_argument_defaults(path)
                self.assertEqual(defaults["streams"], expected_streams)
                source = path.read_text(encoding="utf-8")
                self.assertIn('"pano_config"', source)
                self.assertIn('"pano_mode2_ros2.yaml"', source)
                self.assertIn('LaunchConfiguration("pano_config")', source)

    def test_default_pano_configs_use_mode2_blend_px_32(self):
        for config_name in ("pano_mode2.yaml", "pano_mode2_ros2.yaml"):
            with self.subTest(config_name=config_name):
                values = yaml_scalar_defaults(PACKAGE_ROOT / "config" / config_name)
                for name, expected_value in PANORAMA_ARGUMENT_DEFAULTS.items():
                    self.assertEqual(values[name], expected_value)


if __name__ == "__main__":
    unittest.main()
