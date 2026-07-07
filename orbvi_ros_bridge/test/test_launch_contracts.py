#!/usr/bin/env python3

import ast
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PACKAGE_ROOT.parent
RATE_CHECK_SCRIPT = REPO_ROOT / "scripts" / "orbvi_ros_bridge_rate_check.sh"

VISUAL_STREAMS = "raw,rectified,imu,disparity,depth,vio"
MID360_STREAMS = "raw,rectified,imu,lidar,lidar_imu,disparity,depth,vio"
PANO_STREAMS = "raw,rectified,pano,imu,disparity,depth,vio"

ROS1_USER_LAUNCHES = {
    "orbvi_ros_bridge.launch": VISUAL_STREAMS,
    "orbvi_ros_bridge_mid360.launch": MID360_STREAMS,
    "orbvi_ros_bridge_pano.launch": PANO_STREAMS,
}

ROS2_USER_LAUNCHES = {
    "orbvi_ros_bridge.launch.py": VISUAL_STREAMS,
    "orbvi_ros_bridge_mid360.launch.py": MID360_STREAMS,
    "orbvi_ros_bridge_pano.launch.py": PANO_STREAMS,
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
    "pano_blend": "feather",
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


def ros1_rosparam_files(path):
    root = ET.parse(path).getroot()
    files = []
    for node in root.findall("node"):
        for rosparam in node.findall("rosparam"):
            if rosparam.attrib.get("command") == "load":
                files.append(rosparam.attrib.get("file", ""))
    return files


def ros2_dict_assignment(path, assignment_name):
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        if any(isinstance(target, ast.Name) and target.id == assignment_name
               for target in node.targets):
            return ast.literal_eval(node.value)
    raise AssertionError(f"{path.name} does not define {assignment_name}")


def ros2_argument_defaults(path):
    return ros2_dict_assignment(path, "ARGUMENT_DEFAULTS")


def yaml_scalar_defaults(path):
    values = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        key = key.strip()
        value = value.strip().strip("\"" + chr(39))
        if not value or key in ("/**", "ros__parameters"):
            continue
        values[key] = value.lower() if value.lower() in ("true", "false") else value
    return values


class LaunchContractTest(unittest.TestCase):
    def test_ros1_exposes_three_user_launch_configs(self):
        launch_names = sorted(path.name for path in (PACKAGE_ROOT / "launch").glob("orbvi_ros_bridge*.launch"))
        self.assertEqual(launch_names, sorted(ROS1_USER_LAUNCHES))

    def test_ros2_exposes_three_user_launch_configs(self):
        launch_names = sorted(path.name for path in (PACKAGE_ROOT / "launch" / "ros2").glob("orbvi_ros_bridge*.launch.py"))
        self.assertEqual(launch_names, sorted(ROS2_USER_LAUNCHES))

    def test_ros1_launch_configs_expose_only_host(self):
        for launch_name, expected_streams in ROS1_USER_LAUNCHES.items():
            with self.subTest(launch_name=launch_name):
                path = PACKAGE_ROOT / "launch" / launch_name
                self.assertEqual(ros1_arg_defaults(path), {"host": "127.0.0.1"})
                self.assertEqual(ros1_param_value(path, "streams"), expected_streams)
                self.assertNotIn("pano", VISUAL_STREAMS.split(","))
                self.assertIn("pano", PANO_STREAMS.split(","))

    def test_ros2_launch_configs_expose_only_host(self):
        for launch_name, expected_streams in ROS2_USER_LAUNCHES.items():
            with self.subTest(launch_name=launch_name):
                path = PACKAGE_ROOT / "launch" / "ros2" / launch_name
                self.assertEqual(ros2_argument_defaults(path), {"host": "127.0.0.1"})
                parameters = ros2_dict_assignment(path, "DEFAULT_BRIDGE_PARAMETERS")
                self.assertEqual(parameters["streams"], expected_streams)
                self.assertIn("LaunchConfiguration(\"host\")", path.read_text(encoding="utf-8"))

    def test_only_pano_launch_loads_pano_config(self):
        for launch_name in ROS1_USER_LAUNCHES:
            with self.subTest(ros="ros1", launch_name=launch_name):
                files = ros1_rosparam_files(PACKAGE_ROOT / "launch" / launch_name)
                if launch_name == "orbvi_ros_bridge_pano.launch":
                    self.assertEqual(files, ["$(find orbvi_ros_bridge)/config/pano_mode2.yaml"])
                else:
                    self.assertEqual(files, [])
        for launch_name in ROS2_USER_LAUNCHES:
            with self.subTest(ros="ros2", launch_name=launch_name):
                source = (PACKAGE_ROOT / "launch" / "ros2" / launch_name).read_text(encoding="utf-8")
                if launch_name == "orbvi_ros_bridge_pano.launch.py":
                    self.assertIn("\"pano_mode2_ros2.yaml\"", source)
                else:
                    self.assertNotIn("pano_mode2_ros2.yaml", source)
                self.assertNotIn("LaunchConfiguration(\"pano_config\")", source)

    def test_rate_check_start_bridge_uses_fixed_launch_contract(self):
        source = RATE_CHECK_SCRIPT.read_text(encoding="utf-8")
        self.assertIn("bridge_launch=\"orbvi_ros_bridge_mid360.launch\"", source)
        self.assertIn("--bridge-launch LAUNCH", source)
        self.assertIn("roslaunch orbvi_ros_bridge \"$bridge_launch\"", source)
        for unsupported_arg in (
            "control_port:=",
            "streams:=",
            "image_mode:=",
            "queue_size:=",
            "max_receive_queue_depth:=",
            "max_decode_queue_depth:=",
        ):
            self.assertNotIn(unsupported_arg, source)
        self.assertNotIn("--streams CSV", source)
        self.assertNotIn("--image-mode MODE", source)

    def test_default_pano_configs_use_mode2_blend_px_32(self):
        for config_name in ("pano_mode2.yaml", "pano_mode2_ros2.yaml"):
            with self.subTest(config_name=config_name):
                values = yaml_scalar_defaults(PACKAGE_ROOT / "config" / config_name)
                for name, expected_value in PANORAMA_ARGUMENT_DEFAULTS.items():
                    self.assertEqual(values[name], expected_value)


if __name__ == "__main__":
    unittest.main()
