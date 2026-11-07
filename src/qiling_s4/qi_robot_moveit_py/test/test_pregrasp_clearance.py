"""Offline geometry regression for the configured path; never starts ROS nodes.

Checks the URDF's fixed pinky meshes, not actual closed fingers or table collision.
"""

import ast
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from qi_robot_moveit_py.ruckig_path import plan_joint_path


S4 = Path(__file__).resolve().parents[2]
DESCRIPTION = S4 / "qi_robot_description"
NAMES = [
    f"{side}_{joint}_joint"
    for side in ("left", "right")
    for joint in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
                  "wrist_roll", "wrist_pitch", "wrist_yaw")
]


def _origin(element):
    matrix = np.eye(4)
    if element is not None:
        matrix[:3, 3] = np.fromstring(element.get("xyz", "0 0 0"), sep=" ")
        matrix[:3, :3] = Rotation.from_euler(
            "xyz", np.fromstring(element.get("rpy", "0 0 0"), sep=" ")
        ).as_matrix()
    return matrix


@pytest.fixture(scope="module")
def path():
    launch = S4 / "qi_robot_moveit_config/launch/bringup/robot_stack.launch.py"
    defaults = {}
    for node in ast.walk(ast.parse(launch.read_text())):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                and node.func.id == "DeclareLaunchArgument" and node.args
                and isinstance(node.args[0], ast.Constant)):
            for keyword in node.keywords:
                if keyword.arg == "default_value" and isinstance(keyword.value, ast.Constant):
                    defaults[node.args[0].value] = keyword.value.value
    points = [np.fromstring(defaults[name], sep=",") for name in (
        "home_joint_positions", "pregrasp_transition_joint_positions",
        "pregrasp_middle_joint_positions", "pregrasp_clearance_joint_positions",
        "pregrasp_joint_positions",
    )]
    assert all(len(point) == 14 for point in points)
    limits = {name: float(defaults[f"direct_{name}"]) for name in (
        "max_velocity", "max_acceleration", "max_jerk", "sample_period",
    )}
    samples = plan_joint_path(points, **limits)
    urdf = ET.parse(DESCRIPTION / "urdf/s4_dual_arm.urdf").getroot()
    return points, samples, urdf


def _pinky_geometry(urdf, side):
    joints = {joint.find("child").get("link"): joint for joint in urdf.findall("joint")}
    geometry = []
    for part in ("pinky_proximal", "pinky_distal"):
        name = f"{side}_{part}"
        collision = urdf.find(f"link[@name='{name}']/collision")
        mesh = collision.find("geometry/mesh")
        mesh_path = DESCRIPTION / mesh.get("filename").split("package://qi_robot_description/")[1]
        data = mesh_path.read_bytes()
        count = struct.unpack_from("<I", data, 80)[0]
        assert len(data) == 84 + 50 * count  # Binary STL.
        dtype = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attr", "<u2")])
        vertices = np.frombuffer(data, dtype=dtype, offset=84)["vertices"].reshape(-1, 3)
        vertices = np.unique(vertices, axis=0) * np.fromstring(mesh.get("scale", "1 1 1"), sep=" ")
        transform = _origin(collision.find("origin"))
        vertices = vertices @ transform[:3, :3].T + transform[:3, 3]
        chain = []
        while name != "base_link":
            joint = joints[name]
            index = None if joint.get("type") == "fixed" else NAMES.index(joint.get("name"))
            axis = None if index is None else np.fromstring(joint.find("axis").get("xyz"), sep=" ")
            chain.append((_origin(joint.find("origin")), index, axis))
            name = joint.find("parent").get("link")
        geometry.append((vertices, list(reversed(chain))))
    return geometry


def _minimum_height(geometry, positions):
    heights = []
    for vertices, chain in geometry:
        transform = np.eye(4)
        for origin, index, axis in chain:
            transform = transform @ origin
            if index is not None:
                rotation = np.eye(4)
                rotation[:3, :3] = Rotation.from_rotvec(axis * positions[index]).as_matrix()
                transform = transform @ rotation
        heights.append(np.min(vertices @ transform[2, :3] + transform[2, 3]))
    return min(heights)


def test_path_visits_clearance_and_keeps_original_endpoint_within_joint_limits(path):
    points, samples, urdf = path
    expected_end = [-.55, .45, -.34, -.52, -.65, -.33, .19,
                    -.55, -.45, -.34, -.52, .65, -.33, .19]
    np.testing.assert_allclose(points[-1], expected_end, atol=1e-10)
    limits = [urdf.find(f"joint[@name='{name}']/limit") for name in NAMES]
    lower = [float(limit.get("lower")) for limit in limits]
    upper = [float(limit.get("upper")) for limit in limits]
    positions = np.array([sample[1] for sample in samples])
    assert np.all(positions >= lower) and np.all(positions <= upper)
    for point in points:
        assert np.any(np.max(np.abs(positions - point), axis=1) < 1e-7)


@pytest.mark.parametrize("side", ["LH", "RH"])
def test_late_path_no_longer_dips_below_original_pregrasp_pinky_height(path, side):
    points, samples, urdf = path
    geometry = _pinky_geometry(urdf, side)
    start = next(i for i, sample in enumerate(samples)
                 if np.max(np.abs(np.array(sample[1]) - points[2])) < 1e-7)
    endpoint_height = _minimum_height(geometry, points[-1])
    assert endpoint_height > .09
    assert _minimum_height(geometry, points[2]) > endpoint_height + .06
    assert _minimum_height(geometry, points[3]) > endpoint_height + .05
    heights = [_minimum_height(geometry, sample[1]) for sample in samples[start:]]
    assert min(heights) >= endpoint_height - 1e-4
