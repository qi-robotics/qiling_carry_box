from types import SimpleNamespace

import pytest
from geometry_msgs.msg import Pose

from qi_robot_moveit_py.vision_grasp_sequence import VisionGraspSequence


@pytest.mark.parametrize(
    ("detected_x", "expected_x"),
    [(0.41, 0.43), (0.43, 0.43), (0.44, 0.44), (0.45, 0.45), (0.4501, 0.45), (0.60, 0.45)],
)
def test_visual_x_boundaries(detected_x, expected_x):
    node = SimpleNamespace(
        visual_x_min=0.43,
        visual_x_max=0.45,
        visual_fixed_y=0.0,
        visual_fixed_z=0.055,
        _copy_pose=VisionGraspSequence._copy_pose,
        _publish_status=lambda *args: None,
        get_logger=lambda: SimpleNamespace(info=lambda message: None, error=lambda message: None),
    )
    detected = Pose()
    detected.position.x = detected_x
    detected.position.y = 0.10
    detected.position.z = 0.20

    constrained = VisionGraspSequence._constrain_visual_box_pose(node, detected)

    assert constrained.position.x == pytest.approx(expected_x)
    assert constrained.position.y == 0.0
    assert constrained.position.z == 0.055
    assert detected.position.x == detected_x


def test_invalid_visual_x_is_rejected():
    node = SimpleNamespace(
        _publish_status=lambda *args: None,
        get_logger=lambda: SimpleNamespace(error=lambda message: None),
    )
    detected = Pose()
    detected.position.x = float("nan")

    assert VisionGraspSequence._constrain_visual_box_pose(node, detected) is None
