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


@pytest.mark.parametrize("success", [True, False])
def test_midpoint_policy_uses_fixed_pose_for_either_detection_result(success):
    detected = Pose()
    detected.position.x, detected.position.y, detected.position.z = 0.60, 0.12, 0.20
    response = SimpleNamespace(success=success, message="test", box_pose=detected)
    node = _detect_harness(response, use_midpoint=True)

    pose = VisionGraspSequence._detect_box_pose(node)

    assert (pose.position.x, pose.position.y, pose.position.z) == pytest.approx((0.44, 0.0, 0.055))
    assert pose.orientation.w == 1.0
    assert all(status[2] != "fail" for status in node.statuses)
    assert ("成功" if success else "失败") in node.statuses[-1][3]


@pytest.mark.parametrize("success", [True, False])
def test_disabled_midpoint_policy_preserves_detection_result(success):
    detected = Pose()
    detected.position.x = 0.60
    node = _detect_harness(SimpleNamespace(success=success, message="test", box_pose=detected))

    pose = VisionGraspSequence._detect_box_pose(node)

    assert pose is detected if success else pose is None
    assert node.statuses[-1][2] == ("ok" if success else "fail")


@pytest.mark.parametrize("timeout", [True, False])
def test_midpoint_policy_does_not_continue_without_service_response(timeout):
    node = _detect_harness(None, use_midpoint=True, timeout=timeout)
    assert VisionGraspSequence._detect_box_pose(node) is None
    assert node.statuses[-1][2] == "fail"


def _detect_harness(response, *, use_midpoint=False, timeout=False):
    statuses = []
    return SimpleNamespace(
        detect_service_name="detect",
        service_timeout_sec=1.0,
        detect_client=SimpleNamespace(call_async=lambda request: SimpleNamespace(result=lambda: response)),
        _spin_until_future_done=lambda future, duration: not timeout,
        _publish_status=lambda *args: statuses.append(args),
        get_logger=lambda: SimpleNamespace(info=lambda message: None, error=lambda message: None),
        visual_grasp_use_x_midpoint=use_midpoint,
        visual_x_min=0.43,
        visual_x_max=0.45,
        visual_fixed_y=0.0,
        visual_fixed_z=0.055,
        statuses=statuses,
    )
