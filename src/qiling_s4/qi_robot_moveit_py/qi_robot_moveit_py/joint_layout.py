from typing import List


DEFAULT_JOINT_ORDER = [
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_hip_pitch_joint",
    "left_knee_joint",
    "left_foot_pitch_joint",
    "left_foot_roll_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_hip_pitch_joint",
    "right_knee_joint",
    "right_foot_pitch_joint",
    "right_foot_roll_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "LH_thumb_cmc_yaw",
    "LH_thumb_cmc_pitch",
    "LH_index_mcp_pitch",
    "LH_middle_mcp_pitch",
    "LH_ring_mcp_pitch",
    "LH_pinky_mcp_pitch",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
    "RH_thumb_cmc_yaw",
    "RH_thumb_cmc_pitch",
    "RH_index_mcp_pitch",
    "RH_middle_mcp_pitch",
    "RH_ring_mcp_pitch",
    "RH_pinky_mcp_pitch",
]

DEFAULT_POSITIONS = [
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.1, 0.0, -1.7, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -0.1, 0.0, -1.7, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
]

# Real robot /lowstate: 12 legs + 7 left arm + 7 right arm. Dexterous hands are separate.
REAL_ROBOT_BODY_JOINT_ORDER = (
    DEFAULT_JOINT_ORDER[:12]
    + DEFAULT_JOINT_ORDER[12:19]
    + DEFAULT_JOINT_ORDER[25:32]
)

REAL_ROBOT_BODY_DEFAULT_POSITIONS = (
    DEFAULT_POSITIONS[:12]
    + DEFAULT_POSITIONS[12:19]
    + DEFAULT_POSITIONS[25:32]
)


def joint_order_for_body_dof(body_dof: int) -> List[str]:
    if int(body_dof) <= 26:
        return list(REAL_ROBOT_BODY_JOINT_ORDER)
    return list(DEFAULT_JOINT_ORDER)


def parse_names(value) -> List[str]:
    if isinstance(value, str):
        return [item.strip() for item in value.split(",") if item.strip()]
    return [str(item) for item in value]


def parse_floats(value, expected: int, default: float) -> List[float]:
    if value is None:
        return [default] * expected
    values = [float(item) for item in value]
    if len(values) == expected:
        return values
    if len(values) == 0:
        return [default] * expected
    raise ValueError(f"expected {expected} values, got {len(values)}")
