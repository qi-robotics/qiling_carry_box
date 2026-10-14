from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("qi_robot_dual_arm_full_fixed", package_name="qi_robot_moveit_config")
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .to_moveit_configs()
    )
    default_ik_seed_config = str(
        Path(get_package_share_directory("qi_robot_moveit_py")) / "config" / "ik_seed_presets.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("reference_frame", default_value="base_link"),
            DeclareLaunchArgument(
                "left_joint_names",
                default_value=(
                    "left_shoulder_pitch_joint,left_shoulder_roll_joint,"
                    "left_shoulder_yaw_joint,left_elbow_joint,left_wrist_roll_joint,"
                    "left_wrist_pitch_joint,left_wrist_yaw_joint"
                ),
            ),
            DeclareLaunchArgument(
                "right_joint_names",
                default_value=(
                    "right_shoulder_pitch_joint,right_shoulder_roll_joint,"
                    "right_shoulder_yaw_joint,right_elbow_joint,right_wrist_roll_joint,"
                    "right_wrist_pitch_joint,right_wrist_yaw_joint"
                ),
            ),
            DeclareLaunchArgument(
                "left_controller_action",
                default_value="/left_arm_position_controller/follow_joint_trajectory",
            ),
            DeclareLaunchArgument(
                "right_controller_action",
                default_value="/right_arm_position_controller/follow_joint_trajectory",
            ),
            DeclareLaunchArgument(
                "dual_controller_action",
                default_value="/dual_arm_position_controller/follow_joint_trajectory",
            ),
            DeclareLaunchArgument("plan_only", default_value="false"),
            DeclareLaunchArgument("joint_goal_mode", default_value="plan"),
            DeclareLaunchArgument("both_pose_goal_mode", default_value="cartesian_direct"),
            DeclareLaunchArgument("cartesian_waypoint_count", default_value="10"),
            DeclareLaunchArgument("object_cartesian_waypoint_count", default_value="10"),
            DeclareLaunchArgument("object_to_left_tool_xyz", default_value="0.01,0.184,0.08"),
            DeclareLaunchArgument("object_to_right_tool_xyz", default_value="0.01,-0.184,0.08"),
            DeclareLaunchArgument("left_grasp_delta_xyz", default_value="0.0,0.0,0.0"),
            DeclareLaunchArgument("right_grasp_delta_xyz", default_value="0.0,0.0,0.0"),
            DeclareLaunchArgument("object_to_tool_orientation_mode", default_value="from_tool_orientation"),
            DeclareLaunchArgument("left_orientation_xyzw", default_value="-0.5,0.5,-0.5,0.5"),
            DeclareLaunchArgument("right_orientation_xyzw", default_value="0.5,0.5,0.5,0.5"),
            DeclareLaunchArgument("left_orientation_rpy_offset", default_value="0.0,0.0,0.0"),
            DeclareLaunchArgument("right_orientation_rpy_offset", default_value="0.0,0.0,0.0"),
            DeclareLaunchArgument("object_pose_orientation_mode", default_value="identity"),
            DeclareLaunchArgument("object_path_start_mode", default_value="requested"),
            DeclareLaunchArgument("ik_seed_mode", default_value="current"),
            DeclareLaunchArgument("ik_preferred_joint_positions", default_value=""),
            DeclareLaunchArgument("ik_seed_config_file", default_value=default_ik_seed_config),
            DeclareLaunchArgument("ik_seed_preset", default_value="default"),
            DeclareLaunchArgument("ik_preferred_joint_blend", default_value="0.35"),
            DeclareLaunchArgument("ik_avoid_collisions", default_value="true"),
            DeclareLaunchArgument("goal_service_name", default_value="/moveit_arm_goal"),
            DeclareLaunchArgument("planning_debug_enabled", default_value="true"),
            DeclareLaunchArgument("planning_debug_topic", default_value="/moveit/planning_debug"),
            DeclareLaunchArgument(
                "planning_diagnostics_topic", default_value="/moveit/planning_diagnostics"
            ),
            DeclareLaunchArgument("reject_zero_state_glitches", default_value="true"),
            DeclareLaunchArgument("zero_state_threshold", default_value="0.0001"),
            DeclareLaunchArgument("zero_state_previous_motion_threshold", default_value="0.05"),
            DeclareLaunchArgument("zero_state_confirmation_frames", default_value="3"),
            DeclareLaunchArgument("compute_ik_service", default_value="/compute_ik"),
            DeclareLaunchArgument("plan_kinematic_path_service", default_value="/plan_kinematic_path"),
            DeclareLaunchArgument("moveit_service_wait_sec", default_value="30.0"),
            DeclareLaunchArgument("direct_motion_duration", default_value="3.0"),
            DeclareLaunchArgument("direct_sample_period", default_value="0.02"),
            DeclareLaunchArgument("direct_trajectory_smoothing_window", default_value="5"),
            DeclareLaunchArgument("direct_trajectory_smoothing_passes", default_value="2"),
            DeclareLaunchArgument("direct_smoothing_method", default_value="spline"),
            DeclareLaunchArgument("direct_spline_anchor_stride", default_value="3"),
            DeclareLaunchArgument("direct_spline_smoothing_alpha", default_value="0.35"),
            DeclareLaunchArgument("direct_spline_max_joint_deviation", default_value="0.08"),
            DeclareLaunchArgument("direct_velocity_scale", default_value="0.65"),
            DeclareLaunchArgument("direct_time_parameterization", default_value="jerk_limited"),
            DeclareLaunchArgument("direct_max_velocity", default_value="0.45"),
            DeclareLaunchArgument("direct_max_acceleration", default_value="0.80"),
            DeclareLaunchArgument("direct_max_jerk", default_value="2.50"),
            DeclareLaunchArgument("movel_grasp_lift_enabled", default_value="true"),
            DeclareLaunchArgument("movel_cartesian_step_m", default_value="0.010"),
            DeclareLaunchArgument("movel_max_position_error_m", default_value="0.020"),
            DeclareLaunchArgument("movel_max_orientation_error_rad", default_value="0.15"),
            DeclareLaunchArgument("movel_smoothing_enabled", default_value="false"),
            DeclareLaunchArgument("movel_smoothing_weight", default_value="10.0"),
            DeclareLaunchArgument("movel_smoothing_max_joint_deviation", default_value="0.03"),
            DeclareLaunchArgument("movel_max_speedup", default_value="1.25"),
            DeclareLaunchArgument("movel_grasp_max_speedup", default_value="1.25"),
            DeclareLaunchArgument("direct_min_segment_duration", default_value="0.12"),
            DeclareLaunchArgument("direct_max_duration_scale", default_value="1.50"),
            DeclareLaunchArgument("direct_joint_jump_warn_threshold", default_value="0.25"),
            DeclareLaunchArgument("direct_ik_max_joint_step", default_value="0.20"),
            DeclareLaunchArgument("direct_ik_jump_retry_count", default_value="1"),
            DeclareLaunchArgument("cached_place_start_tolerance", default_value="0.0"),
            DeclareLaunchArgument("cached_push_start_tolerance", default_value="0.0"),
            DeclareLaunchArgument("cached_push_goal_tolerance", default_value="0.0"),
            DeclareLaunchArgument("cached_trajectory_start_blend_enabled", default_value="false"),
            DeclareLaunchArgument("carry_x_offset", default_value="0.0"),
            DeclareLaunchArgument("post_place_push_x", default_value="0.0"),
            DeclareLaunchArgument("allowed_planning_time", default_value="5.0"),
            DeclareLaunchArgument("num_planning_attempts", default_value="5"),
            DeclareLaunchArgument("max_velocity_scaling_factor", default_value="0.4"),
            DeclareLaunchArgument("max_acceleration_scaling_factor", default_value="0.3"),
            DeclareLaunchArgument("visualization_enabled", default_value="true"),
            DeclareLaunchArgument("visualization_marker_topic", default_value="/moveit_goal_markers"),
            DeclareLaunchArgument("goal_axis_length", default_value="0.12"),
            DeclareLaunchArgument("base_axis_length", default_value="0.18"),
            DeclareLaunchArgument("actual_axis_length", default_value="0.12"),
            DeclareLaunchArgument("actual_frame_publish_period", default_value="0.1"),
            DeclareLaunchArgument(
                "actual_frame_names",
                default_value="base_link,left_arm_tool_frame,right_arm_tool_frame",
            ),
            DeclareLaunchArgument("tool_frame_offset_z", default_value="0.10"),
            DeclareLaunchArgument("object_roundtrip_lift_z", default_value="0.10"),
            DeclareLaunchArgument("object_roundtrip_hold_sec", default_value="2.0"),
            Node(
                package="qi_robot_moveit_py",
                executable="moveit_goal_commander",
                name="moveit_goal_commander",
                output="screen",
                parameters=[
                    {
                        "reference_frame": LaunchConfiguration("reference_frame"),
                        "left_joint_names": LaunchConfiguration("left_joint_names"),
                        "right_joint_names": LaunchConfiguration("right_joint_names"),
                        "left_controller_action": LaunchConfiguration("left_controller_action"),
                        "right_controller_action": LaunchConfiguration("right_controller_action"),
                        "dual_controller_action": LaunchConfiguration("dual_controller_action"),
                        "plan_only": ParameterValue(LaunchConfiguration("plan_only"), value_type=bool),
                        "joint_goal_mode": LaunchConfiguration("joint_goal_mode"),
                        "both_pose_goal_mode": LaunchConfiguration("both_pose_goal_mode"),
                        "cartesian_waypoint_count": ParameterValue(
                            LaunchConfiguration("cartesian_waypoint_count"), value_type=int
                        ),
                        "object_cartesian_waypoint_count": ParameterValue(
                            LaunchConfiguration("object_cartesian_waypoint_count"), value_type=int
                        ),
                        "object_to_left_tool_xyz": LaunchConfiguration("object_to_left_tool_xyz"),
                        "object_to_right_tool_xyz": LaunchConfiguration("object_to_right_tool_xyz"),
                        "left_grasp_delta_xyz": LaunchConfiguration("left_grasp_delta_xyz"),
                        "right_grasp_delta_xyz": LaunchConfiguration("right_grasp_delta_xyz"),
                        "object_to_tool_orientation_mode": LaunchConfiguration("object_to_tool_orientation_mode"),
                        "left_orientation_xyzw": LaunchConfiguration("left_orientation_xyzw"),
                        "right_orientation_xyzw": LaunchConfiguration("right_orientation_xyzw"),
                        "left_orientation_rpy_offset": LaunchConfiguration("left_orientation_rpy_offset"),
                        "right_orientation_rpy_offset": LaunchConfiguration("right_orientation_rpy_offset"),
                        "object_pose_orientation_mode": LaunchConfiguration("object_pose_orientation_mode"),
                        "object_path_start_mode": LaunchConfiguration("object_path_start_mode"),
                        "ik_seed_mode": LaunchConfiguration("ik_seed_mode"),
                        "ik_preferred_joint_positions": LaunchConfiguration("ik_preferred_joint_positions"),
                        "ik_seed_config_file": LaunchConfiguration("ik_seed_config_file"),
                        "ik_seed_preset": LaunchConfiguration("ik_seed_preset"),
                        "ik_preferred_joint_blend": ParameterValue(
                            LaunchConfiguration("ik_preferred_joint_blend"), value_type=float
                        ),
                        "ik_avoid_collisions": ParameterValue(
                            LaunchConfiguration("ik_avoid_collisions"), value_type=bool
                        ),
                        "goal_service_name": LaunchConfiguration("goal_service_name"),
                        "planning_debug_enabled": ParameterValue(
                            LaunchConfiguration("planning_debug_enabled"), value_type=bool
                        ),
                        "planning_debug_topic": LaunchConfiguration("planning_debug_topic"),
                        "planning_diagnostics_topic": LaunchConfiguration(
                            "planning_diagnostics_topic"
                        ),
                        "compute_ik_service": LaunchConfiguration("compute_ik_service"),
                        "plan_kinematic_path_service": LaunchConfiguration("plan_kinematic_path_service"),
                        "moveit_service_wait_sec": ParameterValue(
                            LaunchConfiguration("moveit_service_wait_sec"), value_type=float
                        ),
                        "direct_motion_duration": ParameterValue(
                            LaunchConfiguration("direct_motion_duration"), value_type=float
                        ),
                        "direct_sample_period": ParameterValue(
                            LaunchConfiguration("direct_sample_period"), value_type=float
                        ),
                        "direct_trajectory_smoothing_window": ParameterValue(
                            LaunchConfiguration("direct_trajectory_smoothing_window"), value_type=int
                        ),
                        "direct_trajectory_smoothing_passes": ParameterValue(
                            LaunchConfiguration("direct_trajectory_smoothing_passes"), value_type=int
                        ),
                        "direct_smoothing_method": LaunchConfiguration("direct_smoothing_method"),
                        "direct_spline_anchor_stride": ParameterValue(
                            LaunchConfiguration("direct_spline_anchor_stride"), value_type=int
                        ),
                        "direct_spline_smoothing_alpha": ParameterValue(
                            LaunchConfiguration("direct_spline_smoothing_alpha"), value_type=float
                        ),
                        "direct_spline_max_joint_deviation": ParameterValue(
                            LaunchConfiguration("direct_spline_max_joint_deviation"), value_type=float
                        ),
                        "direct_velocity_scale": ParameterValue(
                            LaunchConfiguration("direct_velocity_scale"), value_type=float
                        ),
                        "direct_time_parameterization": LaunchConfiguration("direct_time_parameterization"),
                        "direct_max_velocity": ParameterValue(
                            LaunchConfiguration("direct_max_velocity"), value_type=float
                        ),
                        "direct_max_acceleration": ParameterValue(
                            LaunchConfiguration("direct_max_acceleration"), value_type=float
                        ),
                        "direct_max_jerk": ParameterValue(
                            LaunchConfiguration("direct_max_jerk"), value_type=float
                        ),
                        "movel_grasp_lift_enabled": ParameterValue(
                            LaunchConfiguration("movel_grasp_lift_enabled"), value_type=bool
                        ),
                        "movel_cartesian_step_m": ParameterValue(
                            LaunchConfiguration("movel_cartesian_step_m"), value_type=float
                        ),
                        "movel_max_position_error_m": ParameterValue(
                            LaunchConfiguration("movel_max_position_error_m"), value_type=float
                        ),
                        "movel_max_orientation_error_rad": ParameterValue(
                            LaunchConfiguration("movel_max_orientation_error_rad"), value_type=float
                        ),
                        "movel_smoothing_enabled": ParameterValue(
                            LaunchConfiguration("movel_smoothing_enabled"), value_type=bool
                        ),
                        "movel_smoothing_weight": ParameterValue(
                            LaunchConfiguration("movel_smoothing_weight"), value_type=float
                        ),
                        "movel_smoothing_max_joint_deviation": ParameterValue(
                            LaunchConfiguration("movel_smoothing_max_joint_deviation"), value_type=float
                        ),
                        "movel_max_speedup": ParameterValue(
                            LaunchConfiguration("movel_max_speedup"), value_type=float
                        ),
                        "movel_grasp_max_speedup": ParameterValue(
                            LaunchConfiguration("movel_grasp_max_speedup"), value_type=float
                        ),
                        "direct_min_segment_duration": ParameterValue(
                            LaunchConfiguration("direct_min_segment_duration"), value_type=float
                        ),
                        "direct_max_duration_scale": ParameterValue(
                            LaunchConfiguration("direct_max_duration_scale"), value_type=float
                        ),
                        "direct_joint_jump_warn_threshold": ParameterValue(
                            LaunchConfiguration("direct_joint_jump_warn_threshold"), value_type=float
                        ),
                        "direct_ik_max_joint_step": ParameterValue(
                            LaunchConfiguration("direct_ik_max_joint_step"), value_type=float
                        ),
                        "direct_ik_jump_retry_count": ParameterValue(
                            LaunchConfiguration("direct_ik_jump_retry_count"), value_type=int
                        ),
                        "cached_place_start_tolerance": ParameterValue(
                            LaunchConfiguration("cached_place_start_tolerance"), value_type=float
                        ),
                        "cached_push_start_tolerance": ParameterValue(
                            LaunchConfiguration("cached_push_start_tolerance"), value_type=float
                        ),
                        "cached_push_goal_tolerance": ParameterValue(
                            LaunchConfiguration("cached_push_goal_tolerance"), value_type=float
                        ),
                        "cached_trajectory_start_blend_enabled": ParameterValue(
                            LaunchConfiguration("cached_trajectory_start_blend_enabled"), value_type=bool
                        ),
                        "carry_x_offset": ParameterValue(
                            LaunchConfiguration("carry_x_offset"), value_type=float
                        ),
                        "post_place_push_x": ParameterValue(
                            LaunchConfiguration("post_place_push_x"), value_type=float
                        ),
                        "allowed_planning_time": ParameterValue(
                            LaunchConfiguration("allowed_planning_time"), value_type=float
                        ),
                        "num_planning_attempts": ParameterValue(
                            LaunchConfiguration("num_planning_attempts"), value_type=int
                        ),
                        "max_velocity_scaling_factor": ParameterValue(
                            LaunchConfiguration("max_velocity_scaling_factor"), value_type=float
                        ),
                        "max_acceleration_scaling_factor": ParameterValue(
                            LaunchConfiguration("max_acceleration_scaling_factor"), value_type=float
                        ),
                        "visualization_enabled": ParameterValue(
                            LaunchConfiguration("visualization_enabled"), value_type=bool
                        ),
                        "visualization_marker_topic": LaunchConfiguration("visualization_marker_topic"),
                        "goal_axis_length": ParameterValue(LaunchConfiguration("goal_axis_length"), value_type=float),
                        "base_axis_length": ParameterValue(LaunchConfiguration("base_axis_length"), value_type=float),
                        "actual_axis_length": ParameterValue(
                            LaunchConfiguration("actual_axis_length"), value_type=float
                        ),
                        "actual_frame_publish_period": ParameterValue(
                            LaunchConfiguration("actual_frame_publish_period"), value_type=float
                        ),
                        "actual_frame_names": LaunchConfiguration("actual_frame_names"),
                        "tool_frame_offset_z": ParameterValue(
                            LaunchConfiguration("tool_frame_offset_z"), value_type=float
                        ),
                        "object_roundtrip_lift_z": ParameterValue(
                            LaunchConfiguration("object_roundtrip_lift_z"), value_type=float
                        ),
                        "object_roundtrip_hold_sec": ParameterValue(
                            LaunchConfiguration("object_roundtrip_hold_sec"), value_type=float
                        ),
                        "robot_description_xml": moveit_config.robot_description["robot_description"],
                        "reject_zero_state_glitches": ParameterValue(
                            LaunchConfiguration("reject_zero_state_glitches"), value_type=bool
                        ),
                        "zero_state_threshold": ParameterValue(
                            LaunchConfiguration("zero_state_threshold"), value_type=float
                        ),
                        "zero_state_previous_motion_threshold": ParameterValue(
                            LaunchConfiguration("zero_state_previous_motion_threshold"), value_type=float
                        ),
                        "zero_state_confirmation_frames": ParameterValue(
                            LaunchConfiguration("zero_state_confirmation_frames"), value_type=int
                        ),
                    }
                ],
            ),
        ]
    )
