"""Real-robot dual-arm control stack.

This entry keeps the tested real-robot behavior but no longer depends on the
old local-MuJoCo launch.  It reads SDK lowstate, publishes /joint_states, starts
MoveIt and the arm bridge, then exposes grasp/place commands.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _package_launch(package_name: str, launch_file: str):
    return PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare(package_name), "launch", launch_file])
    )


def _hand_launch_arguments():
    defaults = {
        "enable_hand_control": "true", "hands_cmd_topic": "/handscmd",
        "hands_state_topic": "/handsstate", "hands_state_log_period_sec": "2.0",
        "left_hand_array_index": "0", "right_hand_array_index": "1",
        "left_hand_id": "0", "right_hand_id": "1", "hand_mode": "1",
        "hands_mode": "1", "hands_mode_ctrl": "5",
        "left_hand_grasp_positions": "60,128,200,200,150,90",
        "right_hand_grasp_positions": "60,128,200,200,150,90",
        "left_hand_grasp_step1_positions": "255,255,200,200,150,90",
        "right_hand_grasp_step1_positions": "255,255,200,200,150,90",
        "left_hand_release_positions": "255,255,255,255,255,255",
        "right_hand_release_positions": "255,255,255,255,255,255",
        "left_hand_grasp_durations": "200,200,200,200,200,200",
        "right_hand_grasp_durations": "200,200,200,200,200,200",
        "left_hand_release_durations": "200,200,200,200,200,200",
        "right_hand_release_durations": "200,200,200,200,200,200",
        "hand_grasp_wait_sec": "1", "hand_release_wait_sec": "1",
        "hand_grasp_step_delay_sec": "1.0",
    }
    return [DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()]


def _hand_parameters():
    integer_names = (
        "left_hand_array_index", "right_hand_array_index", "left_hand_id", "right_hand_id",
        "hand_mode", "hands_mode", "hands_mode_ctrl",
    )
    string_names = (
        "hands_cmd_topic", "hands_state_topic", "left_hand_grasp_positions", "right_hand_grasp_positions",
        "left_hand_grasp_step1_positions", "right_hand_grasp_step1_positions",
        "left_hand_release_positions", "right_hand_release_positions",
        "left_hand_grasp_durations", "right_hand_grasp_durations",
        "left_hand_release_durations", "right_hand_release_durations",
    )
    result = {name: LaunchConfiguration(name) for name in string_names}
    result.update({name: ParameterValue(LaunchConfiguration(name), value_type=int) for name in integer_names})
    result["enable_hand_control"] = ParameterValue(LaunchConfiguration("enable_hand_control"), value_type=bool)
    result["hand_grasp_wait_sec"] = ParameterValue(LaunchConfiguration("hand_grasp_wait_sec"), value_type=float)
    result["hand_release_wait_sec"] = ParameterValue(LaunchConfiguration("hand_release_wait_sec"), value_type=float)
    result["hand_grasp_step_delay_sec"] = ParameterValue(LaunchConfiguration("hand_grasp_step_delay_sec"), value_type=float)
    result["hands_state_log_period_sec"] = ParameterValue(
        LaunchConfiguration("hands_state_log_period_sec"), value_type=float
    )
    return result


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("ik_backend", default_value="pick_ik"),
            DeclareLaunchArgument("rviz_delay", default_value="3.0"),
            DeclareLaunchArgument("object_cartesian_waypoint_count", default_value="5"),
            DeclareLaunchArgument("object_roundtrip_lift_z", default_value="0.10"),
            DeclareLaunchArgument("object_roundtrip_hold_sec", default_value="2.0"),
            DeclareLaunchArgument("direct_motion_duration", default_value="5.0"),
            DeclareLaunchArgument("direct_sample_period", default_value="0.02"),
            DeclareLaunchArgument("direct_trajectory_smoothing_window", default_value="7"),
            DeclareLaunchArgument("direct_trajectory_smoothing_passes", default_value="3"),
            DeclareLaunchArgument("direct_smoothing_method", default_value="spline"),
            DeclareLaunchArgument("direct_spline_anchor_stride", default_value="3"),
            DeclareLaunchArgument("direct_spline_smoothing_alpha", default_value="0.35"),
            DeclareLaunchArgument("direct_spline_max_joint_deviation", default_value="0.08"),
            DeclareLaunchArgument("direct_velocity_scale", default_value="0.75"),
            DeclareLaunchArgument("direct_time_parameterization", default_value="jerk_limited"),
            DeclareLaunchArgument("direct_max_velocity", default_value="0.65"),
            DeclareLaunchArgument("direct_max_acceleration", default_value="0.40"),
            DeclareLaunchArgument("direct_max_jerk", default_value="2.00"),
            DeclareLaunchArgument("direct_min_segment_duration", default_value="0.12"),
            DeclareLaunchArgument("direct_max_duration_scale", default_value="2.50"),
            DeclareLaunchArgument("direct_joint_jump_warn_threshold", default_value="0.25"),
            DeclareLaunchArgument("direct_ik_max_joint_step", default_value="0.30"),
            DeclareLaunchArgument("long_range_ik_max_joint_step", default_value="0.60"),
            DeclareLaunchArgument("direct_ik_jump_retry_count", default_value="4"),
            DeclareLaunchArgument("cached_place_start_tolerance", default_value="0.10"),
            DeclareLaunchArgument("cached_push_start_tolerance", default_value="0.10"),
            DeclareLaunchArgument("cached_push_goal_tolerance", default_value="0.10"),
            DeclareLaunchArgument("cached_trajectory_start_blend_enabled", default_value="true"),
            DeclareLaunchArgument("object_to_left_tool_xyz", default_value="0.01,0.180,0.08"),
            DeclareLaunchArgument("object_to_right_tool_xyz", default_value="0.01,-0.180,0.08"),
            DeclareLaunchArgument("object_to_tool_orientation_mode", default_value="from_tool_orientation"),
            DeclareLaunchArgument("left_orientation_xyzw", default_value="-0.5,0.5,-0.5,0.5"),
            DeclareLaunchArgument("right_orientation_xyzw", default_value="0.5,0.5,0.5,0.5"),
            DeclareLaunchArgument("object_pose_orientation_mode", default_value="identity"),
            DeclareLaunchArgument("object_path_start_mode", default_value="requested"),
            DeclareLaunchArgument("start_grasp_command_node", default_value="true"),
            DeclareLaunchArgument("detect_service_name", default_value="detect"),
            DeclareLaunchArgument("goal_service_name", default_value="/moveit_arm_goal"),
            DeclareLaunchArgument("grasp_command_topic", default_value="/vision_grasp/grasp_command"),
            DeclareLaunchArgument("place_command_topic", default_value="/vision_grasp/place_command"),
            DeclareLaunchArgument("status_topic", default_value="/vision_grasp/status"),
            DeclareLaunchArgument("grasp_result_topic", default_value="/vision_grasp/result"),
            DeclareLaunchArgument("visual_x_only_enabled", default_value="true"),
            DeclareLaunchArgument("visual_x_min", default_value="0.43"),
            DeclareLaunchArgument("visual_x_max", default_value="0.48"),
            DeclareLaunchArgument("visual_fixed_y", default_value="0.0"),
            DeclareLaunchArgument("visual_fixed_z", default_value="0.02"),
            DeclareLaunchArgument("joint_state_topic", default_value="/joint_states"),
            # ROS topic lowcmd_arms maps to DDS rt/lowcmd_arms under rmw_cyclonedds.
            DeclareLaunchArgument(
                "lowcmd_topic",
                default_value="lowcmd_arms",
                description="Real-robot arm aux LowCmd topic (ROS name; DDS rt/lowcmd_arms).",
            ),
            DeclareLaunchArgument(
                "lowcmd_mode_ctrl",
                default_value="0",
                description="Unused for source selection on lowcmd_arms; keep 0.",
            ),
            DeclareLaunchArgument("body_dof", default_value="26"),
            DeclareLaunchArgument("reject_zero_state_glitches", default_value="true"),
            DeclareLaunchArgument("zero_state_threshold", default_value="0.0001"),
            DeclareLaunchArgument("zero_state_previous_motion_threshold", default_value="0.05"),
            DeclareLaunchArgument("zero_state_confirmation_frames", default_value="3"),
            DeclareLaunchArgument("approach_x_offset", default_value="0.01"),
            DeclareLaunchArgument("side_offset_y", default_value="0.180"),
            DeclareLaunchArgument("approach_z_offset", default_value="0.08"),
            DeclareLaunchArgument("lift_z", default_value="0.20"),
            DeclareLaunchArgument("carry_x_offset", default_value="-0.07"),
            DeclareLaunchArgument("post_place_push_x", default_value="0.10"),
            DeclareLaunchArgument("left_orientation_rpy_offset", default_value="-0.2,0.25,-0.3"),
            DeclareLaunchArgument("right_orientation_rpy_offset", default_value="0.2,0.25,0.3"),
            DeclareLaunchArgument("object_orientation_xyzw", default_value="0.0,0.0,0.0,1.0"),
            DeclareLaunchArgument("tool_frame_offset_z", default_value="0.10"),
            DeclareLaunchArgument("precheck_grasp_ik_before_pregrasp", default_value="true"),
            DeclareLaunchArgument("precheck_ik_avoid_collisions", default_value="true"),
            DeclareLaunchArgument("precheck_ik_seed_from_pregrasp", default_value="true"),
            DeclareLaunchArgument("use_pregrasp_joint_goal", default_value="true"),
            DeclareLaunchArgument(
                "pregrasp_transition_joint_positions",
                default_value=(
                    "1.2,0.5,0.15,-2.1,0.0,0.0,0.0,"
                    "1.2,-0.5,0.15,-2.1,0.0,0.0,0.0"
                ),
            ),
            DeclareLaunchArgument(
                "pregrasp_joint_positions",
                default_value=(
                    # "0.9,0.2,0.0,-1.9,0.0,0.0,0.0,"
                    # "0.9,-0.2,0.0,-1.9,0.0,0.0,0.0"
                    "-0.55, 0.45, -0.34, -0.52, -0.65, -0.33, 0.19,"
                    "-0.55, -0.45, -0.34, -0.52, 0.65, -0.33, 0.19"
                ),
            ),
            DeclareLaunchArgument(
                "home_joint_positions",
                default_value=(
                    "0.0,0.18,0.0,-0.3,0.0,0.0,0.0,"
                    "0.0,-0.18,0.0,-0.3,0.0,0.0,0.0"
                ),
            ),
            DeclareLaunchArgument(
                "home_transition_joint_positions",
                default_value=(
                    "-0.6,0.4,-0.6,-0.6,-0.57,-0.2,0.17,"
                    "-0.6,-0.4,-0.6,-0.6,0.57,-0.2,0.17;"
                    "0.0,0.4,0.0,-1.5,0.0,0.0,0.0,"
                    "0.0,-0.4,0.0,-1.5,0.0,0.0,0.0;"
                    "0.9,0.2,0.0,-1.9,0.0,0.0,0.0,"
                    "0.9,-0.2,0.0,-1.9,0.0,0.0,0.0"
                ),
            ),
            DeclareLaunchArgument("allow_contact_motion_timeout", default_value="true"),
            DeclareLaunchArgument("home_retry_count", default_value="1"),
            DeclareLaunchArgument("home_retry_wait_sec", default_value="0.5"),
            *_hand_launch_arguments(),
            DeclareLaunchArgument("both_pose_goal_mode", default_value="cartesian_direct"),
            DeclareLaunchArgument("cartesian_waypoint_count", default_value="5"),
            DeclareLaunchArgument("ik_avoid_collisions", default_value="true"),
            DeclareLaunchArgument("arm_kp", default_value="60.0"),
            DeclareLaunchArgument("arm_kd", default_value="4.0"),
            DeclareLaunchArgument("tail_joint_kp", default_value="50.0"),
            DeclareLaunchArgument("tail_joint_kd", default_value="3.0"),
            DeclareLaunchArgument("hold_kp", default_value="60.0"),
            DeclareLaunchArgument("hold_kd", default_value="4.0"),
            DeclareLaunchArgument("tail_hold_kp", default_value="50.0"),
            DeclareLaunchArgument("tail_hold_kd", default_value="3.0"),
            DeclareLaunchArgument("arm_goal_tolerance", default_value="0.30"),
            DeclareLaunchArgument("goal_settle_timeout", default_value="8.0"),
            DeclareLaunchArgument("hold_final_target_on_goal_timeout", default_value="true"),
            DeclareLaunchArgument("trajectory_start_hold_time", default_value="0.50"),
            DeclareLaunchArgument("trajectory_start_blend_time", default_value="1.20"),
            DeclareLaunchArgument("command_smoothing_time_constant", default_value="0.12"),
            DeclareLaunchArgument(
                "publish_rate",
                default_value="50.0",
                description="Arm LowCmd publish rate (Hz); SDK recommends >=50.",
            ),
            DeclareLaunchArgument("enable_gravity_compensation", default_value="true"),
            DeclareLaunchArgument("strip_gravity_comp_urdf", default_value="true"),
            DeclareLaunchArgument("gravity_comp_scale", default_value="0.6"),
            DeclareLaunchArgument("gravity_comp_tau_limits", default_value="12.0"),
            DeclareLaunchArgument("gravity_comp_ramp_time", default_value="2.0"),
            DeclareLaunchArgument("joint_goal_mode", default_value="direct"),
            Node(
                package="qi_robot_moveit_py",
                executable="lowstate_joint_states_bridge",
                name="lowstate_joint_states_bridge",
                output="both",
                parameters=[
                    {
                        "input_topic": "lowstate",
                        "output_topic": LaunchConfiguration("joint_state_topic"),
                        "body_dof": ParameterValue(LaunchConfiguration("body_dof"), value_type=int),
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
            IncludeLaunchDescription(
                _package_launch("qi_robot_moveit_config", "static_virtual_joint_tfs.launch.py")
            ),
            IncludeLaunchDescription(_package_launch("qi_robot_moveit_config", "rsp.launch.py")),
            IncludeLaunchDescription(
                _package_launch("qi_robot_moveit_config", "move_group.launch.py"),
                launch_arguments={"ik_backend": LaunchConfiguration("ik_backend")}.items(),
            ),
            TimerAction(
                period=LaunchConfiguration("rviz_delay"),
                actions=[
                    IncludeLaunchDescription(
                        _package_launch("qi_robot_moveit_config", "moveit_rviz.launch.py"),
                        launch_arguments={"ik_backend": LaunchConfiguration("ik_backend")}.items(),
                    )
                ],
                condition=IfCondition(LaunchConfiguration("use_rviz")),
            ),
            IncludeLaunchDescription(
                _package_launch("qi_robot_moveit_py", "moveit_mit_bridge.launch.py"),
                launch_arguments={
                    "direct_lowcmd": "true",
                    "lowcmd_topic": LaunchConfiguration("lowcmd_topic"),
                    "lowcmd_mode_ctrl": LaunchConfiguration("lowcmd_mode_ctrl"),
                    "joint_state_topic": LaunchConfiguration("joint_state_topic"),
                    "reject_zero_state_glitches": LaunchConfiguration("reject_zero_state_glitches"),
                    "zero_state_threshold": LaunchConfiguration("zero_state_threshold"),
                    "zero_state_previous_motion_threshold": LaunchConfiguration("zero_state_previous_motion_threshold"),
                    "zero_state_confirmation_frames": LaunchConfiguration("zero_state_confirmation_frames"),
                    "arm_kp": LaunchConfiguration("arm_kp"),
                    "arm_kd": LaunchConfiguration("arm_kd"),
                    "tail_joint_kp": LaunchConfiguration("tail_joint_kp"),
                    "tail_joint_kd": LaunchConfiguration("tail_joint_kd"),
                    "hold_kp": LaunchConfiguration("hold_kp"),
                    "hold_kd": LaunchConfiguration("hold_kd"),
                    "tail_hold_kp": LaunchConfiguration("tail_hold_kp"),
                    "tail_hold_kd": LaunchConfiguration("tail_hold_kd"),
                    "arm_goal_tolerance": LaunchConfiguration("arm_goal_tolerance"),
                    "goal_settle_timeout": LaunchConfiguration("goal_settle_timeout"),
                    "hold_final_target_on_goal_timeout": LaunchConfiguration("hold_final_target_on_goal_timeout"),
                    "trajectory_start_hold_time": LaunchConfiguration("trajectory_start_hold_time"),
                    "trajectory_start_blend_time": LaunchConfiguration("trajectory_start_blend_time"),
                    "command_smoothing_time_constant": LaunchConfiguration("command_smoothing_time_constant"),
                    "publish_rate": LaunchConfiguration("publish_rate"),
                    "enable_gravity_compensation": LaunchConfiguration("enable_gravity_compensation"),
                    "strip_gravity_comp_urdf": LaunchConfiguration("strip_gravity_comp_urdf"),
                    "gravity_comp_scale": LaunchConfiguration("gravity_comp_scale"),
                    "gravity_comp_tau_limits": LaunchConfiguration("gravity_comp_tau_limits"),
                    "gravity_comp_ramp_time": LaunchConfiguration("gravity_comp_ramp_time"),
                    "body_dof": LaunchConfiguration("body_dof"),
                }.items(),
            ),
            IncludeLaunchDescription(
                _package_launch("qi_robot_moveit_py", "goal_commander.launch.py"),
                launch_arguments={
                    "goal_service_name": LaunchConfiguration("goal_service_name"),
                    "reject_zero_state_glitches": LaunchConfiguration("reject_zero_state_glitches"),
                    "zero_state_threshold": LaunchConfiguration("zero_state_threshold"),
                    "zero_state_previous_motion_threshold": LaunchConfiguration("zero_state_previous_motion_threshold"),
                    "zero_state_confirmation_frames": LaunchConfiguration("zero_state_confirmation_frames"),
                    "joint_goal_mode": LaunchConfiguration("joint_goal_mode"),
                    "both_pose_goal_mode": LaunchConfiguration("both_pose_goal_mode"),
                    "cartesian_waypoint_count": LaunchConfiguration("cartesian_waypoint_count"),
                    "direct_motion_duration": LaunchConfiguration("direct_motion_duration"),
                    "direct_sample_period": LaunchConfiguration("direct_sample_period"),
                    "direct_trajectory_smoothing_window": LaunchConfiguration("direct_trajectory_smoothing_window"),
                    "direct_trajectory_smoothing_passes": LaunchConfiguration("direct_trajectory_smoothing_passes"),
                    "direct_smoothing_method": LaunchConfiguration("direct_smoothing_method"),
                    "direct_spline_anchor_stride": LaunchConfiguration("direct_spline_anchor_stride"),
                    "direct_spline_smoothing_alpha": LaunchConfiguration("direct_spline_smoothing_alpha"),
                    "direct_spline_max_joint_deviation": LaunchConfiguration("direct_spline_max_joint_deviation"),
                    "direct_velocity_scale": LaunchConfiguration("direct_velocity_scale"),
                    "direct_time_parameterization": LaunchConfiguration("direct_time_parameterization"),
                    "direct_max_velocity": LaunchConfiguration("direct_max_velocity"),
                    "direct_max_acceleration": LaunchConfiguration("direct_max_acceleration"),
                    "direct_max_jerk": LaunchConfiguration("direct_max_jerk"),
                    "direct_min_segment_duration": LaunchConfiguration("direct_min_segment_duration"),
                    "direct_max_duration_scale": LaunchConfiguration("direct_max_duration_scale"),
                    "direct_joint_jump_warn_threshold": LaunchConfiguration("direct_joint_jump_warn_threshold"),
                    "direct_ik_max_joint_step": LaunchConfiguration("direct_ik_max_joint_step"),
                    "direct_ik_jump_retry_count": LaunchConfiguration("direct_ik_jump_retry_count"),
                    "cached_place_start_tolerance": LaunchConfiguration("cached_place_start_tolerance"),
                    "cached_push_start_tolerance": LaunchConfiguration("cached_push_start_tolerance"),
                    "cached_push_goal_tolerance": LaunchConfiguration("cached_push_goal_tolerance"),
                    "cached_trajectory_start_blend_enabled": LaunchConfiguration(
                        "cached_trajectory_start_blend_enabled"
                    ),
                    "carry_x_offset": LaunchConfiguration("carry_x_offset"),
                    "post_place_push_x": LaunchConfiguration("post_place_push_x"),
                    "object_cartesian_waypoint_count": LaunchConfiguration("object_cartesian_waypoint_count"),
                    "object_roundtrip_lift_z": LaunchConfiguration("object_roundtrip_lift_z"),
                    "object_roundtrip_hold_sec": LaunchConfiguration("object_roundtrip_hold_sec"),
                    "object_to_left_tool_xyz": LaunchConfiguration("object_to_left_tool_xyz"),
                    "object_to_right_tool_xyz": LaunchConfiguration("object_to_right_tool_xyz"),
                    "object_to_tool_orientation_mode": LaunchConfiguration("object_to_tool_orientation_mode"),
                    "left_orientation_xyzw": LaunchConfiguration("left_orientation_xyzw"),
                    "right_orientation_xyzw": LaunchConfiguration("right_orientation_xyzw"),
                    "left_orientation_rpy_offset": LaunchConfiguration("left_orientation_rpy_offset"),
                    "right_orientation_rpy_offset": LaunchConfiguration("right_orientation_rpy_offset"),
                    "object_pose_orientation_mode": LaunchConfiguration("object_pose_orientation_mode"),
                    "object_path_start_mode": LaunchConfiguration("object_path_start_mode"),
                    "ik_avoid_collisions": LaunchConfiguration("ik_avoid_collisions"),
                }.items(),
            ),
            Node(
                package="qi_robot_moveit_py",
                executable="vision_grasp_sequence",
                name="manual_grasp_sequence",
                output="both",
                parameters=[
                    {
                        "detect_service_name": LaunchConfiguration("detect_service_name"),
                        "arm_goal_service_name": LaunchConfiguration("goal_service_name"),
                        "result_topic": "/both_arms/result",
                        "grasp_command_topic": LaunchConfiguration("grasp_command_topic"),
                        "place_command_topic": LaunchConfiguration("place_command_topic"),
                        "status_topic": LaunchConfiguration("status_topic"),
                        "grasp_result_topic": LaunchConfiguration("grasp_result_topic"),
                        "pose_source": "vision",
                        "visual_x_only_enabled": ParameterValue(
                            LaunchConfiguration("visual_x_only_enabled"), value_type=bool
                        ),
                        "visual_x_min": ParameterValue(LaunchConfiguration("visual_x_min"), value_type=float),
                        "visual_x_max": ParameterValue(LaunchConfiguration("visual_x_max"), value_type=float),
                        "visual_fixed_y": ParameterValue(LaunchConfiguration("visual_fixed_y"), value_type=float),
                        "visual_fixed_z": ParameterValue(LaunchConfiguration("visual_fixed_z"), value_type=float),
                        "attach_on_grasp_start": False,
                        "attach_after_grasp": False,
                        "detach_after_lift": False,
                        "wait_for_place_command": True,
                        "approach_x_offset": ParameterValue(LaunchConfiguration("approach_x_offset"), value_type=float),
                        "side_offset_y": ParameterValue(LaunchConfiguration("side_offset_y"), value_type=float),
                        "approach_z_offset": ParameterValue(LaunchConfiguration("approach_z_offset"), value_type=float),
                        "lift_z": ParameterValue(LaunchConfiguration("lift_z"), value_type=float),
                        "carry_x_offset": ParameterValue(LaunchConfiguration("carry_x_offset"), value_type=float),
                        "post_place_push_x": ParameterValue(
                            LaunchConfiguration("post_place_push_x"), value_type=float
                        ),
                        "left_orientation_xyzw": LaunchConfiguration("left_orientation_xyzw"),
                        "right_orientation_xyzw": LaunchConfiguration("right_orientation_xyzw"),
                        "left_orientation_rpy_offset": LaunchConfiguration("left_orientation_rpy_offset"),
                        "right_orientation_rpy_offset": LaunchConfiguration("right_orientation_rpy_offset"),
                        "object_orientation_xyzw": LaunchConfiguration("object_orientation_xyzw"),
                        "tool_frame_offset_z": ParameterValue(LaunchConfiguration("tool_frame_offset_z"), value_type=float),
                        "long_range_ik_max_joint_step": ParameterValue(
                            LaunchConfiguration("long_range_ik_max_joint_step"), value_type=float
                        ),
                        "precheck_grasp_ik_before_pregrasp": ParameterValue(
                            LaunchConfiguration("precheck_grasp_ik_before_pregrasp"), value_type=bool
                        ),
                        "precheck_ik_avoid_collisions": ParameterValue(
                            LaunchConfiguration("precheck_ik_avoid_collisions"), value_type=bool
                        ),
                        "precheck_ik_seed_from_pregrasp": ParameterValue(
                            LaunchConfiguration("precheck_ik_seed_from_pregrasp"), value_type=bool
                        ),
                        "use_pregrasp_joint_goal": ParameterValue(
                            LaunchConfiguration("use_pregrasp_joint_goal"), value_type=bool
                        ),
                        "pregrasp_transition_joint_positions": LaunchConfiguration(
                            "pregrasp_transition_joint_positions"
                        ),
                        "pregrasp_joint_positions": LaunchConfiguration("pregrasp_joint_positions"),
                        "home_joint_positions": LaunchConfiguration("home_joint_positions"),
                        "home_transition_joint_positions": LaunchConfiguration("home_transition_joint_positions"),
                        "allow_contact_motion_timeout": ParameterValue(
                            LaunchConfiguration("allow_contact_motion_timeout"), value_type=bool
                        ),
                        "home_retry_count": ParameterValue(LaunchConfiguration("home_retry_count"), value_type=int),
                        "home_retry_wait_sec": ParameterValue(LaunchConfiguration("home_retry_wait_sec"), value_type=float),
                        **_hand_parameters(),
                    }
                ],
                condition=IfCondition(LaunchConfiguration("start_grasp_command_node")),
            ),
        ]
    )
