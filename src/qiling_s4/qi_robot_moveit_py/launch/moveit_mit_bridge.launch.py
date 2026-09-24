from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
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

    return LaunchDescription(
        [
            DeclareLaunchArgument("arm_command_topic", default_value="/moveit_arm_command"),
            DeclareLaunchArgument("direct_lowcmd", default_value="true"),
            DeclareLaunchArgument(
                "lowcmd_topic",
                default_value="lowcmd",
                description=(
                    "ROS LowCmd topic. Real robot arm aux is lowcmd_arms "
                    "(DDS rt/lowcmd_arms). External sim typically keeps lowcmd."
                ),
            ),
            DeclareLaunchArgument(
                "lowcmd_mode_ctrl",
                default_value="0",
                description="LowCmd.mode_ctrl; unused for source on rt/lowcmd_arms, default 0.",
            ),
            DeclareLaunchArgument("joint_state_topic", default_value="/joint_states"),
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
            DeclareLaunchArgument("arm_kp", default_value="60.0"),
            DeclareLaunchArgument("arm_kd", default_value="3.0"),
            DeclareLaunchArgument("tail_joint_kp", default_value="8.0"),
            DeclareLaunchArgument("tail_joint_kd", default_value="0.4"),
            DeclareLaunchArgument("hold_kp", default_value="60.0"),
            DeclareLaunchArgument("hold_kd", default_value="3.0"),
            DeclareLaunchArgument("tail_hold_kp", default_value="4.0"),
            DeclareLaunchArgument("tail_hold_kd", default_value="0.25"),
            DeclareLaunchArgument("arm_goal_tolerance", default_value="0.18"),
            DeclareLaunchArgument("goal_settle_timeout", default_value="5.0"),
            DeclareLaunchArgument("hold_final_target_on_goal_timeout", default_value="false"),
            DeclareLaunchArgument("trajectory_start_hold_time", default_value="0.15"),
            DeclareLaunchArgument("trajectory_start_blend_time", default_value="0.25"),
            DeclareLaunchArgument("command_smoothing_time_constant", default_value="0.03"),
            DeclareLaunchArgument("arm_wait_for_initial_state", default_value="true"),
            DeclareLaunchArgument("initial_state_settle_time", default_value="0.5"),
            DeclareLaunchArgument("reject_zero_state_glitches", default_value="true"),
            DeclareLaunchArgument("zero_state_threshold", default_value="0.0001"),
            DeclareLaunchArgument("zero_state_previous_motion_threshold", default_value="0.05"),
            DeclareLaunchArgument("zero_state_confirmation_frames", default_value="3"),
            DeclareLaunchArgument(
                "publish_rate",
                default_value="60.0",
                description="Arm LowCmd publish rate (Hz). Real-robot SDK recommends >=50.",
            ),
            DeclareLaunchArgument("enable_gravity_compensation", default_value="true"),
            DeclareLaunchArgument("strip_gravity_comp_urdf", default_value="true"),
            DeclareLaunchArgument("gravity_comp_scale", default_value="1.0"),
            DeclareLaunchArgument("gravity_comp_sign", default_value="1.0"),
            DeclareLaunchArgument("gravity_comp_source", default_value="current"),
            DeclareLaunchArgument("gravity_comp_tau_limits", default_value="30.0"),
            DeclareLaunchArgument("gravity_comp_ramp_time", default_value="1.0"),
            DeclareLaunchArgument("gravity_vector", default_value="0.0,0.0,-9.81"),
            DeclareLaunchArgument("gravity_comp_debug_log_period", default_value="0.0"),
            DeclareLaunchArgument(
                "body_dof",
                default_value="38",
                description="Body actuator count: 26 for real robot (no dexterous hands), 38 for S4 sim.",
            ),
            DeclareLaunchArgument(
                "reversed_joint_names",
                default_value="",
                description=(
                    "Comma-separated joints whose MuJoCo/hardware sign differs from MoveIt URDF. "
                    "Real robot defaults are applied when body_dof<=26; external sim should set explicitly."
                ),
            ),
            Node(
                package="qi_robot_moveit_py",
                executable="moveit_mit_arm_bridge",
                name="moveit_mit_arm_bridge",
                output="screen",
                on_exit=Shutdown(reason="moveit_mit_arm_bridge exited; stopping robot stack"),
                parameters=[
                    {
                        "command_topic": LaunchConfiguration("arm_command_topic"),
                        "direct_lowcmd": ParameterValue(
                            LaunchConfiguration("direct_lowcmd"),
                            value_type=bool,
                        ),
                        "lowcmd_topic": LaunchConfiguration("lowcmd_topic"),
                        "lowcmd_mode_ctrl": ParameterValue(
                            LaunchConfiguration("lowcmd_mode_ctrl"),
                            value_type=int,
                        ),
                        "joint_state_topic": LaunchConfiguration("joint_state_topic"),
                        "left_controller_action": LaunchConfiguration("left_controller_action"),
                        "right_controller_action": LaunchConfiguration("right_controller_action"),
                        "dual_controller_action": LaunchConfiguration("dual_controller_action"),
                        "arm_kp": ParameterValue(LaunchConfiguration("arm_kp"), value_type=float),
                        "arm_kd": ParameterValue(LaunchConfiguration("arm_kd"), value_type=float),
                        "tail_joint_kp": ParameterValue(LaunchConfiguration("tail_joint_kp"), value_type=float),
                        "tail_joint_kd": ParameterValue(LaunchConfiguration("tail_joint_kd"), value_type=float),
                        "hold_kp": ParameterValue(LaunchConfiguration("hold_kp"), value_type=float),
                        "hold_kd": ParameterValue(LaunchConfiguration("hold_kd"), value_type=float),
                        "tail_hold_kp": ParameterValue(LaunchConfiguration("tail_hold_kp"), value_type=float),
                        "tail_hold_kd": ParameterValue(LaunchConfiguration("tail_hold_kd"), value_type=float),
                        "arm_goal_tolerance": ParameterValue(
                            LaunchConfiguration("arm_goal_tolerance"),
                            value_type=float,
                        ),
                        "goal_settle_timeout": ParameterValue(
                            LaunchConfiguration("goal_settle_timeout"),
                            value_type=float,
                        ),
                        "hold_final_target_on_goal_timeout": ParameterValue(
                            LaunchConfiguration("hold_final_target_on_goal_timeout"),
                            value_type=bool,
                        ),
                        "trajectory_start_hold_time": ParameterValue(
                            LaunchConfiguration("trajectory_start_hold_time"),
                            value_type=float,
                        ),
                        "trajectory_start_blend_time": ParameterValue(
                            LaunchConfiguration("trajectory_start_blend_time"),
                            value_type=float,
                        ),
                        "command_smoothing_time_constant": ParameterValue(
                            LaunchConfiguration("command_smoothing_time_constant"),
                            value_type=float,
                        ),
                        "wait_for_initial_state": ParameterValue(
                            LaunchConfiguration("arm_wait_for_initial_state"),
                            value_type=bool,
                        ),
                        "initial_state_settle_time": ParameterValue(
                            LaunchConfiguration("initial_state_settle_time"),
                            value_type=float,
                        ),
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
                        "publish_rate": ParameterValue(LaunchConfiguration("publish_rate"), value_type=float),
                        "enable_gravity_compensation": ParameterValue(
                            LaunchConfiguration("enable_gravity_compensation"),
                            value_type=bool,
                        ),
                        "strip_gravity_comp_urdf": ParameterValue(
                            LaunchConfiguration("strip_gravity_comp_urdf"),
                            value_type=bool,
                        ),
                        "robot_description_xml": moveit_config.robot_description["robot_description"],
                        "gravity_comp_scale": ParameterValue(
                            LaunchConfiguration("gravity_comp_scale"),
                            value_type=float,
                        ),
                        "gravity_comp_sign": ParameterValue(
                            LaunchConfiguration("gravity_comp_sign"),
                            value_type=float,
                        ),
                        "gravity_comp_source": LaunchConfiguration("gravity_comp_source"),
                        "gravity_comp_tau_limits": ParameterValue(
                            LaunchConfiguration("gravity_comp_tau_limits"),
                            value_type=str,
                        ),
                        "gravity_comp_ramp_time": ParameterValue(
                            LaunchConfiguration("gravity_comp_ramp_time"),
                            value_type=float,
                        ),
                        "gravity_vector": ParameterValue(
                            LaunchConfiguration("gravity_vector"),
                            value_type=str,
                        ),
                        "gravity_comp_debug_log_period": ParameterValue(
                            LaunchConfiguration("gravity_comp_debug_log_period"),
                            value_type=float,
                        ),
                        "body_dof": ParameterValue(LaunchConfiguration("body_dof"), value_type=int),
                        "reversed_joint_names": LaunchConfiguration("reversed_joint_names"),
                    }
                ],
            ),
        ]
    )
