from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("qi_robot_dual_arm_full_fixed", package_name="qi_robot_moveit_config")
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .to_moveit_configs()
    )

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        "capabilities": ParameterValue(LaunchConfiguration("capabilities"), value_type=str),
        "disable_capabilities": ParameterValue(LaunchConfiguration("disable_capabilities"), value_type=str),
        "publish_planning_scene": LaunchConfiguration("publish_monitored_planning_scene"),
        "publish_geometry_updates": LaunchConfiguration("publish_monitored_planning_scene"),
        "publish_state_updates": LaunchConfiguration("publish_monitored_planning_scene"),
        "publish_transforms_updates": LaunchConfiguration("publish_monitored_planning_scene"),
        "monitor_dynamics": False,
        "trajectory_execution.allowed_execution_duration_scaling": 2.0,
        "trajectory_execution.allowed_goal_duration_margin": 3.0,
        "trajectory_execution.allowed_start_tolerance": 0.02,
    }

    return LaunchDescription(
        [
            DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True),
            DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True),
            DeclareLaunchArgument(
                "capabilities",
                default_value=moveit_config.move_group_capabilities["capabilities"],
            ),
            DeclareLaunchArgument(
                "disable_capabilities",
                default_value=moveit_config.move_group_capabilities["disable_capabilities"],
            ),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    move_group_configuration,
                ],
                remappings=[("/tf", "/moveit/tf"), ("/tf_static", "/moveit/tf_static")],
            ),
        ]
    )
