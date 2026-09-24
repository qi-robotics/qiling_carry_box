from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context):
    ik_backend = LaunchConfiguration("ik_backend").perform(context)
    if ik_backend not in {"pick_ik", "pinocchio"}:
        raise RuntimeError("ik_backend must be 'pick_ik' or 'pinocchio'")
    moveit_config = (
        MoveItConfigsBuilder("qi_robot_dual_arm_full_fixed", package_name="qi_robot_moveit_config")
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .robot_description_kinematics(file_path=f"config/kinematics_{ik_backend}.yaml")
        .to_moveit_configs()
    )

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        "capabilities": (
            LaunchConfiguration("capabilities").perform(context)
            or moveit_config.move_group_capabilities["capabilities"]
        ),
        "disable_capabilities": (
            LaunchConfiguration("disable_capabilities").perform(context)
            or moveit_config.move_group_capabilities["disable_capabilities"]
        ),
        "publish_planning_scene": LaunchConfiguration("publish_monitored_planning_scene"),
        "publish_geometry_updates": LaunchConfiguration("publish_monitored_planning_scene"),
        "publish_state_updates": LaunchConfiguration("publish_monitored_planning_scene"),
        "publish_transforms_updates": LaunchConfiguration("publish_monitored_planning_scene"),
        "monitor_dynamics": False,
        "trajectory_execution.allowed_execution_duration_scaling": 2.0,
        "trajectory_execution.allowed_goal_duration_margin": 3.0,
        "trajectory_execution.allowed_start_tolerance": 0.02,
    }

    return [
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[moveit_config.to_dict(), move_group_configuration],
            remappings=[("/tf", "/moveit/tf"), ("/tf_static", "/moveit/tf_static")],
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("ik_backend", default_value="pick_ik"),
        DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True),
        DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True),
        DeclareLaunchArgument("capabilities", default_value=""),
        DeclareLaunchArgument("disable_capabilities", default_value=""),
        OpaqueFunction(function=_launch_setup),
    ])
