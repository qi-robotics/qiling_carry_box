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
    return [Node(
        package="rviz2", executable="rviz2", output="log",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[moveit_config.planning_pipelines, moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits],
        remappings=[("/tf", "/moveit/tf"), ("/tf_static", "/moveit/tf_static")],
    )]


def generate_launch_description():
    default_rviz = str(
        MoveItConfigsBuilder("qi_robot_dual_arm_full_fixed", package_name="qi_robot_moveit_config")
        .to_moveit_configs().package_path / "config/moveit.rviz"
    )
    return LaunchDescription([
        DeclareBooleanLaunchArg("debug", default_value=False),
        DeclareLaunchArgument("ik_backend", default_value="pick_ik"),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz),
        OpaqueFunction(function=_launch_setup),
    ])
