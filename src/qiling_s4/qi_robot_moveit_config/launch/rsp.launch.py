from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("qi_robot_dual_arm_full_fixed", package_name="qi_robot_moveit_config")
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .to_moveit_configs()
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("publish_frequency", default_value="15.0"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                respawn=True,
                output="screen",
                parameters=[
                    moveit_config.robot_description,
                    {"publish_frequency": LaunchConfiguration("publish_frequency")},
                ],
                remappings=[("/tf", "/moveit/tf"), ("/tf_static", "/moveit/tf_static")],
            ),
        ]
    )
