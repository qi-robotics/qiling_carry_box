from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_static_virtual_joint_tfs_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("qi_robot_dual_arm_full_fixed", package_name="qi_robot_moveit_config")
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .to_moveit_configs()
    )
    return generate_static_virtual_joint_tfs_launch(moveit_config)
