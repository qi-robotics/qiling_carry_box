from setuptools import find_packages, setup

package_name = "qi_robot_moveit_py"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/config", ["config/ik_seed_presets.yaml"]),
        (f"share/{package_name}/launch", ["launch/goal_commander.launch.py", "launch/moveit_mit_bridge.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="zfy",
    maintainer_email="user@example.com",
    description="通过 ROS2 话题驱动 MoveIt 完成双臂规划和控制的 Python 节点。",
    license="BSD",
    entry_points={
        "console_scripts": [
            "moveit_goal_commander = qi_robot_moveit_py.moveit_goal_commander:main",
            "moveit_mit_arm_bridge = qi_robot_moveit_py.moveit_mit_arm_bridge:main",
            "vision_grasp_sequence = qi_robot_moveit_py.vision_grasp_sequence:main",
            "lowstate_joint_states_bridge = qi_robot_moveit_py.lowstate_joint_states_bridge:main",
            "lowcmd_joint_states_echo = qi_robot_moveit_py.lowcmd_joint_states_echo:main",
            "ik_sweep_tester = qi_robot_moveit_py.ik_sweep_tester:main",
        ],
    },
)
