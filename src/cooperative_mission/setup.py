from glob import glob

from setuptools import find_packages, setup


package_name = "cooperative_mission"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="maze",
    maintainer_email="maze@todo.todo",
    description=(
        "Leader/Follower mission coordinator for AprilTag grasp, synchronized "
        "lift and cooperative straight transport."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "leader_mission_node = cooperative_mission.leader_mission_node:main",
            "follower_mission_node = cooperative_mission.follower_mission_node:main",
            "cooperative_demo_plan_node = cooperative_mission.demo_plan_node:main",
            "cooperative_object_path_planner_node = cooperative_mission.object_path_planner_node:main",
            "cooperative_leader_path_adapter_node = cooperative_mission.leader_path_adapter_node:main",
            "cooperative_path_tracking_preview_node = cooperative_mission.path_tracking_preview_node:main",
            "cooperative_path_preview_node = cooperative_mission.path_preview_node:main",
        ],
    },
)
