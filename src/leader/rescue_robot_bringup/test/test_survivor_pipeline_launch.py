"""Contracts for the installed survivor pipeline composition."""

import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch_ros.actions import Node


LAUNCH_FILE = (
    Path(__file__).resolve().parents[1]
    / "launch/survivor_pipeline.launch.py"
)
EXPECTED = {
    "survivor_camera_processing.launch.py": {
        ("rescue_robot_apriltag", "camera_info_qos_bridge.py"),
        ("image_proc", "rectify_node"),
    },
    "survivor_map_transform.launch.py": {
        ("rescue_robot_survivor", "survivor_map_transform_node"),
    },
    "survivor_map_visualizer.launch.py": {
        ("rescue_robot_survivor", "survivor_map_visualizer_node"),
    },
    "survivor_registry.launch.py": {
        ("rescue_robot_survivor", "survivor_registry_node"),
        ("rescue_robot_survivor", "survivor_registry_visualizer_node"),
    },
}


def _description():
    spec = importlib.util.spec_from_file_location(
        "survivor_pipeline", LAUNCH_FILE
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def test_installed_children_and_node_ownership():
    description = _description()
    assert isinstance(description, LaunchDescription)
    assert not any(isinstance(entity, Node) for entity in description.entities)

    groups = [
        entity for entity in description.entities
        if isinstance(entity, GroupAction)
    ]
    assert len(groups) == len(EXPECTED)
    found = {}
    for group in groups:
        # Each child gets its own launch configuration defaults.
        assert any(
            type(entity).__name__ == "ResetLaunchConfigurations"
            for entity in group.get_sub_entities()
        )
        includes = [
            entity for entity in group.get_sub_entities()
            if isinstance(entity, IncludeLaunchDescription)
        ]
        assert len(includes) == 1
        source = includes[0].launch_description_source
        child = source.get_launch_description(LaunchContext())
        path = Path(source.location)
        assert path.is_file()
        package_name = (
            "rescue_robot_bringup"
            if path.name == "survivor_camera_processing.launch.py"
            else "rescue_robot_survivor"
        )
        assert path.parent == (
            Path(get_package_share_directory(package_name)) / "launch"
        )
        found[path.name] = {
            (entity.node_package, entity.node_executable)
            for entity in child.entities if isinstance(entity, Node)
        }
    assert found == EXPECTED


def test_raw_visualizer_switch_only_controls_raw_branch():
    description = _description()
    declarations = [
        entity for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    ]
    assert len(declarations) == 1
    assert declarations[0].name == "enable_raw_visualizer"
    default = "".join(
        part.perform(LaunchContext())
        for part in declarations[0].default_value
    )
    assert default == "true"

    for enabled in ("true", "false"):
        context = LaunchContext()
        context.launch_configurations["enable_raw_visualizer"] = enabled
        active = {}
        for group in description.entities:
            if not isinstance(group, GroupAction):
                continue
            include = next(
                entity for entity in group.get_sub_entities()
                if isinstance(entity, IncludeLaunchDescription)
            )
            include.launch_description_source.get_launch_description(context)
            name = Path(include.launch_description_source.location).name
            active[name] = (
                group.condition is None or group.condition.evaluate(context)
            )
        assert active["survivor_map_visualizer.launch.py"] is (
            enabled == "true"
        )
        assert all(
            value for name, value in active.items()
            if name != "survivor_map_visualizer.launch.py"
        )


def test_integrated_mapping_has_one_vslam_tf_owner_and_no_ekf():
    path = LAUNCH_FILE.with_name("nvblox_vslam_realsense.launch.py")
    spec = importlib.util.spec_from_file_location("mapping_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    includes = [
        entity for entity in module.generate_launch_description().entities
        if isinstance(entity, IncludeLaunchDescription)
    ]
    assert len(includes) == 3
    for entity in includes:
        entity.launch_description_source.get_launch_description(LaunchContext())
    assert sorted(
        Path(entity.launch_description_source.location).name
        for entity in includes
    ) == sorted([
        "visual_slam_realsense.launch.py",
        "nvblox_realsense.launch.py",
        "nvblox_nav2.launch.py",
    ])
    vslam = next(entity for entity in includes if entity.launch_arguments)
    assert dict(vslam.launch_arguments)["publish_map_to_odom_tf"] == "true"
    assert dict(vslam.launch_arguments)["publish_odom_to_base_tf"] == "true"
