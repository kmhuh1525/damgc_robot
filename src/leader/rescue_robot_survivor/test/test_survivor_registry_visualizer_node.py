"""Persistent Registry RViz marker contracts and reset cleanup."""

from math import inf, nan
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
from rescue_robot_interfaces.msg import SurvivorTrack, SurvivorTrackArray
from visualization_msgs.msg import Marker
import yaml

from rescue_robot_survivor.survivor_registry_visualizer_node import (
    MARKER_QOS,
    MAX_SURVIVOR_MARKER_ID,
    REGISTRY_QOS,
    SurvivorRegistryVisualizerNode,
    build_registry_marker_array,
)


def make_track(
    survivor_id,
    filtered,
    *,
    raw=(9.0, 9.0, 9.0),
    visible=True,
    status=SurvivorTrack.STATUS_CONFIRMED,
):
    """Create one typed Registry track."""
    track = SurvivorTrack()
    track.id = survivor_id
    track.raw_position.x, track.raw_position.y, track.raw_position.z = raw
    (track.filtered_position.x,
     track.filtered_position.y,
     track.filtered_position.z) = filtered
    track.visible = visible
    track.status = status
    track.observation_count = 3
    return track


def make_array(tracks, frame_id="map"):
    """Create one stamped Registry snapshot."""
    message = SurvivorTrackArray()
    message.header.frame_id = frame_id
    message.header.stamp.sec = 123
    message.header.stamp.nanosec = 456
    message.tracks = list(tracks)
    return message


def build(message, previous_ids=frozenset()):
    """Use the installed visualizer defaults."""
    return build_registry_marker_array(
        message,
        previous_ids,
        "survivor_registry",
        0.20,
        0.18,
        1.0,
    )


def test_visible_track_uses_filtered_position_id_xyz_and_status_text():
    message = make_array([
        make_track(1, (2.53, -0.06, 0.51)),
    ])

    markers, ids, invalid = build(message)

    assert ids == {1}
    assert invalid == 0
    assert len(markers.markers) == 2
    sphere, label = markers.markers
    assert [(marker.ns, marker.id, marker.action)
            for marker in markers.markers] == [
        ("survivor_registry", 2, Marker.ADD),
        ("survivor_registry", 3, Marker.ADD),
    ]
    assert sphere.type == Marker.SPHERE
    assert label.type == Marker.TEXT_VIEW_FACING
    assert all(marker.header.frame_id == "map"
               for marker in markers.markers)
    assert all(marker.header.stamp == message.header.stamp
               for marker in markers.markers)
    assert (sphere.pose.position.x, sphere.pose.position.y,
            sphere.pose.position.z) == (2.53, -0.06, 0.51)
    assert (label.pose.position.x, label.pose.position.y,
            label.pose.position.z) == (2.53, -0.06, 1.51)
    assert label.text == (
        "Survivor #1\nX: 2.53\nY: -0.06\nZ: 0.51\nVISIBLE"
    )
    assert sphere.pose.position.x != message.tracks[0].raw_position.x
    assert (sphere.color.r, sphere.color.g,
            sphere.color.b, sphere.color.a) == (1.0, 0.35, 0.10, 1.0)
    assert (label.color.r, label.color.g,
            label.color.b, label.color.a) == (1.0, 1.0, 1.0, 1.0)
    assert all((marker.lifetime.sec, marker.lifetime.nanosec) == (0, 0)
               for marker in markers.markers)


def test_lost_track_uses_last_position_yellow_sphere_and_white_text():
    message = make_array([
        make_track(
            4,
            (1.0, 2.0, 0.5),
            visible=False,
            status=SurvivorTrack.STATUS_LOST,
        ),
    ])

    markers, ids, invalid = build(message)

    sphere, label = markers.markers
    assert ids == {4}
    assert invalid == 0
    assert (sphere.id, label.id) == (8, 9)
    assert (sphere.pose.position.x, sphere.pose.position.y,
            sphere.pose.position.z) == (1.0, 2.0, 0.5)
    assert (label.pose.position.x, label.pose.position.y,
            label.pose.position.z) == (1.0, 2.0, 1.5)
    assert "Survivor #4" in label.text
    assert label.text.endswith("LAST SEEN")
    assert (sphere.color.r, sphere.color.g,
            sphere.color.b, sphere.color.a) == (1.0, 0.85, 0.0, 1.0)
    assert (label.color.r, label.color.g,
            label.color.b, label.color.a) == (1.0, 1.0, 1.0, 1.0)
    assert all((marker.lifetime.sec, marker.lifetime.nanosec) == (0, 0)
               for marker in markers.markers)
    assert all(marker.action == Marker.ADD for marker in markers.markers)


def test_empty_registry_always_publishes_deleteall():
    markers, ids, invalid = build(make_array([]), {1, 2})

    assert ids == set()
    assert invalid == 0
    assert len(markers.markers) == 1
    assert markers.markers[0].action == Marker.DELETEALL
    assert markers.markers[0].header.frame_id == "map"
    assert markers.markers[0].ns == "survivor_registry"


def test_removed_track_gets_deterministic_sphere_and_text_deletes():
    message = make_array([make_track(2, (2.0, 0.0, 0.5))])

    markers, ids, invalid = build(message, {1, 2})

    assert ids == {2}
    assert invalid == 0
    assert [(marker.id, marker.action) for marker in markers.markers] == [
        (4, Marker.ADD),
        (5, Marker.ADD),
        (2, Marker.DELETE),
        (3, Marker.DELETE),
    ]


def test_multiple_tracks_and_reordered_array_keep_id_based_markers():
    first = make_array([
        make_track(
            2,
            (2.0, 0.0, 0.5),
            visible=False,
            status=SurvivorTrack.STATUS_LOST,
        ),
        make_track(1, (1.0, 0.0, 0.5)),
    ])
    second = make_array([
        make_track(1, (1.1, 0.0, 0.5)),
        make_track(
            2,
            (2.1, 0.0, 0.5),
            visible=False,
            status=SurvivorTrack.STATUS_LOST,
        ),
    ])

    first_markers, first_ids, _ = build(first)
    second_markers, second_ids, _ = build(second, first_ids)

    assert first_ids == second_ids == {1, 2}
    assert [marker.id for marker in first_markers.markers] == [2, 3, 4, 5]
    assert [marker.id for marker in second_markers.markers] == [2, 3, 4, 5]
    assert all(marker.action == Marker.ADD
               for marker in second_markers.markers)
    visible_sphere, visible_text, lost_sphere, lost_text = (
        second_markers.markers
    )
    assert (visible_sphere.color.r, visible_sphere.color.g,
            visible_sphere.color.b, visible_sphere.color.a) == (
                1.0, 0.35, 0.10, 1.0
            )
    assert (lost_sphere.color.r, lost_sphere.color.g,
            lost_sphere.color.b, lost_sphere.color.a) == (
                1.0, 0.85, 0.0, 1.0
            )
    for label in (visible_text, lost_text):
        assert (label.color.r, label.color.g,
                label.color.b, label.color.a) == (1.0, 1.0, 1.0, 1.0)


def test_invalid_id_status_position_and_duplicate_are_skipped():
    tracks = [
        make_track(0, (0.0, 0.0, 0.0)),
        make_track(MAX_SURVIVOR_MARKER_ID + 1, (0.0, 0.0, 0.0)),
        make_track(1, (nan, 0.0, 0.0)),
        make_track(2, (0.0, inf, 0.0)),
        make_track(3, (3.0, 0.0, 0.5), status=0),
        make_track(4, (4.0, 0.0, 0.5)),
        make_track(4, (4.1, 0.0, 0.5)),
    ]

    markers, ids, invalid = build(make_array(tracks))

    assert ids == {4}
    assert invalid == 6
    assert [marker.id for marker in markers.markers] == [8, 9]


def test_callback_updates_active_ids_and_rejects_wrong_frame():
    harness = SimpleNamespace(
        _map_frame="map",
        _active_ids={1, 2},
        _marker_namespace="survivor_registry",
        _marker_scale=0.20,
        _text_height=0.18,
        _text_z_offset=1.0,
        _publisher=Mock(),
        _warn=Mock(),
    )
    message = make_array([make_track(2, (2.0, 0.0, 0.5))])

    SurvivorRegistryVisualizerNode._tracks_callback(harness, message)
    assert harness._active_ids == {2}
    assert harness._publisher.publish.call_count == 1

    empty = make_array([])
    SurvivorRegistryVisualizerNode._tracks_callback(harness, empty)
    assert harness._active_ids == set()
    published = harness._publisher.publish.call_args.args[0]
    assert published.markers[0].action == Marker.DELETEALL

    wrong_frame = make_array([], frame_id="odom")
    harness._publisher.publish.reset_mock()
    SurvivorRegistryVisualizerNode._tracks_callback(
        harness, wrong_frame
    )
    harness._publisher.publish.assert_not_called()
    harness._warn.assert_called_once()


def test_visualizer_qos_is_transient_local_and_reliable():
    for qos in (REGISTRY_QOS, MARKER_QOS):
        assert qos.depth == 1
        assert qos.reliability == ReliabilityPolicy.RELIABLE
        assert qos.durability == DurabilityPolicy.TRANSIENT_LOCAL


def test_invalid_parameters_are_rejected():
    valid = dict(
        _input_topic="/leader/survivor/tracks",
        _output_topic="/leader/survivor/registry_markers",
        _map_frame="map",
        _marker_namespace="survivor_registry",
        _marker_scale=0.20,
        _text_height=0.18,
        _text_z_offset=1.0,
    )
    for field, value in (
        ("_input_topic", ""),
        ("_output_topic", ""),
        ("_map_frame", ""),
        ("_marker_namespace", ""),
        ("_marker_scale", 0.0),
        ("_marker_scale", inf),
        ("_text_height", -1.0),
        ("_text_z_offset", -0.1),
    ):
        values = valid.copy()
        values[field] = value
        try:
            SurvivorRegistryVisualizerNode._validate_parameters(
                SimpleNamespace(**values)
            )
        except ValueError:
            pass
        else:
            raise AssertionError(f"{field}={value} should be rejected")

    values = valid.copy()
    values["_output_topic"] = values["_input_topic"]
    try:
        SurvivorRegistryVisualizerNode._validate_parameters(
            SimpleNamespace(**values)
        )
    except ValueError:
        pass
    else:
        raise AssertionError("input and output topics must differ")


def test_rviz_config_preserves_raw_and_adds_registry_display():
    workspace_root = Path(__file__).resolve().parents[4]
    config_path = workspace_root / "rviz/vslam_nvblox.rviz"
    source = config_path.read_text(encoding="utf-8")
    parsed = yaml.safe_load(source)

    assert parsed["Visualization Manager"]["Global Options"][
        "Fixed Frame"
    ] == "odom"
    assert "Name: NvbloxMesh" in source
    assert "Name: Survivor Raw" in source
    assert "Value: /leader/survivor/map_markers" in source
    assert "Name: Survivor Registry" in source
    assert "Value: /leader/survivor/registry_markers" in source
    assert "Durability Policy: Transient Local" in source
