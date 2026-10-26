"""Follower AprilTag perception and coherent hybrid alignment commands."""

import time
from math import atan2, cos, hypot, isfinite, radians, sin, sqrt
from typing import List, Optional, Sequence, Tuple

import rclpy
from follower_alignment_msgs.msg import FollowerAlignmentCommand
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, Float64, Int32, String
from tf2_ros import Buffer, TransformException, TransformListener

from follower_supply_perception.approach_logic import (
    ApproachState,
    ApproachStateMachine,
    ApproachThresholds,
    MedianTranslationFilter,
    RelativeMeasurement,
    TagObservation,
    is_valid_translation,
    normalize_quaternion,
    select_observation,
)
from follower_supply_perception.base_alignment_logic import (
    AlignmentDecision,
    BaseAlignmentMeasurement,
    BaseAlignmentStateMachine,
    BaseAlignmentThresholds,
    ControlMode,
    compute_blind_remaining_distance,
    compute_forward_progress,
    is_blind_final_approach_eligible,
    normalize_angle,
)
from follower_supply_perception.base_pose import (
    PlanarNormalMedianFilter,
    TargetGeometry,
    compute_base_metrics,
    compute_target_geometry,
    is_fresh_timestamp,
    rotate_tag_z_to_base_xy,
    select_robot_facing_normal,
    transform_pose_preserving_stamp,
)


class AprilTagApproachNode(Node):
    """Look up Follower tag TF and publish one coherent alignment generation."""

    def __init__(self) -> None:
        super().__init__("apriltag_approach")
        self._declare_parameters()
        self._load_and_validate_parameters()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._detected_pub = self.create_publisher(Bool, "supply/detected", 10)
        self._tag_id_pub = self.create_publisher(Int32, "supply/tag_id", 10)
        self._pose_pub = self.create_publisher(PoseStamped, "supply/relative_pose", 10)
        self._distance_pub = self.create_publisher(Float64, "supply/distance", 10)
        self._lateral_pub = self.create_publisher(
            Float64, "supply/lateral_error", 10
        )
        self._straight_pub = self.create_publisher(
            Float64, "supply/straight_distance", 10
        )
        self._angle_pub = self.create_publisher(Float64, "supply/angle", 10)
        self._state_pub = self.create_publisher(String, "alignment/state", 10)
        self._base_pose_pub = self.create_publisher(
            PoseStamped, "supply/base_relative_pose", 10
        )
        self._base_forward_pub = self.create_publisher(
            Float64, "supply/base_forward_distance", 10
        )
        self._base_lateral_pub = self.create_publisher(
            Float64, "supply/base_lateral_error", 10
        )
        self._base_bearing_pub = self.create_publisher(
            Float64, "supply/base_bearing", 10
        )
        self._base_state_pub = self.create_publisher(
            String, "base_alignment/state", 10
        )
        self._control_mode_pub = self.create_publisher(
            String, "alignment/control_mode", 10
        )
        self._normal_heading_pub = self.create_publisher(
            Float64, "alignment/tag_normal_heading", 10
        )
        self._prealign_target_pub = self.create_publisher(
            PoseStamped, "alignment/prealign_target_pose", 10
        )
        self._final_target_pub = self.create_publisher(
            PoseStamped, "alignment/final_target_pose", 10
        )
        self._control_target_pub = self.create_publisher(
            PoseStamped, "alignment/control_target_pose", 10
        )
        self._command_pub = self.create_publisher(
            FollowerAlignmentCommand, "alignment/command", 10
        )
        self._final_position_error_pub = self.create_publisher(
            Float64, "alignment/final_position_error", 10
        )
        self._final_yaw_error_pub = self.create_publisher(
            Float64, "alignment/final_yaw_error", 10
        )
        self._blind_active_pub = self.create_publisher(
            Bool, "alignment/blind_final_approach_active", 10
        )
        self._last_valid_tag_x_pub = self.create_publisher(
            Float64, "alignment/last_valid_tag_x", 10
        )
        self._blind_distance_pub = self.create_publisher(
            Float64, "alignment/blind_planned_distance", 10
        )
        self._odom_progress_pub = self.create_publisher(
            Float64, "alignment/odom_forward_progress", 10
        )
        self._odom_sub = self.create_subscription(
            Odometry, self._odom_topic, self._on_odom, 10
        )

        self._translation_filter = MedianTranslationFilter(self._filter_window)
        self._normal_filter = PlanarNormalMedianFilter(self._filter_window)
        self._state_machine = ApproachStateMachine(
            ApproachThresholds(
                self._target_distance,
                self._distance_tolerance,
                self._lateral_tolerance,
                self._angle_tolerance_deg,
                self._stable_time,
            )
        )
        self._base_state_machine = BaseAlignmentStateMachine(
            self._base_thresholds()
        )
        self._active_tag_id: Optional[int] = None
        self._last_logged_state: Optional[ApproachState] = None
        self._last_logged_base_state: Optional[ApproachState] = None
        self._last_processed_observation_stamps = {}
        self._last_observation_receipts = {}
        self._last_valid_tag_x: Optional[float] = None
        self._last_valid_receipt: Optional[float] = None
        self._last_valid_yaw_error: Optional[float] = None
        self._last_valid_cross_track: Optional[float] = None
        self._last_fresh_final_observation_receipt: Optional[float] = None
        self._final_approach_grace_eligible = False
        self._final_approach_grace_active = False
        self._last_odom: Optional[Tuple[float, float, float, float, float]] = None
        self._blind_active = False
        self._blind_completed = False
        self._blind_planned_distance = 0.0
        self._blind_start_odom: Optional[Tuple[float, float, float]] = None
        self._blind_previous_odom: Optional[Tuple[float, float, float]] = None
        self._blind_start_receipt: Optional[float] = None
        self._last_odom_progress = 0.0
        self._approach_enabled_sub = self.create_subscription(
            Bool, "approach/enabled", self._on_approach_enabled, 10
        )
        self._timer = self.create_timer(1.0 / self._publish_rate, self._on_timer)

    def _declare_parameters(self) -> None:
        self.declare_parameter(
            "source_frame", "follower/follower_camera_optical_frame"
        )
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tf_lookup_timeout", 0.05)
        self.declare_parameter("tag_frame_pattern", "follower/tag36h11:{id}")
        self.declare_parameter("target_tag_id", 0)
        self.declare_parameter("allowed_tag_ids", [0, 1, 2])
        self.declare_parameter("selection_mode", "priority")
        self.declare_parameter("target_distance", 0.15)
        self.declare_parameter("distance_tolerance", 0.02)
        self.declare_parameter("lateral_tolerance", 0.02)
        self.declare_parameter("angle_tolerance_deg", 5.0)
        self.declare_parameter("tag_timeout", 2.0)
        self.declare_parameter("tag_receipt_timeout", 0.35)
        self.declare_parameter("observation_hold_time", 0.25)
        self.declare_parameter("align_tag_normal", True)
        self.declare_parameter("stable_time", 0.8)
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("filter_window", 5)
        self.declare_parameter("base_target_forward", 0.25)
        self.declare_parameter("base_forward_tolerance", 0.04)
        self.declare_parameter("base_lateral_tolerance", 0.04)
        self.declare_parameter("base_bearing_tolerance_deg", 8.0)
        self.declare_parameter("base_stable_time", 0.30)
        self.declare_parameter("aligned_confirm_samples", 3)
        self.declare_parameter("stabilizing_tag_loss_grace_sec", 0.30)
        self.declare_parameter("final_approach_tag_loss_grace_sec", 0.30)
        self.declare_parameter("pre_align_distance", 0.35)
        self.declare_parameter("orientation_engage_distance", 0.40)
        self.declare_parameter("orientation_disengage_distance", 0.43)
        self.declare_parameter("turn_enter_error_deg", 8.0)
        self.declare_parameter("turn_exit_error_deg", 3.0)
        self.declare_parameter("tag_recenter_enter_deg", 18.0)
        self.declare_parameter("tag_recenter_exit_deg", 11.0)
        self.declare_parameter("near_normal_correction_limit_deg", 6.0)
        self.declare_parameter("final_realign_yaw_error_deg", 12.0)
        self.declare_parameter("blind_final_approach_enabled", False)
        self.declare_parameter("blind_activation_max_tag_x", 0.35)
        self.declare_parameter("blind_max_distance", 0.10)
        self.declare_parameter("blind_last_tag_max_age", 0.25)
        self.declare_parameter("blind_handoff_max_age", 0.40)
        self.declare_parameter("blind_max_duration", 5.0)
        self.declare_parameter("blind_odom_timeout", 0.25)
        self.declare_parameter("blind_max_odom_step", 0.05)
        self.declare_parameter("blind_max_lateral_deviation", 0.03)
        self.declare_parameter("blind_max_yaw_deviation_deg", 12.0)
        self.declare_parameter("blind_reverse_tolerance", 0.01)
        self.declare_parameter("odom_topic", "/follower/odom/raw")

    def _load_and_validate_parameters(self) -> None:
        def value(name):
            return self.get_parameter(name).value

        self._source_frame = str(value("source_frame"))
        self._base_frame = str(value("base_frame"))
        self._tf_lookup_timeout = float(value("tf_lookup_timeout"))
        self._tag_frame_pattern = str(value("tag_frame_pattern"))
        self._target_tag_id = int(value("target_tag_id"))
        self._allowed_tag_ids = [int(tag_id) for tag_id in value("allowed_tag_ids")]
        self._selection_mode = str(value("selection_mode"))
        self._target_distance = float(value("target_distance"))
        self._distance_tolerance = float(value("distance_tolerance"))
        self._lateral_tolerance = float(value("lateral_tolerance"))
        self._angle_tolerance_deg = float(value("angle_tolerance_deg"))
        self._tag_timeout = float(value("tag_timeout"))
        self._tag_receipt_timeout = float(value("tag_receipt_timeout"))
        self._observation_hold_time = float(value("observation_hold_time"))
        self._align_tag_normal = bool(value("align_tag_normal"))
        self._stable_time = float(value("stable_time"))
        self._publish_rate = float(value("publish_rate"))
        self._filter_window = int(value("filter_window"))
        self._base_target_forward = float(value("base_target_forward"))
        self._base_forward_tolerance = float(value("base_forward_tolerance"))
        self._base_lateral_tolerance = float(value("base_lateral_tolerance"))
        self._base_bearing_tolerance_deg = float(
            value("base_bearing_tolerance_deg")
        )
        self._base_stable_time = float(value("base_stable_time"))
        self._aligned_confirm_samples = int(value("aligned_confirm_samples"))
        self._stabilizing_tag_loss_grace_sec = float(
            value("stabilizing_tag_loss_grace_sec")
        )
        self._final_approach_tag_loss_grace_sec = float(
            value("final_approach_tag_loss_grace_sec")
        )
        self._pre_align_distance = float(value("pre_align_distance"))
        self._orientation_engage_distance = float(value("orientation_engage_distance"))
        self._orientation_disengage_distance = float(
            value("orientation_disengage_distance")
        )
        self._turn_enter_error_deg = float(value("turn_enter_error_deg"))
        self._turn_exit_error_deg = float(value("turn_exit_error_deg"))
        self._tag_recenter_enter_deg = float(value("tag_recenter_enter_deg"))
        self._tag_recenter_exit_deg = float(value("tag_recenter_exit_deg"))
        self._near_normal_correction_limit_deg = float(
            value("near_normal_correction_limit_deg")
        )
        self._final_realign_yaw_error_deg = float(
            value("final_realign_yaw_error_deg")
        )
        self._blind_final_approach_enabled = bool(
            value("blind_final_approach_enabled")
        )
        self._blind_activation_max_tag_x = float(value("blind_activation_max_tag_x"))
        self._blind_max_distance = float(value("blind_max_distance"))
        self._blind_last_tag_max_age = float(value("blind_last_tag_max_age"))
        self._blind_handoff_max_age = float(value("blind_handoff_max_age"))
        self._blind_max_duration = float(value("blind_max_duration"))
        self._blind_odom_timeout = float(value("blind_odom_timeout"))
        self._blind_max_odom_step = float(value("blind_max_odom_step"))
        self._blind_max_lateral_deviation = float(
            value("blind_max_lateral_deviation")
        )
        self._blind_max_yaw_deviation_deg = float(
            value("blind_max_yaw_deviation_deg")
        )
        self._blind_reverse_tolerance = float(value("blind_reverse_tolerance"))
        self._odom_topic = str(value("odom_topic"))
        self._validate_parameters()

    def _base_thresholds(self) -> BaseAlignmentThresholds:
        return BaseAlignmentThresholds(
            orientation_engage_distance=self._orientation_engage_distance,
            orientation_disengage_distance=self._orientation_disengage_distance,
            turn_enter_error_deg=self._turn_enter_error_deg,
            turn_exit_error_deg=self._turn_exit_error_deg,
            tag_recenter_enter_deg=self._tag_recenter_enter_deg,
            tag_recenter_exit_deg=self._tag_recenter_exit_deg,
            near_normal_correction_limit_deg=self._near_normal_correction_limit_deg,
            pre_align_position_tolerance=self._base_forward_tolerance,
            final_forward_tolerance=self._base_forward_tolerance,
            final_lateral_tolerance=self._base_lateral_tolerance,
            final_yaw_tolerance_deg=self._base_bearing_tolerance_deg,
            final_realign_yaw_error_deg=self._final_realign_yaw_error_deg,
            stable_time=self._base_stable_time,
            sample_timeout=self._tag_receipt_timeout,
            aligned_confirm_samples=self._aligned_confirm_samples,
            stabilizing_tag_loss_grace_sec=self._stabilizing_tag_loss_grace_sec,
        )

    def _validate_parameters(self) -> None:
        if not self._source_frame or not self._base_frame or not self._odom_topic:
            raise ValueError("source_frame, base_frame, and odom_topic must not be empty")
        if "{id}" not in self._tag_frame_pattern:
            raise ValueError("tag_frame_pattern must contain '{id}'")
        self._tag_frame_pattern.format(id=0)
        if self._target_tag_id < -1:
            raise ValueError("target_tag_id must be -1 or non-negative")
        if any(tag_id < 0 for tag_id in self._allowed_tag_ids):
            raise ValueError("allowed_tag_ids must be non-negative")
        if len(set(self._allowed_tag_ids)) != len(self._allowed_tag_ids):
            raise ValueError("allowed_tag_ids must not contain duplicates")
        if self._target_tag_id == -1 and not self._allowed_tag_ids:
            raise ValueError("allowed_tag_ids must not be empty in multi-tag mode")
        if self._selection_mode not in ("priority", "nearest"):
            raise ValueError("selection_mode must be priority or nearest")
        numeric = (
            self._target_distance,
            self._distance_tolerance,
            self._lateral_tolerance,
            self._angle_tolerance_deg,
            self._tag_timeout,
            self._tag_receipt_timeout,
            self._stable_time,
            self._publish_rate,
            self._tf_lookup_timeout,
            self._base_target_forward,
            self._pre_align_distance,
            self._blind_activation_max_tag_x,
            self._blind_max_distance,
            self._blind_last_tag_max_age,
            self._blind_handoff_max_age,
            self._blind_max_duration,
            self._blind_odom_timeout,
            self._blind_max_odom_step,
            self._blind_max_lateral_deviation,
            self._blind_max_yaw_deviation_deg,
            self._blind_reverse_tolerance,
            self._stabilizing_tag_loss_grace_sec,
            self._final_approach_tag_loss_grace_sec,
        )
        if not all(isfinite(item) for item in numeric):
            raise ValueError("Numeric parameters must be finite")
        ApproachThresholds(
            self._target_distance,
            self._distance_tolerance,
            self._lateral_tolerance,
            self._angle_tolerance_deg,
            self._stable_time,
        ).validate()
        self._base_thresholds().validate()
        if self._tag_timeout <= 0.0 or self._tag_receipt_timeout <= 0.0:
            raise ValueError("tag timeouts must be positive")
        if not 0.0 < self._observation_hold_time <= self._tag_receipt_timeout:
            raise ValueError("observation_hold_time must be positive and <= tag_receipt_timeout")
        if self._publish_rate <= 0.0 or self._filter_window < 1:
            raise ValueError("publish_rate and filter_window must be positive")
        if self._tf_lookup_timeout < 0.0:
            raise ValueError("tf_lookup_timeout must not be negative")
        if self._final_approach_tag_loss_grace_sec < 0.0:
            raise ValueError(
                "final_approach_tag_loss_grace_sec must not be negative"
            )
        if not self._pre_align_distance > self._base_target_forward > 0.0:
            raise ValueError("pre-align distance must exceed Follower final distance")
        if self._blind_activation_max_tag_x <= 0.0:
            raise ValueError("blind activation distance must be positive")
        if self._blind_max_distance < 0.0:
            raise ValueError("blind max distance must not be negative")
        if not 0.0 <= self._blind_last_tag_max_age < self._blind_handoff_max_age:
            raise ValueError("blind handoff age must exceed visual loss age")
        if self._blind_last_tag_max_age >= self._tag_receipt_timeout:
            raise ValueError("blind visual loss age must precede general receipt timeout")
        if (
            self._blind_max_duration <= 0.0
            or self._blind_odom_timeout <= 0.0
            or self._blind_max_odom_step <= 0.0
            or self._blind_max_lateral_deviation <= 0.0
            or self._blind_max_yaw_deviation_deg <= 0.0
            or self._blind_reverse_tolerance < 0.0
        ):
            raise ValueError("blind safety limits are invalid")

    def _candidate_ids(self) -> Sequence[int]:
        if self._target_tag_id >= 0:
            return (self._target_tag_id,)
        return self._allowed_tag_ids

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        ros_now = now.nanoseconds / 1.0e9
        receipt_now = time.monotonic()
        if self._blind_completed:
            self._publish_completed_cycle(ros_now)
            return
        candidate_ids = self._candidate_ids()
        observations = [
            observation
            for tag_id in candidate_ids
            if (observation := self._lookup_observation(tag_id, now)) is not None
        ]
        selected = select_observation(
            observations, list(candidate_ids), self._selection_mode
        )
        if selected is None:
            if self._publish_final_approach_grace(receipt_now):
                return
            self._publish_lost(ros_now, receipt_now)
            return
        is_new = self._accept_observation_stamp(selected, receipt_now)
        if is_new and getattr(self, "_final_approach_grace_active", False):
            if self._expire_final_approach_grace(ros_now, receipt_now):
                return
        sample_receipt = self._last_observation_receipts.get(selected.tag_id)
        if self._blind_active:
            if is_new:
                self._clear_blind_plan()
            else:
                self._publish_blind_cycle(ros_now, receipt_now)
                return
        if not is_new and (sample_receipt is None or receipt_now - sample_receipt > getattr(self, "_observation_hold_time", 0.25)):
            if self._publish_final_approach_grace(receipt_now):
                return
            # A repeated TF is not a visual dropout. Preserve a fresh
            # stabilizing sample; the state machine receives is_new=False
            # and therefore cannot count it as another confirmation.
        if sample_receipt is None:
            self._publish_lost(ros_now, receipt_now)
            return
        receipt_age = receipt_now - sample_receipt
        if receipt_age < 0.0 or receipt_age > self._tag_receipt_timeout:
            self._publish_lost(ros_now, receipt_now)
            return
        if (
            getattr(self, "_blind_final_approach_enabled", False)
            and not is_new
            and self._last_valid_receipt is not None
            and receipt_age >= self._blind_last_tag_max_age
        ):
            self._publish_lost(ros_now, receipt_now)
            return
        if selected.tag_id != self._active_tag_id:
            self._translation_filter.reset()
            self._normal_filter.reset()
            self._state_machine.reset()
            self._base_state_machine.reset()
            self._active_tag_id = selected.tag_id
        measurement = self._translation_filter.add(
            selected.x, selected.y, selected.z, selected.stamp_nanoseconds
        )
        state = self._state_machine.update(measurement, ros_now, selected.tag_id)
        self._publish_valid(
            selected, measurement, state, sample_receipt, receipt_now, is_new
        )

    def _lookup_observation(self, tag_id: int, now: Time) -> Optional[TagObservation]:
        tag_frame = self._tag_frame_pattern.format(id=tag_id)
        try:
            transform = self._tf_buffer.lookup_transform(
                self._source_frame, tag_frame, Time()
            )
        except TransformException:
            return None
        stamp = Time.from_msg(transform.header.stamp)
        age = (now.nanoseconds - stamp.nanoseconds) / 1.0e9
        if stamp.nanoseconds <= 0 or age < 0.0 or age > self._tag_timeout:
            return None
        translation = transform.transform.translation
        if not is_valid_translation(translation.x, translation.y, translation.z):
            return None
        rotation = transform.transform.rotation
        quaternion = normalize_quaternion(
            (rotation.x, rotation.y, rotation.z, rotation.w)
        )
        if quaternion is None:
            return None
        return TagObservation(
            tag_id,
            float(translation.x),
            float(translation.y),
            float(translation.z),
            quaternion,
            stamp.nanoseconds,
        )

    def _accept_observation_stamp(
        self, observation: TagObservation, receipt_now: float
    ) -> bool:
        if observation.stamp_nanoseconds <= 0 or not isfinite(receipt_now):
            return False
        previous = self._last_processed_observation_stamps.get(observation.tag_id)
        if previous is not None and observation.stamp_nanoseconds <= previous:
            return False
        self._last_processed_observation_stamps[
            observation.tag_id
        ] = observation.stamp_nanoseconds
        self._last_observation_receipts[observation.tag_id] = receipt_now
        return True

    def _on_approach_enabled(self, message: Bool) -> None:
        """Reset all mission-temporal state at an approach session boundary."""
        self._translation_filter.reset()
        self._normal_filter.reset()
        self._state_machine.reset()
        self._base_state_machine.reset()
        self._active_tag_id = None
        self._clear_final_sample()
        self._clear_final_approach_grace()
        self._clear_blind_plan()
        self._blind_completed = False
        self.get_logger().info(
            "Approach session state reset on controller %s"
            % ("enable" if message.data else "disable")
        )

    def _on_odom(self, message: Odometry) -> None:
        receipt = time.monotonic()
        ros_now = self.get_clock().now().nanoseconds / 1.0e9
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        values = (
            position.x,
            position.y,
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        if not all(isfinite(value) for value in values):
            self._last_odom = None
            return
        norm = sqrt(sum(value * value for value in values[2:]))
        if norm <= 1.0e-12:
            self._last_odom = None
            return
        qx, qy, qz, qw = (value / norm for value in values[2:])
        yaw = atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1.0e9
        if stamp <= 0.0 or not all(isfinite(v) for v in (stamp, ros_now, receipt, yaw)):
            self._last_odom = None
            return
        self._last_odom = (
            float(position.x),
            float(position.y),
            float(yaw),
            float(stamp),
            float(receipt),
        )

    def _fresh_odom(
        self, ros_now: float, receipt_now: float
    ) -> Optional[Tuple[float, float, float]]:
        if self._last_odom is None or not all(isfinite(v) for v in self._last_odom):
            return None
        x, y, yaw, stamp, receipt = self._last_odom
        source_age = ros_now - stamp
        receipt_age = receipt_now - receipt
        if (
            source_age < 0.0
            or source_age > self._blind_odom_timeout
            or receipt_age < 0.0
            or receipt_age > self._blind_odom_timeout
        ):
            return None
        return x, y, yaw

    def _remember_last_valid_final_sample(
        self,
        metrics,
        geometry: TargetGeometry,
        decision: AlignmentDecision,
        sample_receipt: float,
        is_new: bool,
    ) -> None:
        if (
            is_new
            and decision.mode == ControlMode.FINAL_APPROACH
            and decision.state == ApproachState.FINAL_APPROACH
        ):
            self._last_valid_tag_x = metrics.forward_distance
            self._last_valid_receipt = sample_receipt
            self._last_valid_yaw_error = geometry.final_yaw_error
            self._last_valid_cross_track = geometry.final_y
        elif is_new:
            self._clear_final_sample()

    def _clear_final_sample(self) -> None:
        self._last_valid_tag_x = None
        self._last_valid_receipt = None
        self._last_valid_yaw_error = None
        self._last_valid_cross_track = None

    def _record_final_approach_observation(
        self,
        decision: AlignmentDecision,
        receipt_now: float,
        is_new: bool,
    ) -> None:
        """Arm visual grace only from a fresh FINAL_APPROACH observation."""
        if not is_new:
            return
        is_final_approach = (
            decision.state == ApproachState.FINAL_APPROACH
            and decision.mode == ControlMode.FINAL_APPROACH
        )
        if not is_final_approach:
            self._clear_final_approach_grace()
            return
        if self._final_approach_grace_active:
            self.get_logger().info(
                "FINAL_APPROACH: fresh tag reacquired; resuming visual control"
            )
        self._last_fresh_final_observation_receipt = receipt_now
        self._final_approach_grace_eligible = True
        self._final_approach_grace_active = False

    def _clear_final_approach_grace(self) -> None:
        self._last_fresh_final_observation_receipt = None
        self._final_approach_grace_eligible = False
        self._final_approach_grace_active = False

    def _publish_final_approach_grace(self, receipt_now: float) -> bool:
        """Hold the final phase with no stale target during a short dropout."""
        ros_now = self.get_clock().now().nanoseconds / 1.0e9
        if self._expire_final_approach_grace(ros_now, receipt_now):
            return True
        if not self._final_approach_grace_eligible:
            return False
        if not self._final_approach_grace_active:
            self.get_logger().info(
                "FINAL_APPROACH: tag temporarily lost; holding zero velocity"
            )
        self._final_approach_grace_active = True
        self._detected_pub.publish(Bool(data=False))
        self._tag_id_pub.publish(Int32(data=-1))
        decision = AlignmentDecision(
            ApproachState.FINAL_APPROACH, ControlMode.FINAL_APPROACH
        )
        self._publish_atomic_command(None, decision)
        self._control_mode_pub.publish(String(data=decision.mode.value))
        self._base_state_pub.publish(String(data=decision.state.value))
        self._publish_blind_diagnostics(False)
        self._log_base_state_change(decision.state)
        return True

    def _expire_final_approach_grace(
        self, ros_now: float, receipt_now: float
    ) -> bool:
        """Run normal loss handling after the monotonic grace deadline."""
        last_fresh = self._last_fresh_final_observation_receipt
        if not self._final_approach_grace_eligible or last_fresh is None:
            return False
        age = receipt_now - last_fresh
        if (
            not isfinite(age)
            or age < 0.0
            or age > self._final_approach_tag_loss_grace_sec + 1.0e-9
        ):
            if self._final_approach_grace_active:
                self.get_logger().info(
                    "FINAL_APPROACH: tag loss grace expired; entering TAG_LOST"
                )
            self._clear_final_approach_grace()
            self._publish_lost(
                ros_now,
                receipt_now,
                allow_blind=self._blind_final_approach_enabled,
            )
            return True
        return False

    def _blind_plan(
        self, ros_now: float, receipt_now: float
    ) -> Optional[float]:
        if any(
            value is None
            for value in (
                self._last_valid_tag_x,
                self._last_valid_receipt,
                self._last_valid_yaw_error,
                self._last_valid_cross_track,
            )
        ):
            return None
        odom = self._fresh_odom(ros_now, receipt_now)
        if odom is None:
            return None
        assert self._last_valid_tag_x is not None
        assert self._last_valid_receipt is not None
        assert self._last_valid_yaw_error is not None
        assert self._last_valid_cross_track is not None
        phase = AlignmentDecision(
            ApproachState.FINAL_APPROACH, ControlMode.FINAL_APPROACH
        )
        if not is_blind_final_approach_eligible(
            enabled=self._blind_final_approach_enabled,
            phase=phase,
            last_valid_tag_x=self._last_valid_tag_x,
            last_valid_receipt=self._last_valid_receipt,
            receipt_now=receipt_now,
            last_valid_yaw_error=self._last_valid_yaw_error,
            last_valid_cross_track=self._last_valid_cross_track,
            final_target_distance=self._base_target_forward,
            activation_max_tag_x=self._blind_activation_max_tag_x,
            max_distance=self._blind_max_distance,
            handoff_max_age=self._blind_handoff_max_age,
            yaw_tolerance=radians(self._base_bearing_tolerance_deg),
            cross_track_tolerance=self._base_lateral_tolerance,
            odometry_valid=True,
        ):
            return None
        return compute_blind_remaining_distance(
            self._last_valid_tag_x,
            self._base_target_forward,
            self._blind_max_distance,
        )

    def _clear_blind_plan(self) -> None:
        self._blind_active = False
        self._blind_planned_distance = 0.0
        self._blind_start_odom = None
        self._blind_previous_odom = None
        self._blind_start_receipt = None
        self._last_odom_progress = 0.0

    def _abort_blind(self, ros_now: float, receipt_now: float) -> None:
        self._clear_blind_plan()
        self._clear_final_sample()
        self._publish_base_lost(ros_now, receipt_now)

    def _publish_blind_cycle(self, ros_now: float, receipt_now: float) -> None:
        if (
            self._blind_start_receipt is None
            or receipt_now < self._blind_start_receipt
            or receipt_now - self._blind_start_receipt > self._blind_max_duration
        ):
            self._abort_blind(ros_now, receipt_now)
            return
        odom = self._fresh_odom(ros_now, receipt_now)
        if odom is None or self._blind_start_odom is None:
            self._abort_blind(ros_now, receipt_now)
            return
        progress = compute_forward_progress(
            self._blind_start_odom[0],
            self._blind_start_odom[1],
            self._blind_start_odom[2],
            odom[0],
            odom[1],
        )
        if progress is None or progress < -self._blind_reverse_tolerance:
            self._abort_blind(ros_now, receipt_now)
            return
        total_dx = odom[0] - self._blind_start_odom[0]
        total_dy = odom[1] - self._blind_start_odom[1]
        lateral = abs(
            -sin(self._blind_start_odom[2]) * total_dx
            + cos(self._blind_start_odom[2]) * total_dy
        )
        yaw_deviation = abs(normalize_angle(odom[2] - self._blind_start_odom[2]))
        step = 0.0
        if self._blind_previous_odom is not None:
            step = hypot(
                odom[0] - self._blind_previous_odom[0],
                odom[1] - self._blind_previous_odom[1],
            )
        if (
            not all(isfinite(v) for v in (progress, step, lateral, yaw_deviation))
            or step > self._blind_max_odom_step
            or lateral > self._blind_max_lateral_deviation
            or yaw_deviation > radians(self._blind_max_yaw_deviation_deg)
            or progress > self._blind_max_distance
        ):
            self._abort_blind(ros_now, receipt_now)
            return
        self._blind_previous_odom = odom
        self._last_odom_progress = max(0.0, progress)
        if self._last_odom_progress >= self._blind_planned_distance:
            self._blind_active = False
            self._blind_completed = True
            self._publish_blind_command(ApproachState.ALIGNED, ControlMode.ALIGNED)
            return
        self._publish_blind_command(
            ApproachState.FINAL_APPROACH, ControlMode.BLIND_FINAL_APPROACH
        )

    def _publish_lost(
        self, ros_now: float, receipt_now: float, *, allow_blind: bool = True
    ) -> None:
        if self._blind_completed:
            self._publish_completed_cycle(ros_now)
            return
        if self._blind_active:
            self._detected_pub.publish(Bool(data=False))
            self._tag_id_pub.publish(Int32(data=-1))
            self._state_pub.publish(String(data=ApproachState.TAG_LOST.value))
            self._publish_blind_cycle(ros_now, receipt_now)
            return
        planned = self._blind_plan(ros_now, receipt_now) if allow_blind else None
        self._translation_filter.reset()
        self._normal_filter.reset()
        self._active_tag_id = None
        camera_state = self._state_machine.update(None, ros_now, None)
        self._detected_pub.publish(Bool(data=False))
        self._tag_id_pub.publish(Int32(data=-1))
        self._state_pub.publish(String(data=camera_state.value))
        self._log_state_change(camera_state)
        if planned is None:
            self._clear_final_sample()
            self._publish_base_lost(ros_now, receipt_now)
            return
        odom = self._fresh_odom(ros_now, receipt_now)
        if odom is None:
            self._clear_final_sample()
            self._publish_base_lost(ros_now, receipt_now)
            return
        self._blind_active = True
        self._blind_planned_distance = planned
        self._blind_start_odom = odom
        self._blind_previous_odom = odom
        self._blind_start_receipt = receipt_now
        self._last_odom_progress = 0.0
        self._publish_blind_cycle(ros_now, receipt_now)

    def _publish_base_lost(self, ros_now: float, receipt_now: float) -> None:
        del ros_now
        clear_grace = getattr(self, "_clear_final_approach_grace", None)
        if clear_grace is not None:
            clear_grace()
        self._normal_filter.reset()
        decision = self._base_state_machine.update(None, receipt_now, None)
        self._publish_atomic_command(None, decision)
        self._control_mode_pub.publish(String(data=decision.mode.value))
        self._base_state_pub.publish(String(data=decision.state.value))
        self._publish_blind_diagnostics(False)
        self._log_base_state_change(decision.state)

    def _publish_base_outputs(
        self,
        camera_pose: PoseStamped,
        sample_receipt: float,
        receipt_now: float,
        is_new: bool,
    ) -> None:
        stamp = camera_pose.header.stamp
        ros_now_ns = self.get_clock().now().nanoseconds
        if not is_fresh_timestamp(
            stamp.sec, stamp.nanosec, ros_now_ns, self._tag_timeout
        ):
            self._publish_base_lost(ros_now_ns / 1.0e9, receipt_now)
            return
        try:
            transform = self._tf_buffer.lookup_transform(
                self._base_frame,
                camera_pose.header.frame_id,
                Time.from_msg(stamp),
                timeout=Duration(seconds=self._tf_lookup_timeout),
            )
            base_pose = transform_pose_preserving_stamp(
                camera_pose, transform, self._base_frame
            )
        except Exception as error:
            self.get_logger().warning(
                "Skipping invalid base transform: %s" % error,
                throttle_duration_sec=5.0,
            )
            self._publish_base_lost(ros_now_ns / 1.0e9, receipt_now)
            return
        if base_pose is None:
            self._publish_base_lost(ros_now_ns / 1.0e9, receipt_now)
            return
        try:
            metrics = compute_base_metrics(
                base_pose.pose.position.x, base_pose.pose.position.y
            )
            if getattr(self, "_align_tag_normal", True):
                orientation = base_pose.pose.orientation
                normal_x, normal_y = rotate_tag_z_to_base_xy(
                    (orientation.x, orientation.y, orientation.z, orientation.w)
                )
                normal_x, normal_y = select_robot_facing_normal(
                    base_pose.pose.position.x,
                    base_pose.pose.position.y,
                    normal_x,
                    normal_y,
                )
                normal_x, normal_y = self._normal_filter.add(
                    normal_x,
                    normal_y,
                    stamp.sec * 1_000_000_000 + stamp.nanosec,
                )
            else:
                # Keep both staging and final targets on the same center ray.
                # A noisy tag-face yaw must not create a lateral detour.
                normal_x = -base_pose.pose.position.x
                normal_y = -base_pose.pose.position.y
            geometry = compute_target_geometry(
                base_pose.pose.position.x,
                base_pose.pose.position.y,
                normal_x,
                normal_y,
                self._pre_align_distance,
                self._base_target_forward,
            )
        except ValueError as error:
            self.get_logger().warning(
                "Skipping invalid tag-normal geometry: %s" % error,
                throttle_duration_sec=5.0,
            )
            self._publish_base_lost(ros_now_ns / 1.0e9, receipt_now)
            return
        decision = self._base_state_machine.update(
            BaseAlignmentMeasurement(
                base_pose.pose.position.x,
                base_pose.pose.position.y,
                geometry.prealign_x,
                geometry.prealign_y,
                geometry.final_x,
                geometry.final_y,
                geometry.final_yaw_error,
                sample_receipt,
            ),
            receipt_now,
            self._active_tag_id,
            is_new,
        )
        record_final = getattr(self, "_record_final_approach_observation", None)
        if record_final is not None:
            record_final(decision, receipt_now, is_new)
        self._remember_last_valid_final_sample(
            metrics, geometry, decision, sample_receipt, is_new
        )
        prealign_pose = self._make_target_pose(base_pose, geometry, True)
        final_pose = self._make_target_pose(base_pose, geometry, False)
        control_pose = self._make_control_pose(base_pose, geometry, decision)
        self._base_pose_pub.publish(base_pose)
        self._base_forward_pub.publish(Float64(data=metrics.forward_distance))
        self._base_lateral_pub.publish(Float64(data=metrics.lateral_error))
        self._base_bearing_pub.publish(Float64(data=metrics.bearing))
        self._normal_heading_pub.publish(
            Float64(data=atan2(geometry.normal_y, geometry.normal_x))
        )
        self._prealign_target_pub.publish(prealign_pose)
        self._final_target_pub.publish(final_pose)
        self._final_position_error_pub.publish(
            Float64(data=geometry.final_position_error)
        )
        self._final_yaw_error_pub.publish(Float64(data=geometry.final_yaw_error))
        self._publish_atomic_command(control_pose, decision)
        self._control_target_pub.publish(control_pose)
        self._control_mode_pub.publish(String(data=decision.mode.value))
        self._base_state_pub.publish(String(data=decision.state.value))
        self._publish_blind_diagnostics(False)
        self._log_base_state_change(decision.state)

    def _make_target_pose(
        self, base_pose: PoseStamped, geometry: TargetGeometry, prealign: bool
    ) -> PoseStamped:
        target = PoseStamped()
        target.header = base_pose.header
        target.pose.position.x = geometry.prealign_x if prealign else geometry.final_x
        target.pose.position.y = geometry.prealign_y if prealign else geometry.final_y
        target.pose.orientation.z = sin(geometry.target_yaw / 2.0)
        target.pose.orientation.w = cos(geometry.target_yaw / 2.0)
        return target

    def _make_control_pose(
        self,
        base_pose: PoseStamped,
        geometry: TargetGeometry,
        decision: AlignmentDecision,
    ) -> PoseStamped:
        target = PoseStamped()
        target.header = base_pose.header
        target.pose.position.x = decision.control_x
        target.pose.position.y = decision.control_y
        target.pose.orientation.z = sin(geometry.target_yaw / 2.0)
        target.pose.orientation.w = cos(geometry.target_yaw / 2.0)
        return target

    def _publish_atomic_command(
        self,
        control_pose: Optional[PoseStamped],
        decision: AlignmentDecision,
    ) -> None:
        command = FollowerAlignmentCommand()
        if control_pose is None:
            command.header.frame_id = self._base_frame
            command.header.stamp = self.get_clock().now().to_msg()
        else:
            command.header = control_pose.header
            command.target_pose = control_pose.pose
        command.control_mode = decision.mode.value
        command.alignment_state = decision.state.value
        self._command_pub.publish(command)

    def _publish_blind_command(
        self, state: ApproachState, mode: ControlMode
    ) -> None:
        pose = PoseStamped()
        pose.header.frame_id = self._base_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self._blind_planned_distance
        pose.pose.orientation.w = 1.0
        decision = AlignmentDecision(state, mode)
        self._publish_atomic_command(pose, decision)
        self._control_target_pub.publish(pose)
        self._control_mode_pub.publish(String(data=mode.value))
        self._base_state_pub.publish(String(data=state.value))
        self._publish_blind_diagnostics(mode == ControlMode.BLIND_FINAL_APPROACH)
        self._log_base_state_change(state)

    def _publish_completed_cycle(self, ros_now: float) -> None:
        del ros_now
        self._detected_pub.publish(Bool(data=False))
        self._tag_id_pub.publish(Int32(data=-1))
        self._state_pub.publish(String(data=ApproachState.TAG_LOST.value))
        self._publish_blind_command(ApproachState.ALIGNED, ControlMode.ALIGNED)

    def _publish_blind_diagnostics(self, active: bool) -> None:
        self._blind_active_pub.publish(Bool(data=active))
        self._last_valid_tag_x_pub.publish(
            Float64(data=self._last_valid_tag_x or 0.0)
        )
        self._blind_distance_pub.publish(Float64(data=self._blind_planned_distance))
        self._odom_progress_pub.publish(Float64(data=self._last_odom_progress))

    def _publish_valid(
        self,
        selected: TagObservation,
        measurement: RelativeMeasurement,
        state: ApproachState,
        sample_receipt: float,
        receipt_now: float,
        is_new: bool,
    ) -> None:
        pose = PoseStamped()
        pose.header.frame_id = self._source_frame
        pose.header.stamp = Time(nanoseconds=selected.stamp_nanoseconds).to_msg()
        pose.pose.position.x = measurement.x
        pose.pose.position.y = measurement.y
        pose.pose.position.z = measurement.z
        (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ) = selected.quaternion
        self._detected_pub.publish(Bool(data=True))
        self._tag_id_pub.publish(Int32(data=selected.tag_id))
        self._pose_pub.publish(pose)
        self._distance_pub.publish(Float64(data=measurement.distance))
        self._lateral_pub.publish(Float64(data=measurement.lateral_error))
        self._straight_pub.publish(Float64(data=measurement.straight_distance))
        self._angle_pub.publish(Float64(data=measurement.angle))
        self._state_pub.publish(String(data=state.value))
        self._log_state_change(state)
        self._publish_base_outputs(pose, sample_receipt, receipt_now, is_new)

    def _log_state_change(self, state: ApproachState) -> None:
        if state != self._last_logged_state:
            self.get_logger().info("Alignment state changed to %s" % state.value)
            self._last_logged_state = state

    def _log_base_state_change(self, state: ApproachState) -> None:
        if state != self._last_logged_base_state:
            self.get_logger().info(
                "Base alignment state changed to %s" % state.value
            )
            self._last_logged_base_state = state


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[AprilTagApproachNode] = None
    try:
        node = AprilTagApproachNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass


if __name__ == "__main__":
    main()
