# Manual-grasp cooperative transport

This package controls software readiness, path transfer, drive ownership and
scheduled starts. It never imports gripper helpers, launches Dynamixel or
sends grasp/lift commands. The operator physically attaches both robots.

## Operator keys

1. Assemble the robots **opposite-facing with neutral, straight hinges**.
   Geometry defaults from follower PR #1: axle→hinge 12.5cm,
   hinge→contact 13.25cm, contact→object centre 5.25cm; axle separation 62cm.
   This is a required physical reference, **not a measured relative pose**.
2. Press **B** in the leader keyboard terminal. Wait `COOP leader WAIT_PLAN`.
   Automatic Nav2 motor selection is disabled; depth mapping enters HOLD.
3. Select a **new** RViz Nav2 goal. The planner's `/plan` is validated for
   the combined footprint, passive hinges and constant forward/reverse travel.
   Existing goals or paths from before B are not used. Both robots stay stopped.
4. Wait **COOP leader READY**. The follower has independently checked the
   frozen path, acknowledged its hash and set its selector to STOP.
5. Press **N**. Both selectors are armed while commands remain zero, then the
   leader submits the frozen axle path to the existing Nav2 RPP `FollowPath`
   action. After acknowledgements, both gates use a common scheduled start.
6. **Space** aborts both. Arrow keys abort cooperation and take leader teleop
   ownership. **M/P/H/L** abort cooperation and return to their original use.
   After an abort or goal completion, use B and a new goal for another run.

In cooperative mode, the updated main keyboard also ignores Z/X/C/V and
suspends its gripper-command publisher.

To add B/N without restarting an existing map or keyboard, open another
interactive terminal and run `ros2 run cooperative_transport transport_keys`.
Use B/N in that window; the old keyboard retains its original controls.

Preparation is intentionally operator-driven; startup, receiving a path and
READY alone never start motion. Pressing N before READY is rejected.
`motion_enabled` defaults to false on both peers. In this mode, preparation
and previews work but START/ARM cannot enable motion. Restart with
`motion_enabled:=true` only for an intentional drive run.

## Launch

Leader integration is required: the checked shared branch does not yet provide
the COOPERATION selector source, Nav2 goal-selection gate, or the mapping-control
request relay. The intended integrated mapping launch starts the leader peer
dormant until B. Once those interfaces are implemented, a running Leader stack
can add the peer separately:

```bash
source /opt/ros/humble/setup.bash
source install_docker/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 launch cooperative_transport manual_transport.launch.py role:=leader
```

On the follower, with no competing mission/command publisher:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 launch cooperative_transport manual_transport.launch.py \
  role:=follower follower_drive:=true use_stm32_bridge:=true
```

This is a preparation-only launch: `motion_enabled:=false` and
`i2c_write_enabled:=false` are the defaults. The bridge may read wheel odometry
but does not write motor commands. Hardware-free launch inspection can use
`use_stm32_bridge:=false`; a valid odometry source is still required for READY.
For a later drive run, explicitly add BOTH `motion_enabled:=true` and
`i2c_write_enabled:=true` on the follower and enable motion on the leader.
The standalone `transport_peer` executable also defaults to motion disabled.

This follower launch includes ONLY selector, velocity guard, STM32 drive/
odometry bridge and peer. The selector starts STOP and guard disabled.
If those drive nodes already run with the mission cmd_vel remap, use
`follower_drive:=false`; the peer refuses readiness with another publisher.
Do not launch the original grasp/lift mission alongside this workflow.

## Communication and control

- `/cooperation/transport/control`: PREPARE, START, ABORT (leader only).
- `/cooperation/transport/{leader,follower}`: reliable JSON protocol v1,
  session ID, SHA-256 plan hash, PREPARE/READY, ARM/ACK, COMMIT/ACK,
  heartbeat and STOP. Paths are frozen and resent until acknowledged.
- `/cooperation/transport/{leader,follower}/path`: latched axle paths for RViz.
  Follower preview uses **its own** local odometry frame, not leader coordinates.
- `/cooperation/transport/{leader,follower}/status`: latched human status.
- Leader drive input: `/leader/cooperation/cmd_vel`, source COOPERATION.
  Nav2 RPP commands are gated and limited to 0.05m/s, retaining curvature.
- Follower drive input: `/follower/mission/cmd_vel` via COOPERATION selector
  and existing final velocity guard; reverse-capable pure pursuit.
- Raw wheel odometry initializes the follower's one-session SE(2) transform
  from the required physical neutral assembly. Equal `odom` frame names do
  not establish a shared origin. Moving either robot after READY requires B
  and a new path. Arbitrary assembly/hinge poses need measured relative pose.
- Both controllers compare progress and stop for stale odometry/heartbeat,
  conflicting publishers, invalid tracking, inadmissible turn commands,
  missed start acknowledgement/deadline or Nav2 action failure.
- The follower scales its speed with the leader RPP command ratio, including
  collision pauses; the leader keeps Nav2 collision regulation.
- Combined envelope collision checks use a fresh known global costmap at
  preparation and N. Unknown/obstructed paths are rejected rather than reused.
  The combined footprint is also supplied to Nav2 in cooperative mode.

Clock offset is estimated from round-trip timestamp exchanges (RTT ≤200ms).
The start is scheduled 2 seconds ahead; this is **best-effort synchronization**.
Network asymmetry, ROS scheduling and motor latency can still create skew;
network partitions cannot provide an atomic two-robot start. Stale heartbeat
closes local gates. Actual start skew and loaded tracking require real tests.

## Provenance / limits

`hinged_formation.py` and `path_tracking.py` originate from
`kmhuh1525/damgc_robot`, PR #1, commit
`dbc24b97e6e1c39947dd326ef0d0d8ceeac740ea` (Apache-2.0). `motion.py` retains
only pose/command types, with no Dynamixel helpers. Communication and manual
trigger logic are new. Sharp solo Nav2 curves and reversal cusps can be
rejected by the formation validator; the current planner does not automatically
search a replacement cooperative route. Straight departure with neutral
hinges is required. Hardware-loaded operation has not been tested here.

Follower preparation and verification: see
[`FOLLOWER_MANUAL_TRANSPORT_PREPARATION.md`](../../docs/FOLLOWER_MANUAL_TRANSPORT_PREPARATION.md).
