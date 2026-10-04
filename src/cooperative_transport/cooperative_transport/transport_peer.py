"""Two-stage manual-grasp transport. No Dynamixel nodes, topics or services.

Reliable session-scoped packets freeze one checked path on both computers.
Motor ownership is independent of Nav2, and commands remain zero until a
scheduled start has been acknowledged. Scheduling is best effort, not atomic
or hard real-time across DDS and two motor controllers.
"""
import hashlib
import json
import math
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rcl_interfaces.srv import SetParameters
from std_srvs.srv import SetBool
from action_msgs.srv import CancelGoal
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from std_msgs.msg import String

from .motion import Pose2D, normalize_angle
from .hinged_formation import HingeGeometry, leader_path_to_object_path, drive_direction
from .path_tracking import compute_tracking_command

TERMINAL = {'IDLE', 'STOPPED', 'DONE', 'ERROR'}

def yaw(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

def pose(p):
    result = Pose2D(float(p.position.x), float(p.position.y), yaw(p.orientation))
    if not all(math.isfinite(v) for v in (result.x, result.y, result.yaw)):
        raise ValueError('nonfinite pose')
    return result

def digest(body):
    return hashlib.sha256(json.dumps(body, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()

def map_pose(p, source, destination):
    a = normalize_angle(destination.yaw-source.yaw)
    dx, dy = p.x-source.x, p.y-source.y
    return Pose2D(destination.x + math.cos(a)*dx-math.sin(a)*dy,
                  destination.y + math.sin(a)*dx+math.cos(a)*dy, normalize_angle(p.yaw+a))

class TransportPeer(Node):
    def __init__(self):
        super().__init__('transport_peer')
        self.declare_parameter('role', 'leader')
        self.role = str(self.get_parameter('role').value)
        if self.role not in ('leader', 'follower'):
            raise ValueError('role must be leader or follower')
        self.leader = self.role == 'leader'
        self.declare_parameter('motion_enabled', False)
        self.motion_enabled = bool(self.get_parameter('motion_enabled').value)
        self.declare_parameter('validate_curvature', True)
        self.validate_curvature = bool(self.get_parameter('validate_curvature').value)
        defaults = {'odom_topic': '/leader/odometry/local' if self.leader else '/follower/odom/raw',
                    'command_topic': '/leader/cooperation/cmd_vel' if self.leader else '/follower/mission/cmd_vel',
                    'axle_to_hinge': .125, 'hinge_to_contact': .1325,
                    'object_center_to_contact': .0525, 'hinge_limit_deg': 15.,
                    'speed': .05, 'max_path_error': .10, 'body_half_width': .145,
                    'body_rear_extent': .06, 'peer_timeout': .6}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.geometry = HingeGeometry(self.p('axle_to_hinge'), self.p('hinge_to_contact'),
                                      self.p('object_center_to_contact'), math.radians(self.p('hinge_limit_deg')))
        self.geometry.validate()
        for name in ('speed', 'max_path_error', 'body_half_width', 'body_rear_extent', 'peer_timeout'):
            if not math.isfinite(self.p(name)) or self.p(name) <= 0:
                raise ValueError('invalid '+name)
        self.command_topic = str(self.get_parameter('command_topic').value)
        self.cmd = self.create_publisher(Twist, self.command_topic, 1)
        root = '/cooperation/transport/'
        self.wire = self.create_publisher(String, root+self.role, 10)
        self.create_subscription(String, root+('follower' if self.leader else 'leader'), self.receive, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, root+self.role+'/status', latched)
        self.operator_pub = self.create_publisher(String, '/leader/mapping/status', 10) if self.leader else None
        self.preview = self.create_publisher(Path, root+self.role+'/path', latched)
        self.create_subscription(Odometry, str(self.get_parameter('odom_topic').value), self.odometry, qos_profile_sensor_data)
        self.selector = self.create_client(SetParameters, '/'+self.role+'/command_selector/set_parameters')
        self.guard = self.create_client(SetBool, '/follower/velocity_guard/enable') if not self.leader else None
        self.follow_handle = None
        self.rpp_command, self.rpp_time = Twist(), 0.
        self.peer_speed_ratio = 0.
        if self.leader:
            from rclpy.action import ActionClient
            from nav2_msgs.action import FollowPath
            self.follow_type = FollowPath
            self.follow = ActionClient(self, FollowPath, '/follow_path')
            self.create_subscription(Twist, '/nav2/cmd_vel', self.rpp_velocity, 1)
        self.cancel = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal') if self.leader else None
        self.state, self.session, self.path_hash = 'IDLE', '', ''
        self.robot, self.odom_frame, self.odom_time = None, '', 0.
        self.start_pose, self.path, self.object_path = None, (), ()
        self.index, self.progress, self.peer_progress = 0, 0., 0.
        self.peer_time, self.peer_state = 0., 'IDLE'
        self.packet = None
        self.pending = False
        self.pending_since = 0.
        self.generation = 0
        self.ready_deadline = 0.
        self.start_mono, self.start_wall, self.offset = None, None, None
        self.clock_rtt = None
        self.pings = {}
        self.last_prepare_send = 0.
        self.grid, self.grid_time = None, 0.
        self.plan_gate_ns, self.new_goal_ns = 0, 0
        self.buffered_plan = None
        if self.leader:
            self.create_subscription(String, root+'control', self.control, 10)
            self.create_subscription(String, '/leader/command_selector/request', self.takeover, 10)
            self.create_subscription(Path, '/plan', self.plan, 1)
            self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.goal_status, latched)
            self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.costmap, latched)
            self.create_subscription(OccupancyGridUpdate, '/global_costmap/costmap_updates', self.costmap_update, 10)
        self.create_timer(.05, self.tick)
        self.create_timer(.1, self.communicate)
        self.report('IDLE', 'B=prepare, RViz goal, wait READY, N=start; manual grasp only')

    def p(self, key):
        return float(self.get_parameter(key).value)

    def report(self, state, detail):
        self.state = state
        text = f'COOP {self.role} {state}: {detail}'
        self.status_pub.publish(String(data=text))
        if self.operator_pub:
            self.operator_pub.publish(String(data=text))
        self.get_logger().info(text)

    def send(self, kind, **values):
        body = dict(v=1, kind=kind, session=self.session, hash=self.path_hash, **values)
        self.wire.publish(String(data=json.dumps(body, separators=(',', ':'), allow_nan=False)))

    def odometry(self, msg):
        try:
            self.robot = pose(msg.pose.pose)
            self.odom_frame = msg.header.frame_id
            self.odom_time = time.monotonic()
            self.moving = abs(msg.twist.twist.linear.x) > .01 or abs(msg.twist.twist.angular.z) > .03
        except ValueError:
            self.robot = None

    def local_check(self, stationary=False):
        if not self.robot or not self.odom_frame or time.monotonic()-self.odom_time > .35:
            raise ValueError('odometry unavailable/stale')
        if stationary and self.moving:
            raise ValueError('robot is still moving')
        if not self.selector.service_is_ready():
            raise ValueError('command selector unavailable')
        if self.guard and not self.guard.service_is_ready():
            raise ValueError('follower velocity guard unavailable')
        if self.leader and not self.follow.server_is_ready():
            raise ValueError('Nav2 FollowPath action unavailable')
        if self.count_publishers(self.command_topic) != 1:
            raise ValueError('another publisher owns '+self.command_topic)
        if self.start_pose and stationary:
            if math.hypot(self.robot.x-self.start_pose.x, self.robot.y-self.start_pose.y) > .025 or abs(normalize_angle(self.robot.yaw-self.start_pose.yaw)) > math.radians(3):
                raise ValueError('robot moved since prepare; press B and plan again')

    def parameter(self, name, value, callback):
        if not self.selector.service_is_ready():
            self.stop('selector unavailable')
            return
        generation = self.generation
        request = SetParameters.Request()
        request.parameters = [Parameter(name, value=value).to_parameter_msg()]
        self.pending = True
        self.pending_since = time.monotonic()
        def done(future):
            if generation != self.generation:
                return
            self.pending = False
            try:
                results = future.result().results
                if not results or not all(r.successful for r in results):
                    raise ValueError('selector rejected '+name)
                callback()
            except Exception as error:
                self.stop(str(error))
        self.selector.call_async(request).add_done_callback(done)

    def source(self, source, callback):
        self.parameter('source_mode', source, callback)

    def takeover(self, msg):
        if msg.data in ('STOP', 'TELEOP') and self.state not in TERMINAL:
            self.stop('keyboard takeover', selector_stop=False)

    def control(self, msg):
        if msg.data == 'PREPARE':
            self.stop('new preparation')
            self.generation += 1
            self.session, self.path_hash = uuid.uuid4().hex, ''
            self.path, self.object_path, self.packet = (), (), None
            self.start_pose, self.start_mono, self.offset = None, None, None
            self.peer_time = 0.
            self.clock_rtt = None
            self.report('LOCKING', 'stopping before accepting a new RViz path')
            self.ready_deadline = time.monotonic()+8
            self.parameter('enable_nav2_goal_selection', False,
                           lambda: self.source('STOP', self.wait_for_goal))
        elif msg.data == 'START':
            try:
                if not self.motion_enabled:
                    raise ValueError('motion disabled; restart with motion_enabled:=true for drive tests')
                if self.state != 'READY' or self.peer_state != 'READY' or time.monotonic()-self.peer_time > .3:
                    raise ValueError('both peers must report READY first')
                self.local_check(stationary=True)
                self.check_collision()
                if self.offset is None or self.clock_rtt is None or self.clock_rtt > .2:
                    raise ValueError('clock handshake unavailable/too slow')
                self.report('ARMING', 'N received; opening selectors with zero commands')
                self.ready_deadline = time.monotonic()+5
                self.source('COOPERATION', lambda: self.report('WAIT_ARM', 'waiting for follower arm acknowledgement'))
            except ValueError as error:
                self.report(self.state, 'START rejected: '+str(error))
        elif msg.data == 'ABORT':
            self.stop('operator abort')

    def costmap(self, msg):
        self.grid = msg
        self.grid_time = time.monotonic()

    def costmap_update(self, msg):
        g = self.grid
        if not g or msg.header.frame_id != g.header.frame_id:
            return
        if msg.x+msg.width > g.info.width or msg.y+msg.height > g.info.height:
            self.grid = None
            return
        for row in range(msg.height):
            a = (msg.y+row)*g.info.width+msg.x
            g.data[a:a+msg.width] = msg.data[row*msg.width:(row+1)*msg.width]
        self.grid_time = time.monotonic()

    def check_collision(self):
        g = self.grid
        if not g or time.monotonic()-self.grid_time > 10 or g.header.frame_id != self.odom_frame:
            raise ValueError('fresh global costmap in local odometry frame required')
        origin = pose(g.info.origin)
        resolution = float(g.info.resolution)
        if resolution <= 0 or len(g.data) != g.info.width*g.info.height:
            raise ValueError('invalid costmap')
        # Conservative swept rectangle enclosing BOTH chassis, arms and object.
        extent = self.geometry.axle_to_hinge+self.geometry.longitudinal_offset+self.p('body_rear_extent')+.02
        half = self.p('body_half_width')+.02
        step = min(.025, resolution/2)
        nx, ny = math.ceil(2*extent/step), math.ceil(2*half/step)
        dense = []
        for first, second in zip(self.object_path,self.object_path[1:]):
            count = max(1,math.ceil(math.hypot(second.x-first.x,second.y-first.y)/step))
            for j in range(count):
                u = j/count
                dense.append(Pose2D(first.x+u*(second.x-first.x),first.y+u*(second.y-first.y),first.yaw+u*normalize_angle(second.yaw-first.yaw)))
        dense.append(self.object_path[-1])
        for center in dense:
            c, s = math.cos(center.yaw), math.sin(center.yaw)
            for ix in range(nx+1):
                dx = -extent+2*extent*ix/nx
                for iy in range(ny+1):
                    dy = -half+2*half*iy/ny
                    wx, wy = center.x+c*dx-s*dy-origin.x, center.y+s*dx+c*dy-origin.y
                    gx = math.floor((math.cos(origin.yaw)*wx+math.sin(origin.yaw)*wy)/resolution)
                    gy = math.floor((-math.sin(origin.yaw)*wx+math.cos(origin.yaw)*wy)/resolution)
                    if not (0 <= gx < g.info.width and 0 <= gy < g.info.height):
                        raise ValueError('combined footprint leaves known costmap')
                    value = g.data[gy*g.info.width+gx]
                    if value < 0 or value >= 99:
                        raise ValueError('combined footprint intersects obstacle/unknown space')

    def wait_for_goal(self):
        self.buffered_plan = None
        self.new_goal_ns = 0
        self.plan_gate_ns = self.get_clock().now().nanoseconds
        if self.cancel.service_is_ready():
            self.cancel.call_async(CancelGoal.Request())
        self.report('WAIT_PLAN', 'select a NEW RViz Nav2 goal; leader remains stopped')

    def goal_status(self, msg):
        if self.state != 'WAIT_PLAN':
            return
        for entry in msg.status_list:
            stamp = entry.goal_info.stamp.sec*1_000_000_000+entry.goal_info.stamp.nanosec
            if entry.status in (1,2) and stamp > self.plan_gate_ns:
                self.new_goal_ns = max(self.new_goal_ns,stamp)
        if self.new_goal_ns and self.buffered_plan:
            message, self.buffered_plan = self.buffered_plan, None
            self.plan(message)

    def plan(self, msg):
        if self.state != 'WAIT_PLAN' or self.pending:
            return
        path_stamp = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        if not self.new_goal_ns:
            self.buffered_plan = msg
            return
        if path_stamp < self.new_goal_ns:
            return
        try:
            self.local_check(stationary=True)
            if msg.header.frame_id != self.odom_frame or len(msg.poses) < 3 or len(msg.poses) > 4000:
                raise ValueError('path needs 3..4000 poses in leader odometry frame')
            points = tuple(pose(p.pose) for p in msg.poses)
            first = points[0]
            if math.hypot(first.x-self.robot.x, first.y-self.robot.y) > .05 or abs(normalize_angle(first.yaw-self.robot.yaw)) > math.radians(3):
                raise ValueError('path does not begin at current leader pose/heading')
            if any(math.hypot(b.x-a.x,b.y-a.y) > .08 for a,b in zip(points,points[1:])):
                raise ValueError('path spacing exceeds 8cm')
            obj, formation = leader_path_to_object_path(points, self.geometry, lateral_tolerance=.01,
                                                       validate_curvature=self.validate_curvature)
            if self.validate_curvature and abs(formation.leader_hinge_angles[0]) > math.radians(2):
                raise ValueError('path must begin with neutral hinges; start with straight section')
            for path in (formation.leader, formation.follower):
                if drive_direction(path) not in ('FORWARD','REVERSE'):
                    raise ValueError('mixed forward/reverse or lateral paths unsupported')
            self.path, self.object_path = formation.leader, obj
            self.check_collision()
            self.start_pose = self.robot
            d = 2*(self.geometry.axle_to_hinge+self.geometry.longitudinal_offset)
            expected = Pose2D(self.robot.x+d*math.cos(self.robot.yaw), self.robot.y+d*math.sin(self.robot.yaw), normalize_angle(self.robot.yaw+math.pi))
            body = {'frame': msg.header.frame_id,
                    'geometry': [self.geometry.axle_to_hinge, self.geometry.hinge_to_contact,
                                 self.geometry.object_center_to_contact, self.geometry.hinge_limit],
                    'leader': [[p.x,p.y,p.yaw] for p in formation.leader],
                    'follower': [[p.x,p.y,p.yaw] for p in formation.follower],
                    'expected_follower': [expected.x, expected.y, expected.yaw],
                    'speed': self.p('speed')}
            self.path_hash, self.packet = digest(body), body
            self.index = 0
            self.progress = self.peer_progress = 0.
            self.peer_time, self.offset = 0., None
            self.publish_path()
            self.report('WAIT_READY', 'path frozen and sent; waiting for stationary follower READY')
            self.ready_deadline = time.monotonic()+20
            if self.cancel.service_is_ready():
                self.cancel.call_async(CancelGoal.Request())
        except (ValueError, TypeError) as error:
            self.path, self.object_path = (), ()
            self.report('WAIT_PLAN', 'path rejected: '+str(error)+'; choose another RViz goal')

    def publish_path(self):
        msg = Path()
        msg.header.frame_id = self.odom_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        for p in self.path:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x, ps.pose.position.y = p.x, p.y
            ps.pose.orientation.z, ps.pose.orientation.w = math.sin(p.yaw/2), math.cos(p.yaw/2)
            msg.poses.append(ps)
        self.preview.publish(msg)

    def receive(self, msg):
        try:
            received_wall = time.time()
            p = json.loads(msg.data)
            if not isinstance(p, dict):
                raise ValueError('protocol packet must be an object')
            if p.get('v') != 1:
                return
            kind = p['kind']
            if not self.leader and kind == 'PREPARE':
                self.prepare_follower(p, received_wall)
                return
            if not self.session or p.get('session') != self.session or p.get('hash') != self.path_hash:
                return
            if kind == 'STOP':
                self.stop('peer stopped: '+str(p.get('detail','')), notify=False, state='DONE' if p.get('detail') == 'DONE' else 'STOPPED')
            elif kind == 'HB':
                self.peer_time = time.monotonic()
                self.peer_state = str(p['state'])
                self.peer_progress = float(p['progress'])
                self.peer_speed_ratio = float(p.get('speed_ratio',0.))
                if not math.isfinite(self.peer_speed_ratio) or not 0 <= self.peer_speed_ratio <= 1:
                    raise ValueError('invalid peer speed ratio')
                if not math.isfinite(self.peer_progress) or not 0 <= self.peer_progress <= 1:
                    raise ValueError('invalid peer progress')
                if self.state not in TERMINAL and self.peer_state in ('ERROR','STOPPED'):
                    self.stop('peer no longer ready')
                elif self.state in ('RUNNING','ARRIVED') and self.peer_state in ('ARRIVED','DONE'):
                    remaining = math.hypot(self.path[-1].x-self.robot.x,self.path[-1].y-self.robot.y)
                    if remaining <= .07:
                        self.stop('goal reached; both command gates stopped', state='DONE')
                    else:
                        self.stop('peer reached goal but local goal error exceeds 7cm')
            elif not self.leader and kind == 'PING' and self.state == 'READY':
                self.local_check(stationary=True)
                self.send('READY', nonce=p['nonce'], t1=p['t1'], t2=received_wall, t3=time.time())
            elif self.leader and kind == 'READY' and self.state in ('WAIT_READY','READY'):
                t1 = float(p['t1'])
                started = self.pings.pop(p['nonce'], None)
                if started is None:
                    return
                rtt = time.monotonic()-started-(float(p['t3'])-float(p['t2']))
                offset = ((float(p['t2'])-t1)+(float(p['t3'])-received_wall))/2
                if not all(math.isfinite(v) for v in (rtt,offset)) or not 0 <= rtt <= .2:
                    return
                if self.clock_rtt is None or rtt <= self.clock_rtt:
                    self.clock_rtt, self.offset = rtt, offset
                self.peer_time, self.peer_state = time.monotonic(), 'READY'
                if self.state != 'READY':
                    self.local_check(stationary=True)
                    self.report('READY', 'both paths verified, motors stopped. Manual assembly must already match neutral opposite-facing geometry. Press N to start.')
            elif not self.leader and kind == 'ARM' and self.state == 'READY':
                if not self.motion_enabled:
                    raise ValueError('motion disabled on follower')
                self.local_check(stationary=True)
                self.report('ARMING', 'start key received; arming with zero commands')
                self.ready_deadline = time.monotonic()+5
                self.source('COOPERATION', self.arm_guard)
            elif self.leader and kind == 'ARM_ACK' and self.state == 'WAIT_ARM':
                self.report('ARM_CONTROLLER', 'follower armed; submitting frozen path to Nav2 RPP behind zero gate')
                self.ready_deadline = time.monotonic()+5
                self.start_controller()
            elif not self.leader and kind == 'COMMIT' and self.state in ('ARMED','SCHEDULED'):
                deadline = float(p['start'])+float(p['offset'])
                if self.state == 'ARMED':
                    delay = deadline-time.time()
                    if not math.isfinite(delay) or not .5 < delay < 4:
                        raise ValueError('invalid/late scheduled start')
                    self.local_check(stationary=True)
                    self.start_wall, self.start_mono = deadline, time.monotonic()+delay
                    self.report('SCHEDULED', 'common start accepted; zero until deadline')
                self.send('COMMIT_ACK')
            elif self.leader and kind == 'COMMIT_ACK' and self.state == 'WAIT_COMMIT':
                if self.start_mono-time.monotonic() < .5:
                    raise ValueError('start acknowledgement too late')
                self.report('SCHEDULED', 'both acknowledged; starting at scheduled deadline')
        except (ValueError, KeyError, TypeError, OverflowError) as error:
            if self.state not in TERMINAL:
                self.stop('protocol rejected: '+str(error))
            else:
                self.get_logger().warning('Ignored malformed protocol: '+str(error))

    def rpp_velocity(self, msg):
        values = (msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z)
        if not all(math.isfinite(v) for v in values) or any(abs(v) > 1e-9 for v in (msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y)):
            self.rpp_time = 0.
            return
        self.rpp_command, self.rpp_time = msg, time.monotonic()

    def start_controller(self):
        generation = self.generation
        goal = self.follow_type.Goal()
        goal.path.header.frame_id = self.odom_frame
        goal.path.header.stamp = self.get_clock().now().to_msg()
        for point in self.path:
            ps = PoseStamped()
            ps.header = goal.path.header
            ps.pose.position.x, ps.pose.position.y = point.x, point.y
            ps.pose.orientation.z, ps.pose.orientation.w = math.sin(point.yaw/2), math.cos(point.yaw/2)
            goal.path.poses.append(ps)
        goal.controller_id = 'FollowPath'
        goal.goal_checker_id = 'general_goal_checker'
        self.rpp_time = 0.
        def accepted(future):
            try:
                handle = future.result()
                if generation != self.generation:
                    if handle.accepted:
                        handle.cancel_goal_async()
                    return
                if not handle.accepted:
                    raise ValueError('Nav2 rejected cooperative FollowPath')
                self.follow_handle = handle
                handle.get_result_async().add_done_callback(finished)
                self.start_wall, self.start_mono = time.time()+2., time.monotonic()+2.
                self.report('WAIT_COMMIT', 'Nav2 RPP accepted; scheduling common gate in 2 seconds')
            except Exception as error:
                if generation == self.generation:
                    self.stop(str(error))
        def finished(future):
            if generation != self.generation:
                return
            try:
                response = future.result()
                if response.status != 4:
                    raise ValueError('Nav2 FollowPath ended with status '+str(response.status))
                if self.state not in ('RUNNING','ARRIVED'):
                    raise ValueError('Nav2 finished before synchronized departure')
                self.progress = 1.
                self.report('ARRIVED', 'Nav2 RPP reached goal; holding zero')
            except Exception as error:
                self.stop(str(error))
        self.follow.send_goal_async(goal).add_done_callback(accepted)

    def prepare_follower(self, p, t2):
        if self.state not in TERMINAL and p.get('session') != self.session:
            return  # A new preparation cannot steal an active robot.
        body = p['body']
        if digest(body) != p['hash']:
            raise ValueError('path hash mismatch')
        if p.get('session') == self.session and p.get('hash') == self.path_hash and self.state == 'READY':
            self.local_check(stationary=True)
            self.send('READY', nonce=p['nonce'], t1=p['t1'], t2=t2, t3=time.time())
            return
        if self.state not in TERMINAL:
            return
        self.generation += 1
        self.session, self.path_hash = str(p['session']), str(p['hash'])
        self.report('VALIDATING', 'checking new path before readiness')
        self.ready_deadline = time.monotonic()+5
        self.start_pose, self.start_mono = None, None
        self.local_check(stationary=True)
        actual_geometry = [self.geometry.axle_to_hinge, self.geometry.hinge_to_contact,
                           self.geometry.object_center_to_contact, self.geometry.hinge_limit]
        if len(body['geometry']) != 4 or any(abs(float(a)-b) > 1e-6 for a,b in zip(body['geometry'],actual_geometry)):
            raise ValueError('formation geometry differs between robots')
        leader_path = tuple(Pose2D(*map(float,xyz)) for xyz in body['leader'])
        if not 3 <= len(leader_path) <= 4000:
            raise ValueError('invalid path size')
        _, formation = leader_path_to_object_path(leader_path, self.geometry, lateral_tolerance=.01,
                                                 validate_curvature=self.validate_curvature)
        path = tuple(Pose2D(*map(float,xyz)) for xyz in body['follower'])
        if len(path) != len(formation.follower) or any(math.hypot(a.x-b.x,a.y-b.y) > .005 or abs(normalize_angle(a.yaw-b.yaw)) > .01 for a,b in zip(path,formation.follower)):
            raise ValueError('follower path disagrees with independent formation check')
        if drive_direction(path) not in ('FORWARD','REVERSE'):
            raise ValueError('unsupported path direction')
        expected = Pose2D(*map(float,body['expected_follower']))
        if not all(math.isfinite(v) for point in path+(expected,) for v in (point.x,point.y,point.yaw)):
            raise ValueError('nonfinite path')
        self.start_pose = self.robot
        # Physical neutral opposite-facing assembly defines this one-session SE(2)
        # transform. Equal odom frame strings NEVER imply equal origins.
        self.path = tuple(map_pose(point, expected, self.start_pose) for point in path)
        if math.hypot(self.path[0].x-self.robot.x,self.path[0].y-self.robot.y) > .05 or abs(normalize_angle(self.path[0].yaw-self.robot.yaw)) > math.radians(3):
            raise ValueError('initial follower alignment does not match neutral assembly')
        self.speed = min(float(body['speed']), self.p('speed'))
        if not math.isfinite(self.speed) or self.speed <= 0:
            raise ValueError('invalid speed')
        self.index, self.progress, self.peer_progress = 0, 0., 0.
        self.peer_time, self.peer_state = time.monotonic(), 'WAIT_READY'
        self.publish_path()
        self.report('LOCKING', 'path verified independently; setting selector STOP')
        self.ready_deadline = time.monotonic()+5
        self.source('STOP', lambda: self.report('READY', 'path frozen; motors stopped; awaiting leader start key'))
        # READY with clock timestamps is sent on the next PREPARE retry, after STOP acknowledgement.

    def arm_guard(self):
        generation = self.generation
        request = SetBool.Request(data=True)
        self.pending = True
        self.pending_since = time.monotonic()
        def done(future):
            if generation != self.generation:
                return
            self.pending = False
            try:
                if not future.result().success:
                    raise ValueError('guard did not arm')
                self.report('ARMED', 'selector and guard armed; command gate still zero')
            except Exception as error:
                self.stop(str(error))
        self.guard.call_async(request).add_done_callback(done)

    def communicate(self):
        if not self.session:
            return
        ratio = min(1., abs(self.rpp_command.linear.x)/self.p('speed')) if self.leader and time.monotonic()-self.rpp_time < .5 and self.follow_handle else 0.
        self.send('HB', state=self.state, progress=self.progress, speed_ratio=ratio)
        if self.leader and self.packet and self.state in ('WAIT_READY','READY') and time.monotonic()-self.last_prepare_send >= .5:
            self.last_prepare_send = time.monotonic()
            nonce = uuid.uuid4().hex
            self.pings[nonce] = time.monotonic()
            self.pings = {k:v for k,v in self.pings.items() if time.monotonic()-v < 1}
            if self.state == 'READY':
                self.send('PING', nonce=nonce, t1=time.time())
            else:
                self.send('PREPARE', body=self.packet, nonce=nonce, t1=time.time())
        elif self.leader and self.state == 'WAIT_ARM':
            self.send('ARM')
        elif not self.leader and self.state == 'ARMED':
            self.send('ARM_ACK')
        elif self.leader and self.state in ('WAIT_COMMIT','SCHEDULED'):
            self.send('COMMIT', start=self.start_wall, offset=self.offset)
        elif self.state in ('STOPPED','ERROR','DONE'):
            self.send('STOP', detail=self.state)

    def tick(self):
        command = Twist()
        try:
            now = time.monotonic()
            if self.pending and now-self.pending_since > 2.:
                raise ValueError('selector/guard service response timed out')
            if self.state not in TERMINAL and self.state not in ('WAIT_PLAN','LOCKING'):
                self.local_check(stationary=self.state not in ('RUNNING','ARRIVED'))
                if self.peer_time and now-self.peer_time > self.p('peer_timeout'):
                    raise ValueError('peer heartbeat lost')
            if self.state in ('LOCKING','WAIT_READY','ARMING','WAIT_ARM','ARMED','ARM_CONTROLLER') and now > self.ready_deadline:
                raise ValueError('readiness/arming timed out')
            if self.state == 'WAIT_COMMIT' and self.start_mono-now < .5:
                raise ValueError('no timely commit acknowledgement')
            if self.state == 'SCHEDULED' and now >= self.start_mono:
                if now-self.start_mono > .15:
                    raise ValueError('control callback missed start deadline')
                if self.peer_state not in ('SCHEDULED','RUNNING') or now-self.peer_time > .3:
                    raise ValueError('peer did not confirm scheduled state')
                self.report('RUNNING', 'scheduled gate opened; following frozen path')
            if self.state == 'RUNNING':
                result = compute_tracking_command(self.path, self.robot, self.index,
                                                  max_path_error=self.p('max_path_error'),
                                                  max_linear_speed=self.p('speed') if self.leader else self.speed,
                                                  max_angular_speed=.2, goal_tolerance=.05)
                self.index = result.progress_index
                self.progress = self.index/max(1,len(self.path)-1)
                if result.reached_goal:
                    self.progress = 1.
                    self.report('ARRIVED', 'at goal; holding zero while peer finishes')
                elif result.detail != 'tracking':
                    raise ValueError(result.detail)
                else:
                    if abs(self.progress-self.peer_progress) > .15:
                        raise ValueError('robots diverged in path progress')
                    factor = max(0., min(1., 1.-10*max(0.,self.progress-self.peer_progress-.02)))
                    factor *= min(1., max(0., (now-self.start_mono)/.2))
                    if self.leader:
                        if now-self.rpp_time > .5:
                            raise ValueError('Nav2 RPP command stream stale')
                        raw = self.rpp_command
                        scale = min(1., self.p('speed')/max(abs(raw.linear.x),1e-6))*factor
                        command.linear.x, command.angular.z = raw.linear.x*scale, raw.angular.z*scale
                        if abs(command.linear.x) < 1e-5 and abs(command.angular.z) > 1e-5:
                            raise ValueError('RPP requested in-place rotation while coupled')
                    else:
                        factor *= self.peer_speed_ratio
                        command.linear.x = result.command.linear_x*factor
                        command.angular.z = result.command.angular_z*factor
                    # Pure pursuit must not demand a turn tighter than the hinges.
                    if self.validate_curvature and abs(command.linear.x) > 1e-5 and abs(command.angular.z/command.linear.x) > self.geometry.curvature_limit(.8)*1.1:
                        raise ValueError('tracking command exceeds cooperative turn limit')
        except (ValueError, TypeError) as error:
            self.stop(str(error))
        # Idle peer must not stream zeros into another owner's topic.
        if self.state not in TERMINAL:
            self.cmd.publish(command)

    def stop(self, detail, notify=True, state='STOPPED', selector_stop=True):
        was_active = self.state not in TERMINAL
        if self.leader and self.follow_handle:
            self.follow_handle.cancel_goal_async()
            self.follow_handle = None
        self.generation += 1
        self.pending = False
        self.cmd.publish(Twist())
        self.start_mono = None
        if was_active and selector_stop and self.selector.service_is_ready():
            request = SetParameters.Request()
            request.parameters = [Parameter('source_mode', value='STOP').to_parameter_msg()]
            self.selector.call_async(request)
        if was_active and self.guard and self.guard.service_is_ready():
            self.guard.call_async(SetBool.Request(data=False))
        if notify and self.session:
            self.send('STOP', detail=detail)
        if was_active or state == 'DONE':
            self.report(state, detail)

def main(args=None):
    rclpy.init(args=args)
    node = TransportPeer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop('process shutdown')
            # Give DDS and STOP services a short chance to flush without starting motion.
            for _ in range(3):
                rclpy.spin_once(node, timeout_sec=.05)
        node.destroy_node()
        rclpy.try_shutdown()
