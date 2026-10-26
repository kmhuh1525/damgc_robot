"""Live odometry trail -> hinge-aware follower target; no arbitrary time lag."""
from collections import deque
from dataclasses import dataclass
import math

from cooperative_transport.motion import Pose2D, normalize_angle
from cooperative_transport.hinged_formation import HingeGeometry


CURVE_GEOMETRY=HingeGeometry(hinge_limit=math.radians(60.0))

def neutral_follower_pose(base,geometry):
    distance=2*(geometry.axle_to_hinge+geometry.longitudinal_offset)
    return Pose2D(base.x+distance*math.cos(base.yaw),base.y+distance*math.sin(base.yaw),normalize_angle(base.yaw+math.pi))


def finite_pose(p):
    return all(math.isfinite(x) for x in (p.x,p.y,p.yaw))


def map_pose(p,source,destination):
    a=normalize_angle(destination.yaw-source.yaw)
    dx,dy=p.x-source.x,p.y-source.y
    return Pose2D(destination.x+math.cos(a)*dx-math.sin(a)*dy,destination.y+math.sin(a)*dx+math.cos(a)*dy,normalize_angle(p.yaw+a))


def advance_follower(previous_leader,leader,follower,geometry):
    """Integrate the no-lateral-slip follower and rigid hinge-to-hinge span.

    The front hinge follows observed leader motion. The follower steers toward
    that hinge; its distance increment solves the fixed-span velocity constraint.
    This handles changing curvature without a constant-curvature pose shortcut.
    """
    a,b=geometry.axle_to_hinge,geometry.longitudinal_offset
    hl0=(previous_leader.x+a*math.cos(previous_leader.yaw),previous_leader.y+a*math.sin(previous_leader.yaw))
    hl1=(leader.x+a*math.cos(leader.yaw),leader.y+a*math.sin(leader.yaw))
    c,s=math.cos(follower.yaw),math.sin(follower.yaw)
    dx,dy=follower.x+a*c-hl0[0],follower.y+a*s-hl0[1]
    length=max(math.hypot(dx,dy),1e-6);ux,uy=dx/length,dy/length
    tx,ty=hl0[0]-follower.x,hl0[1]-follower.y
    body_y=-s*tx+c*ty
    curvature=2*body_y/max(tx*tx+ty*ty,.01)
    limit=min(3.,geometry.curvature_limit(.8))
    curvature=max(-limit,min(limit,curvature))
    denominator=ux*(c-a*curvature*s)+uy*(s+a*curvature*c)
    if abs(denominator)<.1:denominator=math.copysign(.1,denominator or -1.)
    distance=math.hypot(leader.x-previous_leader.x,leader.y-previous_leader.y)
    numerator=ux*(hl1[0]-hl0[0])+uy*(hl1[1]-hl0[1])-(length-2*b)*distance/max(2*b,.01)
    ds=max(-3*distance,min(3*distance,numerator/denominator))
    dyaw=curvature*ds
    return Pose2D(follower.x+ds*math.cos(follower.yaw+dyaw/2),follower.y+ds*math.sin(follower.yaw+dyaw/2),normalize_angle(follower.yaw+dyaw))


@dataclass(frozen=True)
class CurveOutput:
    linear: float
    angular: float
    lag: float=0.
    cross_track: float=0.
    position_error: float=0.


class FollowerSmoother:
    """Smooth quantized wheel speed and bound final follower acceleration."""
    def __init__(self):
        self.reset()

    def reset(self):
        self.measured=0.
        self.linear=0.
        self.angular=0.

    def step(self,commanded,measured,dt,compute):
        dt=max(0.,min(.1,dt))
        if abs(commanded)<1e-8:
            self.reset()
            return CurveOutput(0.,0.)
        alpha=dt/(.25+dt)
        self.measured+=alpha*(min(abs(commanded)*1.5,abs(measured))-self.measured)
        result=compute(math.copysign(self.measured,-commanded))
        self.linear+=max(-.02*dt,min(.02*dt,result.linear-self.linear))
        self.angular+=max(-.04*dt,min(.04*dt,result.angular-self.angular))
        # Keep curvature bounded even while linear speed ramps down.
        self.angular=max(-2.5*abs(self.linear),min(2.5*abs(self.linear),self.angular))
        return CurveOutput(self.linear,self.angular,result.lag,result.cross_track,result.position_error)


class LeaderTrail:
    def __init__(self,geometry=CURVE_GEOMETRY,step=.005,capacity=512):
        geometry.validate()
        if not 0 < step <= .02 or capacity<3:raise ValueError('invalid trail limits')
        self.geometry,self.step,self.capacity=geometry,step,capacity
        self.points=deque(maxlen=capacity)
        self.seeds=deque(maxlen=capacity)
        self.origin=None
        self.revision=0
        self.snapshot_seq=0

    def begin(self,pose):
        if not finite_pose(pose):raise ValueError('invalid initial leader odometry')
        self.origin=pose;self.points.clear();self.seeds.clear();self.snapshot_seq=0
        self.points.append((pose.x,pose.y,pose.yaw,0.,0.));self.revision=1
        self.seeds.append(neutral_follower_pose(pose,self.geometry))

    def add(self,pose):
        if not finite_pose(pose):raise ValueError('invalid leader odometry')
        if self.origin is None:return
        x,y,yaw,arc,k=self.points[-1]
        dx,dy=pose.x-x,pose.y-y
        distance=math.hypot(dx,dy)
        if distance>.15:raise ValueError('leader odometry discontinuity/reset')
        if distance<self.step:return
        dyaw=normalize_angle(pose.yaw-yaw)
        signed=math.cos(yaw+.5*dyaw)*dx+math.sin(yaw+.5*dyaw)*dy
        if abs(signed)<distance*.5:raise ValueError('leader odometry is not longitudinal')
        measured=max(-1.,min(1.,dyaw/signed))
        alpha=distance/(.10+distance)
        k=k+alpha*(measured-k)
        nominal=advance_follower(Pose2D(x,y,yaw),pose,self.seeds[-1],self.geometry)
        self.points.append((pose.x,pose.y,pose.yaw,arc+distance,k));self.seeds.append(nominal);self.revision+=1

    def packet(self,session):
        g=self.geometry
        self.snapshot_seq+=1
        return dict(version=1,session=session,sequence=self.snapshot_seq,revision=self.revision,
                    origin=[self.origin.x,self.origin.y,self.origin.yaw],
                    follower_seed=[self.seeds[0].x,self.seeds[0].y,self.seeds[0].yaw],
                    geometry=[g.axle_to_hinge,g.hinge_to_contact,g.object_center_to_contact,g.hinge_limit],points=list(self.points))


class FollowerTrail:
    def __init__(self,geometry=CURVE_GEOMETRY):
        geometry.validate();self.geometry=geometry
        self.reset(None)

    def reset(self,local_origin):
        self.local_origin=local_origin
        self.leader_origin=None
        self.path=[];self.arcs=[];self.revision=-1;self.received=None;self.snapshot_seq=-1

    @property
    def ready(self):return self.local_origin is not None and bool(self.path)

    def receive(self,packet,now):
        if packet.get('version')!=1:raise ValueError('trail version mismatch')
        revision=packet.get('revision')
        if type(revision) is not int:raise ValueError('invalid trail revision')
        seq=packet.get('sequence')
        if type(seq) is not int or seq<=self.snapshot_seq:return
        g=self.geometry;expected=(g.axle_to_hinge,g.hinge_to_contact,g.object_center_to_contact,g.hinge_limit)
        values=packet.get('geometry',[])
        if len(values)!=4 or not all(isinstance(v,(float,int)) and math.isfinite(v) and abs(v-e)<1e-6 for v,e in zip(values,expected)):
            raise ValueError('leader/follower geometry mismatch')
        origin=packet.get('origin',[])
        if len(origin)!=3 or not all(isinstance(v,(float,int)) and math.isfinite(v) for v in origin):raise ValueError('invalid leader origin')
        if self.leader_origin is not None and origin!=self.leader_origin:raise ValueError('leader origin changed during session')
        if self.local_origin is None:raise ValueError('follower initial odometry unavailable')
        if revision==self.revision:
            self.snapshot_seq=seq;self.received=now
            return
        if revision<self.revision:raise ValueError('trail revision went backwards')
        records=packet.get('points',[])
        if not 1<=len(records)<=512:raise ValueError('invalid trail size')
        expected_start=neutral_follower_pose(Pose2D(*origin),g)
        seed=packet.get('follower_seed',[])
        if len(seed)!=3 or not all(isinstance(v,(float,int)) and math.isfinite(v) for v in seed):raise ValueError('invalid rolling follower seed')
        nominal=Pose2D(*seed);previous=None
        path=[];arcs=[]
        for record in records:
            if len(record)!=5 or not all(isinstance(v,(float,int)) and math.isfinite(v) for v in record):raise ValueError('invalid trail sample')
            x,y,yaw,arc,k=record
            if arc<0 or (arcs and arc<=arcs[-1]):raise ValueError('nonmonotonic trail progress')
            current=Pose2D(x,y,yaw)
            if previous is not None:nominal=advance_follower(previous,current,nominal,g)
            previous=current
            path.append(map_pose(nominal,expected_start,self.local_origin));arcs.append(arc)
        self.path,self.arcs=path,arcs
        self.leader_origin=list(origin);self.revision=revision;self.snapshot_seq=seq;self.received=now

    def output(self,robot,leader_linear,now):
        if not finite_pose(robot):raise ValueError('invalid follower odometry')
        if abs(leader_linear)<1e-8:return CurveOutput(0.,0.)
        if not self.ready or self.received is None or now-self.received>.60:raise ValueError('leader trail updates missing while moving')
        target=self.path[-1]
        dx,dy=target.x-robot.x,target.y-robot.y
        ex=math.cos(robot.yaw)*dx+math.sin(robot.yaw)*dy
        ey=-math.sin(robot.yaw)*dx+math.cos(robot.yaw)*dy
        yaw_error=normalize_angle(target.yaw-robot.yaw)
        ratio,k=1.,0.
        if len(self.path)>=2:
            a,b=self.path[-2:];ds=self.arcs[-1]-self.arcs[-2]
            dyaw=normalize_angle(b.yaw-a.yaw)
            signed=math.cos(a.yaw+.5*dyaw)*(b.x-a.x)+math.sin(a.yaw+.5*dyaw)*(b.y-a.y)
            ratio=max(.5,min(1.5,abs(signed)/max(ds,1e-9)))
            k=dyaw/math.copysign(max(abs(signed),.001),signed)
        direction=1. if leader_linear<0 else -1.
        ff=-leader_linear*ratio
        # Large errors reduce speed continuously; there is no tracking-error FAULT.
        lateral_scale=max(.15,1./(1.+12.*abs(ey)+3.*abs(yaw_error)))
        linear=direction*max(abs(ff)*.1,min(.04,direction*(ff*lateral_scale+.6*ex)))
        angular=ff*k+direction*2.0*ey+.8*yaw_error
        turn_limit=min(.08,min(2.5,self.geometry.curvature_limit(.8))*abs(linear))
        angular=max(-turn_limit,min(turn_limit,angular))
        distance=math.hypot(dx,dy)
        nearest_arc=self.arcs[0];cross=float('inf')
        if len(self.path)==1:cross=distance
        for i,(a,b) in enumerate(zip(self.path,self.path[1:])):
            vx,vy=b.x-a.x,b.y-a.y;length2=vx*vx+vy*vy
            u=max(0.,min(1.,((robot.x-a.x)*vx+(robot.y-a.y)*vy)/max(length2,1e-12)))
            e=math.hypot(robot.x-a.x-u*vx,robot.y-a.y-u*vy)
            if e<cross:cross=e;nearest_arc=self.arcs[i]+u*(self.arcs[i+1]-self.arcs[i])
        lag=max(0.,self.arcs[-1]-nearest_arc)
        return CurveOutput(linear,angular,lag,cross,distance)


def leader_scale(lag,cross_track):
    if not all(math.isfinite(x) and x>=0 for x in (lag,cross_track)):raise ValueError('invalid follower progress feedback')
    return max(.15,min(1.,.02/(.02+max(0.,lag-.01)+cross_track)))
