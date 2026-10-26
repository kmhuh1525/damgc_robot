import copy
import math
import pytest
from cooperative_manual.curve import CURVE_GEOMETRY,LeaderTrail,FollowerTrail,leader_scale,neutral_follower_pose
from cooperative_transport.motion import Pose2D,normalize_angle


def advance(p,v,w,dt):
    return Pose2D(p.x+v*math.cos(p.yaw+w*dt/2)*dt,p.y+v*math.sin(p.yaw+w*dt/2)*dt,normalize_angle(p.yaw+w*dt))


def test_user_hinge60_and_neutral_separation():
    assert math.degrees(CURVE_GEOMETRY.hinge_limit)==pytest.approx(60.)
    p=neutral_follower_pose(Pose2D(0.,0.,0.),CURVE_GEOMETRY)
    assert p.x==pytest.approx(.62) and p.y==0 and abs(p.yaw)==pytest.approx(math.pi)


@pytest.mark.parametrize('sign',[-1.,1.])
def test_recorded_changing_curve_tracks_without_time_delay(sign):
    leader=Pose2D(0.,0.,0.);follower=Pose2D(.62,0.,math.pi)
    trail=LeaderTrail();trail.begin(leader);tracker=FollowerTrail();tracker.reset(follower)
    errors=[];lateral=[]
    for i in range(3000):
        t=i*.02
        k=0. if t<5 or t>50 else sign*.8*min(1.,(t-5)/10,(50-t)/10)
        leader=advance(leader,-.02,-.02*k,.02);trail.add(leader)
        if i%5==0:tracker.receive(trail.packet('s'),t)
        out=tracker.output(follower,-.02,t)
        assert 0<=out.linear<=.04 and abs(out.angular)<=.08
        follower=advance(follower,out.linear,out.angular,.02)
        errors.append(out.position_error);lateral.append(follower.y)
    assert max(errors)<.012  # Ideal odometry/unicycle model, not a hardware guarantee.
    assert max(lateral)-min(lateral)>.04
    assert tracker.output(follower,0.,60.).linear==tracker.output(follower,0.,60.).angular==0.


def test_rolling_trail_is_bounded_and_recomputes_consistent_seed():
    trail=LeaderTrail(capacity=8);trail.begin(Pose2D(0.,0.,0.))
    track=FollowerTrail();track.reset(Pose2D(.62,0.,math.pi))
    for i in range(1,30):
        trail.add(Pose2D(-i*.01,0.,0.));track.receive(trail.packet('s'),i*.1)
    assert len(trail.points)==len(track.path)==8
    assert track.path[-1].x==pytest.approx(.62-.29,abs=.0001)


def test_separate_odom_origins_align_without_shared_frame():
    trail=LeaderTrail();trail.begin(Pose2D(0.,0.,0.));trail.add(Pose2D(-.01,0.,0.))
    track=FollowerTrail();track.reset(Pose2D(4.,-2.,math.pi/2))
    track.receive(trail.packet('s'),0.)
    assert track.path[-1].x==pytest.approx(4.)
    assert track.path[-1].y==pytest.approx(-1.99)


def test_large_tracking_error_slows_and_corrects_without_exception_or_latch():
    trail=LeaderTrail();trail.begin(Pose2D(0.,0.,0.));trail.add(Pose2D(-.01,0.,0.))
    track=FollowerTrail();track.reset(Pose2D(.62,0.,math.pi));track.receive(trail.packet('s'),0.)
    bad=Pose2D(.9,.3,math.pi)
    out=track.output(bad,-.02,.1)
    assert out.position_error>.25 and out.linear>0 and abs(out.angular)<=.08
    assert .0<leader_scale(out.lag,out.cross_track)<1.


def test_explicit_snapshot_heartbeat_updates_even_when_stationary_but_replay_does_not():
    trail=LeaderTrail();trail.begin(Pose2D(0.,0.,0.));track=FollowerTrail();track.reset(Pose2D(.62,0.,math.pi))
    first=trail.packet('s');track.receive(first,0.)
    track.receive(first,.4);assert track.received==0.
    track.receive(trail.packet('s'),.4);assert track.received==.4
    with pytest.raises(ValueError,match='updates missing'):track.output(Pose2D(.62,0.,math.pi),-.02,1.1)
    assert track.output(Pose2D(.62,0.,math.pi),0.,2.).linear==0.


def test_geometry_mismatch_and_changed_origin_are_configuration_errors():
    trail=LeaderTrail();trail.begin(Pose2D(0.,0.,0.));track=FollowerTrail();track.reset(Pose2D(.62,0.,math.pi))
    p=trail.packet('s');bad=copy.deepcopy(p);bad['geometry'][3]=math.radians(15)
    with pytest.raises(ValueError,match='geometry mismatch'):track.receive(bad,0.)
    track.receive(p,0.);bad=trail.packet('s');bad['origin'][0]=1.
    with pytest.raises(ValueError,match='origin changed'):track.receive(bad,.1)


def test_reverse_curvature_estimate_uses_signed_distance():
    trail=LeaderTrail();trail.begin(Pose2D(0.,0.,0.));p=Pose2D(0.,0.,0.)
    for _ in range(150):p=advance(p,-.02,-.01,.1);trail.add(p)
    assert trail.points[-1][4]>0.4  # w/v=+0.5; Euclidean ds would invert the sign.


def test_sensor_reset_is_distinct_from_tracking_error():
    trail=LeaderTrail();trail.begin(Pose2D(0.,0.,0.))
    with pytest.raises(ValueError,match='discontinuity'):trail.add(Pose2D(1.,0.,0.))


def test_follower_output_smooths_encoder_zero_pulses_and_stops_immediately():
    from cooperative_manual.curve import FollowerSmoother,CurveOutput
    smoother=FollowerSmoother()
    previous=CurveOutput(0.,0.)
    for i in range(200):
        result=smoother.step(-.02,.04 if i%2 else 0.,.02,
            lambda speed:CurveOutput(abs(speed),abs(speed)*2))
        assert abs(result.linear-previous.linear)<=.000400001
        assert abs(result.angular-previous.angular)<=.000800001
        if i>50:assert result.linear>.005
        previous=result
    result=smoother.step(0.,.04,.02,lambda speed:CurveOutput(.04,.08))
    assert result.linear==result.angular==0.
    assert smoother.measured==0.
