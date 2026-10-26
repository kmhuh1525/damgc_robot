import math
import pytest
from cooperative_manual.logic import Limits, StraightSync


def pair():
    leader, follower = StraightSync('leader'), StraightSync('follower')
    leader.enable(0., 'new-session')
    follower.enable(0.)
    leader.armed_ok()
    follower.armed_ok()
    return leader, follower


def cycle(leader, follower, now, command=-.03, deliver_l=True, deliver_f=True):
    if command is not None:
        leader.input(now, command)
    lv, lp = leader.step(now)
    if deliver_l:
        for p in lp:
            follower.receive(p, now)
    fv, fp = follower.step(now)
    if deliver_f:
        for p in fp:
            leader.receive(p, now)
    return lv, fv


def moving_pair():
    l, f = pair()
    for i in range(70):
        cycle(l, f, i*.02)
    return l, f


def test_no_motion_without_explicit_arming_and_peer_ready():
    l, f = StraightSync('leader'), StraightSync('follower')
    assert l.step(0.)[0] == f.step(0.)[0] == 0.
    l.enable(0., 'new')
    l.input(.1, -.03)
    assert l.step(.1)[0] == 0.
    l.armed_ok()
    assert l.step(.2)[0] == 0.


def test_one_profile_opposite_body_signs_same_world_direction():
    l, f = pair()
    previous = 0.
    for i in range(80):
        lv, fv = cycle(l, f, i*.02)
        assert lv <= 0 and fv >= 0
        assert fv == pytest.approx(-lv)
        assert abs(lv-previous) <= .03*.02 + 1e-9
        previous = lv
    assert lv == pytest.approx(-.03)


def test_normal_release_decelerates_shared_profile_to_zero():
    l, f = moving_pair()
    previous = l.value
    for i in range(70, 135):
        lv, fv = cycle(l, f, i*.02, command=0.)
        assert fv == pytest.approx(-lv)
        assert abs(lv-previous) <= .03*.02 + 1e-9
        previous = lv
    assert lv == fv == 0.


def test_operator_stop_both_zero_and_requires_new_enable():
    l, f = moving_pair()
    l.stop(1.4, 'operator stop', fault=False)
    lv, packets = l.step(1.4)
    for p in packets:
        f.receive(p, 1.4)
    assert lv == f.step(1.4)[0] == 0.
    assert not l.enabled and not f.enabled
    l.input(1.5, -.03)
    assert l.step(1.5)[0] == 0.


@pytest.mark.parametrize('lost_direction',['leader','follower'])
def test_partition_stops_both_and_recovery_cannot_restart(lost_direction):
    l, f = moving_pair()
    for i in range(70, 100):
        cycle(l, f, i*.02, deliver_l=lost_direction!='leader', deliver_f=lost_direction!='follower')
    assert l.state == f.state == 'FAULT'
    assert l.value == f.value == 0.
    for i in range(100, 110):
        assert cycle(l, f, i*.02) == (0.,0.)


def test_duplicate_command_does_not_refresh_watchdog():
    _, f = moving_pair()
    old_time = f.peer_time
    f.receive(dict(version=1, role='leader', kind='COMMAND', session=f.session, seq=f.received_seq, linear=-.03),old_time+.2)
    assert f.peer_time == old_time
    assert f.step(old_time+.26)[0] == 0.
    assert f.state == 'FAULT'


def test_stale_session_is_ignored():
    l, f = moving_pair()
    f.receive(dict(version=1,role='leader',session='old-session',kind='COMMAND',seq=99999,linear=-.03),1.4)
    assert f.received_seq < 99999
    l.receive(dict(version=1,role='follower',session='old-session',kind='STATUS',heartbeat_seq=99999,armed=True,state='ACTIVE',ack=l.seq),1.4)
    assert l.peer_status_seq < 99999


def test_manual_publisher_death_stops_both():
    l, f = moving_pair()
    for i in range(70,100):
        cycle(l,f,i*.02,command=None)
    assert l.state == f.state == 'FAULT'
    assert l.value == f.value == 0.


def test_fresh_heartbeats_without_command_ack_cannot_keep_leader_moving():
    l, f = moving_pair()
    for i in range(70,95):
        now=i*.02
        l.input(now,-.03)
        l.receive(dict(version=1,role='follower',session=l.session,kind='STATUS',heartbeat_seq=1000+i,state='ACTIVE',armed=True,ack=-1),now)
        l.step(now)
    assert l.state == 'FAULT'
    assert 'acknowledgement' in l.reason


@pytest.mark.parametrize('linear,axes',[(-.01,(0.,0.,0.,0.,.1)),(math.nan,(0.,)*5),(-.01,(0.,math.inf,0.,0.,0.))])
def test_turns_and_nonfinite_commands_rejected(linear,axes):
    l,_=moving_pair()
    l.input(1.4,linear,axes)
    assert l.state == 'FAULT' and l.value == 0.


def test_pre_enable_input_is_not_reused():
    l=StraightSync('leader')
    l.input(0.,-.03)
    l.enable(1.,'new')
    assert l.input_time is None and l.target == 0.


def test_delayed_loop_cannot_create_large_acceleration_jump():
    l,f=moving_pair()
    l.target=0.
    # Preserve liveness independently, then cap dt despite a delayed timer.
    l.peer_time=1.6;l.last_ack_sent=1.6;l.input_time=1.6
    before=l.value
    after=l.step(1.6)[0]
    assert abs(after-before)<=l.limits.acceleration*l.limits.max_dt+1e-9


def test_follower_can_wait_for_operator_before_leader_enables():
    f=StraightSync('follower');f.enable(0.);f.armed_ok()
    assert f.step(20.)[0]==0. and f.state=='WAIT_PEER'
    f.receive(dict(version=1,role='leader',session='later',kind='PREPARE'),20.)
    assert f.session=='later' and f.enabled_at==20.


def test_new_session_while_moving_cannot_silently_take_control():
    _,f=moving_pair()
    f.receive(dict(version=1,role='leader',session='intruder',kind='PREPARE'),1.4)
    assert f.state=='FAULT' and f.value==0.


@pytest.mark.parametrize('linear',[math.nan,math.inf,.5,-.5,True])
def test_follower_rejects_out_of_contract_velocity(linear):
    _,f=moving_pair()
    f.receive(dict(version=1,role='leader',session=f.session,kind='COMMAND',seq=f.received_seq+1,linear=linear),1.4)
    assert f.state=='FAULT' and f.value==0.


def test_peer_timeout_is_latched_and_start_without_peer_times_out():
    l=StraightSync('leader');l.enable(0.,'alone');l.armed_ok()
    assert l.step(8.1)[0]==0. and l.state=='FAULT'


def test_forward_leader_is_reverse_follower():
    l,f=pair()
    for i in range(75):
        lv,fv=cycle(l,f,i*.02,command=.03)
        assert lv>=0 and fv<=0
        assert fv==pytest.approx(-lv)
    assert lv==pytest.approx(.03)


def test_direction_change_passes_through_zero_without_jump():
    l,f=moving_pair();values=[];previous=l.value
    for i in range(70,200):
        lv,fv=cycle(l,f,i*.02,command=.03)
        values.append(lv)
        assert fv==pytest.approx(-lv)
        assert abs(lv-previous)<=.03*.02+1e-9
        previous=lv
    assert 0. in values and values[-1]==pytest.approx(.03)
