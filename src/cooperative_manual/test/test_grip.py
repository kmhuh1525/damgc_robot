import pytest
from cooperative_manual.grip import PairedGrip


def pair():
    l=PairedGrip('leader',True,1000,480,.2)
    f=PairedGrip('follower',True,1000,331,.1)
    l.request('GRASP','session',0.)
    return l,f


def cycle(l,f,t,logs,drop_commit=False,ack=True):
    lc,lp=l.tick(t,True,True)
    logs[0].extend(lc)
    for target in lc:
        if ack:l.motor_status('OK rx64=-1 rx28=%d rx64_torque=-1 rx28_torque=1'%target)
    for p in lp:
        if not drop_commit or p['kind']!='GRIP_COMMIT':f.receive(p,'session',t,True,True)
    fc,fp=f.tick(t,True,True)
    logs[1].extend(fc)
    for target in fc:
        if ack:f.motor_status('OK rx64=-1 rx28=%d rx64_torque=-1 rx28_torque=1'%target)
    for p in fp:l.receive(p,'session',t,True,True)


def test_both_targets_once_and_done_waits_for_both():
    l,f=pair();logs=[[],[]]
    for i in range(60):cycle(l,f,i*.02,logs)
    assert l.phase==f.phase=='DONE'
    assert logs==[[480],[331]]
    l.request('RELEASE','session',1.2)
    for i in range(60,120):cycle(l,f,i*.02,logs)
    assert l.phase==f.phase=='DONE'
    assert logs==[[480,1000],[331,1000]]


def test_no_command_until_base_stopped_and_peer_ready():
    l,f=pair()
    commands,packets=l.tick(.1,False,True)
    assert commands==packets==[]
    commands,packets=l.tick(.2,True,True)
    assert not commands and packets[0]['kind']=='GRIP_PREPARE'
    f.receive(packets[0],'session',.2,False,True)
    assert not f.busy and not f.commands
    f.receive(packets[0],'session',.3,True,True)
    assert f.phase=='READY' and not f.commands


def test_repeated_commit_cannot_reexecute_motor():
    l,f=pair();logs=[[],[]]
    for i in range(40):cycle(l,f,i*.02,logs)
    old=f.packet('GRIP_COMMIT');old['role']='leader'
    for i in range(10):
        f.receive(old,'session',1.+i*.02,True,True)
        commands,_=f.tick(1.+i*.02,True,True)
        assert not commands
    assert logs==[[480],[331]]


def test_missing_ack_fails_without_reissuing_local_motor_command():
    l,f=pair();logs=[[],[]]
    for i in range(220):cycle(l,f,i*.02,logs,ack=False)
    assert l.phase=='ERROR' and f.phase=='ERROR'
    assert logs==[[480],[331]]


def test_commit_loss_retries_remote_but_local_executes_once():
    l,f=pair();logs=[[],[]]
    for i in range(80):cycle(l,f,i*.02,logs,drop_commit=i<15)
    assert l.phase==f.phase=='DONE'
    assert logs==[[480],[331]]


def test_unavailable_follower_prevents_leader_motor_command():
    l,f=pair()
    _,packets=l.tick(.1,True,True)
    f.receive(packets[0],'session',.1,True,False)
    _,reply=f.tick(.1,True,False)
    for p in reply:l.receive(p,'session',.1,True,True)
    assert l.phase=='ERROR' and not l.commands


def test_motor_error_or_failure_to_stop_cancels_action():
    l,f=pair();logs=[[],[]]
    for i in range(5):cycle(l,f,i*.02,logs)
    l.motor_status('ERROR bus lost')
    assert l.phase=='ERROR'
    l,f=pair();l.tick(3.1,False,True)
    assert l.phase=='ERROR' and not l.commands


def test_unconfigured_leader_cannot_enable_gripper():
    with pytest.raises(ValueError):PairedGrip('leader',True)


def test_old_session_and_action_packets_do_not_actuate():
    l,f=pair();_,p=l.tick(.1,True,True)
    f.receive(p[0],'different-session',.1,True,True)
    assert f.phase=='IDLE'
