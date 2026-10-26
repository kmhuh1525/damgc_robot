from collections import Counter
from types import SimpleNamespace
from cooperative_manual.follower_prepare import graph, REQUIRED, CONFLICTS


def test_graph_filters_leader_and_counts_duplicates():
    n=SimpleNamespace(get_node_names_and_namespaces=lambda:[('manual_straight','/leader'),('manual_straight','/follower'),('velocity_guard','/follower'),('velocity_guard','/follower')])
    assert graph(n)==Counter(manual_straight=1,velocity_guard=2)


def test_preflight_requires_all_drive_and_motor_owners():
    assert set(REQUIRED)=={'command_selector','velocity_guard','stm32_bridge','dynamixel_orin_node','manual_straight'}
    assert {'gripper_sequence','mission_executor','transport_peer'}<=set(CONFLICTS)


def test_complete_launch_has_no_startup_gripper_motion_and_retains_hold():
    import importlib.util
    from pathlib import Path
    from launch import LaunchContext
    from launch.actions import IncludeLaunchDescription
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
    path=Path(__file__).parents[1]/'launch/follower_manual_ready.launch.py'
    spec=importlib.util.spec_from_file_location('follower_all_ready_test',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    description=module.generate_launch_description()
    children=[x for x in description.entities if isinstance(x,IncludeLaunchDescription)]
    assert len(children)==2
    context=LaunchContext();context.launch_configurations.update(gripper_port='/dev/ttyUSB0',i2c_device='/dev/i2c-7')
    values=[{k:perform_substitutions(context,normalize_to_list_of_substitutions(v)) for k,v in child.launch_arguments} for child in children]
    motor,drive=values
    assert motor['startup_pose_enabled']==motor['startup_torque']==motor['lift_enabled']=='false'
    assert motor['shutdown_torque_off']=='false' and motor['rx28_speed']=='40'
    assert drive['role']=='follower' and drive['drive_nodes']=='true' and drive['close_raw']=='331'
