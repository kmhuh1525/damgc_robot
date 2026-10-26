"""Keep legacy fake-drive ROS tests out of the live robots' ROS domain."""
import os
import pytest


@pytest.fixture(autouse=True)
def isolated_fake_drive_tests(request):
    if request.node.fspath.basename not in ('test_follower_peer.py','test_follower_launch.py'):
        return
    if os.environ.get('ROS_DOMAIN_ID')!='176' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        pytest.skip('fake drive/services require isolated localhost ROS domain 176')
