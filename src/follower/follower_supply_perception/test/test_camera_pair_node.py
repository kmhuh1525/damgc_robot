from types import SimpleNamespace as NS
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo
from follower_supply_perception.camera_pair_node import CameraPairNode


def harness():
    images, infos = [], []
    bridge = CvBridge()
    image = bridge.cv2_to_imgmsg(np.zeros((8, 8), dtype=np.uint8), encoding="mono8")
    image.header.stamp.sec = 10
    image.header.frame_id = "camera_optical"
    info = CameraInfo(width=8, height=8)
    info.k = [10.,0.,4.,0.,10.,4.,0.,0.,1.]
    info.p = [10.,0.,4.,0.,0.,10.,4.,0.,0.,0.,1.,0.]
    info.r = np.eye(3).reshape(-1).tolist()
    info.d = [0.,0.,0.,0.,0.]
    node = NS(info=info, maps=None, key=None, bridge=bridge, image_pub=NS(publish=images.append), info_pub=NS(publish=infos.append), get_clock=lambda:NS(now=lambda:NS(nanoseconds=10_500_000_000)))
    return node, image, images, infos


def test_rectified_image_and_info_share_acquisition_header():
    n, image, images, infos = harness()
    CameraPairNode.on_image(n, image)
    assert len(images) == len(infos) == 1
    assert images[0].header == infos[0].header == image.header
    assert images[0].encoding == "mono8"
    assert n.info.header.stamp.sec == 0  # Cached calibration is not mutated.


def test_stale_image_cannot_be_relabelled_fresh():
    n, image, images, infos = harness()
    image.header.stamp.sec = 7
    CameraPairNode.on_image(n, image)
    assert not images and not infos


def test_calibration_resolution_mismatch_is_rejected():
    n, image, images, infos = harness()
    n.info.width = 640
    CameraPairNode.on_image(n, image)
    assert not images and not infos
