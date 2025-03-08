import numpy as np
import rospy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
try:
    from phoxi_camera.srv import GetFrame
except ImportError:
    pass
from .depth_camera_parser_base import DepthCameraParserBase


class PhotoneoMotionCamParser(DepthCameraParserBase):
    """
    This camera does not publish depthmaps, but pointclouds directly, we will read the poitncloud as depthmap and process it to its shape
    """


    def __init__(self, *args, camera_frame_name='MotionCam_sensor', buffered=True, save_depth_as_numpy=True, **kwargs):
        self.camera_frame_name = camera_frame_name # frame of the scan
        super().__init__(*args, buffered=buffered, save_depth_as_numpy=save_depth_as_numpy, **kwargs)
        self.scan_srv = rospy.ServiceProxy(f'/{self.camera_name}/get_frame', GetFrame)

    @property
    def wait_for_data(self):
        return False

    def scan(self):
        resp = self.scan_srv(-1)
        return None

    def _get_camera_name_base(self):
        return 'motioncam'

    def _get_camera_frame(self):
        return self.camera_frame_name

    def _get_color_topic(self):
        color_topic = '/{}/rgb_texture'.format(self.camera_name)
        return color_topic

    def _get_depth_topic(self):
        depth_topic = '/{}/pointcloud'.format(self.camera_name)
        return depth_topic

    def _get_pc_topic(self):
        pc_topic = '/{}/pointcloud'.format(self.camera_name)
        return pc_topic

    def _get_color_info_topic(self):
        color_info_topic = '/{}/camera_info'.format(self.camera_name)
        return color_info_topic

    def _get_depth_info_topic(self):
        depth_info_topic = '/{}/camera_info'.format(self.camera_name)
        return depth_info_topic


    def _get_optical_frame_name(self):
        optical_frames = {
            'color': self.camera_frame_name,
            'depth': self.camera_frame_name,
        }
        return optical_frames

    def _get_processors_message_types(self):
        processors = super()._get_processors_message_types()
        processors_override = {
            'depth': self._process_depth_msg, # depth is received as pointcloud, so we need to process it accordingly
        }
        processors.update(processors_override)
        return processors

    def _get_message_types(self):
        msgs_types = super()._get_message_types()
        msgs_types_override = {
            'depth': PointCloud2, # depth is received as pointcloud, so we need to process it accordingly
        }
        msgs_types.update(msgs_types_override)
        return msgs_types

    def _process_depth_msg(self, pc_msg):
        # Convert the pointcloud to numpy array
        pc_ar, pc_frame = self._process_point_cloud_msg(pc_msg)
        # reshape the pointcloud to the image shape
        pc_ar = pc_ar.reshape((pc_msg.height, pc_msg.width, 6))
        depth_ar = pc_ar[:, :, 2:3] # (h, w, 6) -> (h, w, 1) selecting only the z
        # NOTE: The return is (h, w, 1) for consistency (same with Realsense parsers)
        return depth_ar

    def get_image_color(self):
        img = self._get_topic_data('color')
        return img