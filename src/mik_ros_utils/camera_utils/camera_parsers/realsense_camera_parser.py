import numpy as np
import cv2
import os

from mik_ros_utils.camera_utils.camera_parsers.depth_camera_parser_base import DepthCameraParserBase


class RealSenseCameraParser(DepthCameraParserBase):

    def __init__(self, *args, aligned_depth=False, **kwargs):
        self.aligned_depth = aligned_depth
        super().__init__(*args, **kwargs)

    def _get_optical_frame_name(self):
        if self.aligned_depth:
            depth_optical_frame_name = '{}_color_optical_frame'.format(self.camera_name)
        else:
            depth_optical_frame_name = '{}_depth_optical_frame'.format(self.camera_name)
        frame_names = {
            'color': '{}_color_optical_frame'.format(self.camera_name),
            'depth': depth_optical_frame_name,
        }
        return frame_names

    def _get_color_topic(self):
        color_topic = '/{}/color/image_raw'.format(self.camera_name)
        return color_topic

    def _get_depth_topic(self):
        if self.aligned_depth:
            depth_topic = '/{}/aligned_depth_to_color/image_raw'.format(self.camera_name)
        else:
            depth_topic = '/{}/depth/image_rect_raw'.format(self.camera_name)
        return depth_topic

    def _get_pc_topic(self):
        pc_topic = '/{}/depth/color/points'.format(self.camera_name)
        return pc_topic

    def _get_color_info_topic(self):
        color_info_topic = '/{}/color/camera_info'.format(self.camera_name)
        return color_info_topic

    def _get_depth_info_topic(self):
        if self.aligned_depth:
            depth_info_topic = '/{}/aligned_depth_to_color/camera_info'.format(self.camera_name)
        else:
            depth_info_topic = '/{}/depth/camera_info'.format(self.camera_name)
        return depth_info_topic

    def _get_camera_name_base(self):
        return 'camera'

    def _process_color_image_data(self, ros_data):
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        return image_np