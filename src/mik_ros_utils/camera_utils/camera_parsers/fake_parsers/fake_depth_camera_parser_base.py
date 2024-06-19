import abc
from abc import ABC

import numpy as np
from mik_ros_utils.camera_utils.camera_parsers.fake_parsers.fake_camera_parser_base import FakeCameraParserBase
from mik_ros_utils.camera_utils.camera_parsers import DepthCameraParserBase


class FakeDepthCameraParserBase(FakeCameraParserBase, DepthCameraParserBase, ABC):

    def _get_image_depth_shape(self):
        # Overide this if needed
        fake_depth_shape = (25, 25, 1)
        return fake_depth_shape

    def get_image_depth(self):
        # Return an array representing the fake image
        fake_depth_img_shape = self._get_image_depth_shape()
        fake_depth_img = np.zeros(fake_depth_img_shape)
        return fake_depth_img

    def get_point_cloud(self, ref_frame=None, return_ref_frame=False):
        # num_points = np.prod(np.array(self._get_image_depth_shape())[:-1])
        num_points = 100
        fake_pc_array = np.zeros((num_points, 6))
        fake_out_ref_frame = 'fake_frame'
        if return_ref_frame:
            return fake_pc_array, fake_out_ref_frame
        else:
            return fake_pc_array

    def get_camera_info_depth(self):
        fake_camera_info = self._get_fake_camera_info()
        return fake_camera_info

    def transform_pc(self, pc, origin_frame, target_frame):
        fake_pc_array_tr = pc
        return fake_pc_array_tr

