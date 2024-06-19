import numpy as np
import cv2
import os

from mik_ros_utils.camera_utils.camera_parsers.fake_parsers.fake_depth_camera_parser_base import FakeDepthCameraParserBase
from mik_ros_utils.camera_utils.camera_parsers import RealSenseCameraParser
from mik_ros_utils.camera_utils.camera_details import realsense_camera_details


class FakeRealSenseCameraParser(FakeDepthCameraParserBase, RealSenseCameraParser):

    def _get_image_color_shape(self):
        realsense_color_img_shape = realsense_camera_details['color_img_shape']
        return realsense_color_img_shape

    def _get_image_depth_shape(self):
        # Overide this if needed
        realsense_depth_img_shape = realsense_camera_details['color_img_shape']
        return realsense_depth_img_shape