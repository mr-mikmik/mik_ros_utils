import numpy as np
import cv2
import os

from mik_ros_utils.camera_utils.camera_parsers.fake_parsers.fake_depth_camera_parser_base import FakeDepthCameraParserBase
from mik_ros_utils.camera_utils.camera_parsers import PicoFlexxCameraParser
from mik_ros_utils.camera_utils.camera_details import picoflexx_camera_details


class FakePicoFlexxCameraParser(FakeDepthCameraParserBase, PicoFlexxCameraParser):

    def _get_image_color_shape(self):
        realsense_color_img_shape = picoflexx_camera_details['color_img_shape']
        return realsense_color_img_shape

    def _get_image_depth_shape(self):
        # Overide this if needed
        realsense_depth_img_shape = picoflexx_camera_details['color_img_shape']
        return realsense_depth_img_shape