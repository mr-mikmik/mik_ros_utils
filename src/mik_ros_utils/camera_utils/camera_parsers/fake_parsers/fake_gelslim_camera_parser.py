import abc
import numpy as np
from mik_ros_utils.camera_utils.camera_parsers.fake_parsers.fake_camera_parser_base import FakeCameraParserBase
from mik_ros_utils.camera_utils.camera_parsers import GelslimCameraParser
from mik_ros_utils.camera_utils.camera_details import gelslim_camera_details


class FakeGelslimCameraParser(FakeCameraParserBase, GelslimCameraParser):
    def _get_image_color_shape(self):
        gelslim_color_img_shape = gelslim_camera_details['color_img_shape']
        return gelslim_color_img_shape
