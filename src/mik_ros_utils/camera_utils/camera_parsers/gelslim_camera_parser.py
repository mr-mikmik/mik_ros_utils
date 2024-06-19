import os
from sensor_msgs.msg import Image, CompressedImage, CameraInfo

from mik_ros_utils.camera_utils.camera_parsers.camera_parser_base import CameraParserBase


class GelslimCameraParser(CameraParserBase):
    """
    Note: Gelslim cameras have only color.
    """
    # def _process_image_msg(self, img_msg):
    #     # Assume it is a Image message
    #     np_img = process_compressed_img_msg(img_msg)
    #     return np_img
    def _get_message_types(self):
        msgs_types = super()._get_message_types()
        msgs_types['color'] = CompressedImage
        return msgs_types

    def _get_optical_frame_name(self):
        frame_names = {
            'color': '{}_optical_frame'.format(self.camera_name),
            'depth': '{}_camera_frame'.format(self.camera_name),
        }
        return frame_names

    def _get_color_topic(self):
        color_topic = '/{}/image/compressed'.format(self.camera_name)
        return color_topic

    def _get_color_info_topic(self):
        color_info_topic = '/{}/camera_info'.format(self.camera_name)
        return color_info_topic

    def _get_camera_name_base(self):
        return 'gelslim'
