import os
from mik_ros_utils.camera_utils.camera_parsers.depth_camera_parser_base import DepthCameraParserBase


class PicoFlexxCameraParser(DepthCameraParserBase):
    """
    Note: PicoFlexx cameras do not have chromatic sensing, and therefore the color topic is the illuminance
    """
    def _get_optical_frame_name(self):
        frame_names = {
            'color': '{}_optical_frame'.format(self.camera_name),
            'depth': '{}_optical_frame'.format(self.camera_name),
        }
        return frame_names

    def _get_color_topic(self):
        color_topic = '/{}/image_mono16'.format(self.camera_name)
        return color_topic

    def _get_depth_topic(self):
        depth_topic = '/{}/image_depth'.format(self.camera_name)
        return depth_topic

    def _get_pc_topic(self):
        pc_topic = '/{}/points'.format(self.camera_name)
        return pc_topic

    def _get_color_info_topic(self):
        color_info_topic = '/{}/camera_info'.format(self.camera_name)
        return color_info_topic

    def _get_depth_info_topic(self):
        depth_info_topic = '/{}/camera_info'.format(self.camera_name)
        return depth_info_topic

    def _get_camera_name_base(self):
        return 'pico_flexx'