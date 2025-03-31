import numpy as np
import cv2
import os

from mik_ros_utils.camera_utils.camera_parsers.depth_camera_parser_base import DepthCameraParserBase
from mik_tools.recording_utils.data_recording_wrappers import ImageSelfSavedWrapper


class RealSenseCameraParser(DepthCameraParserBase):

    def __init__(self, *args, aligned_depth=False, infra=False, **kwargs):
        self.infra = infra
        self.aligned_depth = aligned_depth
        super().__init__(*args, **kwargs)

    def record_infra(self):
        """
        Record infrared images if the camera has infrared capabilities.
        This function can be called to ensure that infrared images are recorded.
        """
        if self.infra:
            # Record infrared images
            self._record_data('infra1')
            self._record_data('infra2')
        else:
            print("Infrared recording is not enabled for this camera.")

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

    def _get_infra1_topic(self):
        infra_topic = '/{}/infra1/image_rect_raw'.format(self.camera_name)
        return infra_topic

    def _get_infra2_topic(self):
        infra_topic = '/{}/infra1/image_rect_raw'.format(self.camera_name)
        return infra_topic

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

    def _get_topics(self):
        topics = super()._get_topics()
        if self.infra:
            added_topics = {
                'infra1': self._get_infra1_topic(),
                'infra2': self._get_infra2_topic(),
            }
            topics.update(added_topics)
        return topics

    def _get_processors_message_types(self):
        processors = super()._get_processors_message_types()
        added_processors = {
            'infra1': self._process_image_msg,
            'infra2': self._process_image_msg,
        }
        processors.update(added_processors)
        return processors

    def _get_data_wrappers(self):
        data_wrappers = super()._get_data_wrappers()
        added_wrappers = {
            'infra1': ImageSelfSavedWrapper,
            'infra2': ImageSelfSavedWrapper,
        }
        data_wrappers.update(added_wrappers)
        return data_wrappers

    def _get_additional_data_params(self):
        additional_data_params = super()._get_additional_data_params()
        additional_data_params['infra1'] = {'image_name': 'infra1'}
        additional_data_params['infra2'] = {'image_name': 'infra2'}
        return additional_data_params

    def _process_color_image_data(self, ros_data):
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        return image_np