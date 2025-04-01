import numpy as np
import cv2
import os
from sensor_msgs.msg import Image

from mik_ros_utils.camera_utils.camera_parsers.depth_camera_parser_base import DepthCameraParserBase
from mik_tools.recording_utils.data_recording_wrappers import ImageSelfSavedWrapper
import mik_tools.recording_utils.recording_utils as record_utils


class RealSenseCameraParser(DepthCameraParserBase):

    def __init__(self, *args, aligned_depth=False, infra=False, **kwargs):
        self.infra = infra
        self.aligned_depth = aligned_depth
        super().__init__(*args, **kwargs)

    def get_infra1(self):
        infra1_img = self._get_topic_data('infra1')
        return infra1_img

    def get_infra2(self):
        infra2_img = self._get_topic_data('infra2')
        return infra2_img

    def record(self, fc=None, **kwargs):
        super().record(fc=fc, **kwargs)
        if self.infra:
            self.record_infra(fc=fc)

    def record_image_depth(self, fc=None):
        """
        Record depth image in addition to color image. This is overridden to ensure Infrared images are also recorded.
        """
        if self.infra:
            # If infrared is enabled, we will not record depth image since it is not available when infrared is enabled in realsense2_camera.
            return None
        else:
            return super().record_image_depth(fc=fc)

    def record_infra(self, fc=None):
        """
        Record infrared images if the camera has infrared capabilities.
        This function can be called to ensure that infrared images are recorded.
        """
        if fc is None:
            fc1 = self.counter['infra1']
            fc2 = self.counter['infra2']
        else:
            # Allow passing in a frame count to record at a specific frame count.
            fc1 = fc
            fc2 = fc
        print('called record_infra')
        if self.infra:
            # Get depth images
            infra1_img = self.get_infra1()
            infra2_img = self.get_infra2()
            # Record infrared images
            record_utils.record_image(infra1_img, self.save_path, self.scene_name, self.camera_name, fc1, image_name='infra1', save_as_numpy=self.save_depth_as_numpy)
            record_utils.record_image(infra2_img, self.save_path, self.scene_name, self.camera_name, fc2, image_name='infra2', save_as_numpy=self.save_depth_as_numpy)
            self.counter['infra1'] += 1
            self.counter['infra2'] += 1
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
            # remove the depth topic and depth_infro since both are not published at the same time.
            topics.pop('depth', None)  # remove depth if it exists, since we will use aligned depth for infra
            topics.pop('depth_info', None)  # remove depth_info if it exists, since we will use aligned depth for infra
        return topics

    def _get_message_types(self):
        msgs_types = super()._get_message_types()
        added_msgs_types = {
            'infra1': Image,
            'infra2': Image,
        }
        msgs_types.update(added_msgs_types)
        return msgs_types

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