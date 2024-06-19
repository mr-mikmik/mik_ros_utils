import abc
import numpy as np

from mik_ros_utils.camera_utils.camera_parsers.camera_parser_base import CameraParserBase


class FakeCameraParserBase(CameraParserBase):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def _get_listeners(self):
        fake_listeners = {topic: None for topic, topic_name in self.topics.items()}
        return fake_listeners

    def _get_tf_listener(self):
        return None

    def _get_fake_camera_info(self):
        fake_camera_info = {
            'header.seq': 0,
            'header.stamp.secs': 0,
            'header.stamp.nsecs': 0,
            'header.frame_id': 'fake_camera_frame_id',
            'height': 0,
            'width': 0,
            'distortion_model': 'plumb_bob',
            'D': (0.0, 0.0, 0.0, 0.0, 0.0),
            'K': np.eye(3),
            'R': np.eye(3),
            'P': np.array([[1., 0., 1., 0.],
                           [0., 1., 1., 0.],
                           [0., 0., 1., 0.]]),
            'binning_x': 0,
            'binning_y': 0,
            'roi.x_offset': 0,
            'roi.y_offset': 0,
            'roi.height': 0,
            'roi.width': 0,
            'roi.do_rectify': False
        }
        return fake_camera_info

    def _get_image_color_shape(self):
        # Overide this if needed
        fake_color_shape = (25, 25, 3)
        return fake_color_shape

    def get_image_color(self):
        # Return an array representing the fake image
        fake_color_img_shape = self._get_image_color_shape()
        fake_color_img = np.zeros(fake_color_img_shape)
        return fake_color_img

    def get_camera_info_color(self):
        fake_camera_info = self._get_fake_camera_info()
        return fake_camera_info

