import numpy as np
import rospy
import time

from mik_ros_utils.utils.recording_utils import WrenchRecorder
from .ros_base_env import ROSBaseEnv
try:
    from netft_utils.ft_sensor import FTSensor
except ImportError:
    pass


class FTSensorBaseEnv(ROSBaseEnv):
    """
    This environment adds 2 scene cameras
    """
    def __init__(self, *args, ft_sensor_name='netft', ft_topic_name='netft_data', **kwargs):
        self.ft_sensor_name = ft_sensor_name
        self.topic_name = ft_topic_name
        self.ft_sensor_frames = self._get_ft_sensor_frames()
        super().__init__(*args, **kwargs)
        self.ft_sensor = self._get_ft_sensor()
        self.ft_sensor_wrench_recorder = self._get_ft_sensor_wrench_recorder()

    @classmethod
    def get_name(cls):
        return 'ft_sensor_base_env'

    def _get_ft_sensor(self):
        ft_sensor = FTSensor(ns=self.ft_sensor_name)
        return ft_sensor

    def _get_ft_sensor_frames(self):
        ft_sensor_frames = ['link_ft']
        return ft_sensor_frames

    def _get_ft_sensor_wrench_recorder(self):
        ft_sensor_wrench_recorder = WrenchRecorder(f'/{self.ft_sensor_name}/{self.topic_name}', scene_name=self.scene_name,
                                                   save_path=self.save_path,
                                                   wrap_data=self.wrap_data, wrench_name='ft_sensor_wrenches')
        return ft_sensor_wrench_recorder

    def _get_wrench_frames(self):
        wrench_frames = [] # frames for which to record teh wrench
        wrench_frames += self.ft_sensor_frames
        return wrench_frames

    def _get_ft_wrench(self):
        frame_names = self._get_wrench_frames()
        wrench = self.ft_sensor_wrench_recorder.get_wrench(frame_names=frame_names)
        return wrench

    def _get_tf_frames(self):
        tf_frames = super()._get_tf_frames() + self.ft_sensor_frames
        return tf_frames

    def _get_ft_sensor_observation(self):
        obs = {}
        ft_sensor_wrench = self._get_ft_wrench()
        obs['ft_sensor_wrench'] = ft_sensor_wrench
        return obs
