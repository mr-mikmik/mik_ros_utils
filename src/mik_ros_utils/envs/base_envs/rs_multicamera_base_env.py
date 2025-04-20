from mik_ros_utils.camera_utils.camera_parsers import RealSenseCameraParser
from mik_tools.recording_utils.data_recording_wrappers import DictSelfSavedWrapper
from .multicamera_base_env import MultiCameraBaseEnv


class RealSenseMultiCameraBaseEnv(MultiCameraBaseEnv):
    """
        This environment handles an arbitrary number of cameras.
        """

    def __init__(self, *args, rs_camera_names=(), record_scene_pcs=True, aligned_depth=False, infra=False, save_depth_as_numpy=True, save_color_as_numpy=False, **kwargs):
        self.rs_camera_names = rs_camera_names
        self.aligned_depth = aligned_depth
        self.infra = infra  # Enable infrared if needed
        self.record_scene_pcs = record_scene_pcs
        self.save_depth_as_numpy = save_depth_as_numpy
        self.save_color_as_numpy = save_color_as_numpy
        super().__init__(*args, **kwargs)
        self.rs_camera_parsers = self._get_rs_camera_parsers()

    @classmethod
    def get_name(cls):
        return 'real_sense_multi_camera_base_env'

    def _get_rs_camera_parsers(self):
        rs_camera_parsers = []
        for camera_name in self.rs_camera_names:
            camera_parser = self._get_rs_camera_parser(camera_name)
            rs_camera_parsers.append(camera_parser)
        return rs_camera_parsers

    def _get_rs_camera_parser(self, camera_name):
        rs_camera_parser = RealSenseCameraParser(camera_name=camera_name,
                                                  scene_name=self.scene_name, save_path=self.save_path,
                                                  verbose=False, buffered=self.buffered,
                                                  save_depth_as_numpy=self.save_depth_as_numpy,
                                                  save_img_as_numpy=self.save_color_as_numpy,
                                                  wrap_data=self.wrap_data,
                                                  infra=self.infra,  # Enable infrared if needed
                                                  aligned_depth=self.aligned_depth,
                                                  record_pointcloud=self.record_scene_pcs)
        return rs_camera_parser

    def _get_rs_camera_observation(self):
        rs_camera_observation = {}
        for i, camera_parser in enumerate(self.rs_camera_parsers):
            obs_i = self._get_rs_camera_observation_from_camera_parser(camera_parser)
            camera_name = camera_parser.camera_name
            for k, v in obs_i.items():
                rs_camera_observation['{}_{}'.format(camera_name, k)] = v
        if self.wrap_data:
            rs_camera_observation = DictSelfSavedWrapper(rs_camera_observation)
        return rs_camera_observation

    def _get_rs_camera_observation_from_camera_parser(self, camera_parser):
        obs = {}
        obs['camera_info_color'] = camera_parser.get_camera_info_color()
        obs['color_img'] = camera_parser.get_image_color()
        if self.infra:
            obs['infra1'] = camera_parser.get_infra1()  # Get infrared image 1
            obs['infra2'] = camera_parser.get_infra2()
        else:
            obs['depth_img'] = camera_parser.get_image_depth()
            obs['camera_info_depth'] = camera_parser.get_camera_info_depth()
        if self.record_scene_pcs:
            obs['point_cloud'] = camera_parser.get_point_cloud()
        return obs

    def _get_tf_frames(self):
        frames = super()._get_tf_frames()
        for camera_parser in self.rs_camera_parsers:
            frames += camera_parser.get_camera_frames()
        return frames