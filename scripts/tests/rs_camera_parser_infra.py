import rospy
import gym
import numpy as np
from collections import OrderedDict
from mik_tools import get_dataset_path
from mik_ros_utils.camera_utils.camera_parsers import RealSenseCameraParser
from mik_ros_utils.envs import RealSenseMultiCameraBaseEnv
from mik_tools.dataset_tools.data_collection import EnvDataCollector
from tqdm import tqdm



def test_parser_record():

    rospy.init_node('rs_camera_parser_infra_node')
    save_path = get_dataset_path('rs_camera_parser_infra_test2')
    parser = RealSenseCameraParser(camera_name='camera_1', save_path=save_path, record_pointcloud=False,
                                   aligned_depth=True, infra=False, wrap_data=True)


    parser.get_camera_info_depth = lambda:None

    for i in tqdm(range(10)):
        parser.record()
        rospy.sleep(1)



class TestEnv(RealSenseMultiCameraBaseEnv):
    """
    Test the multi camera env with RealSenseCameraParser
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, rs_camera_names=['camera_1'], **kwargs)

    @classmethod
    def get_name(cls):
        return 'test_rs_env'

    def _get_action_space(self):
        action_space_dict = OrderedDict()
        # action is defined as a change in pose w.r.t the world frame
        action_space_dict['x'] = gym.spaces.Box(low=-0.02, high=0.02, shape=())
        action_space = gym.spaces.Dict(action_space_dict)
        return action_space

    def _get_observation_space(self):
        pass

    def _get_observation(self):
        obs = {}
        rs_obs = self._get_rs_camera_observation()
        obs.update(rs_obs)
        return obs



def test_rs_mulitcamera_env():
    rospy.init_node('rs_camera_parser_infra_node')
    data_path = get_dataset_path('rs_multicamera_env_infra_test2')
    env = TestEnv(wrap_data=True, record_scene_pcs=False, save_depth_as_numpy=False, infra=True, buffered=True)

    dc = EnvDataCollector(env=env, data_path=data_path, scene_name='mik')
    dc.collect_data(10)


if __name__ == '__main__':
    # test_parser_record()
    test_rs_mulitcamera_env()