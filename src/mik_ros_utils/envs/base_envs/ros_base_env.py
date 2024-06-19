import rospy
import tf2_ros as tf2


from mik_tools.env_tools.base_env import BaseEnv
from mik_tools.recording_utils.data_recording_wrappers import DictSelfSavedWrapper
from mik_ros_utils.ros_utils.tf_utils.tf_utils import get_tfs, get_tf
from mik_ros_utils.ros_utils.tf2_wrapper import TF2Wrapper
from mik_ros_utils.utils.recording_utils.data_recording_wrappers import TFSelfSavedWrapper


class ROSBaseEnv(BaseEnv):
    """
    Environment with:
     - Med robot
     - Wrench recorder
     - Tf listener
    """
    def __init__(self, save_path=None, scene_name='default_scene', ref_frame='med_base', wrap_data=False, verbose=False, buffered=False):
        self.save_path = self._get_save_path(save_path)
        self.ref_frame = ref_frame
        self.scene_name = scene_name
        self.wrap_data = wrap_data
        self.verbose = verbose
        self.buffered = buffered
        self._init_ros_node()
        self.tf_buffer = tf2.Buffer(cache_time=rospy.Duration(secs=10))
        self.tf_listener = self._get_tf_listener()
        self.tf2_listener = self._get_tf2_listener()
        super().__init__()

    @classmethod
    def get_name(cls):
        return 'ros_base_env'

    # Functions to extend to add more frames to record
    def _get_tf_frames(self):
        tf_frames = [] # return a list of frames to be recorded
        return tf_frames

    # Other useful functions:
    def _get_save_path(self, save_path):
        if save_path is None:
            save_path = '/tmp/{}_data'.format(self.get_name())
        return save_path

    def _get_tf_listener(self):
        tf_listener = tf2.TransformListener(buffer=self.tf_buffer, queue_size=1000, buff_size=500000)
        return tf_listener

    def _get_tf2_listener(self):
        tf2_listener = TF2Wrapper(buffer=self.tf_buffer, listener=self.tf_listener)
        return tf2_listener

    def _init_ros_node(self):
        try:
            rospy.init_node('{}_node'.format(self.get_name()))
        except (rospy.exceptions.ROSInitException, rospy.exceptions.ROSException):
            pass

    def _get_tfs(self):
        tf_frames = self._get_tf_frames()
        parent_names = self.ref_frame
        tfs = get_tfs(tf_frames, parent_names, verbose=self.verbose, buffer=self.tf_buffer) # df of the frames
        if self.wrap_data:
            tfs = TFSelfSavedWrapper(tfs, data_params={'save_path': self.save_path, 'scene_name': self.scene_name})
        return tfs

    def _get_tf(self, frame_id, ref_frame=None):
        if ref_frame is None:
            ref_frame = self.ref_frame
        tf_pose = get_tf(child_name=frame_id, parent_name=ref_frame, buffer=self.tf_buffer)
        return tf_pose

    def get_observation(self):
        # adds the wrapping into a self-saved data for easy saving when collecting data
        obs = self._get_observation()
        if self.wrap_data:
            obs = DictSelfSavedWrapper(obs, data_params={'save_path': self.save_path, 'scene_name': self.scene_name})
        return obs