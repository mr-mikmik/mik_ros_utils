import os
import rospy

from geometry_msgs.msg import WrenchStamped

from mik_ros_utils.aux import package_path
from mik_ros_utils.ros_utils import pack_message_as_dict, TF2Wrapper, Listener
from mik_ros_utils.utils.recording_utils.recording_utils import record_wrenches, pack_msgs_to_df
from mik_ros_utils.utils.recording_utils.data_recording_wrappers import WrenchSelfSavedWrapper



class WrenchRecorder(object):
    def __init__(self, wrench_topic, scene_name='scene', save_path=None, wrench_name='wrenches', wrap_data=False):
        self.wrench_topic = wrench_topic
        self.scene_name = scene_name
        self.wrench_name = wrench_name
        self.wrap_data = wrap_data
        self.save_path = self._get_save_path(save_path)
        self.wrench_listener = self._get_wrench_listener()
        self.tf2_listener = self._get_tf2_listener()
        self.counter = 0

    def _get_wrench_listener(self):
        wrench_listener = Listener(self.wrench_topic, WrenchStamped, wait_for_data=True)
        return wrench_listener

    def _get_tf2_listener(self):
        tf2_listener = TF2Wrapper()
        return tf2_listener

    def _get_save_path(self, save_path=None):
        if save_path is None:
            # Get some default directory based on the current working directory
            save_path = os.path.join(package_path, 'wrench_data')
        else:
            if save_path.startswith("/"):
                save_path = save_path  # we provide the full path (absolute)
            else:
                exec_path = os.getcwd()
                save_path = os.path.join(exec_path, save_path)  # we save the data on the path specified (relative)
        return save_path

    def get_wrench(self, frame_names=None):
        wrench = self.wrench_listener.get(block_until_data=True)
        wrenches = [wrench]
        if frame_names is not None:
            # record only the frame that the topic is published to
            for frame_name_i in frame_names:
                # import pdb; pdb.set_trace()
                wrench_frame_i = self.tf2_listener.transform_to_frame(wrench, target_frame=frame_name_i,
                                                                      timeout=rospy.Duration(secs=1))
                wrenches.append(wrench_frame_i)

        if self.wrap_data:
            wrenches = WrenchSelfSavedWrapper(wrenches, data_params={'scene_name': self.scene_name, 'save_path': self.save_path, 'wrench_name':self.wrench_name})
        return wrenches

    def record(self, fc=None, frame_names=None):
        if fc is None:
            fc = self.counter + 1
        wrenches = self.get_wrench(frame_names=frame_names)
        record_wrenches(wrenches, self.save_path, self.scene_name, fc)
        self.counter += 1

    def _pack_wrenches(self, wrenches):
        wrenches_df = pack_msgs_to_df(wrenches)
        return wrenches_df

    def _pack_message_as_dict(self, msg):
        msg_dict = pack_message_as_dict(msg)
        return msg_dict




