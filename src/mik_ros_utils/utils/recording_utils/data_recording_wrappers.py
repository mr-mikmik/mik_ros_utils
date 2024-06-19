from mik_tools.recording_utils.data_recording_wrappers import DataSelfSavedWrapper
from mik_ros_utils.utils.recording_utils.recording_utils import record_wrenches
from mik_ros_utils.ros_utils.tf_utils.tf_utils import record_tfs


class WrenchSelfSavedWrapper(DataSelfSavedWrapper):
    def save_fc(self, fc):
        save_path = self.data_params['save_path']
        scene_name = self.data_params['scene_name']
        wrench_name = None
        if 'wrench_name' in self.data_params:
            wrench_name = self.data_params['wrench_name']
        record_wrenches(self.data, save_path=save_path, scene_name=scene_name, fc=fc, wrench_name=wrench_name)


class TFSelfSavedWrapper(DataSelfSavedWrapper):
    def save_fc(self, fc):
        save_path = self.data_params['save_path']
        scene_name = self.data_params['scene_name']
        if 'file_name' in self.data_params:
            record_tfs(self.data, save_path=save_path, scene_name=scene_name, fc=fc, file_name=self.data_params['file_name'])
        else:
            record_tfs(self.data, save_path=save_path, scene_name=scene_name, fc=fc)