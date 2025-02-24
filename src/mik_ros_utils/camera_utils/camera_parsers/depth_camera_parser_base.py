import abc
import os
import tf
import rospy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2

from mik_tools.camera_tools.pointcloud_utils import save_pointcloud, tr_pointcloud
from mik_tools import tr, pose_to_matrix
import mik_tools.recording_utils.recording_utils as record_utils
from mik_tools.recording_utils.data_recording_wrappers import PointCloudSelfSavedWrapper, DepthImageSelfSavedWrapper, \
    CameraInfoDepthSelfSavedWrapper
from mik_ros_utils.ros_utils.ros_msg_functions import process_point_cloud_msg
from mik_ros_utils.camera_utils.camera_parsers.camera_parser_base import CameraParserBase
from mik_ros_utils.ros_utils.tf_utils.tf_utils import get_tf


class DepthCameraParserBase(CameraParserBase):
    def __init__(self, *args, record_pointcloud=True, save_depth_as_numpy=False, **kwargs):
        self.save_depth_as_numpy = save_depth_as_numpy
        self.record_pointcloud = record_pointcloud
        super().__init__(*args, **kwargs)

    @abc.abstractmethod
    def _get_depth_topic(self):
        pass

    @abc.abstractmethod
    def _get_pc_topic(self):
        pass

    @abc.abstractmethod
    def _get_depth_info_topic(self):
        pass

    def _get_topics(self):
        topics = super()._get_topics()
        added_topics = {
            'depth': self._get_depth_topic(),
            'depth_info': self._get_depth_info_topic(),
        }
        if self.record_pointcloud:
            added_topics['pointcloud'] = self._get_pc_topic()
        topics.update(added_topics)
        return topics

    def _get_message_types(self):
        msgs_types = super()._get_message_types()
        added_msgs_types = {
            'depth': Image,
            'pointcloud': PointCloud2,
            'depth_info': CameraInfo,
        }
        msgs_types.update(added_msgs_types)
        return msgs_types

    def _get_processors_message_types(self):
        processors = super()._get_processors_message_types()
        added_processors = {
            'depth': self._process_image_msg,
            'pointcloud': self._process_point_cloud_msg,
            'depth_info': self._process_camera_info_msg,
        }
        processors.update(added_processors)
        return processors

    def _get_additional_data_params(self):
        additional_data_params = super()._get_additional_data_params()
        additional_data_params['depth'] = {'save_as_numpy': self.save_depth_as_numpy}
        return additional_data_params

    def _get_data_wrappers(self):
        data_wrappers = super()._get_data_wrappers()
        added_wrappers = {
            'depth': DepthImageSelfSavedWrapper,
            'pointcloud': self.__wrap_pointcloud_and_frame,
            'depth_info': CameraInfoDepthSelfSavedWrapper,
        }
        data_wrappers.update(added_wrappers)
        return data_wrappers

    def get_image_depth(self):
        img = self._get_topic_data('depth')
        return img

    def get_point_cloud(self, ref_frame=None, return_ref_frame=False):
        pc_array, pc_frame = self._get_topic_data('pointcloud')
        out_ref_frame = pc_frame
        if ref_frame is not None:
            if ref_frame != pc_frame:
                # Get tf between them and transform the pointcloud so it is in the frame specified
                pc_tr = self.transform_pc(pc_array, origin_frame=pc_frame, target_frame=ref_frame)
                if pc_tr is not None:
                    # The transformation was successful
                    pc_array = pc_tr
                    out_ref_frame = ref_frame

        if return_ref_frame:
            return pc_array, out_ref_frame
        else:
            return pc_array

    def get_camera_info_depth(self):
        camera_info = self._get_topic_data('depth_info')
        return camera_info

    def transform_pc(self, pc, origin_frame, target_frame):
        # Lisent for the transformation between frames:
        # following line for checking frames' existence no longer works with newer yaml libraries
        # if self.tf_listener.frameExists(origin_frame) and self.tf_listener.frameExists(target_frame):
        try:
            # t, q = self.tf_listener.lookupTransform(pc_frame, ref_frame, rospy.Time(0))
            tf_pose_of = get_tf(child_name=target_frame, parent_name=origin_frame, tf_listener=self.tf_listener)
            of_X_tf = pose_to_matrix(tf_pose_of)
            tf_X_of = tr.inverse_matrix(of_X_tf)
            pc_array_tr = self._transform_pc(pc, T=tf_X_of)
            self.log_message(
                'returning the poincloud in frame {} (original frame: {})'.format(target_frame, origin_frame))
            return pc_array_tr
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            self.log_message('Could not transform point cloud from frame {} to {}'.format(origin_frame, target_frame))
            return None

    def _transform_pc(self, pc_array, R=None, t=None, T=None):
        pc_array_tr = tr_pointcloud(pc_array, R=R, t=t, T=T)
        return pc_array_tr

    def _process_point_cloud_msg(self, pc_msg):
        # Try xyz rgb
        pc_ar = process_point_cloud_msg(pc_msg)
        pc_frame = pc_msg.header.frame_id
        processed_pc = (pc_ar, pc_frame)
        return processed_pc

    def save_point_cloud(self, pc, filename, path=None):
        if path is None:
            save_path = self.save_path
        else:
            save_path = path
        if not os.path.exists(save_path):
            os.makedirs(path)
        save_pointcloud(pc, filename, save_path)

    def record(self, fc=None, ref_frame=None):
        super().record(fc=fc)
        self.record_image_depth(fc=fc)
        if self.record_pointcloud:
            self.record_point_cloud(fc=fc, ref_frame=ref_frame)

    def record_point_cloud(self, ref_frame=None, fc=None):
        if fc is None:
            fc = self.counter['pointcloud']
        pc = self.get_point_cloud(ref_frame=ref_frame)
        record_utils.record_point_cloud(pc, self.save_path, self.scene_name, self.camera_name, fc)
        self.counter['pointcloud'] += 1
        return pc

    def record_image_depth(self, fc=None):
        if fc is None:
            fc = self.counter['depth']
        depth_img = self.get_image_depth()
        record_utils.record_image_depth(depth_img, self.save_path, self.scene_name, self.camera_name, fc,
                                        save_as_numpy=self.save_depth_as_numpy)
        self.counter['depth'] += 1
        return depth_img

    def record_camera_info(self, fc=None):
        if fc is None:
            fc = self.counter['camera_info']
        camera_info_depth = self.get_camera_info_depth()
        camera_info_color = self.get_camera_info_color()
        record_utils.record_camera_info_depth(camera_info_depth, save_path=self.save_path, scene_name=self.scene_name, camera_name=self.camera_name, fc=fc)
        record_utils.record_camera_info_color(camera_info_color, save_path=self.save_path, scene_name=self.scene_name, camera_name=self.camera_name, fc=fc)
        self.counter['camera_info'] += 1
        combined_info = {
            'color': camera_info_color,
            'depth': camera_info_depth,
        }
        return combined_info

    def __wrap_pointcloud_and_frame(self, pointcloud_and_frame, data_params=None):
        pointcloud, frame = pointcloud_and_frame
        wrapped_pointcloud = PointCloudSelfSavedWrapper(pointcloud, data_params=data_params)
        return (wrapped_pointcloud, frame)