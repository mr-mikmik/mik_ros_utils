import abc
import numpy as np
import rospy
import tf2_ros as tf2
from threading import Lock
import os
from collections import defaultdict
from sensor_msgs.msg import Image, CompressedImage, CameraInfo

from mik_ros_utils.ros_utils.ros_msg_functions import unpack_camera_info
from mik_ros_utils.ros_utils.listener import Listener
import mik_tools.recording_utils.recording_utils as record_utils
from mik_tools.recording_utils.data_recording_wrappers import PointCloudSelfSavedWrapper, DepthImageSelfSavedWrapper, ColorImageSelfSavedWrapper, CameraInfoDepthSelfSavedWrapper, CameraInfoColorSelfSavedWrapper
from mik_ros_utils.ros_utils.ros_msg_functions import process_point_cloud_msg, process_image_msg
from mik_ros_utils.ros_utils.tf_utils.tf_utils import record_tfs, get_tfs
from mik_ros_utils import package_path


class CameraParserBase(abc.ABC):

    def __init__(self, camera_name=None, camera_indx=None, scene_name='scene', save_path=None, save_img_as_numpy=False, verbose=True, wrap_data=False, buffered=False, no_listeners=False, buffer_wait=True, parent_frame=None):
        self.scene_name = scene_name
        self.verbose = verbose # TODO: Log into file or terminal
        self.wrap_data = wrap_data
        self.save_img_as_numpy = save_img_as_numpy
        self.buffered = buffered
        self.buffer_wait = buffer_wait
        self.parent_frame = parent_frame
        self.no_listeners = no_listeners
        self.camera_name = self._get_camera_name(camera_name)
        self.camera_indx = self._get_camera_indx(camera_indx)
        self.save_path = self._get_save_path(save_path)
        self.data_params = {'scene_name': self.scene_name, 'save_path': self.save_path, 'camera_name': self.camera_name}
        self.optical_frame = self._get_optical_frame_name()
        self.topics = self._get_topics()
        self.msgs_types = self._get_message_types()
        self.counter = {topic_key: 0 for topic_key in list(self.topics.keys())+['camera_info', 'tf']}
        self.additional_data_params = self._get_additional_data_params()
        self.data_wrappers = self._get_data_wrappers()
        self.msg_processors = self._get_processors_message_types() # dict of functions for each topic that take the raw message and porcess them.
        self.callbacks = {topic: [] for topic in self.topics.keys()}
        self.locks = {topic: Lock() for topic in self.topics.keys()}
        self.buffers = {topic: None for topic in self.topics.keys()} # TODO: Extend this to store multiple messages
        self.listeners = self._get_listeners()
        self.tf_buffer = tf2.Buffer(cache_time=rospy.Duration(secs=10))
        self.tf_listener = self._get_tf_listener()
        rospy.sleep(.2)

    @property
    def wait_for_data(self):
        return self.buffered

    def _get_listeners(self):
        if self.no_listeners:
            return None
        listeners = {
            topic: Listener(topic_name, self.msgs_types[topic], callback=self._create_topic_callback(topic),
                            wait_for_data=self.wait_for_data, queue_size=1)
            for topic, topic_name in self.topics.items()}
        return listeners

    def _get_camera_frame(self):
        camera_frame = '{}_link'.format(self.camera_name)
        return camera_frame

    def _get_tf_listener(self):
        tf_listener = tf2.TransformListener(buffer=self.tf_buffer, queue_size=1000, buff_size=500000)
        return tf_listener

    def _get_from_buffer(self, topic_key):
        with self.locks[topic_key]:
            topic_data = self.buffers[topic_key]
        if self.buffer_wait:
            while topic_data is None:
                rospy.sleep(.01)
                with self.locks[topic_key]:
                    topic_data = self.buffers[topic_key]
        return topic_data

    def _clean_buffer(self, topic_name=None):
        if topic_name is None:
            # clean all buffers
            for topic in self.buffers.keys():
                self.buffers[topic] = None
        else:
            # only clean the buffer for the specified topic
            self.buffers[topic_name] = None

    def _create_topic_callback(self, topic):
        def callback(topic_data):
            try:
                processed_data = self.msg_processors[topic](topic_data)
            except Exception as e:
                print('Error processing {} message: {}'.format(topic, e))
                processed_data = None
            for registered_callback in self.callbacks[topic]:
                registered_callback(processed_data)
            if self.buffered:
                with self.locks[topic]:
                    # add the data to the buffers
                    self.buffers[topic] = processed_data
        return callback

    @abc.abstractmethod
    def _get_color_topic(self):
        pass

    @abc.abstractmethod
    def _get_color_info_topic(self):
        pass

    @abc.abstractmethod
    def _get_camera_name_base(self):
        pass

    @abc.abstractmethod
    def _get_optical_frame_name(self):
        # TODO: Consider using the frame_id from the camera_info
        pass

    def get_camera_frames(self):
        """
        Return a list of the camera's important frames
        :return:
        """
        camera_frames = [self._get_camera_frame()] + list(self.optical_frame.values())
        camera_frames = list(np.unique(camera_frames))
        return camera_frames

    def _get_camera_name(self, camera_name):
        if camera_name is None:
            camera_name = '{}{}'.format(self.
                                        _get_camera_name_base(), self.camera_indx)
        return camera_name

    def _get_save_path(self, save_path=None):
        if save_path is None:
            # Get some default directory based on the current working directory
            save_path = os.path.join(package_path, 'pointcloud_data')
        else:
            if save_path.startswith("/"):
                save_path = save_path  # we provide the full path (absolute)
            else:
                exec_path = os.getcwd()
                save_path = os.path.join(exec_path, save_path)  # we save the data on the path specified (relative)
        return save_path

    def _get_topics(self):
        topics = {
            'color': self._get_color_topic(),
            'color_info': self._get_color_info_topic(),
        }
        return topics

    def _get_message_types(self):
        msgs_types = {
            'color': Image,
            'color_info': CameraInfo,
        }
        return msgs_types

    def _get_processors_message_types(self):
        processors = {
            'color': self._process_image_msg,
            'color_info': self._process_camera_info_msg,
        }
        return processors

    def _get_additional_data_params(self):
        additional_data_params = defaultdict(dict)
        additional_data_params['color'] = {'save_as_numpy': self.save_img_as_numpy}
        return additional_data_params

    def _get_data_wrappers(self):
        data_wrappers = {
            'color': ColorImageSelfSavedWrapper,
            'color_info': CameraInfoColorSelfSavedWrapper,
        }
        return data_wrappers

    def _get_camera_indx(self, camera_indx):
        if camera_indx is None:
            camera_indx = ''
        else:
            camera_indx = '_{}'.format(camera_indx)
        return camera_indx

    def restart(self, scene_name=None):
        self.counter = {topic_key: 0 for topic_key in list(self.topics.keys())+['camera_info', 'tf']}
        if scene_name is None:
            self.scene_name = scene_name

    def _get_topic_data(self, topic_key):
        if topic_key not in self.topics:
            self.log_message('Error: topic_key "{}" not found in topics: {}'.format(topic_key, list(self.topics.keys())))
            return None
        if self.buffered:
            # read from the buffer
            topic_data = self._get_from_buffer(topic_key)
        else:
            # force a new message, and wait until we have one
            ros_data = None
            while not rospy.is_shutdown():
                try:
                    ros_data = rospy.wait_for_message(self.topics[topic_key], self.msgs_types[topic_key])
                    self.log_message('{} received'.format(topic_key))
                    break
                except rospy.ROSInterruptException:
                    self.log_message('waiting for {}...'.format(topic_key))
            topic_data = self.msg_processors[topic_key](ros_data)
        if self.wrap_data:
            data_params = self.data_params.copy()
            additional_data_params = self.additional_data_params[topic_key]
            data_params.update(additional_data_params)
            topic_data = self.data_wrappers[topic_key](topic_data, data_params)
        return topic_data

    def get_image_color(self):
        img = self._get_topic_data('color')
        return img

    def get_camera_info_color(self):
        camera_info = self._get_topic_data('color_info')
        return camera_info

    def _process_image_msg(self, img_msg):
        # Assume it is a Image message
        np_img = process_image_msg(img_msg)
        return np_img

    def _process_camera_info_msg(self, camera_info_msg):
        processed_camera_info_msg = unpack_camera_info(camera_info_msg)
        return processed_camera_info_msg

    def save_image(self, img, filename, path=None):
        if path is None:
            save_path = self.save_path
        else:
            save_path = path

        file_save_path = record_utils.save_image(img, filename, save_path, save_as_numpy=self.save_img_as_numpy)
        self.log_message('Image saved at {}'.format(file_save_path))

    def save_camera_info(self, camera_info, filename, path=None):
        # save it as a yaml
        if path is None:
            save_path = self.save_path
        else:
            save_path = path
        if not os.path.exists(save_path):
            os.makedirs(save_path)
        file_path = os.path.join(save_path, '{}.npy'.format(filename))
        np.save(file_path, camera_info)

    def record(self, fc=None, ref_frame=None):
        self.record_image_color(fc=fc)
        self.record_camera_info(fc=fc)
        self.record_tfs(fc=fc)

    def record_image_color(self, fc=None):
        if fc is None:
            fc = self.counter['color']
        color_img = self.get_image_color()
        record_utils.record_image_color(color_img, self.save_path, self.scene_name, self.camera_name, fc, save_as_numpy=self.save_img_as_numpy)
        self.counter['color'] += 1
        return color_img

    def record_camera_info(self, fc=None):
        if fc is None:
            fc = self.counter['camera_info']
        camera_info_color = self.get_camera_info_color()
        record_utils.record_camera_info_color(camera_info_color, save_path=self.save_path, scene_name=self.scene_name, camera_name=self.camera_name, fc=fc)
        self.counter['camera_info'] += 1
        combined_info = {
            'color': camera_info_color,
        }
        return combined_info

    def record_tfs(self, fc=None):
        if fc is None:
            fc = self.counter['tf']
        tf_frames = self.get_camera_frames()
        if self.parent_frame is None:
            parent_frame = self._get_camera_frame()
            self.log_message('No parent frame provided, using the camera_frame: {}'.format(parent_frame))
        else:
            parent_fame = self.parent_frame
        tfs_df = get_tfs(tf_frames, parent_frame, verbose=self.verbose, buffer=self.tf_buffer)
        record_tfs(tfs_df, self.save_path, f'{self.scene_name}/{self.camera_name}', fc, file_name='tfs')
        self.counter['tf'] += 1

    def log_message(self, *args):
        # TODO: log in file
        if self.verbose:
            print(*args)

    def validate_topic(self, topic):
        if topic not in self.topics:
            raise RuntimeError(f"topic {topic} unknown, choose from {self.topics.keys()}")

    def register_callback(self, topic, callback):
        self.validate_topic(topic)
        self.callbacks[topic].append(callback)

    def clear_callbacks(self, topic):
        self.validate_topic(topic)
        self.callbacks[topic] = []