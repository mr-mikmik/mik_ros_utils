

import rospy
import tf2_ros as tf
# import tf as tf
import copy
import threading
import rospy
import threading
import abc
import copy
import time

from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped, Pose, PoseWithCovarianceStamped
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from mik_ros_utils.ros_utils import TFBroadcaster, pack_tf


class CameraApriltagTFDetection(object):
    """
    This class listens to topics {camera_name}/tag_detections contained the detected apriltags.
    Then, it broadcasts the position of the tags with respect to the common frame, which we assume fixed (camera frame or other frame).

    # TODO: Combine multiple detections from different cameras
    """
    def __init__(self, continuous=False, tag_ids=None, parent_frame_name=None, tag_ids_names={}, rate=5.0, time_on_buffer=1.0, clean_buffer=True, add_camera_name_to_tag=True):
        """
         :param continuous: <bool>
            * If True, it will keep updating the position of the cameras with respect to the detected tag.
            * If False, the reading is only done once, and it keeps broadcasting the same tf.
        :param tag_ids: Single or collection of ids of the tags. Example tags_ids=(1,2,3) will look fot the tags 1, 2, and 3.
            For bundles, it is only necessary to specify one of the bundle tags, and the pose will be the bundle pose.
        :param parent_frame_name: <str> Name of the frame to set as parent for the detected tags. If None, it is the same as the camera frame
        :param tag_ids_names: <dict <int>, <str>> mapping of spacial names to the tags to be detacted
        :param rate: <float> Frequency at which to publish the detected tags
        :param time_on_buffer: <float> time in seconds to keep a tag in the buffer before being cleaned.
        :param clean_buffer: <bool> if false, we will not clean the buffer, and detected tags will be published until the process is closed.
            * Note: If they are detected again, they will still be updated.
        :param add_camera_name_to_tag: <bool> If True, it will add the camera name to the tag name. Example: tag_1_{camera_name}

        """
        if type(tag_ids) is not list:
            tag_ids = list(tag_ids)
        self.tag_ids = tag_ids
        self.tag_ids_names = tag_ids_names
        self.continuous = continuous
        self.rate = rate
        self.clean_buffer = clean_buffer
        self.time_on_buffer = time_on_buffer
        self.add_camera_name_to_tag = add_camera_name_to_tag

        self.broadcast_parent_frame_name = parent_frame_name

        self.lock = threading.Lock()
        rospy.init_node('camera_tag_detection')
        self.tf_buffer = tf.BufferCore()
        self.transformer = tf.TransformListener(self.tf_buffer)
        self.tf_broadcaster = TFBroadcaster(rate=self.rate)
        self.detections = {}
        self.tfs = {} # Buffer containing the detected tfs to be broadcasted for each of the tags, key
        self.subscribers = self._get_subscribers()
        self.broadcast_tfs()
        rospy.spin()

    def broadcast_tfs(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            try:
                self._update_tfs()
                packed_tfs = []
                for camera_id, tfs_i in self.tfs.items():
                    for tf_i in tfs_i:
                        packed_tf_i = pack_tf(tf_i)
                        packed_tfs.append(packed_tf_i)
                self.tf_broadcaster.tfs = packed_tfs
                if self.continuous:
                    self.perform_buffer_cleaning()
            except:
                continue
            rate.sleep()

    def perform_buffer_cleaning(self):
        # Remove from the self.tfs the tags that have been there fore more than self.time_on_buffer
        if not self.clean_buffer:
            # do not clean the buffer
            pass
        elif self.time_on_buffer is None or self.time_on_buffer <= 0.:
            # clean the full buffer
            self.tfs = {}
        else:
            # update the buffer removing the tags that expired (time in buffer > timeout)
            for camera_id, tfs_i in self.tfs.items():
                tfs_to_keep = []
                for tf_i in tfs_i:
                    time_since_received = time.time() - tf_i['received_time']
                    if time_since_received > self.time_on_buffer:
                        # we have had the tag longer than the timeout, and therefore needs to be removed.
                        pass
                    else:
                        tfs_to_keep.append(tf_i)
                self.tfs[camera_id] = tfs_to_keep

    def _get_transform_from_detection(self, detection, tag_id=None):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = detection.pose.header.frame_id
        if tag_id is None:
            t.child_frame_id = 'tag_{}'.format(detection.id[0])
        else:
            t.child_frame_id = 'tag_{}'.format(tag_id)
        t.transform.translation.x = detection.pose.pose.pose.position.x
        t.transform.translation.y = detection.pose.pose.pose.position.y
        t.transform.translation.z = detection.pose.pose.pose.position.z
        t.transform.rotation.x = detection.pose.pose.pose.orientation.x
        t.transform.rotation.y = detection.pose.pose.pose.orientation.y
        t.transform.rotation.z = detection.pose.pose.pose.orientation.z
        t.transform.rotation.w = detection.pose.pose.pose.orientation.w
        return t

    def _get_subscribers(self):
        # this listents to all topic and returns a list of subscribers for the topics {camera_name}/tag_detections_i
        # - Get all topics:
        all_topics = rospy.get_published_topics()
        # - Filter out all topics not {camera_name}/tag_detections
        detected_topics = [t[0] for t in all_topics if t[0][-15:]== '/tag_detections']
        # - Create a Subscriber for each topic
        subscribers = []
        for i, topic in enumerate(detected_topics):
            # topic composed by /{camera_name}/tag_detections
            camera_name = topic[1:-15] # remove the first '/' and the last '/tag_detections'
            subscriber_i = rospy.Subscriber(topic, AprilTagDetectionArray, self._update_detections, camera_name)
            subscribers.append(subscriber_i)
        return subscribers

    def _update_detections(self, msg, camera_name):
        self.lock.acquire()
        try:
            self.detections[camera_name] = msg.detections
        finally:
            self.lock.release()

    def _update_tfs(self):
        self.lock.acquire()
        try:
            for camera_name, detections in self.detections.items():
                if camera_name in self.tfs and self.continuous is False:
                    continue
                else:
                    for detection in detections:
                        for tag_id in self.tag_ids:
                            if tag_id in detection.id:
                                pose = detection.pose # PoseWithCovarianceStampted
                                # get the transformation from tag_id to camera_i_link
                                t = self._get_transform_from_detection(detection, tag_id=tag_id)
                                self.tf_buffer.set_transform(t, 'default_authority')
                                camera_frame_name = f'{camera_name}_link'
                                ltr = self.tf_buffer.lookup_transform_core('tag_{}'.format(tag_id), camera_frame_name, rospy.Time(0))
                                # pos, rot = b[0], b[1]
                                # pack the transform
                                # Calibration Frame Transform:
                                if self.broadcast_parent_frame_name is None:
                                    parent_name = f'{camera_name}_link'
                                else:
                                    parent_name = self.broadcast_parent_frame_name
                                ltr = self.tf_buffer.lookup_transform_core(parent_name, 'tag_{}'.format(tag_id), rospy.Time(0))
                                tf_i = {}
                                tf_i['frame_id'] = parent_name
                                child_id = self._get_child_id(tag_id, camera_name)
                                tf_i['child_id'] = child_id
                                tf_i['x'] = ltr.transform.translation.x
                                tf_i['y'] = ltr.transform.translation.y
                                tf_i['z'] = ltr.transform.translation.z
                                tf_i['qx'] = ltr.transform.rotation.x
                                tf_i['qy'] = ltr.transform.rotation.y
                                tf_i['qz'] = ltr.transform.rotation.z
                                tf_i['qw'] = ltr.transform.rotation.w
                                tf_i['received_time'] = time.time() # record also the time at which we recieve it for computing the time in the buffer and claring it.
                                if camera_name in self.tfs:
                                    # search for the tf in the buffer with the same child_id to update it
                                    # TODO: Consider changing self.tfs as a dict of dicts instead of a dict of lists for more efficient search.
                                    tf_exists_indx = None
                                    for i, existing_tf_i in enumerate(self.tfs[camera_name]):
                                        if existing_tf_i['child_id'] == child_id:
                                            tf_exists_indx = i
                                            break
                                    if tf_exists_indx is None:
                                        self.tfs[camera_name].append(tf_i)
                                    else:
                                        self.tfs[camera_name][tf_exists_indx] = tf_i # override the tf_i
                                else:
                                    self.tfs[camera_name] = [tf_i]
        finally:
            self.lock.release()

    def _get_child_id(self, tag_id, camera_name):
        if tag_id not in self.tag_ids_names:
            child_id = 'tag_{}'.format(tag_id) # TODO: Integrate multiple cameras
        else:
            child_id = self.tag_ids_names['tag_id']
        if self.add_camera_name_to_tag:
            child_id += f'_{camera_name}' # TODO: Integrate multiple cameras
        return child_id