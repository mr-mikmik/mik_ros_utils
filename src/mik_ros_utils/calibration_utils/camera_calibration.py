#!/usr/bin/env python
import rospy
import tf2_ros as tf
import numpy as np
import copy
import threading
import time
from geometry_msgs.msg import TransformStamped
from apriltag_ros.msg import AprilTagDetectionArray

from mik_tools import tr
from mik_tools.camera_tools.pointcloud_utils import find_best_transform


class CameraApriltagCalibration(object):
    """
    Calibrate a set of n cameras given a tag
    Usage:
     - take_measurement:
     - broadcast_tf: when we have callected all the measurements, call broadcast_tf to compute and broadcast the estimated tf
    """

    def __init__(self, tag_id=None, calibration_frame_name='calibration_frame', parent_frame_name=None, only_camera_ids=None, rate=10):
        """
        Calibration class for a set of cameras given a tag
        :param tag_id (str): id of the tag to be used for calibration
            if None, it will pick the first detection tag we find
        :param calibration_frame_name: <str> name of the frame
        :param parent_frame_name: <str> name of the frame which we will broadcast the cameras tfs wrt
            if None, it will be set to the calibration_frame_name
        :param only_camera_ids: <list> if provided, we only broadcast the tfs for the cameras with the ids in the list.
            if None, we will broadcast tfs for all cameras available
        :param rate:
        """
        self.tag_id = tag_id
        self.calibration_frame_name = calibration_frame_name
        self.only_camera_ids = only_camera_ids
        self.broadcast_parent_frame_name = parent_frame_name
        if self.broadcast_parent_frame_name is None:
            self.broadcast_parent_frame_name = self.calibration_frame_name
        self.rate = rate
        self.is_alive = True
        self.lock = threading.Lock()

        # rospy.init_node('camera_calibration')
        self.tf_buffer = tf.BufferCore(cache_time=rospy.Duration(60))
        self.transformer = tf.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.latest_detections = {} # stores the latest tag detections -- (shared resource)
        self.tfs_measures = {} # stores the measured tfs between the broadcast_parent_frame_name and the camera frames
        self.calibration_buffer = {} # stores the tfs between cameras and calibration_frame, and parent_frame_name and calibration_frame. It is used for aggregation purposes.
        self.tfs = {} # Stores the computed and aggregated tfs to be broadcasted
        self.subscribers = self._get_subscribers()
        self.broadcast_thread = threading.Thread(target=self._broadcast_tfs_callback)

    def reset(self):
        self.tfs_measures = {}
        self.tfs = {}
        self.subscribers = self._get_subscribers()

    def finish(self):
        with self.lock:
            self.is_alive = False
        self.broadcast_thread.join()

    def _get_subscribers(self):
        """
        Listens to all topic and returns a list of subscribers for the topics /tag_detections_i
        :return: <list> containing Subscriber objects for each tag_detections topics available
        """
        # - Get all topics:
        all_topics = rospy.get_published_topics()
        # - Filter out all topics not /tag_detections_i
        detected_topics = [t[0] for t in all_topics if 'tag_detections' in t[0]]
        detected_topics = [t for t in detected_topics if 'image' not in t]# remove image topics
        # - Create a Subscriber for each topic
        subscribers = []
        for i, topic in enumerate(detected_topics):
            camera_id = topic.split('tag_detections_')[-1]
            subscriber_i = rospy.Subscriber(topic, AprilTagDetectionArray, self._update_detections, camera_id)
            subscribers.append(subscriber_i)
        return subscribers

    def _update_detections(self, msg, indx):
        # Callback for subscriber for camera with index indx
        self.lock.acquire()
        try:
            self.latest_detections[indx] = msg.detections
        finally:
            self.lock.release()

    def take_mesurement(self):
        """
        Add a new measurement into the calibration buffer.
        :return:
        """
        # Clear detections buffer to make sure we have a new measurement (latest available)
        self.lock.acquire()
        try:
            self.latest_detections = {}
        finally:
            self.lock.release()
        # Wait a moment to make sure the detected tags are updated
        time.sleep(2.0)
        # Update each camera measures adding the new detections for each camera
        self._update_measurements()

    def _update_measurements(self):
        self.lock.acquire()
        try:
            # self.tf_buffer = tf.BufferCore()
            # self.transformer = tf.TransformListener(self.tf_buffer)
            for camera_id, detections in self.latest_detections.items():
                for detection in detections:
                    if self.tag_id in detection.id:
                        pose = detection.pose # PoseWithCovarianceStampted
                        # get the transformation from tag_id to camera_i_link
                        t = self._get_transform_from_detection(detection, tag_id=self.tag_id)
                        self.tf_buffer.set_transform(t, 'default_authority')
                        ctr = self.tf_buffer.lookup_transform_core('tag_{}'.format(self.tag_id), 'camera_{}_link'.format(camera_id), rospy.Time(0))
                        # pos, rot = b[0], b[1]
                        # pack the transform
                        # Calibration Frame Transform:
                        calib_frame_tr = copy.deepcopy(ctr)
                        calib_frame_tr.header.frame_id = self.calibration_frame_name
                        self.tf_buffer.set_transform(calib_frame_tr, 'default_authority')
                        rospy.sleep(.1)
                        # Get transformation between the
                        ltr = self.tf_buffer.lookup_transform_core(self.broadcast_parent_frame_name, 'camera_{}_link'.format(camera_id), rospy.Time(0))
                        tf_i = {}
                        tf_i['frame_id'] = self.broadcast_parent_frame_name
                        tf_i['child_id'] = 'camera_{}_link'.format(camera_id)
                        tf_i['x'] = ltr.transform.translation.x
                        tf_i['y'] = ltr.transform.translation.y
                        tf_i['z'] = ltr.transform.translation.z
                        tf_i['qx'] = ltr.transform.rotation.x
                        tf_i['qy'] = ltr.transform.rotation.y
                        tf_i['qz'] = ltr.transform.rotation.z
                        tf_i['qw'] = ltr.transform.rotation.w
                        # Add tf to measurements:
                        if camera_id in self.tfs_measures:
                            self.tfs_measures[camera_id].append(tf_i)
                        else:
                            self.tfs_measures[camera_id] = [tf_i]

                        # Add camera_frame-to-calibration_frame and broadcast_frame-to-calibration_frame for aggregation
                        cam_2_cf_tr_raw_i = self.tf_buffer.lookup_transform_core('camera_{}_link'.format(camera_id), self.calibration_frame_name, rospy.Time(0))
                        bf_2_cf_tr_raw_i = self.tf_buffer.lookup_transform_core(self.broadcast_parent_frame_name, self.calibration_frame_name, rospy.Time(0))
                        cam_2_cf_tr_i = self._unpack_tf_transformation(cam_2_cf_tr_raw_i)
                        bf_2_cf_tr_i = self._unpack_tf_transformation(bf_2_cf_tr_raw_i)
                        if camera_id in self.calibration_buffer:
                            self.calibration_buffer[camera_id]['cam2cf'].append(cam_2_cf_tr_i) # camera to calibration_frame
                            self.calibration_buffer[camera_id]['bf2cf'].append(bf_2_cf_tr_i) # broadcast_frame to calibration_frame
                        else:
                            self.calibration_buffer[camera_id] = {
                                'cam2cf': [cam_2_cf_tr_i], # camera to calibration_frame
                                'bf2cf':  [bf_2_cf_tr_i] # broadcast_frame to calibration_frame
                            }

        finally:
            self.tf_buffer.clear()
            self.lock.release()

    def _get_transform_from_detection(self, detection, tag_id=None):
        """
        Transform the detection message into a TransformStamped
        :param detection:
        :param tag_id:
        :return:
        """
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

    def broadcast_tfs(self):
        self.broadcast_thread.start()

    def _broadcast_tfs_callback(self):
        """
        Given all the measurements on the calibration buffer adquired by calling take_measurement,
        aggregate them and broadcast the resulting tf.
        :return:
        """
        self._compute_tfs() # from the stored measures, compute the final tf (do some average)
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            try:
                with self.lock:
                    for camera_id, tf_i in self.tfs.items():
                        if self.only_camera_ids is None:
                            self._broadcast_tf(tf_i)
                        elif camera_id in self.only_camera_ids:
                            self._broadcast_tf(tf_i)
                        else:
                            # we do not want to broadcast that camera
                            pass
            except:
                continue
            with self.lock:
                if not self.is_alive:
                    return
            rate.sleep()

    def _compute_tfs(self):
        # Compute the tf for each camera
        for camera_id, measurements in self.tfs_measures.items():
            if len(measurements) >= 3:
                # Aggregate them:
                t, R = find_best_transform(np.array(self.calibration_buffer[camera_id]['bf2cf']), np.array(self.calibration_buffer[camera_id]['cam2cf'])) # we obtain the tf from broadcast_frame to camera_frame
                R_ext = np.eye(4)
                R_ext[:3,:3] = R
                q_i = tr.quaternion_from_matrix(R_ext)
                tf_measure_i = {
                    'frame_id': self.broadcast_parent_frame_name,
                    'child_id': 'camera_{}_link'.format(camera_id),
                    'x': t[0],
                    'y': t[1],
                    'z': t[2],
                    'qx': q_i[0],
                    'qy': q_i[1],
                    'qz': q_i[2],
                    'qw': q_i[3],
                }
            else:
                print('WARNING: Camera {} only has {} measurements, not enough to aggregate them, so we broadcast a point estimator'.format(camera_id, len(measurements)))
                # We do not have enough measurements to do an aggregation estimation, so broadcast the first one
                tf_measure_i = measurements[0]
            self.tfs[camera_id] = tf_measure_i

    def _broadcast_tf(self, tf_i):
        """
        Pack and broadcast a transform dictionary
        :param tf_i: <dict> containing all tf parameters
        """
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = tf_i['frame_id']
        t.child_frame_id = tf_i['child_id']
        t.transform.translation.x = tf_i['x']
        t.transform.translation.y = tf_i['y']
        t.transform.translation.z = tf_i['z']
        t.transform.rotation.x = tf_i['qx']
        t.transform.rotation.y = tf_i['qy']
        t.transform.rotation.z = tf_i['qz']
        t.transform.rotation.w = tf_i['qw']
        self.tf_broadcaster.sendTransform(t)

    def _unpack_tf_transformation(self, tf_tr):
        unpacked_tf = [tf_tr.transform.translation.x,
                       tf_tr.transform.translation.y,
                       tf_tr.transform.translation.z,
                       tf_tr.transform.rotation.x,
                       tf_tr.transform.rotation.y,
                       tf_tr.transform.rotation.z,
                       tf_tr.transform.rotation.w]
        return unpacked_tf

