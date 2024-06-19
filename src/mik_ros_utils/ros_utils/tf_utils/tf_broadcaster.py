import rospy
import threading
import abc
import copy
import tf2_ros as tf
from geometry_msgs.msg import TransformStamped


class TFBroadcaster(abc.ABC):
    def __init__(self, rate=5.0):
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.rate = rospy.Rate(rate)
        self._tfs = [] # list of TrasformStamped messages
        self.alive = True
        self.lock = threading.Lock()
        self.broadcaster_thread = threading.Thread(target=self._broadcast_loop)
        self.broadcaster_thread.start()

    def _broadcast_loop(self):
        while not rospy.is_shutdown():
            self._precall_loop()
            tfs = self.tfs
            if tfs is not None:
                for tf_i in tfs:
                    tf_i.header.stamp = rospy.Time.now() # Update the time
                    self.tf_broadcaster.sendTransform(tf_i)
                self.rate.sleep()
            with self.lock:
                if not self.alive:
                    return

    def _precall_loop(self):
        pass

    @property
    def tfs(self):
        with self.lock:
            return self._tfs

    @tfs.setter
    def tfs(self, value):
        with self.lock:
            self._tfs = value

    def finish(self):
        with self.lock:
            self.alive = False
        self.broadcaster_thread.join()

    def add_tf(self, tf):
        """
        :param tf: TransformStamped message
        """
        with self.lock:
            self._tfs.append(tf)

    def add_tf_from_dict(self, tf_dict):
        tf_i = self._pack_tf(tf_dict)
        with self.lock:
            self._tfs.append(tf_i)

    def add_tf_from_pose(self, pose, frame_id='frame_name', ref_id='world'):
        tf_i = self._pack_tf_from_pose(pose, frame_id, ref_id)
        with self.lock:
            self._tfs.append(tf_i)

    def _pack_tf_from_pose(self, pose, frame_id, ref_id):
        tf_i = TransformStamped()
        tf_i.header.stamp = rospy.Time.now()
        tf_i.header.frame_id = ref_id
        tf_i.child_frame_id = frame_id
        tf_i.transform.translation.x = pose[0]
        tf_i.transform.translation.y = pose[1]
        tf_i.transform.translation.z = pose[2]
        tf_i.transform.rotation.x = pose[3]
        tf_i.transform.rotation.y = pose[4]
        tf_i.transform.rotation.z = pose[5]
        tf_i.transform.rotation.w = pose[6]
        return tf_i

    def _pack_tf(self, tf_dict):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = tf_dict['frame_id']
        t.child_frame_id = tf_dict['child_id']
        t.transform.translation.x = tf_dict['x']
        t.transform.translation.y = tf_dict['y']
        t.transform.translation.z = tf_dict['z']
        t.transform.rotation.x = tf_dict['qx']
        t.transform.rotation.y = tf_dict['qy']
        t.transform.rotation.z = tf_dict['qz']
        t.transform.rotation.w = tf_dict['qw']
        return t
