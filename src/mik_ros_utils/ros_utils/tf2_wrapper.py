# ----
# Adapted from: https://github.com/UM-ARM-Lab/arc_utilities/blob/master/src/arc_utilities/tf2wrapper.py
# ----

import numpy as np
import rospy
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from mik_tools import matrix_to_pose, pose_to_matrix


class TF2Wrapper:
    def __init__(self, buffer=None, listener=None):
        if buffer is None:
            self.tf_buffer = tf2_ros.Buffer()
        else:
            self.tf_buffer = buffer
        if listener is None:
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        else:
            self.tf_listener = listener
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.static_tfs = []

    def get_transform(self,
                      parent,
                      child,
                      verbose=True,
                      spin_delay=rospy.Duration(secs=0, nsecs=100 * 1000 * 1000),
                      time=rospy.Time()):
        """
        Waits for a transform to become available. Blocks until a transform is available or an exception is raised.

        :param parent: frame name for the parent (see below)
        :param child: frame name for the child (see below)
        :param verbose: If verbose is True, then output messages are sent on rosinfo as the function waits for
                        a transform, otherwise on rosdebug
        :param spin_delay: How long to wait between output messages
        :param time: The timepoint to request a transform at. Defaults to "latest available".
        :return: A matrix representation of the transform (numpy). Returns None if a tf2 exception is raised.

        The notation here follows the following convention:

        p_measured_in_parent = returned_transform * p_measured_in_child
        p_measured_in_target = returned_transform * p_measured_in_source
        """

        try:
            transform = self.get_transform_msg(parent=parent, child=child, verbose=verbose, spin_delay=spin_delay,
                                               time=time)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("No transform available: %s to %s", parent, child)
            return None

        # build the matrix
        transformation_matrix = self._transform_to_matrix(transform.transform.translation, transform.transform.rotation)
        return transformation_matrix

    def get_transform_msg(self,
                          parent,
                          child,
                          verbose=True,
                          spin_delay=rospy.Duration(secs=0, nsecs=100 * 1000 * 1000),
                          time=rospy.Time()):
        while not self.tf_buffer.can_transform(target_frame=parent, source_frame=child,
                                               time=time, timeout=spin_delay):
            if rospy.is_shutdown():
                raise KeyboardInterrupt("ROS has shutdown")
            if verbose:
                rospy.loginfo("Waiting for TF frames %s and %s", parent, child)
            else:
                rospy.logdebug("Waiting for TF frames %s and %s", parent, child)
        transform = self.tf_buffer.lookup_transform(target_frame=parent, source_frame=child, time=time)
        return transform

    def send_transform_matrix(self, transform, parent, child, is_static=False, time=None):
        """
        :param parent: frame name for the parent (see below)
        :param child: frame name for the child (see below)
        :param transform: A matrix representation of the transform (presumably numpy)
        :param time: The timestamp for the transform, defaults to now()

        The notation here follows the following convention:

        p_measured_in_parent = transform * p_measured_in_child
        p_measured_in_target = transform * p_measured_in_source
        """
        [translation, quaternion] = self._matrix_to_transform(transform)
        self.send_transform(translation, quaternion, parent, child, is_static, time)

    def send_transform_from_pose_msg(self, pose: Pose, parent, child, is_static=False, time=None):
        if time is None:
            time = rospy.Time.now()

        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = time
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation = pose.orientation

        if is_static:
            self.static_tfs.append(t)
            self.static_tf_broadcaster.sendTransform(self.static_tfs)  # send all at once.
        else:
            self.tf_broadcaster.sendTransform(t)

    def send_transform(self, translation, quaternion, parent, child, is_static=False, time=None):
        """
        :param parent: frame name for the parent (see below)
        :param child: frame name for the child (see below)
        :param translation: [x, y, z]
        :param quaternion: [x, y, z, w]
        :param time: The timestamp for the transform, defaults to now()

        The notation here follows the following convention:

        p_measured_in_parent = transform * p_measured_in_child
        p_measured_in_target = transform * p_measured_in_source
        """
        if time is None:
            time = rospy.Time.now()

        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = time
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]
        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]

        if is_static:
            self.static_tfs.append(t)
            self.static_tf_broadcaster.sendTransform(self.static_tfs) # send all at once.
        else:
            self.tf_broadcaster.sendTransform(t)

    def transform_to_frame(self, object_stamped, target_frame, timeout=rospy.Duration(0), new_type=None):
        """
        Transforms many "stamped" data types between frames. The specific package for the type of stamped object needs
         to be imported prior to use. Examples are tf2_geometry_msgs and tf2_py.
        If new_type is not None, the type specified must have a valid conversion from the input type, else the function
         will raise an exception.
        Example usage:
            from arc_utilities import ros_helpers
            import tf2_geometry_msgs
            ...
            self.tf2 = ros_helpers.TF2Wrapper()
            ...
            p_in_native_frame = PointStamped()
            p_in_native_frame.header.stamp = rospy.Time.now() # This will likely cause an extrapolation warning/exception without a timeout set
            p_in_native_frame.header.frame_id = frame_point_is_measured_in
            p_in_native_frame.point = ...
            p_in_world = self.tf2.transform_to_frame(object_stamped=p_in_native_frame, target_frame=world_frame_name)

        :param object_stamped: The timestamped object the transform.
        :param target_frame: Name of the frame to transform the input into.
        :param timeout: (Optional) Time to wait for the target frame to become available.
        :param new_type: (Optional) Type to convert the object to.
        :return: The transformed, timestamped output, possibly converted to a new type.
        """
        return self.tf_buffer.transform(object_stamped, target_frame, timeout, new_type)

    def _transform_to_matrix(self, translation, quaternion):
        pos = np.array([translation.x, translation.y, translation.z])
        quat = np.array([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        pose = np.concatenate([pos, quat]) # (7,)
        matrix = pose_to_matrix(pose)
        return matrix

    def _matrix_to_transform(self, matrix):
        pose = matrix_to_pose(matrix)
        pos = pose[:3] # (3,)
        quat = pose[3:] # (4,)
        transform = (pos, quat)
        return transform


