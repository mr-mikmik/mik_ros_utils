#!/usr/bin/env python

import rospy
import os
import numpy as np
import tf2_ros as tf
import pandas as pd
import time as pytime

from geometry_msgs.msg import TransformStamped
from mik_ros_utils.ros_utils.tf_utils.tf_broadcaster import TFBroadcaster


def get_all_frame_names(tf_listener=None, verbose=True):
    # returns all tfs
    if tf_listener is None:
        buffer = tf.Buffer()
        rospy.sleep(5.0)
        if verbose:
            print('Listening to the TFs to be saved during 5s')
        tf_listener = tf.TransformListener(buffer)
    all_tfs = tf_listener.buffer._getFrameStrings()
    return all_tfs


def get_tf(child_name, parent_name, buffer=None, verbose=False, spin_delay=rospy.Duration(secs=1, nsecs=100 * 1000 * 1000),
                      time=rospy.Time(0), tf_listener=None):
    """
    Return tfs as a dataframe
    :param child_names:
    :param parent_names:
    :param same_path:
    :param file_name:
    :return:
    """
    if buffer is None:
        if tf_listener is None:
            buffer = tf.Buffer()
            tf_listener = tf.TransformListener(buffer)
            rospy.sleep(5.0)
            if verbose:
                print('Listening to the TFs to be saved during 5s')
        else:
            buffer = tf_listener.buffer

    if buffer._frameExists(child_name):
        if buffer._frameExists(parent_name):
            for i in range(10):
                try:
                    tf_i = get_transform_msg(buffer, parent=parent_name, child=child_name, verbose=verbose, spin_delay=spin_delay, time=time)
                    # tf_i = buffer.lookup_transform_full(target_frame=parent_name, source_frame=child_name, source_time=rospy.Time(0), target_time=rospy.Time(), fixed_frame=parent_name)
                    # tf_i = tf_listener.buffer.lookup_transform(parent_name, child_name, rospy.Time())
                    tf_array = np.asarray([tf_i.transform.translation.x,
                                                             tf_i.transform.translation.y,
                                                             tf_i.transform.translation.z,
                                                             tf_i.transform.rotation.x,
                                                             tf_i.transform.rotation.y,
                                                             tf_i.transform.rotation.z,
                                                             tf_i.transform.rotation.w
                                                             ])
                    return tf_array
                except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    rospy.logwarn('TF between {} and {} raised exception {}'.format(child_name, parent_name, e))
                    import pdb; pdb.set_trace()
                    pass
        else:
            if verbose:
                print('WARNING: parent frame {} does not exist'.format(parent_name))
    else:
        if verbose:
            print('WARNING: child frame {} does not exist'.format(child_name))
    return None


def get_tfs(child_names, parent_names, buffer=None, verbose=True, spin_delay=rospy.Duration(secs=1, nsecs=100 * 1000 * 1000),
                      time=rospy.Time(0), tf_listener=None):
    """
    Return tfs as a dataframe
    :param child_names:
    :param parent_names:
    :param same_path:
    :param file_name:
    :return:
    """
    if type(child_names) is str:
        child_names = [child_names]
    if type(parent_names) is str:
        # we only provide one parent name, so all tfs will be with respect to this frame
        parent_names = [parent_names]*len(child_names)
    if buffer is None:
        if tf_listener is None:
            buffer = tf.Buffer()
            tf_listener = tf.TransformListener(buffer)
        else:
            buffer = tf_listener.buffer
        rospy.sleep(5.0)
        if verbose:
            print('Listening to the TFs to be saved during 5s')

    child_names = np.unique(child_names)

    tfs_to_save = []
    for i, child_name in enumerate(child_names):
        if buffer._frameExists(child_name):
            if parent_names is not None:
                parent_name = parent_names[i]
            else:
                # if parent_names is None, we will record them with respect to their direct parent in tf tree
                parent_name = buffer.getParent(child_name)
            if buffer._frameExists(parent_name):
                for i in range(10):
                    try:
                        tf_i = get_transform_msg(buffer, parent=parent_name, child=child_name, verbose=verbose, spin_delay=spin_delay, time=time)
                        # tf_i = buffer.lookup_transform_full(target_frame=parent_name, source_frame=child_name, source_time=rospy.Time(), target_time=rospy.Time(), fixed_frame=parent_name)
                        # tf_i = buffer.lookup_transform_full(target_frame=parent_name, source_frame=child_name, source_time=rospy.Time(0), target_time=rospy.Time(), fixed_frame=parent_name)
                        # tf_i = tf_listener.buffer.lookup_transform(parent_name, child_name, rospy.Time())
                        tf_line_i = [parent_name, child_name] + [tf_i.transform.translation.x,
                                                                 tf_i.transform.translation.y,
                                                                 tf_i.transform.translation.z,
                                                                 tf_i.transform.rotation.x,
                                                                 tf_i.transform.rotation.y,
                                                                 tf_i.transform.rotation.z,
                                                                 tf_i.transform.rotation.w
                                                                 ]
                        tfs_to_save.append(tf_line_i)
                        break
                    except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                        rospy.logwarn('TF between {} and {} raised exception {}'.format(child_name, parent_name, e))
                        import pdb; pdb.set_trace()
                        pass
            else:
                if verbose:
                    print('WARNING: Frame {} not found'.format(parent_name))

        else:
            if verbose:
                print('WARNING: Frame {} not found'.format(child_name))
    # pack the tfs registered into a df
    df = pd.DataFrame(tfs_to_save, columns=['parent_frame', 'child_frame', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
    return df


def get_transform_msg(tf_buffer, parent, child, verbose=True, spin_delay=rospy.Duration(secs=0, nsecs=100 * 1000 * 1000),
                          time=rospy.Time(), max_time=10.0):
    start_time = pytime.time()
    while not tf_buffer.can_transform(target_frame=parent, source_frame=child,
                                           time=time, timeout=spin_delay):
        if rospy.is_shutdown():
            raise KeyboardInterrupt("ROS has shutdown")
        if verbose:
            rospy.loginfo("Waiting for TF frames %s and %s", parent, child)
        else:
            rospy.logdebug("Waiting for TF frames %s and %s", parent, child)
        if pytime.time() - start_time > max_time:
            rospy.logwarn("Waiting for TF frames %s and %s -- TIMEOUT", parent, child)
            break
    transform = tf_buffer.lookup_transform(target_frame=parent, source_frame=child, time=time)
    return transform


def save_tfs(child_names, parent_names, save_path, file_name='tfs', verbose=True, buffer=None):
    """
    Save tfs as a .csv file
    :param child_names:
    :param parent_names:
    :param same_path:
    :param file_name:
    :return:
    """
    df = get_tfs(child_names, parent_names, verbose=verbose, buffer=buffer)
    # save df
    if not os.path.exists(save_path):
        os.makedirs(save_path)
    file_path = os.path.join(save_path, '{}.csv'.format(file_name))
    df.to_csv(file_path)
    if verbose:
        rospy.loginfo('TF {} saved at {}'.format(child_names, file_path))


def record_tfs(tfs_df, save_path, scene_name, fc, file_name='tfs'):
    filename = '{}_{:06d}'.format(file_name, fc)
    save_path = os.path.join(save_path, scene_name, 'tfs')
    if not os.path.exists(save_path):
        os.makedirs(save_path)
    full_save_path = os.path.join(save_path, '{}.csv'.format(filename))
    tfs_df.to_csv(full_save_path, index=False)
    return full_save_path


def load_tfs(file_path, rate=None, verbose=True):
    """
    Load tfs from a .csv file and broacast them.
    :param file_path:
    :return:
    """
    df = pd.read_csv(file_path)
    # Read the transforms and pack them:

    # Pack the tfs:
    tfs = [] # List of TransformStamped
    for indx, line_i in df.iterrows():
        line_i['frame_id'] = line_i['parent_frame']
        line_i['child_id'] = line_i['child_frame']
        tf_i = pack_tf(line_i)
        tfs.append(tf_i)
    if verbose:
        rospy.loginfo('Loaded TFs {} from {}'.format([tf_i.child_frame_id for tf_i in tfs], file_path))
    # import pdb; pdb.set_trace()
    if rate is None:
        # Static publishers
        broadcasters = []
        # for i, tf_i in enumerate(tfs):
        #     broadcasters.append(tf.StaticTransformBroadcaster())
        #     broadcasters[-1].sendTransform(tf_i)
        #     print(i)
        br = tf.StaticTransformBroadcaster()
        br.sendTransform(tfs) # we send all t
        # NOTE: We can only have one StaticTransformBroadcaster per thread. We need to broadcast all static tfs at once.
    else:
        broadcaster_i = TFBroadcaster(rate=rate)
        broadcaster_i.tfs = tfs
    rospy.spin()


def pack_tf(tf_i):
    """
    Connvert a tf_dict to a TransformSTamped.
    :param tf_i:
    :return:
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
    return t


def unpack_tf(transform_stamped):
    tf_i = {}
    tf_i['frame_id'] = transform_stamped.header.frame_id
    tf_i['child_id'] = transform_stamped.child_frame_id
    tf_i['x'] = transform_stamped.transform.translation.x
    tf_i['y'] = transform_stamped.transform.translation.y
    tf_i['z'] = transform_stamped.transform.translation.z
    tf_i['qx'] = transform_stamped.transform.rotation.x
    tf_i['qy'] = transform_stamped.transform.rotation.y
    tf_i['qz'] = transform_stamped.transform.rotation.z
    tf_i['qw'] = transform_stamped.transform.rotation.w
    return tf_i
