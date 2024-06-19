#! /usr/bin/env python
import rospy
import sys
import argparse
import tf2_ros as tf

from mik_ros_utils.calibration_utils.store_camera_calibration import load_camera_calibration_tfs, save_camera_calibration_tfs



if __name__ == '__main__':

    rospy.init_node('load_save_camera_tfs')

    parser = argparse.ArgumentParser('Camera TFS')
    parser.add_argument('--filename', type=str, default=None, help='name of the tf configuration')
    parser.add_argument('--parent_frame', type=str, default='med_base', help='name of the parent framet to save the configuration')
    parser.add_argument('--rate', type=float, default=None, help='TF publishing rate (if None, then are loaded as static)')
    parser.add_argument('--save', action='store_true', help='if to save the tfs to a file')
    parser.add_argument('--load', action='store_true', help='if to load the tfs from a file')
    parser.add_argument('--package', type=str, default=None, help='name of the package where the config file is stored')
    parser.add_argument('--camera_ids', type=int, nargs='+', default=None,
                        help='scene name for the data. For organization purposes')

    args = parser.parse_args()
    if args.save:
        buffer = tf.Buffer()
        print('Listening to the TFs to be saved during 5s')
        rospy.sleep(5.0)
        tf_listener = tf.TransformListener(buffer)
        save_camera_calibration_tfs(camera_ids=args.camera_ids, filename=args.filename, tf_listener=tf_listener, parent_frame=args.parent_frame, package=args.package)
    if args.load:
        load_camera_calibration_tfs(filename=args.filename, rate=args.rate, package=args.package)
    if not args.save and not args.load:
        print('Promgram finished without doing anything. Select --save or --load.')


