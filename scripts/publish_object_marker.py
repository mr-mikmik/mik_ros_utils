#! /usr/bin/env python
import rospy
import numpy as np
import argparse
from mik_ros_utils.ros_utils import TFBroadcaster, TF2Wrapper
from mik_ros_utils.aux.load_confs import get_available_object_ids, load_object_params, get_get_available_object_ids_fn
from mik_ros_utils.visualization import GeometryApriltagMarkerPublisher

colors = {
    'orange': np.array([252, 157, 3, 255]) / 255.0,  # orange
    'turquoise': np.array([52, 235, 152, 255.]) / 255.0,  # turquoise
    'blue': np.array([0, 0, 255, 255]) / 255.0,  # blue
    'green': np.array([0, 255, 0, 255]) / 255.0,  # green
    'red': np.array([255, 0, 0, 255]) / 255.0,  # red
    'purple': np.array([128, 0, 128, 255]) / 255.0  # purple
    # Add more colors as needed
}

"""
This script publishes a marker for a specified object using ROS.
It uses the `GeometryApriltagMarkerPublisher` to create a marker for the object based on its ID and camera ID.
The marker's color can be specified from a predefined set of colors
and the marker is published while the object is visible in the camera's frame.
"""

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--package_name', type=str, default='mik_ros_utils', help='name of the package to load object parameters from')
    parser.add_argument('--obj_params_file_name', type=str, default='object_params.yaml', help='name of the object parameters file to load from the package')
    args, _ = parser.parse_known_args()
    available_object_ids = get_get_available_object_ids_fn(args.package_name, args.obj_params_file_name)()
    parser.add_argument('--object_id', type=str, default=None, choices=available_object_ids,
                        help='object id to publish')
    parser.add_argument('--camera_id', type=str, default='camera_1', help='camera id to use')
    parser.add_argument('--marker_name', type=str, default='object_name', help='name of the marker to be published')
    parser.add_argument('--color', type=str, default='orange', choices=list(colors.keys()), help='color of the marker. Default: orange.')
    args, _ = parser.parse_known_args()

    object_id = args.object_id
    camera_id = args.camera_id

    rospy.init_node(f'obj_marker_publisher_{object_id}')

    tf2_listener = TF2Wrapper()
    tf_broadcaster = TFBroadcaster()

    marker_color = colors[args.color]  # Use the specified color from the dictionary

    mps = []
    mp_i = GeometryApriltagMarkerPublisher(object_id=object_id,
                                           topic_name=args.marker_name,
                                           camera_id=camera_id,
                                           package_name=args.package_name,
                                           obj_params_file_name=args.obj_params_file_name,
                                           )
    mp_i.marker_color = marker_color
    mp_i.set_tag_reference_mode()
    mps.append(mp_i)

    for mp_i in mps:
        mp_i.show = True

    # publish the object frame tf:
    object_params = mp_i.object_params
    tag_id = object_params['tag_id']
    tag_pose_of = np.asarray(object_params['tag_pose_of'])
    frame_id = object_id

    tf_broadcaster.add_tf_from_pose(tag_pose_of, frame_id=frame_id, ref_id=f'tag_{tag_id}_{camera_id}')

    # publish the object frame while the tag is visible, if not, hide it
    while not rospy.is_shutdown():
        frames = tf2_listener.tf_buffer._getFrameStrings()
        for mp_i in mps:
            tag_i = mp_i.get_tag_frame_id()
            if tf2_listener.tf_buffer._frameExists(tag_i) and tf2_listener.tf_buffer.can_transform(frame_id, tag_i,
                                                                                                   time=rospy.Time(0)):
                mp_i.show = True
            else:
                mp_i.show = False
    rospy.spin()
