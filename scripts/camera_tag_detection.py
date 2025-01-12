#!/usr/bin/env python
import argparse
from mik_ros_utils.apriltag_utils.camera_apriltag_tf_detection import CameraApriltagTFDetection

if __name__ == '__main__':
    """
    This class listens to topics /tag_detections_i contained the detected apriltags.
    Then, it broadcasts the position of the tags with respect to the common frame, which we assume fixed (camera frame or other frame).
    """

    parser = argparse.ArgumentParser('ApriltagTFDetection')
    parser.add_argument('--continuous', action='store_true', help='option to launch continous detection. If continuous, ti will keep updating the position of the cameras with respect to the detected tag')
    parser.add_argument('--no_camera_name_to_tag', action='store_true', help='Avoids to add the camera name to the tag name. Example: tag_1_{camera_name} -> tag_1')
    parser.add_argument('--parent_frame', type=str, default=None, help='option to launch continous detection.')
    parser.add_argument('--rate', type=float, default=10.0, help='rate to publish the tags.')
    parser.add_argument('--tag_ids', type=int, nargs='+', default=(1, 2, 3), help='tags to be detected')
    parser.add_argument('--tag_ids_names', type=str, nargs='+', default=None, help='spacial names to the tags to be detacted')
    parser.add_argument('--time_on_buffer', type=float, default=1.0, help='time in seconds to keep a tag in the buffer before being cleaned')
    parser.add_argument('--clean_buffer', type=bool, default=True, help='if false, we will not clean the buffer, and detected tags will be published until the process is closed. '
                                                                          'If they are detected again, they will still be updated')

    args, _ = parser.parse_known_args()
    tag_ids = args.tag_ids
    _tag_ids_names = args.tag_ids_names
    if _tag_ids_names is None:
        tag_ids_names = {} # No special names
    else:
        tag_ids_names = dict(zip(tag_ids, _tag_ids_names))
        # remove Nones:
        keys_to_be_removed = []
        for k, v in tag_ids_names.items():
            if v == 'None':
                keys_to_be_removed.append(k)
        for k in keys_to_be_removed:
            tag_ids_names.pop(k)

    camera_apriltag_fusion = CameraApriltagTFDetection(continuous=args.continuous,
                                                       tag_ids=tag_ids,
                                                       tag_ids_names=tag_ids_names,
                                                       parent_frame_name=args.parent_frame,
                                                       rate=args.rate,
                                                       time_on_buffer=args.time_on_buffer,
                                                       add_camera_name_to_tag=not args.no_camera_name_to_tag,
                                                       clean_buffer=args.clean_buffer)
