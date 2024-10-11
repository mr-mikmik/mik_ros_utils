#!/usr/bin/env python
import rospy
import roslaunch
import time
import argparse
from mik_ros_utils.aux.load_confs import load_camera_names_and_ids

"""
Realsense D4 cameras (D4115, D435,...) only allow some (width, height, rate) configurations:
For Z16 bit encoding (default), we have
    * 1280 x 720 : 6, 15, 30
    * 848 x 480 : 6, 15, 30, 60, 90
    * 640 x 480 : 6, 15, 30, 60, 90
    * 640 x 360 : 6, 15, 30, 60, 90
Realsense D405:
For Z16 bit encoding (default), we have
    * 1280 x 720 : 5, 15, 30
    * 848 x 480 : 5, 15, 30, 60, 90
    * 640 x 360 : 5, 15, 30, 60, 90
    * 480 x 270 : 5, 15, 30, 60, 90
    * 424 x 240 : 5, 15, 30, 60, 90

For this to work, we need to specify all parameters.



For more info check: https://www.intelrealsense.com/wp-content/uploads/2022/11/Intel-RealSense-D400-Series-Datasheet-November-2022.pdf
"""

resolution_options = {
    '640x480': (640, 480),
    '848x480': (848, 480),
    '1280x720': (1280, 720),
    '640x360': (640, 369),
    '480x270': (480, 270),
    '424x240': (424, 240)
}

parser = argparse.ArgumentParser('Camera Launcher')
parser.add_argument('--camera_config_package_name', type=str, default='mik_ros_utils', help=f'Name of the package to where to load the configuration for the cameras (cameras.yaml). Default: mik_ros_utils')
parser.add_argument('--camera_config_file_name', type=str, default='cameras.yaml', help=f'Name of the file to load the cameras configuration. Default: cameras.yaml')
parser.add_argument('--tags_config_package_name', type=str, default='apriltag_ros', help=f'Name of the package to where to load the configuration for the tags. Default: apriltag_ros')
parser.add_argument('--resolution', type=str, default='640x480', choices=resolution_options.keys(), help=f'Resolution of the cameras. Options: {resolution_options.keys()}')
parser.add_argument('--rate', type=int, default=15, help=f'Rate of the cameras. Default: 15')
parser.add_argument('--not_apriltags', action='store_true', help='Do not launch the apriltags detection')
parser.add_argument('--enable_pointcloud', action='store_true', help='Enable the pointcloud publishing')
parser.add_argument('--align_depth', action='store_true', help='Align the depth image with the color image. Default: False')
args, _ = parser.parse_known_args()

# Unpack the arguments
rate = args.rate
enable_pointcloud = args.enable_pointcloud
align_depth = args.align_depth
camera_config_package_name = args.camera_config_package_name
camera_config_file_name = args.camera_config_file_name
tags_config_package_name = args.tags_config_package_name
image_size = resolution_options[args.resolution]
img_width = image_size[0]
img_height = image_size[1]


camera_names_and_ids = load_camera_names_and_ids(camera_config_package_name, file_name=camera_config_file_name) # dictionary of  {camera_name: camera_id}

# Initialize the node
rospy.init_node("launch_camera_node")


uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
uuids = []
roslaunch.configure_logging(uuid)
launch_files = []
launches = []

# Launch the cameras ------------------------------------------------------------------------------------------------
print('\n\nLaunching cameras with ids: {}\n\n'.format(camera_names_and_ids))
time.sleep(.5)
for i, (camera_name, camera_id) in enumerate(camera_names_and_ids.items()):
    # Process arguments
    cli_args_i = ['realsense2_camera',  # package
                  'rs_camera.launch',   # launch file
                  'camera:={}'.format(camera_name),
                  'serial_no:={}'.format(camera_id),
                  'enable_pointcloud:={}'.format(enable_pointcloud),
                  'fisheye_fps:={}'.format(rate),
                  'depth_fps:={}'.format(rate),
                  'depth_width:={}'.format(img_width),
                  'depth_height:={}'.format(img_height),
                  'color_width:={}'.format(img_width),
                  'color_height:={}'.format(img_height),
                  'infra_fps:={}'.format(rate),
                  'color_fps:={}'.format(rate),
                  'align_depth:={}'.format(align_depth),
                  'filters:=disparity,temporal,spatial,decimation' # Consider removing some of the filters
                  ]
    roslaunch_file_i = roslaunch.rlutil.resolve_launch_arguments(cli_args_i)[0]
    roslaunch_args_i = cli_args_i[2:]
    # Launch the file:
    uuid_i = roslaunch.rlutil.get_or_generate_uuid(None, False)
    uuids.append(uuid_i)
    launch_i = roslaunch.parent.ROSLaunchParent(uuid_i, [(roslaunch_file_i, roslaunch_args_i)])
    launches.append(launch_i)
    launch_i.start()
    rospy.sleep(2)


if not args.not_apriltags:
    rospy.sleep(5)
    # Lanuch the continuous detection for each camera:  ------------------------------------------------------------------------------------------------
    for i, (camera_name, camera_id) in enumerate(camera_names_and_ids.items()):
        cli_args_i = ['mik_ros_utils', # package
                       'apriltag_continuous_detection.launch', # launch file
                       f'camera_name:=/{camera_name}',
                       f'camera_frame:=/{camera_name}_color_optical_frame',
                       'image_topic:=/color/image_raw',
                       'node_namespace:=apriltag_ros_continuous_node_{}'.format(camera_id),
                       'publish_tf:=false', # Do not publish the tf for the tags to avoid multiple parent frames
                       'params_package_name:={}'.format(args.tags_config_package_name),
                       ]
        roslaunch_file_i = roslaunch.rlutil.resolve_launch_arguments(cli_args_i)[0]
        roslaunch_args_i = cli_args_i[2:]
        # Launch the file:
        uuid_i = roslaunch.rlutil.get_or_generate_uuid(None, False)
        uuids.append(uuid_i)
        launch_i = roslaunch.parent.ROSLaunchParent(uuid_i, [(roslaunch_file_i, roslaunch_args_i)])
        launches.append(launch_i)
        launch_i.start()
        rospy.sleep(2)

rospy.sleep(3)

try:
    for launch in launches:
        launch.spin()
finally:
    for launch in launches:
        launch.shutdown()
