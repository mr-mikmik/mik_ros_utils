#!/usr/bin/env python
import rospy
import roslaunch
import time
import argparse
from mik_ros_utils.aux.load_confs import load_camera_names_and_ids
from mik_ros_utils.camera_utils.rs_utils import launch_rs_cameras

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
parser.add_argument('--enable_infra', action='store_true', help='Enable the pointcloud publishing')
parser.add_argument('--align_depth', action='store_true', help='Align the depth image with the color image. Default: False')
parser.add_argument('--settings_filename', type=str, default='settings.yaml', help='Name of the settings file to use for the cameras. Default: settings.yaml')
parser.add_argument('--tags_filename', type=str, default='tags.yaml', help='Name of the tags file to use for the apriltags detection. Default: tags.yaml')
args, _ = parser.parse_known_args()

# Unpack the arguments
rate = args.rate
enable_pointcloud = args.enable_pointcloud
enable_infra = args.enable_infra
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
launch_rs_cameras(camera_names_and_ids, enable_pointcloud=enable_pointcloud, enable_infra=enable_infra, align_depth=align_depth, rate=rate, 
                  img_width=img_width, img_height=img_height, not_apriltags=args.not_apriltags, tags_config_package_name=args.tags_config_package_name, 
                  settings_filename=args.settings_filename, tags_filename=args.tags_filename)
