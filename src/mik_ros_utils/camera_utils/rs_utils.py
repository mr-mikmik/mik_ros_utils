import rospy
import roslaunch
import time
import argparse


def launch_rs_cameras(camera_names_and_ids:dict, enable_pointcloud=False, enable_infra=False, align_depth=False, rate=15, img_width=640, img_height=480, not_apriltags=False, tags_config_package_name='apriltag_ros'):
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
                      # 'enable_infra:=true', # Enable the infrared streams
                      'enable_infra1:={}'.format(enable_infra), # Enable the infrared streams
                      'enable_infra2:={}'.format(enable_infra), # Enable the infrared streams
                      # 'infra_rgb:=true'
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


    if not not_apriltags:
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
                           'params_package_name:={}'.format(tags_config_package_name),
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