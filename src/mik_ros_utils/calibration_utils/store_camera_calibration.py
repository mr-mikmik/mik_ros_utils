import os

from mik_ros_utils.aux.conf_utils import camera_saved_calibrations_path, get_package_camera_saved_calibrations_path
from mik_ros_utils.ros_utils.tf_utils.tf_utils import save_tfs, load_tfs
from mik_ros_utils.camera_utils.camera_utils import list_camera_tf_frames


def save_camera_calibration_tfs(camera_ids=None, filename=None, parent_frame='med_base', tf_listener=None, verbose=True, package=None):
    if filename is None:
        filename = 'calibration_scene'
    if package is None:# default filename
        tf_save_path = camera_saved_calibrations_path
    else:
        tf_save_path = get_package_camera_saved_calibrations_path(package)
    if camera_ids is None:
        tf_frames = list_camera_tf_frames()
    else:
        tf_frames = ['camera_{}_link'.format(id) for id in camera_ids]
    save_tfs(child_names=tf_frames, parent_names=parent_frame, save_path=tf_save_path, file_name=filename,
             buffer=tf_listener.buffer, verbose=verbose)


def load_camera_calibration_tfs(filename=None, rate=1.0, verbose=True, package=None):
    if filename is None:
        filename = 'calibration_scene' # default filename
    if package is None:# default filename
        tf_save_path = camera_saved_calibrations_path
    else:
        tf_save_path = get_package_camera_saved_calibrations_path(package)
    filepath = os.path.join(tf_save_path, '{}.csv'.format(filename))
    load_tfs(filepath, rate=rate, verbose=verbose)
