import yaml
import os
import numpy as np
import pandas as pd

from mik_ros_utils.aux.package_utils import package_path, package_name, config_path, find_package_path

camera_calibration_joint_sequence_conf_path = os.path.join(package_path, 'config', 'camera_calibration_joint_sequences')
camera_saved_calibrations_path = os.path.join(package_path, 'config', 'camera_saved_calibrations')


def get_package_camera_calibration_joint_sequence_conf_path(package_name):
    package_path = find_package_path(package_name)
    camera_calibration_joint_sequence_conf_path = os.path.join(package_path, 'config', 'camera_calibration_joint_sequences')
    return camera_calibration_joint_sequence_conf_path


def get_package_camera_saved_calibrations_path(package_name):
    package_path = find_package_path(package_name)
    camera_saved_calibrations_path = os.path.join(package_path, 'config', 'camera_saved_calibrations')
    return camera_saved_calibrations_path


def load_joint_sequence(name, package=None):
    camera_calibration_joint_sequence_conf_path = get_package_camera_calibration_joint_sequence_conf_path(package)
    joint_sequence_path_i = os.path.join(camera_calibration_joint_sequence_conf_path, '{}.csv'.format(name))
    joint_df = pd.read_csv(joint_sequence_path_i)
    joints = [joints.values for i, joints in joint_df.iterrows()]
    return joints


def save_joint_sequence(joint_sequence, name, package=None):
    # joint_sequence is a collection of 7-dim arrays
    # We save it as a dict where lines are configs and we have a column for each joint.
    num_joints = len(joint_sequence[0])
    if package is None:
        package = package_name
    camera_calibration_joint_sequence_conf_path = get_package_camera_calibration_joint_sequence_conf_path(package)
    joint_sequence_path_i = os.path.join(camera_calibration_joint_sequence_conf_path, '{}.csv'.format(name))
    col_names = ['Joint_{}'.format(i) for i in range(num_joints)]
    joint_df = pd.DataFrame(np.asarray(joint_sequence), columns=col_names)
    joint_df.to_csv(joint_sequence_path_i, index=False)


def get_camera_calibration_sequence_names(package=None):
    if package is None:
        package = package_name
    camera_calibration_joint_sequence_conf_path = get_package_camera_calibration_joint_sequence_conf_path(package)
    if not os.path.exists(camera_calibration_joint_sequence_conf_path):
        print(f"{camera_calibration_joint_sequence_conf_path} doesn't exist. Creating path.")
        os.mkdir(camera_calibration_joint_sequence_conf_path)
        
    all_files = [f for f in os.listdir(camera_calibration_joint_sequence_conf_path) if
                 os.path.isfile(os.path.join(camera_calibration_joint_sequence_conf_path, f))]
    joint_names = [f.split('.csv')[0] for f in all_files if '.csv' in f]
    return joint_names


def load_camera_calibration_joint_sequences(package=None):
    # get all joint sequences names:
    joint_names = get_camera_calibration_sequence_names(package=package)
    camera_calibration_joint_sequences = {}
    for joint_name in joint_names:
        joint_seq_i = load_joint_sequence(joint_name, package=package)
        camera_calibration_joint_sequences[joint_name] = joint_seq_i
    return camera_calibration_joint_sequences


def save_camera_calibration_joint_sequcences(data_dict, package=None):
    # save all
    for seq_name, joint_sequence in data_dict.items():
        save_joint_sequence(joint_sequence, seq_name, package=package)


def update_camera_calibration_joint_sequcences(update_dict, package=None):
    # save only the ones that do not exist
    current_seq_nams = get_camera_calibration_sequence_names(package=package)
    for seq_name, joint_sequence in update_dict.items():
        if seq_name not in current_seq_nams:
            save_joint_sequence(joint_sequence, seq_name, package=package)


def _load_config_from_path(path):
    config = None
    with open(path) as f:
        config = yaml.load(f, Loader=yaml.SafeLoader)
    return config


def _save_config_to_path(data_dict, path):
    with open(path) as f:
        yaml.safe_dump(data_dict, f)



