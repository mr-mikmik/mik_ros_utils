import copy
import numpy as np
import yaml
import sys
import os
import pathlib
from .package_utils import package_path, find_package_path


def load_robot_configurations():
    robot_configs_path = os.path.join(package_path, 'config', 'robot_confs.yaml')
    robot_configs = _load_config_from_path(robot_configs_path)
    return robot_configs


def load_tool_grasp_params():
    bubble_reconstruction_configs_path = os.path.join(package_path, 'config', 'tool_grasp_params.yaml')
    bubble_reconstruction_configs = _load_config_from_path(bubble_reconstruction_configs_path)
    return bubble_reconstruction_configs


def load_camera_config(package_name='mik_ros_utils', file_name='cameras.yaml'):
    package_path = find_package_path(package_name)
    camera_configs_path = os.path.join(package_path, 'config', file_name)
    camera_configs = _load_config_from_path(camera_configs_path)
    # camera_config is {'camera_name':'camera_id'} dictionary
    return camera_configs


def load_camera_names_and_ids(package_name='mik_ros_utils', file_name='cameras.yaml'):
    camera_configs = load_camera_config(package_name, file_name=file_name)
    camera_names_and_ids = camera_configs
    return camera_names_and_ids

def _load_config_from_path(path):
    config = None
    with open(path) as f:
        config = yaml.load(f, Loader=yaml.SafeLoader)
    return config



if __name__ == '__main__':
    camera_config = load_camera_config()
    print(camera_config)
