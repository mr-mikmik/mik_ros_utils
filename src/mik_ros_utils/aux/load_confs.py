import copy
import numpy as np
import yaml
import sys
import os
import pathlib
from .package_utils import package_path, find_package_path, package_name


def load_robot_configurations():
    robot_configs_path = os.path.join(package_path, 'config', 'robot_confs.yaml')
    robot_configs = _load_config_from_path(robot_configs_path)
    return robot_configs


def load_tool_grasp_params():
    bubble_reconstruction_configs_path = os.path.join(package_path, 'config', 'tool_grasp_params.yaml')
    bubble_reconstruction_configs = _load_config_from_path(bubble_reconstruction_configs_path)
    return bubble_reconstruction_configs


def get_load_object_params_fn(package_name='mik_ros_utils', file_name='object_params.yaml'):
    package_path = find_package_path(package_name)
    robot_configs_path = os.path.join(package_path, 'config', file_name)

    def load_object_params():
        obj_params = _load_config_from_path(robot_configs_path)
        return obj_params

    return load_object_params

def get_get_available_object_ids_fn(package_name='mik_ros_utils', file_name='object_params.yaml'):
    load_object_params_fn = get_load_object_params_fn(package_name, file_name=file_name)

    def get_available_object_ids():
        object_params = load_object_params_fn()
        object_ids = list(object_params.keys())
        return object_ids

    return get_available_object_ids


load_object_params = get_get_available_object_ids_fn(package_name)
get_available_object_ids = get_get_available_object_ids_fn(package_name)


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
