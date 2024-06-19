import copy
import numpy as np
import yaml
import sys
import os
import pathlib
from mik_tools import tr
from mik_ros_utils import package_path


def load_robot_configurations():
    robot_configs_path = os.path.join(package_path, 'config', 'robot_confs.yaml')
    robot_configs = _load_config_from_path(robot_configs_path)
    return robot_configs


def load_tool_grasp_params():
    bubble_reconstruction_configs_path = os.path.join(package_path, 'config', 'tool_grasp_params.yaml')
    bubble_reconstruction_configs = _load_config_from_path(bubble_reconstruction_configs_path)
    return bubble_reconstruction_configs


def _load_config_from_path(path):
    config = None
    with open(path) as f:
        config = yaml.load(f, Loader=yaml.SafeLoader)
    return config


