import pandas as pd
from collections import defaultdict

from mik_ros_utils.ros_utils.ros_msg_functions import pack_message_as_dict
import mik_tools.data_utils.data_path_tools as data_path_tools
from mik_tools.recording_utils.recording_utils import save_wrenches


def pack_msgs_to_df(msg_list):
    msg_dict = defaultdict(list)
    for i, msg_i in enumerate(msg_list):
        msg_dict_i = pack_message_as_dict(msg_i)
        for k, v in msg_dict_i.items():
            msg_dict[k].append(v)
    wrenches_df = pd.DataFrame(msg_dict)
    return wrenches_df


def record_wrenches(wrenches, save_path, scene_name, fc, wrench_name=None):
    wrenched_df = pack_msgs_to_df(wrenches)
    full_path = data_path_tools.get_wrench_path(save_path=save_path, scene_name=scene_name, fc=fc, wrench_name=wrench_name)
    save_path, filename = data_path_tools.split_full_path(full_path)
    filename_only, extension = data_path_tools.split_filename(filename)
    save_wrenches(wrenched_df, filename=filename_only, save_path=save_path)