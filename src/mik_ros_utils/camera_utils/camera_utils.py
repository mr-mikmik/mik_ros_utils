from mmint_camera_utils.tf_utils.tf_utils import get_all_frame_names


def list_camera_tf_frames():
    # return all tfs frames belonging to cameras (scene cameras)
    all_tfs = get_all_frame_names()
    camera_tfs = [tf_i for tf_i in all_tfs if (('camera_' in tf_i) and ('_link' in tf_i))]
    return camera_tfs
