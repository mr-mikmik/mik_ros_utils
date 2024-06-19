import rospy
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
import numpy as np
import ros_numpy as rn
import cv2
import struct
import cv_bridge


def create_image_msg(img):
    img_msg = cv_bridge.CvBridge().cv2_to_imgmsg(img)
    return img_msg


def process_image_msg(img_msg):
    if type(img_msg) in [CompressedImage]:
        np_img = _process_compressed_img_msg(img_msg)
    else:
        encoding_dict = {
            'rgb8': np.uint8,
            '8UC3': np.uint8,
            '16FC1': np.uint16,
            '32FC1': np.float32,
            '64FC2': np.float64,
            '16UC1': np.uint16,
            'mono8': np.uint8,
            'mono16': np.uint16,
            'mono32': np.uint32,
        }
        if not img_msg.encoding in encoding_dict:
            raise NotImplementedError(
                'encoding {} not supported yet (supported: {})'.format(img_msg.encoding, encoding_dict.keys()))
        encoding_type = encoding_dict[img_msg.encoding]
        np_img = np.frombuffer(img_msg.data, dtype=encoding_type).reshape(img_msg.height, img_msg.width, -1)
    return np_img


def pack_pointcloud_msg(pc_array, frame_id):
    header = Header()
    header.frame_id = frame_id
    has_color = pc_array.shape[-1] > 3
    xyz_points = pc_array[:, :3].astype(np.float32)
    # import pdb; pdb.set_trace()
    if has_color:
        color_points = pc_array[:, 3:].astype(np.float32)
        pc2_msg = create_colored_cloud(header, xyz_points, color_points)
    else:
        # pc_array only contains xyz
        pc2_msg = pc2.create_cloud_xyz32(header, xyz_points)
    return pc2_msg


def process_point_cloud_msg(pc_msg):
    """
    Extract from a pointcloud message the pointcloud as an numpy array
    Args:
        pc_msg:

    Returns:

    """
    cloud_arr = rn.point_cloud2.pointcloud2_to_array(pc_msg)
    cloud_xyz = rn.point_cloud2.get_xyz_points(cloud_arr)
    pc_frame = pc_msg.header.frame_id
    try:
        cloud_xyzrgb = rn.point_cloud2.split_rgb_field(cloud_arr)
        cloud_rgb = np.stack([cloud_xyzrgb['r'], cloud_xyzrgb['g'], cloud_xyzrgb['b']], axis=-1)
        cloud_rgb = np.array(cloud_rgb) / 255
    except ValueError:
        # some pcs do not contain color, since they are only infrared (picoflexx case for example)
        cloud_rgb = np.zeros_like(cloud_xyz)
    pc_ar = np.concatenate([cloud_xyz, cloud_rgb], axis=-1)
    return pc_ar


def create_colored_cloud(header, pts, color):
    ptsimage = pts.reshape((-1, 3))
    rgb = color.reshape((-1, 3))
    rgb_uint8 = (255*rgb).astype(np.uint8)
    bgra = np.concatenate([np.flip(rgb_uint8, axis=-1), 255*np.ones((rgb.shape[0], 1), dtype=np.uint8)], axis=-1)
    rgba_uint32_values = bgra.view(dtype=np.uint32)
    points = np.rec.fromarrays([ptsimage[:,0], ptsimage[:,1], ptsimage[:,2], rgba_uint32_values[:,0]]) # this concatenates and preserves the types
    fields = [pc2.PointField('x', 0, pc2.PointField.FLOAT32, 1),
              pc2.PointField('y', 4, pc2.PointField.FLOAT32, 1),
              pc2.PointField('z', 8, pc2.PointField.FLOAT32, 1),
              pc2.PointField('rgba', 12, pc2.PointField.UINT32, 1)]
    pc = pc2.create_cloud(header, fields, points)
    return pc


def pack_message_as_dict(msg):
    output_dict = {}
    for slot_name in msg.__slots__:
        slot_value = getattr(msg, slot_name)
        if isinstance(slot_value, rospy.rostime.Time):
            keys = ['secs', 'nsecs']
            sub_dict_updated = {}
            for k in keys:
                sub_dict_updated['{}.{}'.format(slot_name, k)] = getattr(slot_value, k)
            output_dict.update(sub_dict_updated)
        elif '__slots__' in slot_value.__dir__():
            sub_dict = pack_message_as_dict(slot_value)
            # update keys:
            sub_dict_updated = {}
            for k, v in sub_dict.items():
                sub_dict_updated['{}.{}'.format(slot_name, k)] = v
            output_dict.update(sub_dict_updated)
        else:
            output_dict[slot_name] = slot_value

    return output_dict


def unpack_dict_to_message(msg_dict, Msg):
    """
    Pack the msg dict into the Msg
    :param msg_dict:
    :param Msg:
    :return:
    """
    # TODO: Implement
    pass


def unpack_camera_info(camera_info_msg):
    unpacked_camera_info = pack_message_as_dict(camera_info_msg)
    unpacked_camera_info['K'] = np.asarray(camera_info_msg.K).reshape(3, 3)
    unpacked_camera_info['R'] = np.asarray(camera_info_msg.R).reshape(3, 3)
    unpacked_camera_info['P'] = np.asarray(camera_info_msg.P).reshape(3, 4)
    return unpacked_camera_info


def _process_compressed_img_msg(img_msg):
    encoding_type = np.uint8
    np_img = np.frombuffer(img_msg.data, dtype=encoding_type)
    np_img = cv2.imdecode(np_img, cv2.IMREAD_COLOR)
    np_img = np.flip(np_img, axis=-1)
    return np_img


