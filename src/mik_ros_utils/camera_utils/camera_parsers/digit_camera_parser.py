import os
import rospy
from std_srvs.srv import Empty, EmptyResponse, EmptyRequest
from sensor_msgs.msg import Image, CompressedImage, CameraInfo

from mik_ros_utils.camera_utils.camera_parsers.camera_parser_base import CameraParserBase


class DigitCameraParser(CameraParserBase):
    """
    Note: Digit cameras have only color.
    """
    # def _process_image_msg(self, img_msg):
    #     # Assume it is a Image message
    #     np_img = process_compressed_img_msg(img_msg)
    #     return np_img

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.reset_service_proxy = rospy.ServiceProxy(f'/{self.camera_name}/reset', Empty)

    def _get_message_types(self):
        msgs_types = super()._get_message_types()
        msgs_types['color'] = Image
        return msgs_types

    def _get_optical_frame_name(self):
        frame_names = {
            'color': '{}_optical_frame'.format(self.camera_name),
            'depth': '{}_camera_frame'.format(self.camera_name),
        }
        return frame_names

    def _get_color_topic(self):
        color_topic = '/{}/image'.format(self.camera_name)
        return color_topic

    def _get_color_info_topic(self):
        color_info_topic = '/{}/camera_info'.format(self.camera_name)
        return color_info_topic

    def _get_camera_name_base(self):
        return 'digit_'

    def reset_digit(self):
        """
        Reset the digit camera by calling the reset service
        """
        rospy.loginfo(f'Resetting digit camera {self.camera_name}')
        try:
            self.reset_service_proxy(EmptyRequest())
            rospy.loginfo(f'Digit camera {self.camera_name} reset successfully')
        except rospy.ServiceException as e:
            rospy.logerr(f'Failed to reset digit camera {self.camera_name}: {e}')




# TEST:
if __name__ == '__main__':
    import matplotlib.pyplot as plt
    from mik_ros_utils import safe_init_node
    from mik_tools import get_dataset_path
    dataset_path = get_dataset_path('digit_test_data')
    safe_init_node('digit_parser_test')
    digit_parser = DigitCameraParser(camera_name='digit_D21202', save_path=dataset_path)
    img = digit_parser.get_image_color()
    print(img.shape)
    plt.imshow(img)
    plt.show()
    # save
    digit_parser.wrap_data = True
    img_wr = digit_parser.get_image_color()
    img_wr.save_fc(0)

