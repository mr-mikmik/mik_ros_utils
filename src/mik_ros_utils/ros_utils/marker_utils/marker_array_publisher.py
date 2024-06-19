import rospy
import numpy as np

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from mik_ros_utils.ros_utils.marker_utils.marker_publisher import MarkerPublisher


class MarkerArrayPublisher(MarkerPublisher):
    """
    Here data is a list of poses
    """
    Marker = Marker()
    def __init__(self, topic_name, **kwargs):
        super().__init__(topic_name=topic_name, **kwargs)

    def _get_msg_type(self):
        return MarkerArray

    def _publish_loop(self):
        while not rospy.is_shutdown():
            if self.data is not None:
                markers = []
                current_poses = self.data
                # print('num_poses: ', len(current_poses))
                for i, pose in enumerate(current_poses):
                    marker_i = self._create_marker(marker_pose=pose, id=i)
                    if self.show:
                        marker_i.action = Marker.ADD
                    else:
                        marker_i.action = Marker.DELETE
                    markers.append(marker_i)
                # wrap the markers into an
                marker_array = MarkerArray()
                marker_array.markers = markers
                # publish
                self.publisher.publish(marker_array)
            self.rate.sleep()
            with self.lock:
                if not self.alive:
                    return

    def add_pose(self, pose):
        with self.lock:
            self._data.append(pose)




# DEBUG:
if __name__ == '__main__':
    rospy.init_node('marker_array_publisher')
    marker_array_publisher = MarkerArrayPublisher('plane_points', scale=(0.01, 0.01, 0.01),
                                                  marker_type=MarkerArrayPublisher.Marker.SPHERE, show=True, rate=5.0,
                                                  frame_id='med_base')
    # reset:
    marker_array_publisher.data = []
    for i in range(10):
        pos_xy_i = np.array([0.5, 0.]) + np.random.uniform(-0.1, 0.1, 2)
        pos_i = np.array([pos_xy_i[0], pos_xy_i[1], 0.])
        pose_i = np.concatenate([pos_i, [0, 0, 0, 1]])
        marker_array_publisher.add_pose(pose_i)
        print('num_poses: ', len(marker_array_publisher.data))
        # wait some time
        rospy.sleep(1.5)



