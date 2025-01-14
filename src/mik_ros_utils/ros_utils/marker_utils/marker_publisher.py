import rospy
import numpy as np

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from mik_ros_utils.ros_utils.publisher_wrapper import PublisherWrapper


class MarkerPublisher(PublisherWrapper):
    Marker = Marker()
    def __init__(self, topic_name, path=None, marker_color=(1.0, 0, 0, 1.0), marker_type=None, 
                show=True, scale=(1.0, 1.0, 1.0), queue_size=100, rate=5.0, frame_id='world', marker_points=None):
        super().__init__(topic_name=topic_name, msg_type=self._get_msg_type(),
                        queue_size=queue_size, rate=rate)
        self.Marker = Marker()
        self._marker_color = marker_color
        self._marker_type = marker_type
        self._marker_points = marker_points
        self._scale = scale
        self._show = show
        self._frame_id = frame_id
        self.path = path

    def _get_msg_type(self):
        return Marker

    def _publish_loop(self):
        while not rospy.is_shutdown():
            if self.data is not None:
                marker = self._create_marker(marker_pose=self.data)
                if self.show:
                    marker.action = Marker.ADD
                else:
                    marker.action = Marker.DELETE
                self.publisher.publish(marker)
            self.rate.sleep()
            with self.lock:
                if not self.alive:
                    return

    def _create_marker(self, marker_pose, id=0):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time()
        marker.id = id
        marker.type = self.marker_type
        if marker.type == Marker.MESH_RESOURCE and self.path is not None:
            # Important Note: if it is a global path to the file, it must start with 'file://'
            path = self.path
            if path[0] == '/':
                path = 'file://' + path
            marker.mesh_resource = path
        marker.action = Marker.ADD
        marker.pose.position.x = marker_pose[0]
        marker.pose.position.y = marker_pose[1]
        marker.pose.position.z = marker_pose[2]
        marker.pose.orientation.x = marker_pose[3]
        marker.pose.orientation.y = marker_pose[4]
        marker.pose.orientation.z = marker_pose[5]
        marker.pose.orientation.w = marker_pose[6]
        marker.scale.x = self.scale[0]
        marker.scale.y = self.scale[1]
        marker.scale.z = self.scale[2]
        marker.color.r = self.marker_color[0]
        marker.color.g = self.marker_color[1]
        marker.color.b = self.marker_color[2]
        marker.color.a = self.marker_color[3]
        if self.marker_points is not None:
            marker.points = self.marker_points
        return marker

    @property
    def pose(self):
        return self.data

    @pose.setter
    def pose(self, value):
        self.data = value

    @property
    def marker_color(self):
        with self.lock:
            return self._marker_color

    @marker_color.setter
    def marker_color(self, value):
        with self.lock:
            self._marker_color = value

    @property
    def scale(self):
        with self.lock:
            return self._scale

    @scale.setter
    def scale(self, value):
        with self.lock:
            self._scale = value

    @property
    def marker_type(self):
        with self.lock:
            return self._marker_type

    @marker_type.setter
    def marker_type(self, value):
        with self.lock:
            self._marker_type = value

    @property
    def marker_points(self):
        with self.lock:
            return self._marker_points

    @marker_points.setter
    def marker_points(self, value):
        points = []
        for point_i in value:
            point_msg_i = Point()
            point_msg_i.x = point_i[0]
            point_msg_i.y = point_i[1]
            point_msg_i.z = point_i[2]
            points.append(point_msg_i)
        with self.lock:
            self._marker_points = points

    @property
    def show(self):
        with self.lock:
            return self._show

    @show.setter
    def show(self, value):
        with self.lock:
            self._show = value    

    @property
    def frame_id(self):
        with self.lock:
            return self._frame_id

    @frame_id.setter
    def frame_id(self, value):
        with self.lock:
            self._frame_id = value
