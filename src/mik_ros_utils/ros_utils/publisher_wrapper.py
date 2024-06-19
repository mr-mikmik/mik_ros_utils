import rospy
import threading
import abc
import copy
import tf2_ros as tf
from geometry_msgs.msg import TransformStamped


class PublisherWrapper(abc.ABC):
    def __init__(self, topic_name, msg_type, queue_size=100, rate=5.0):
        self.topic_name = topic_name
        self.msg_type = msg_type
        self.queue_size = queue_size
        self.publisher = rospy.Publisher(self.topic_name, self.msg_type, queue_size=self.queue_size)
        self.rate = rospy.Rate(rate)
        self._data = None
        self.alive = True
        self.lock = threading.Lock()
        self.publisher_thread = threading.Thread(target=self._publish_loop)
        self.publisher_thread.start()

    def _publish_loop(self):
        while not rospy.is_shutdown():
            if self.data is not None:
                self.publisher.publish(self.data)
                self.rate.sleep()
            with self.lock:
                if not self.alive:
                    return

    @property
    def data(self):
        with self.lock:
            return self._data

    @data.setter
    def data(self, value):
        with self.lock:
            self._data = value

    def finish(self):
        with self.lock:
            self.alive = False
        self.publisher_thread.join()

