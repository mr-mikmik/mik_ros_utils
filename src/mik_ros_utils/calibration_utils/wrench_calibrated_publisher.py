import numpy as np
import rospy
from threading import Lock
from geometry_msgs.msg import WrenchStamped


class WrenchCalibratedPublisher(object):
    def __init__(self, wrench_topic, calibration_samples=100):
        self.wrench_topic = wrench_topic
        self.calibration_samples = calibration_samples
        self.wrench_offsets = np.zeros(6)
        self._calibration_in_process = False
        self.lock = Lock()
        self.calibration_buffer = []
        self.wrench_sub = rospy.Subscriber(self.wrench_topic, WrenchStamped, self._wrench_offsets_cb)
        self.calibrated_wrench_pub = rospy.Publisher(self.wrench_topic + '_calibrated', WrenchStamped, queue_size=1)

    def calibrate(self):
        with self.lock:
            self._calibration_in_process = True
            self.calibration_buffer = []
        buffer_full = False
        while not buffer_full:
            rospy.sleep(0.1)
            with self.lock:
                buffer_full = len(self.calibration_buffer) >= self.calibration_samples
        with self.lock:
            self.wrench_offsets = np.mean(self.calibration_buffer, axis=0)
            self._calibration_in_process = False
            self.calibration_buffer = []


    def _wrench_offsets_cb(self, msg):
        wrench = self.unpack_wrench(msg)
        with self.lock:
            if self._calibration_in_process:
                self.calibration_buffer.append(wrench)
            else:
                wrench_calibrated = self._calibrate_wrench(wrench)
                wrench_calibrated_msg = self.pack_wrench(wrench_calibrated)
                wrench_calibrated_msg.header = msg.header
                self.calibrated_wrench_pub.publish(wrench_calibrated_msg)

    def _calibrate_wrench(self, wrench):
        wrench_calibrated = wrench - self.wrench_offsets
        return wrench_calibrated

    @staticmethod
    def unpack_wrench(msg):
        # Convert a WrenchStamped into a numpy array
        wrench = np.array([msg.wrench.force.x,
                           msg.wrench.force.y,
                           msg.wrench.force.z,
                           msg.wrench.torque.x,
                           msg.wrench.torque.y,
                           msg.wrench.torque.z])
        return wrench

    @staticmethod
    def pack_wrench(wrench):
        # Convert a numpy array into a WrenchStamped
        msg = WrenchStamped()
        msg.wrench.force.x = wrench[0]
        msg.wrench.force.y = wrench[1]
        msg.wrench.force.z = wrench[2]
        msg.wrench.torque.x = wrench[3]
        msg.wrench.torque.y = wrench[4]
        msg.wrench.torque.z = wrench[5]
        return msg


# TEST:
if __name__ == '__main__':
    rospy.init_node('wrench_calibrated_publisher_test')
    wrench_topic = '/med/wrench'
    wrench_calibrated_publisher = WrenchCalibratedPublisher(wrench_topic)