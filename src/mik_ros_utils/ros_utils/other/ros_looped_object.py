import abc
import threading

import rospy


class ThreadedObject(abc.ABC):
    def __init__(self):
        self.alive = False
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self.threaded_function)

    @abc.abstractmethod
    def threaded_function(self):
        pass

    def start(self):
        with self.lock:
            self.alive = True
        self.thread.start()

    def finish(self):
        with self.lock:
            self.alive = False
        self.thread.join()


class ROSLoopedObject(ThreadedObject):

    def __init__(self, rate=1.0):
        self.rate = rospy.Rate(rate)
        super().__init__()

    @abc.abstractmethod
    def looped_function(self):
        pass

    def threaded_function(self):
        while not rospy.is_shutdown():
            self.looped_function()
            with self.lock:
               if not self.alive:
                   return
            self.rate.sleep()
