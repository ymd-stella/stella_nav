from .recognizer import Recognizer
import numpy as np
import rospy

class StopRecognizer(Recognizer):
    def __init__(self, distance, limit):
        super(StopRecognizer, self).__init__()
        self._distance = distance
        self._limit = limit
        self._pose = None
        self._start = None
        self.desc = "StopRecognizer(distance={} limit={})".format(distance, limit)

    def _is_stopping(self, pose):
        distance = np.hypot(
            self._pose.pose.position.x - pose.pose.position.x,
            self._pose.pose.position.y - pose.pose.position.y)
        return distance < self._distance

    def _start_checking(self, pose):
        self._pose = pose
        self._start = rospy.Time.now()

    def detect(self, pose, goal):
        if self._pose is None or not self._is_stopping(pose):
            self._start_checking(pose)
            return False
        return self._limit < (rospy.Time.now() - self._start).to_sec(), self.desc
