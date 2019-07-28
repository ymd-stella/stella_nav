from .recognizer import Recognizer
import numpy as np
import rclpy
from stella_nav_core.stella_nav_node import get_node

class StopRecognizer(Recognizer):
    def __init__(self, distance, limit):
        super(StopRecognizer, self).__init__()
        self._distance = distance
        self._limit = rclpy.Duration(seconds=limit)
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
        self._start = get_node().get_clock().now()

    def detect(self, pose, goal):
        if self._pose is None or not self._is_stopping(pose):
            self._start_checking(pose)
            return False
        return self._limit < get_node().get_clock().now() - self._start, self.desc
