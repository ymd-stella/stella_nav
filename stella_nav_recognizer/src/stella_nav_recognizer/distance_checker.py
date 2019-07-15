from .recognizer import Recognizer
import numpy as np

class DistanceChecker(Recognizer):
    def __init__(self, distance):
        super(DistanceChecker, self).__init__()
        self._distance = distance
        self.desc = "DistanceChecker(distance={})".format(distance)

    def detect(self, pose, goal):
        dist = np.hypot(pose.pose.position.x - goal.pose.position.x, pose.pose.position.y - goal.pose.position.y)
        return dist < self._distance, self.desc
