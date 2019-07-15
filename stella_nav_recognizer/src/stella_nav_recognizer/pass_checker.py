from .recognizer import Recognizer
from stella_nav_core.geometry_utils import GeometryUtils
import numpy as np

class PassChecker(Recognizer):
    def __init__(self, distance=0.0):
        super(PassChecker, self).__init__()
        self._distance = distance
        self.desc = "PassChecker(distance={})".format(distance)

    def detect(self, pose, goal):
        p = np.array((pose.pose.position.x, pose.pose.position.y))
        g = np.array((goal.pose.position.x, goal.pose.position.y))
        yaw = GeometryUtils.get_yaw(goal.pose.orientation)
        signed_distance = np.dot(np.array((np.cos(yaw), np.sin(yaw))), p - g)
        return signed_distance > self._distance, self.desc
