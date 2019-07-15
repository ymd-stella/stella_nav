import numpy as np
import tf
from geometry_msgs.msg import Quaternion

class GeometryUtils(object):
    @staticmethod
    def get_yaw(orientation):
        qx, qy, qz, qw = orientation.x, orientation.y, orientation.z, orientation.w
        return np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

    @staticmethod
    def get_quaternion(theta):
        return Quaternion(*tf.transformations.quaternion_from_euler(0, 0, theta))

    @staticmethod
    def regulate_rad(rad):
        pi_2 = 2 * np.pi
        rad = rad.copy()
        rad[rad > np.pi] = rad[rad > np.pi] - pi_2 * np.round(rad[rad > np.pi] / pi_2)
        rad[rad < np.pi] = rad[rad < np.pi] + pi_2 * np.round(rad[rad < np.pi] / pi_2)
        return rad
