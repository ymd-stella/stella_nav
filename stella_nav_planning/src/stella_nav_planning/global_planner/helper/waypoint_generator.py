import numpy as np
from std_msgs.msg import Header
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion, PoseStamped
import rclpy
import tf
from stella_nav_core.goal import Goal
from stella_nav_core.stella_nav_node import get_node


class WaypointGenerator(object):
    @staticmethod
    def generate(trajectory, subgoal_distance=1.0):
        """
        trajectory: sequence of (x, y) in world coordinates
        subgoal_distance: distance between waypoints
        """
        # pickup point from trajectory
        trajectory_tmp = [trajectory[0]]
        for p in trajectory:
            if np.linalg.norm(p - trajectory_tmp[-1]) > subgoal_distance:
                trajectory_tmp.append(p)
        trajectory_array = np.array(trajectory_tmp)
        if trajectory_array.shape[0] < 2:
            get_node().get_logger().warn("trajectory is too short.")

        # calculate yaw
        vec = trajectory_array[1:] - np.roll(trajectory_array, 1, axis=0)[1:]
        yaw = np.arctan2(vec[:, 1], vec[:, 0])
        trajectory_result = np.hstack((trajectory_array[1:, :], yaw[:, np.newaxis]))

        subgoals = [Goal(x=p[0], y=p[1], theta=p[2]) for p in trajectory_result]
        return subgoals
