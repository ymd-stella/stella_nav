import rclpy
import numpy as np
from stella_nav_core.stella_nav_node import get_node

class PoseUpdateListener(object):
    def __init__(self, navigation, observers, costmaps, initial_setup, **kwargs):
        self._navigation = navigation
        self._update_pose_observers = [observers["force_recovery"]]
        self._costmaps = costmaps
        self.initial_pose = None
        self._initial_setup = initial_setup

    def __call__(self, msg):
        self._navigation.update_pose(msg)
        for observer in self._update_pose_observers:
            observer.update_pose(msg)
        origin = np.array((msg.pose.position.x, msg.pose.position.y))
        costmap = self._costmaps["robot_costmap"]
        costmap.update_origin(costmap.world_to_map_abs(origin))
        self.initial_pose = msg
        if not self._initial_setup["pose"]:
            self._initial_setup["pose"] = True
            get_node().get_logger().info("pose initialized")
