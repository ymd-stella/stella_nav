import rclpy
import numpy as np
from ros_numpy import numpify
from stella_nav_core.stella_nav_node import get_node

class PointCloudListener(object):
    def __init__(self, handlers, costmaps, initial_setup, **kwargs):
        self._costmaps = costmaps
        self._handler_costmap_pairs = [(handlers["robot_costmap"], costmaps["robot_costmap"])]
        self._initial_setup = initial_setup

    def __call__(self, msg):
        assert msg.header.frame_id == "map"
        data = numpify(msg)
        if data.size == 0:
            get_node().get_logger().warn("pointcloud is empty")
            return
        w = np.array(data[["x", "y"]].tolist())
        stamp = msg.header.stamp
        for costmap in self._costmaps.values():
            costmap.write_from_world(w, stamp)
        # keys = ["robot_costmap", "goal_costmap"]
        keys = ["robot_costmap"]
        for handler, costmap in self._handler_costmap_pairs:
            handler.update_costmap(costmap)
        if not self._initial_setup["obstacle"]:
            self._initial_setup["obstacle"] = True
            get_node().get_logger().info("obstacle initialized")
