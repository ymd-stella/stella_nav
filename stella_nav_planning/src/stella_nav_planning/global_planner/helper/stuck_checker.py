import rospy
import numpy as np


class StuckChecker(object):
    @staticmethod
    def is_stuck(costmap, pose):
        w = np.array((pose.pose.position.x, pose.pose.position.y))
        is_contained, m = costmap.world_to_map(w)
        if not is_contained:
            rospy.logdebug("warning: pose is not contained in map")
            return False
        return costmap.get_value(m) > 0.999
