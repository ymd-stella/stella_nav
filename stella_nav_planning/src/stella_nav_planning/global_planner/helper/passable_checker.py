import rospy
import numpy as np
from stella_nav_handler.goal_handler import GoalHandler


class PassableChecker(object):
    @staticmethod
    def bresenhamline(p1, p2):
        d = p2 - p1
        steps = np.max(np.abs(d))
        if np.all(d == 0):
            return np.zeros(d.shape, dtype=p1.dtype)
        d_normalized = d / steps.astype(np.float64)
        t = np.arange(0, steps + 1)[:, np.newaxis]
        line = p1 + d_normalized * t
        return np.array(np.rint(line), dtype=p1.dtype)

    @staticmethod
    def get_planning_goals(costmap, pose, goals, goal_idx, goal_lookahead):
        poses = [np.array((pose.pose.position.x, pose.pose.position.y)).reshape((1, 2))]
        goal = goals[goal_idx]
        planning_goals = []
        i = 0
        while True:
            goal_w = np.array((goal.pose.position.x, goal.pose.position.y)).reshape((1, 2))
            if not costmap.world_to_map(goal_w)[0]:
                break
            poses.append(goal_w)
            planning_goals.append(goal)
            i += 1
            next_goal_idx = GoalHandler.get_next_goal_idx(goals, goal_idx)
            if next_goal_idx is None or goal.data.get("explicit", None) or i > goal_lookahead:
                break
            goal = goals[next_goal_idx]
            goal_idx = next_goal_idx
        return poses, goal, planning_goals

    @staticmethod
    def is_passable_at_poses(costmap, poses):
        is_passable = True
        for i in range(len(poses) - 1):
            w1 = poses[i]
            w2 = poses[i + 1]
            is_contained1, m1 = costmap.world_to_map(w1)
            if not is_contained1:
                continue
            is_contained2, m2 = costmap.world_to_map(w2)
            if not is_contained2:
                continue
            line = PassableChecker.bresenhamline(m1, m2)
            if np.any(costmap.get_value(line) > 0.999):
                is_passable = False
                rospy.logdebug("impassable", (i, poses[i]), (i+1, poses[i+1]))
                break
        return is_passable

    @staticmethod
    def is_passable(costmap, goal_idx, goals, pose, passable_check_length=5):
        poses, goal, planning_goals = PassableChecker.get_planning_goals(costmap, pose, goals, goal_idx, passable_check_length)
        return PassableChecker.is_passable_at_poses(costmap, poses)
