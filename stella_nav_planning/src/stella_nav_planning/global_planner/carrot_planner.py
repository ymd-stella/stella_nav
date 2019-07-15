import numpy as np
from .helper import PassableChecker, StuckChecker


class CarrotPlanner(object):
    def __init__(self, costmaps, costmap, passable_check_length=5):
        self._costmaps = costmaps
        self._costmap = costmaps[costmap]
        self._passable_check_length = passable_check_length

    def set_check_planner(self, planner):
        pass

    def plan(self, goal_idx, goals, subgoal_idx, subgoals, pose):
        self._costmap.lock.acquire()
        costmap = self._costmap.clone()
        self._costmap.lock.release()

        subgoals_ret = None
        cost = None

        triggers = []
        goal_is_passable = PassableChecker.is_passable(costmap, goal_idx, goals, pose, self._passable_check_length)
        triggers.append("goalPassable" if goal_is_passable else "goalImpassable")
        if subgoal_idx is not None:
            subgoal_is_passable = PassableChecker.is_passable(costmap, subgoal_idx, subgoals, pose, self._passable_check_length)
            triggers.append("subgoalPassable" if subgoal_is_passable else "subgoalImpassable")
        if StuckChecker.is_stuck(costmap, pose):
            triggers.append("isStuck")
        else:
            triggers.append("stuckRecovered")
        return subgoals_ret, cost, triggers, None, None
