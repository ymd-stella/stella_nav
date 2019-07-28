import numpy as np
import rclpy
import threading
import yaml
import copy
from stella_nav_core.goal import Goal
from os.path import expanduser
from stella_nav_core.stella_nav_node import get_node


class GoalHandler(object):
    def __init__(self, goals_filepath=None, yaml_format=True):
        super(GoalHandler, self).__init__()
        if goals_filepath is not None and len(goals_filepath) > 0:
            self.set_goals_from_file(goals_filepath, yaml_format)
        else:
            self.set_goals([])

    def set_goals_from_file(self, goals_filepath, yaml_format):
        with open(expanduser(goals_filepath), "r") as fp:
            if yaml_format:
                goals = GoalHandler._parse_yaml_format(fp)
            else:
                goals = GoalHandler._parse_txt_format(fp)
        self.set_goals(goals)

    def set_goals(self, goals):
        self._goals = goals
        if goals:
            self._current_goal_idx = 0
            self._current_goal = self._goals[self._current_goal_idx]
        else:
            self._current_goal_idx = None
            self._current_goal = None
        self._data = np.array([(g.pose.position.x, g.pose.position.y) for g in self._goals])

    @staticmethod
    def _parse_yaml_format(fp):
        data = yaml.load(fp)
        goals = [Goal(**d)
            for d in data["goals"]]
        return goals

    @staticmethod
    def _parse_txt_format(fp):
        goals = [Goal(x=x, y=y, theta=theta)
                 for x, y, theta in [float(wp_s) for wp_s in line.split()]
                 for line in fp]
        return goals

    def get_current_goal(self):
        return self._current_goal

    def get_current_goal_idx(self):
        return self._current_goal_idx

    def get_nearest_goal_idx(self, pos):
        return np.argmin(np.linalg.norm((self._data - pos), axis=1))

    def get_goals(self):
        return copy.copy(self._goals)

    def set_current_goal(self, idx):
        self._current_goal_idx = idx
        self._current_goal = self._goals[idx]
        self._current_goal.header.stamp = get_node().get_clock().now().to_msg()

    def is_end(self):
        return self._current_goal_idx is None

    def clear(self):
        self.set_goals([])

    @staticmethod
    def get_next_goal_idx(goals, goal_idx):
        if goals[goal_idx].data.get("goto", None) is None:
            if goal_idx < len(goals) - 1:
                idx = goal_idx + 1
            else:
                idx = None
        else:
            idx = goals[goal_idx].data.get("goto", None)
        return idx

    def achieve(self):
        trigger = self._goals[self._current_goal_idx].data.get("trigger", None)
        idx = GoalHandler.get_next_goal_idx(self._goals, self._current_goal_idx)
        if idx is None:
            is_end = True
            self._current_goal_idx = None
            self._current_goal = None
        else:
            is_end = False
            self.set_current_goal(idx)
        return is_end, trigger

    def add_goal(self, goal):
        self._goals = self._goals + [goal]
        if self._current_goal_idx is None:
            self._current_goal_idx = len(self._goals) - 1
            self._current_goal = self._goals[self._current_goal_idx]
        if self._data.size != 0:
            self._data = np.vstack((self._data, np.array([(goal.pose.position.x, goal.pose.position.y)])))
        else:
            self._data = np.array([(goal.pose.position.x, goal.pose.position.y)])

