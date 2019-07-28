from .recognizer import Recognizer
import rclpy
from stella_nav_core.stella_nav_node import get_node

class TimeChecker(Recognizer):
    def __init__(self, limit, interval=0):
        super(TimeChecker, self).__init__()
        self._initial_limit = rclpy.Duration(seconds=limit)
        self._limit = self._initial_limit
        self._interval = rclpy.Duration(seconds=interval)
        self._latest_goal = None
        self._stop_time = rclpy.Time()
        self.desc = "TimeChecker(limit={})".format(limit)

    def stop(self):
        self._t_stopping_start = get_node().get_clock().now()

    def start(self):
        self._stop_time += get_node().get_clock().now() - self._t_stopping_start

    def detect(self, pose, goal):
        if self._latest_goal is None:
            self._latest_goal = goal
        elif self._latest_goal is not goal:
            self._limit = self._initial_limit
            self._latest_goal = goal
        time_is_over = self._limit < (get_node().get_clock().now() - goal.header.stamp) - self._stop_time
        if time_is_over:
            self._limit += self._interval
        return time_is_over, self.desc
