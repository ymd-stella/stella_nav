from .recognizer import Recognizer
import rospy

class TimeChecker(Recognizer):
    def __init__(self, limit, interval=0):
        super(TimeChecker, self).__init__()
        self._initial_limit = limit
        self._limit = limit
        self._interval = interval
        self._latest_goal = None
        self._stop_time = rospy.Time(0)
        self.desc = "TimeChecker(limit={})".format(limit)

    def stop(self):
        self._t_stopping_start = ropsy.Time.now()

    def start(self):
        self._stop_time += rospy.Time.now() - self._t_stopping_start

    def detect(self, pose, goal):
        if self._latest_goal is None:
            self._latest_goal = goal
        elif self._latest_goal is not goal:
            self._limit = self._initial_limit
            self._latest_goal = goal
        time_is_over = self._limit < (rospy.Time.now() - goal.header.stamp).to_sec() - self._stop_time.to_sec()
        if time_is_over:
            self._limit += self._interval
        return time_is_over, self.desc
