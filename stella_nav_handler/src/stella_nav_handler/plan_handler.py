import rclpy
import threading

class PlanHandler(object):
    def __init__(self):
        super(PlanHandler, self).__init__()
        self._plan = None
        self._lock = threading.RLock()

    def update_plan(self, plan):
        self._lock.acquire()
        self._plan = plan
        self._lock.release()

    def plan(self):
        self._lock.acquire()
        plan = self._plan
        self._lock.release()
        return plan
