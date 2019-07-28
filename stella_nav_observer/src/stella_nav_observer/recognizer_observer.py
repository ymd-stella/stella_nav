import rclpy
import threading
from .observer import Observer
from stella_nav_core.stella_nav_node import get_node


class RecognizerObserver(Observer):
    def __init__(self, recognizers, recognizer, check_frequency=20.0, **kwargs):
        super(RecognizerObserver, self).__init__(**kwargs)
        self._recognizer = recognizers[recognizer]
        self._timer = get_node().create_timer(1.0/check_frequency, self._dispatch_event)
        self._pose = None
        self._goal = None
        self._lock = threading.RLock()

    def update_goal(self, goal):
        self._lock.acquire()
        self._goal = goal
        self._lock.release()

    def update_pose(self, pose):
        self._lock.acquire()
        self._pose = pose
        self._lock.release()

    def _dispatch_event(self):
        if self._pose is not None and self._goal is not None:
            result, desc = self._recognizer.detect(self._pose, self._goal)
            if result:
                self._call_event(msg=None)
