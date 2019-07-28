import rclpy
import threading
from .observer import Observer
from stella_nav_core.stella_nav_node import get_node


class PeriodicObserver(Observer):
    def __init__(self, rate, **kwargs):
        super(PeriodicObserver, self).__init__(**kwargs)
        if rate is None or rate <= 0:
            raise ValueError("invalid rate")
        self._timer = get_node().create_timer(1.0/rate, self._dispatch_event)

    def _dispatch_event(self):
        self._call_event(msg=None)
