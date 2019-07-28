import rclpy
import threading
from .observer import Observer
from stella_nav_core.stella_nav_node import get_node


class TopicObserver(Observer):
    def __init__(self, call_event_frequency=20.0, **kwargs):
        super(TopicObserver, self).__init__(**kwargs)
        self._latest_msg = None
        self._timer = get_node().create_timer(1.0/call_event_frequency, self._dispatch_event)
        self._lock = threading.RLock()

    def _update_msg(self, msg):
        self._lock.acquire()
        self._latest_msg = msg
        self._lock.release()

    def _dispatch_event(self):
        self._lock.acquire()
        msg = self._latest_msg
        can_call_event = msg is not None
        self._lock.release()
        if can_call_event:
            self._call_event(msg)
            self._lock.acquire()
            self._latest_msg = None
            self._lock.release()

