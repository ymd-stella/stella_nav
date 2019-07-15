import rospy
import threading
from .observer import Observer


class TopicObserver(Observer):
    def __init__(self, call_event_frequency=20.0, **kwargs):
        super(TopicObserver, self).__init__(**kwargs)
        self._latest_msg = None
        self._call_event_rate = rospy.Rate(call_event_frequency)
        self._lock = threading.RLock()
        self._worker = threading.Thread(target=self._dispatch_event, name="topic_observer")
        self._worker.start()

    def join(self):
        self._worker.join()

    def _update_msg(self, msg):
        self._lock.acquire()
        self._latest_msg = msg
        self._lock.release()

    def _dispatch_event(self):
        while not rospy.is_shutdown():
            self._lock.acquire()
            msg = self._latest_msg
            can_call_event = msg is not None
            self._lock.release()
            if can_call_event:
                self._call_event(msg)
                self._lock.acquire()
                self._latest_msg = None
                self._lock.release()
            try:
                self._call_event_rate.sleep()
            except rospy.ROSInterruptException as e:
                rospy.logdebug("PoseObserver: {}".format(e))

