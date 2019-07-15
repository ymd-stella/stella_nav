import rospy
import threading
from .observer import Observer


class PeriodicObserver(Observer):
    def __init__(self, rate, **kwargs):
        super(PeriodicObserver, self).__init__(**kwargs)
        if rate is None or rate <= 0:
            raise ValueError("invalid rate")
        self._rate = rospy.Rate(rate)
        self._worker = threading.Thread(target=self._dispatch_event, name="current_goal_observer")
        self._worker.start()

    def join(self):
        self._worker.join()

    def _dispatch_event(self):
        while not rospy.is_shutdown():
            self._call_event(msg=None)
            try:
                self._rate.sleep()
            except rospy.ROSInterruptException as e:
                rospy.logdebug("PeriodicObserver: {}".format(e))
