import rospy
import threading
from .observer import Observer


class RecognizerObserver(Observer):
    def __init__(self, recognizers, recognizer, check_frequency=20.0, **kwargs):
        super(RecognizerObserver, self).__init__(**kwargs)
        self._recognizer = recognizers[recognizer]
        self._check_rate = rospy.Rate(check_frequency)
        self._pose = None
        self._goal = None
        self._lock = threading.RLock()
        self._worker = threading.Thread(target=self._dispatch_event, name="recognizer_observer")
        self._worker.start()

    def join(self):
        self._worker.join()

    def update_goal(self, goal):
        self._lock.acquire()
        self._goal = goal
        self._lock.release()

    def update_pose(self, pose):
        self._lock.acquire()
        self._pose = pose
        self._lock.release()

    def _dispatch_event(self):
        while not rospy.is_shutdown():
            if self._pose is not None and self._goal is not None:
                result, desc = self._recognizer.detect(self._pose, self._goal)
                if result:
                    self._call_event(msg=None)
            try:
                self._check_rate.sleep()
            except rospy.ROSInterruptException as e:
                rospy.logdebug("PoseObserver: {}".format(e))
