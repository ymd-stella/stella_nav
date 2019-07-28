import rclpy
import threading

class GridMap2DHandler(object):
    def __init__(self):
        super(GridMap2DHandler, self).__init__()
        self._costmap = None
        self._lock = threading.RLock()

    def update_costmap(self, costmap):
        self._lock.acquire()
        self._costmap = costmap.clone()
        self._lock.release()

    def costmap(self):
        self._lock.acquire()
        costmap = self._costmap
        self._lock.release()
        return costmap
