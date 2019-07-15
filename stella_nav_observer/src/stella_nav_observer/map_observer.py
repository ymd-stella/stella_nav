import rospy
from nav_msgs.msg import OccupancyGrid
from .topic_observer import TopicObserver

class MapObserver(TopicObserver):
    def __init__(self, **kwargs):
        super(MapObserver, self).__init__(**kwargs)
        self._map_subscriber = rospy.Subscriber("~map", OccupancyGrid, self._update_msg, queue_size=1)
