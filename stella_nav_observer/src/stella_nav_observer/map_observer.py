import rclpy
from nav_msgs.msg import OccupancyGrid
from .topic_observer import TopicObserver
from stella_nav_core.stella_nav_node import get_node

class MapObserver(TopicObserver):
    def __init__(self, **kwargs):
        super(MapObserver, self).__init__(**kwargs)
        self._map_subscriber = get_node().create_subscription(OccupancyGrid, "~map", self._update_msg, rclpy.qos.QoSProfile(history=rclpy.qos.QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST, depth=1))
