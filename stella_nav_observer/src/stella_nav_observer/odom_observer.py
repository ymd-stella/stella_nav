import rclpy
from nav_msgs.msg import Odometry
from .topic_observer import TopicObserver
from stella_nav_core.stella_nav_node import get_node

class OdomObserver(TopicObserver):
    def __init__(self, **kwargs):
        super(OdomObserver, self).__init__(**kwargs)
        self._odom_subscriber = get_node().create_subscription(Odometry, "~odom", self._update_msg, rclpy.qos.QoSProfile(history=rclpy.qos.QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST, depth=1))
