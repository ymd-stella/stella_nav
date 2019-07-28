import rclpy
from .topic_observer import TopicObserver
from std_msgs.msg import String
from stella_nav_core.stella_nav_node import get_node


class TriggerObserver(TopicObserver):
    def __init__(self, **kwargs):
        super(TriggerObserver, self).__init__(**kwargs)
        self._trigger_subscriber = get_node().create_subscription(String, "~trigger", self._update_msg, rclpy.qos.QoSProfile(history=rclpy.qos.QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_ALL))
