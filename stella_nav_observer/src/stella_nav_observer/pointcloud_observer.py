import rclpy
from .topic_observer import TopicObserver
from sensor_msgs.msg import PointCloud2
from stella_nav_core.stella_nav_node import get_node


class PointCloudObserver(TopicObserver):
    def __init__(self, **kwargs):
        super(PointCloudObserver, self).__init__(**kwargs)
        self._pointcloud_subscriber = get_node().create_subscription(PointCloud2, "~cloud", self._update_msg, rclpy.qos.QoSProfile(history=rclpy.qos.QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST, depth=1))
