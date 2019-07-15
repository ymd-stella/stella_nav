import rospy
from .topic_observer import TopicObserver
from sensor_msgs.msg import PointCloud2


class PointCloudObserver(TopicObserver):
    def __init__(self, **kwargs):
        super(PointCloudObserver, self).__init__(**kwargs)
        self._pointcloud_subscriber = rospy.Subscriber("~cloud", PointCloud2, self._update_msg, queue_size=1)
