import rospy
from nav_msgs.msg import Odometry
from .topic_observer import TopicObserver

class OdomObserver(TopicObserver):
    def __init__(self, **kwargs):
        super(OdomObserver, self).__init__(**kwargs)
        self._odom_subscriber = rospy.Subscriber("~odom", Odometry, self._update_msg, queue_size=1)
