import rospy
from .topic_observer import TopicObserver
from std_msgs.msg import String


class TriggerObserver(TopicObserver):
    def __init__(self, **kwargs):
        super(TriggerObserver, self).__init__(**kwargs)
        self._trigger_subscriber = rospy.Subscriber("~trigger", String, self._update_msg)
