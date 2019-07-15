import threading
import rospy
from std_msgs.msg import Float32, ColorRGBA
from jsk_rviz_plugins.msg import OverlayText
from numbers import Real


class PublisherWrapper(threading.Thread):
    def __init__(self, topic, t, publish_rate=10, queue_size=1):
        super(PublisherWrapper, self).__init__()
        self._topic = topic
        self._lock = threading.RLock()
        self._pub = rospy.Publisher(topic, t, queue_size=queue_size)
        self._msg = None
        self._rate = rospy.Rate(publish_rate)
        self.start()

    def update_msg(self, msg):
        self._lock.acquire()
        self._msg = msg
        self._lock.release()

    def _get_msg(self):
        self._lock.acquire()
        msg = self._msg
        self._lock.release()
        return msg

    def run(self):
        while not rospy.is_shutdown():
            msg = self._get_msg()
            if msg is not None:
                try:
                    self._pub.publish(msg)
                except rospy.ROSException as e:
                    rospy.logdebug("PublisherWrapper {}: {}".format(self._topic, e))
            try:
                self._rate.sleep()
            except rospy.ROSInterruptException as e:
                rospy.logdebug("PublisherWrapper {}: {}".format(self._topic, e))


class CommonMsgHandler(object):
    def __init__(self, publish_rate=10, queue_size=1):
        self._pubs = {}
        self._lock = threading.RLock()
        self._publish_rate = publish_rate
        self._queue_size = queue_size

    def join(self):
        self._lock.acquire()
        for pub in self._pubs.values():
            pub.join()
        self._lock.release()

    def update_msg(self, topic, data):
        self._lock.acquire()
        if isinstance(data, str):
            t = OverlayText
            msg = OverlayText()
            msg.action = OverlayText.ADD
            msg.text = str(data)
        elif isinstance(data, Real):
            t = Float32
            msg = Float32()
            msg.data = float(data)
        else:
            raise ValueError("invalid msg type")
        if not topic in self._pubs:
            self._pubs[topic] = PublisherWrapper(topic, t, self._publish_rate, self._queue_size)
        self._pubs[topic].update_msg(msg)
        self._lock.release()

