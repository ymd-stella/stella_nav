import threading
import rclpy
from std_msgs.msg import Float32, ColorRGBA, String
from numbers import Real
from stella_nav_core.stella_nav_node import get_node


class PublisherWrapper(object):
    def __init__(self, topic, t, publish_rate=10):
        super(PublisherWrapper, self).__init__()
        self._topic = topic
        self._lock = threading.RLock()
        self._pub = get_node().create_publisher(t, topic, rclpy.qos.QoSProfile(history=rclpy.qos.QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST, depth=1))
        self._msg = None
        self._timer= get_node().create_timer(1.0/publish_rate, self._work)

    def update_msg(self, msg):
        self._lock.acquire()
        self._msg = msg
        self._lock.release()

    def _get_msg(self):
        self._lock.acquire()
        msg = self._msg
        self._lock.release()
        return msg

    def _work(self):
        msg = self._get_msg()
        if msg is not None:
            try:
                self._pub.publish(msg)
            except RuntimeError as e:
                get_node().get_logger().debug("PublisherWrapper {}: {}".format(self._topic, e))


class CommonMsgHandler(object):
    def __init__(self, publish_rate=10):
        self._pubs = {}
        self._lock = threading.RLock()
        self._publish_rate = publish_rate

    def join(self):
        self._lock.acquire()
        for pub in self._pubs.values():
            pub.join()
        self._lock.release()

    def update_msg(self, topic, data):
        self._lock.acquire()
        if isinstance(data, str):
            t = String
            msg = String()
            msg.data = data
        elif isinstance(data, Real):
            t = Float32
            msg = Float32()
            msg.data = float(data)
        else:
            t = String
            msg = String()
            msg.data = str(data)
        if not topic in self._pubs:
            self._pubs[topic] = PublisherWrapper(topic, t, self._publish_rate)
        self._pubs[topic].update_msg(msg)
        self._lock.release()

