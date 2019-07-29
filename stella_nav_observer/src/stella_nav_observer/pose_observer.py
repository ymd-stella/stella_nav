import rclpy
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
import threading
import tf2_ros
import tf2_geometry_msgs
import time
from .observer import Observer
from stella_nav_core.stella_nav_node import get_node


class PoseObserver(Observer):
    identical_pose = PoseStamped(
        header=Header(frame_id="map"),
        pose=Pose(
            position=Point(x=0, y=0, z=0),
            orientation=Quaternion(x=0, y=0, z=0, w=1)))

    def __init__(self, robot_frame_id, fixed_frame_id="map", rate=20.0, **kwargs):
        super(PoseObserver, self).__init__(**kwargs)
        self._robot_frame_id = robot_frame_id
        self._fixed_frame_id = fixed_frame_id
        self.tf_buffer = tf2_ros.Buffer(rclpy.Duration(10))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self._timer = get_node.create_timer(1.0/rate, self._work)

    def join(self):
        self._pose_update_thread.join()

    def _work(self):
        start = time.clock()
        pose = self.get_pose()
        if pose is not None:
            self._call_event(pose)

    def get_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(self._fixed_frame_id, self._robot_frame_id, rclpy.Time(), rclpy.Duration(seconds=0.01))
        except (tf2_ros.TransformException, rclpy.ROSInterruptException) as e:
            get_node().get_logger().warn("PoseObserver get Exception: {}".format(e),
                throttle_duration_sec=1)
            return None
        if transform:
            pose = tf2_geometry_msgs.do_transform_pose(PoseObserver.identical_pose, transform)
        else:
            get_node().get_logger().warn("PoseObserver get None")
            pose = None
        return pose
