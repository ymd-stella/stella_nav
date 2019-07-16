import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
import threading
import tf2_ros
import tf2_geometry_msgs
import time
from .observer import Observer


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
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self._pose_update_thread = threading.Thread(target=self._worker, name="pose_observer")
        self._pose_update_thread.start()
        self._rate = rospy.Rate(rate)

    def join(self):
        self._pose_update_thread.join()

    def _worker(self):
        while not rospy.is_shutdown():
            start = time.clock()
            pose = self.get_pose()
            if pose is not None:
                self._call_event(pose)
            try:
                self._rate.sleep()
            except rospy.ROSInterruptException as e:
                rospy.logdebug("PoseObserver: {}".format(e))

    def get_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(self._fixed_frame_id, self._robot_frame_id, rospy.Time(0), rospy.Duration(0.01))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException, rospy.ROSInterruptException) as e:
            rospy.logwarn_throttle(5, "PoseObserver get Exception: {}".format(e))
            return None
        if transform:
            pose = tf2_geometry_msgs.do_transform_pose(PoseObserver.identical_pose, transform)
        else:
            rospy.logwarn("PoseObserver get None")
            pose = None
        return pose
