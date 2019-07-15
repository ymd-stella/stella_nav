#!/usr/bin/env python
import rospy
import numpy as np
from observer import PoseObserver
from geometry_utils import GeometryUtils
import yaml
from os.path import expanduser

class AutoGoalSetter(object):
    def __init__(self, dist_tolerance=1.0, rotation_tolerance=1.0):
        self._dist_tolerance = dist_tolerance
        self._rotation_tolerance = rotation_tolerance
        self._previous_goal = None
        self._previous_theta = None
        self._yaml_dict = {"goals": []}

    def pose_update_listener(self, msg):
        pos = np.array((msg.pose.position.x, msg.pose.position.y))
        theta = GeometryUtils.get_yaw(msg.pose.orientation)
        if self._previous_goal is not None and self._previous_theta is not None:
            dist = np.linalg.norm(self._previous_goal - pos)
            rotation = np.abs(self._previous_theta - theta)
            if dist > self._dist_tolerance or rotation > self._rotation_tolerance:
                goal_dict = {"x": float(pos[0]), "y": float(pos[1]), "theta": float(theta)}
                rospy.loginfo("add {x:+.3f}, {y:+.3f}, {theta:+.3f}".format(**goal_dict))
                self._yaml_dict["goals"].append(goal_dict)
                self._previous_goal = pos
                self._previous_theta = theta
        else:
            self._previous_goal = pos
            self._previous_theta = theta

    def save(self, filename):
        with open(expanduser(filename), "w") as fp:
            fp.write(yaml.dump(self._yaml_dict, default_flow_style=False))
            rospy.loginfo("{} saved".format(filename))

def main():
    rospy.init_node("auto_goal_setter_node")

    savefilename = rospy.get_param("~savefilename", "goals.yaml")
    robot_frame_id = rospy.get_param("~robot_frame_id", "base_link")
    auto_goal_setter = AutoGoalSetter(dist_tolerance=1.0, rotation_tolerance=3.0/180.0*np.pi)
    pose_observer = PoseObserver(robot_frame_id=robot_frame_id)
    pose_observer.add_listener(auto_goal_setter.pose_update_listener)

    rospy.spin()

    pose_observer.join()
    auto_goal_setter.save(savefilename)

if __name__ == "__main__":
    main()
