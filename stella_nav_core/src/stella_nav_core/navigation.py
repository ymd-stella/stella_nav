import threading
import rclpy
from geometry_msgs.msg import PoseStamped, Vector3, Twist
import time
import numpy as np
import copy
from stella_nav_core.stella_nav_node import get_node


class Navigation(threading.Thread):
    def __init__(self, state_to_local_planner, state_to_global_planner, state_machine, goal_handler,
                 subgoal_handler, recognizers, plan_handlers, global_costmap_handler,
                 controller_frequency=10.0):
        super(Navigation, self).__init__()
        self._goal_handler = goal_handler
        self._subgoal_handler = subgoal_handler
        self._recognizers = recognizers
        self._plan_handlers = plan_handlers
        self._global_costmap_handler = global_costmap_handler
        self._pose = None
        self._cmd_vel_pub = get_node().create_publisher(Twist, "~cmd_vel", rclpy.qos.QoSProfile(history=rclpy.qos.QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST, depth=1))
        self._controller_frequency = controller_frequency
        self._sleep_dur = 1.0/controller_frequency
        self._timer = get_node().create_timer(self._sleep_dur, self._work)
        self._state_to_local_planner = state_to_local_planner
        self._state_to_global_planner = state_to_global_planner
        self._state_machine = state_machine
        self._latest_plan = None
        self._lock = threading.RLock()

    def update_pose(self, pose):
        self._lock.acquire()
        self._pose = pose
        self._lock.release()

    def pose(self):
        self._lock.acquire()
        pose = self._pose
        self._lock.release()
        return pose

    def _achieve(self):
        is_end, trigger = self._goal_handler.achieve()
        self._state_machine.trigger("goalAchieved")
        if trigger is not None:
            self._state_machine.trigger(trigger)
        if is_end:
            cmd_vel = Twist(linear=Vector3(x=0, y=0, z=0), angular=Vector3(x=0, y=0, z=0))
            self._subgoal_handler.clear()
        else:
            cmd_vel = None
        return cmd_vel

    def _global_plan(self, state, pose, goals, goal_idx):
        subgoal_idx = self._subgoal_handler.get_current_goal_idx()
        subgoals = self._subgoal_handler.get_goals()
        subgoals_new, cost, triggers, global_plan, global_costmap = self._state_to_global_planner[state].plan(goal_idx, goals, subgoal_idx, subgoals, pose)
        if global_costmap is not None:
            self._global_costmap_handler.update_costmap(global_costmap)
        if subgoals_new is not None:
            self._subgoal_handler.set_goals(subgoals_new)
            if "global_plan" in self._plan_handlers and global_plan is not None:
                self._plan_handlers["global_plan"].update_plan(global_plan)
        return triggers

    def _local_plan(self, state, pose, goal):
        min_u, local_plan = self._state_to_local_planner[state].plan(pose, goal)
        if "local_plan" in self._plan_handlers and local_plan is not None:
            self._plan_handlers["local_plan"].update_plan(local_plan)
        return min_u, []

    def _subgoal_navigate(self, pose, goal):
        subgoal_idx = self._subgoal_handler.get_current_goal_idx()
        subgoals = self._subgoal_handler.get_goals()
        triggers = []
        subgoal = subgoals[subgoal_idx]
        achievement_recognizer_sub = self._recognizers["sub"]
        result, reason = achievement_recognizer_sub.detect(pose, subgoal)
        if result:
            get_node().get_logger().info("subgoal was achieved ({}, {})".format(achievement_recognizer_sub.desc, reason))
            is_end, _ = self._subgoal_handler.achieve()
            if is_end:
                get_node().get_logger().info("all subgoals was achieved")
                self._subgoal_handler.clear()
                triggers.append("subgoalsAchieved")
        return subgoal, triggers

    def _plan(self, pose, goals, goal_idx):
        goal = goals[goal_idx]
        state = self._state_machine.state
        triggers = []
        triggers_global = self._global_plan(state, pose, goals, goal_idx)
        if self._subgoal_handler.is_end():
            subgoal = goal
            triggers_subgoal = []
        else:
            subgoal, triggers_subgoal = self._subgoal_navigate(pose, goal)
        min_u, triggers_local = self._local_plan(state, pose, goal=subgoal)
        triggers.extend(triggers_global)
        triggers.extend(triggers_subgoal)
        triggers.extend(triggers_local)
        for trigger in triggers:
            self._state_machine.trigger(trigger)
        return Twist(linear=Vector3(x=min_u[0], y=0, z=0), angular=Vector3(x=0, y=0, z=min_u[1]))

    def _navigate(self, pose, goals, goal_idx):
        goal = goals[goal_idx]
        achievement_recognizer = self._recognizers[goal.data.get("recognizer", "default")]
        result, reason = achievement_recognizer.detect(pose, goal)
        if result:
            get_node().get_logger().info("goal was achieved (Recognizer: {}, Reasons: {})".format(achievement_recognizer.desc, reason))
            cmd_vel = self._achieve()
        else:
            cmd_vel = self._plan(pose, goals, goal_idx)
        return cmd_vel

    def _work(self):
        start_time = get_node().get_clock().now()

        pose = self.pose()
        goal_idx = self._goal_handler.get_current_goal_idx()
        goals = self._goal_handler.get_goals()
        if pose is None or goal_idx is None:
            cmd_vel = Twist(linear=Vector3(x=0, y=0, z=0), angular=Vector3(x=0, y=0, z=0))
        else:
            cmd_vel = self._navigate(pose, goals, goal_idx)

        if cmd_vel is not None:
            try:
                self._cmd_vel_pub.publish(cmd_vel)
            except RuntimeError as e:
                get_node().get_logger().debug("Navigation: {}".format(e))

        dur = get_node().get_clock().now() - start_time
        if dur > self._sleep_dur:
            get_node().get_logger().warn("Control loop took {:.4f} seconds (desired rate is {} Hz)".format(dur, self._controller_frequency))
