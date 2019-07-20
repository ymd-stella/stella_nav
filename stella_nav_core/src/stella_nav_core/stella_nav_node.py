#!/usr/bin/env python
import rospy
import time
import sys
from stella_nav_recognizer.recognizer import Recognizer
from stella_nav_core.navigation import Navigation
from stella_nav_core.state_machine import StateMachine
import importlib


def _get_class(plugin_modules, class_name):
    c = None
    for module in plugin_modules:
        if module in sys.modules:
            if hasattr(sys.modules[module], class_name):
                c = getattr(sys.modules[module], class_name)
        else:
            rospy.error("module {} is not found".format(module))
    if c is None:
        raise ValueError("class {} is not found in {}".format(class_name, plugin_modules))
    return c


def _except_key(key, d):
    return {k: d[k]for k in d if k != key}


def _get_recognizer(key, value, plugin_modules):
    value["type"] = key
    return _get_obj({}, value, plugin_modules)


def _get_obj(args, value, plugin_modules):
    c = _get_class(plugin_modules, value["type"])
    assert value["type"] == c.__name__
    cat_args = {}
    cat_args.update(args)
    cat_args.update(value)
    rospy.logdebug("initialize: {}, args: {}".format(c, cat_args))
    try:
        obj = c(**_except_key("type", cat_args))
    except TypeError as e:
        raise TypeError("initializing {}: {}".format(c, e))
    return obj


class RecognizerParser(object):
    def __init__(self, plugin_modules):
        self._plugin_modules = plugin_modules

    def _parse_recognizer(self, to_parse):
        recognizers = []
        for key, value in to_parse.iteritems():
            if "All" == key:
                recognizer = Recognizer.all(self._parse_recognizer(value))
            elif "Any" == key:
                recognizer = Recognizer.any(self._parse_recognizer(value))
            else:
                recognizer = _get_recognizer(key, value, self._plugin_modules)
            recognizers.append(recognizer)
        return recognizers

    def parse_recognizer(self, to_parse):
        recognizers = {}
        for key, item in to_parse.iteritems():
            recognizers_top = self._parse_recognizer(item)
            if len(recognizers_top) == 1:
                recognizers[key] = recognizers_top[0]
            else:
                recognizers[key] = Recognizer.any(recognizers_top)
        return recognizers

def main():
    rospy.init_node("stella_nav_node")
    recognizer_plugin_modules = rospy.get_param("~recognizer_plugin_modules")
    planner_plugin_modules = rospy.get_param("~planner_plugin_modules")
    costmap_plugin_modules = rospy.get_param("~costmap_plugin_modules")
    listener_plugin_modules = rospy.get_param("~listener_plugin_modules")
    observer_plugin_modules = rospy.get_param("~observer_plugin_modules")
    handler_plugin_modules = rospy.get_param("~handler_plugin_modules")
    plugin_modules_list = [
        recognizer_plugin_modules, planner_plugin_modules, costmap_plugin_modules,
        listener_plugin_modules, observer_plugin_modules, handler_plugin_modules]
    for plugin_modules in plugin_modules_list:
        for module in plugin_modules:
            importlib.import_module(module)
    start_nearest_goal = rospy.get_param("~start_nearest_goal", False)
    recognizer_parser = RecognizerParser(recognizer_plugin_modules)
    recognizers = recognizer_parser.parse_recognizer(rospy.get_param("~recognizers"))
    default_achievement_recognizer = recognizers["default"]
    achievement_recognizer_sub = recognizers["sub"]
    recognizers = recognizer_parser.parse_recognizer(rospy.get_param("~recognizers"))
    costmaps = {key: _get_obj({}, value, costmap_plugin_modules) for key, value in rospy.get_param("~costmaps").iteritems()}

    planner_args = {"costmaps": costmaps}
    local_planners = {key: _get_obj(planner_args, value, planner_plugin_modules) for key, value in rospy.get_param("~local_planners").iteritems()}
    state_to_local_planner = {
        key: local_planners[value]
        for key, value in rospy.get_param("~state_to_local_planner").iteritems()
    }
    global_planners = {key: _get_obj(planner_args, value, planner_plugin_modules) for key, value in rospy.get_param(
        "~global_planners").iteritems()}
    state_to_global_planner = {
        key: global_planners[value]
        for key, value in rospy.get_param("~state_to_global_planner").iteritems()
    }
    state_machine = StateMachine(**rospy.get_param(
        "~state_machine",
        dict(states=["initial"], transitions=[])))

    handlers = {key: _get_obj({}, value, handler_plugin_modules) for key, value in rospy.get_param("~handlers").iteritems()}
    plan_handlers = {"local_plan": handlers["local_plan"], "global_plan": handlers["global_plan"]}
    gridmap_handlers = {"robot_costmap": handlers["robot_costmap"], "global_costmap": handlers["global_costmap"]}

    controller_frequency = rospy.get_param("~controller_frequency", 10.0)
    navigation = Navigation(
        state_to_local_planner, state_to_global_planner, state_machine,
        handlers["goal"], handlers["subgoal"], recognizers, plan_handlers, gridmap_handlers["global_costmap"],
        controller_frequency)
    initial_setup = {"obstacle": False, "pose": False}

    observer_args = {"handlers": handlers, "recognizers": recognizers}
    observers = {key: _get_obj(observer_args, value, observer_plugin_modules) for key, value in rospy.get_param("~observers").iteritems()}

    listener_args = {"handlers": handlers, "observers": observers, "initial_setup": initial_setup, "costmaps": costmaps, "local_planners": local_planners, "state_machine": state_machine, "navigation": navigation}
    listeners = {key: _get_obj(listener_args, value, listener_plugin_modules) for key, value in rospy.get_param("~listeners").iteritems()}

    for key, value in rospy.get_param("~add_listener").items():
        for l in value:
            observers[key].add_listener(listeners[l])
    workers = [
        handlers["common_msg"],
        navigation]
    workers.extend(observers.values())
    rospy.loginfo("waiting for initialize")
    while not all(initial_setup.values()):
        time.sleep(1)
        if rospy.is_shutdown():
            return
    if "pose_update" in listeners:
        initial_pose = listeners["pose_update"].initial_pose
    else:
        initial_pose = None
    if start_nearest_goal and initial_pose is not None:
        nearest_goal_idx = goal_handler.get_nearest_goal_idx((initial_pose.pose.position.x, initial_pose.pose.position.y))
        goal_handler.set_current_goal(nearest_goal_idx)
        goals = goal_handler.get_goals()
        rospy.loginfo("nearest goal is {} {}".format(nearest_goal_idx, goals[nearest_goal_idx]))
    rospy.loginfo("send \"start\" trigger to start")
    navigation.start()
    rospy.logdebug("threads started")
    rospy.spin()
    for worker in workers:
        worker.join()

def profile():
    import yappi
    # yappi.set_clock_type("WALL")
    yappi.start()
    main()
    yappi.get_func_stats().strip_dirs().sort("tsub").print_all()
    yappi.get_thread_stats().print_all()

if __name__ == '__main__':
    use_profiler = rospy.get_param("use_profiler", False)
    if use_profiler:
        profile()
    else:
        main()
