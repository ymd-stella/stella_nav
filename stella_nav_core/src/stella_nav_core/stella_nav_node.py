#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import sys
from stella_nav_recognizer.recognizer import Recognizer
from stella_nav_core.navigation import Navigation
from stella_nav_core.state_machine import StateMachine
import importlib
import threading
from os.path import expanduser
import yaml

def get_node():
    return StellaNavNode.instance()


class StellaNavNode(Node):
    _instance = None
    def __init__(self):
        super().__init__("stella_nav_node")

    @staticmethod
    def instance(cls):
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance


def _get_class(plugin_modules, class_name):
    c = None
    for module in plugin_modules:
        if module in sys.modules:
            if hasattr(sys.modules[module], class_name):
                c = getattr(sys.modules[module], class_name)
        else:
            rclpy.error("module {} is not found".format(module))
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
    get_node().get_logger().debug("initialize: {}, args: {}".format(c, cat_args))
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
    rclpy.init(args=sys.argv)
    node = get_node()
    default_config_filepath = expanduser("~/ros2_ws/src/stella_nav/stella_nav_launch/samples/config.yaml.sample")
    config_filepath = get_node().get_parameter_or("config_filepath", default_config_filepath)
    config = yaml.load(config_filepath)
    recognizer_plugin_modules = config["recognizer_plugin_modules"]
    planner_plugin_modules = config["planner_plugin_modules"]
    costmap_plugin_modules = config["costmap_plugin_modules"]
    listener_plugin_modules = config["listener_plugin_modules"]
    observer_plugin_modules = config["observer_plugin_modules"]
    handler_plugin_modules = config["handler_plugin_modules"]
    plugin_modules_list = [
        recognizer_plugin_modules, planner_plugin_modules, costmap_plugin_modules,
        listener_plugin_modules, observer_plugin_modules, handler_plugin_modules]
    for plugin_modules in plugin_modules_list:
        for module in plugin_modules:
            importlib.import_module(module)
    start_nearest_goal = config.get("start_nearest_goal", False)
    recognizer_parser = RecognizerParser(recognizer_plugin_modules)
    recognizers = recognizer_parser.parse_recognizer(config["recognizers"])
    default_achievement_recognizer = recognizers["default"]
    achievement_recognizer_sub = recognizers["sub"]
    recognizers = recognizer_parser.parse_recognizer(config["recognizers"])
    costmaps = {key: _get_obj({}, value, costmap_plugin_modules) for key, value in config["costmaps"].iteritems()}

    planner_args = {"costmaps": costmaps, "node": node}
    local_planners = {key: _get_obj(planner_args, value, planner_plugin_modules) for key, value in config["local_planners"].iteritems()}
    state_to_local_planner = {
        key: local_planners[value]
        for key, value in config["state_to_local_planner"].iteritems()
    }
    global_planners = {key: _get_obj(planner_args, value, planner_plugin_modules) for key, value in config[
        "global_planners"].iteritems()}
    state_to_global_planner = {
        key: global_planners[value]
        for key, value in config["state_to_global_planner"].iteritems()
    }
    state_machine = StateMachine(**config.get(
        "state_machine",
        dict(states=["initial"], transitions=[])))

    handlers = {key: _get_obj({}, value, handler_plugin_modules) for key, value in config["handlers"].iteritems()}
    plan_handlers = {"local_plan": handlers["local_plan"], "global_plan": handlers["global_plan"]}
    gridmap_handlers = {"robot_costmap": handlers["robot_costmap"], "global_costmap": handlers["global_costmap"]}

    controller_frequency = config.get("controller_frequency", 10.0)
    navigation = Navigation(
        state_to_local_planner, state_to_global_planner, state_machine,
        handlers["goal"], handlers["subgoal"], recognizers, plan_handlers, gridmap_handlers["global_costmap"],
        controller_frequency)
    initial_setup = {"obstacle": False, "pose": False}

    observer_args = {"handlers": handlers, "recognizers": recognizers, "node": node}
    observers = {key: _get_obj(observer_args, value, observer_plugin_modules) for key, value in config["observers"].iteritems()}

    listener_args = {"handlers": handlers, "observers": observers, "initial_setup": initial_setup, "costmaps": costmaps, "local_planners": local_planners, "state_machine": state_machine, "navigation": navigation, "node": node}
    listeners = {key: _get_obj(listener_args, value, listener_plugin_modules) for key, value in config["listeners"].iteritems()}

    for key, value in config["add_listener"].items():
        for l in value:
            observers[key].add_listener(listeners[l])
    workers = [
        handlers["common_msg"],
        navigation]
    workers.extend(observers.values())
    node.get_logger().info("waiting for initialize")
    while not all(initial_setup.values()):
        time.sleep(1)
        if rclpy.is_shutdown():
            return
    if "pose_update" in listeners:
        initial_pose = listeners["pose_update"].initial_pose
    else:
        initial_pose = None
    if start_nearest_goal and initial_pose is not None:
        nearest_goal_idx = goal_handler.get_nearest_goal_idx((initial_pose.pose.position.x, initial_pose.pose.position.y))
        goal_handler.set_current_goal(nearest_goal_idx)
        goals = goal_handler.get_goals()
        node.get_logger().info("nearest goal is {} {}".format(nearest_goal_idx, goals[nearest_goal_idx]))
    node.get_logger().info("send \"start\" trigger to start")
    navigation.start()
    node.get_logger().debug("threads started")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    for worker in workers:
        worker.join()
    node.destroy_node()
    rclpy.shutdown()

def profile():
    import yappi
    # yappi.set_clock_type("WALL")
    yappi.start()
    main()
    columns = {0:("name",80), 1:("ncall", 5), 2:("tsub", 8), 3:("ttot", 8), 4:("tavg",8)}
    with open("yappi_prof.txt", "w") as of:
        yappi.get_func_stats().strip_dirs().sort("tsub").print_all(out=of, columns=columns)
    yappi.get_thread_stats().print_all()

if __name__ == '__main__':
    use_profiler = False
    if use_profiler:
        profile()
    else:
        main()
