cmake_minimum_required(VERSION 2.8.3)
project(stella_nav_listener)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  ros_numpy
)

catkin_python_setup()

catkin_package()
