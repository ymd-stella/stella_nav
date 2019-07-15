cmake_minimum_required(VERSION 2.8.3)
project(stella_nav_core)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  jsk_rviz_plugins
  tf2
)

catkin_python_setup()
