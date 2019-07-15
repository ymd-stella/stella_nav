cmake_minimum_required(VERSION 2.8.3)
project(stella_nav_costmap)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
)

catkin_python_setup()

add_custom_target(stella_nav_costmap_cython_modules ALL
  SOURCES ${PROJECT_SOURCE_DIR}/src/stella_nav_costmap/optimized/inflation.so)

add_custom_command(
  OUTPUT ${PROJECT_SOURCE_DIR}/src/stella_nav_costmap/optimized/inflation.so
  COMMAND pwd && python setup.py build_ext --inplace
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/src/stella_nav_costmap)

catkin_package()
