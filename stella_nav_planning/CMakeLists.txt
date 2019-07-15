cmake_minimum_required(VERSION 2.8.3)
project(stella_nav_planning)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
)

catkin_python_setup()

set(OPT_GP ${PROJECT_SOURCE_DIR}/src/stella_nav_planning/global_planner/optimized)
add_custom_target(stella_nav_planning_cython_modules ALL
  SOURCES ${OPT_GP}/ompl_planner_impl.so ${OPT_GP}/ompl_wrapper.so ${OPT_GP}/test_planning.so)
add_custom_command(
  OUTPUT ${OPT_GP}/ompl_planner_impl.so ${OPT_GP}/ompl_wrapper.so ${OPT_GP}/test_planning.so
  DEPENDS ${OPT_GP}/ompl_planner_impl.pyx ${OPT_GP}/ompl.pxd ${OPT_GP}/ompl_wrapper.pyx ${OPT_GP}/setup.py
  COMMAND pwd && ./build_module.sh
  WORKING_DIRECTORY ${OPT_GP})

catkin_package()
