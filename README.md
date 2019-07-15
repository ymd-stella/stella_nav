# stella_nav
Navigation System written in Python (and Cython)

# Getting Started
## Prerequisites
*  docker-compose

## Build and Run
```
cd stella_nav/docker/minimal
../build.sh
../run.sh
```

## Demo
```
roslaunch stella_nav_launch simulation.launch
roslaunch stella_nav_launch navigation.launch config:=${HOME}/catkin_ws/src/stella_nav/stella_nav_launch/samples/config.yaml.sample goal:=${HOME}/catkin_ws/src/stella_nav/stella_nav_launch/samples/goals.yaml.sample
roslaunch stella_nav_launch rviz.launch
rostopic pub trigger std_msgs/String "start"
```

# License
stella_nav is released under the MIT License, see [LICENSE](/LICENSE).
