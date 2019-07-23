#!/bin/bash
source /opt/ros/kinetic/setup.bash
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:$HOME/ORB_SLAM2/Examples/ROS
source $HOME/catkin_ws/devel/setup.bash

LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
HISTSIZE=50000

# set a fancy prompt (non-color, unless we know we "want" color)
case "$TERM" in
    xterm-color|*-256color) color_prompt=yes;;
esac

if [ "$color_prompt" = yes ]; then
    PS1='${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
else
    PS1='${debian_chroot:+($debian_chroot)}\u@\h:\w\$ '
fi
