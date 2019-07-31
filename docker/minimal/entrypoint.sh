#!/bin/bash

export HOME=/home/${USER_NAME}
cd ${HOME}

if [ ! -e ${HOME}/.bashrc_profile ]; then
    echo "#!/bin/bash"       >  ${HOME}/.bash_profile
    echo ". ${HOME}/.bashrc" >> ${HOME}/.bash_profile
fi

if [ ! -e ${HOME}/ros2_ws/install ]; then
    source /opt/ros/${ROS_DISTRO}/setup.bash
    mkdir -p ros2_ws/src
    cd ros2_ws/src
    git clone -b ros2 https://github.com/ROBOTIS-GIT/turtlebot3.git
    git clone -b ros2 https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
    git clone -b dashing-devel https://github.com/ros-planning/navigation2.git
    git clone -b dashing https://github.com/ros2/cartographer_ros.git
    git clone -b dashing https://github.com/ros2/geometry2.git
    git clone -b ros2 https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
    git clone -b dashing https://github.com/ros2/geometry2.git
    git clone -b dashing https://github.com/ros2/rviz.git
    cd ..
    colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
    source install/setup.bash
    cd ..
fi

exec $@
