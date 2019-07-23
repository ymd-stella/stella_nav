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
    cd ros2_ws
    colcon build --symlink-install
    source install/setup.bash
    cd ..
fi

exec $@
