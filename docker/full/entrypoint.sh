#!/bin/bash

export HOME=/home/${USER_NAME}
cd ${HOME}

if [ ! -e ${HOME}/.bashrc_profile ]; then
    echo "#!/bin/bash"       >  ${HOME}/.bash_profile
    echo ". ${HOME}/.bashrc" >> ${HOME}/.bash_profile
fi

if [ ! -e ${HOME}/catkin_ws/src/CMakeLists.txt ]; then
    source /opt/ros/${ROS_DISTRO}/setup.bash
    mkdir -p catkin_ws/src
    cd catkin_ws/src
    catkin_init_workspace
    cd ..
    catkin_make
    source devel/setup.bash
    cd ..
fi

exec $@
