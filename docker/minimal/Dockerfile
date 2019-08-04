FROM ubuntu:bionic
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y tzdata gnupg2 lsb-release && \
    rm -rf /var/lib/apt/lists/*

ARG ROS_DISTRO
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list' && \
    apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-desktop \
    x11-apps \
    tmux \
    build-essential \
    ccache \
    cmake \
    git \
    pkg-config \
    wget \
    unzip \
    python-pip \
    python3-dev \
    python3-pip \
    mlocate \
    sudo \
    gdb \
    libeigen3-dev \
    libboost-dev \
    libode-dev \
    nomacs \
    graphviz-dev \
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-tf2 \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-rqt-plot \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-cartographer \
    ros-${ROS_DISTRO}-behaviortree-cpp \
    ros-${ROS_DISTRO}-test-interface-files \
    ros-${ROS_DISTRO}-rcl-interfaces \
    ros-${ROS_DISTRO}-test-msgs \
    libsdl-dev libsdl-image1.2-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /work
RUN wget https://ompl.kavrakilab.org/install-ompl-ubuntu.sh \
    && sed "s/make$/make -j4/" install-ompl-ubuntu.sh | sed "s/make update_bindings$/make update_bindings -j4/" > install-ompl-ubuntu-mod.sh \
    && chmod u+x install-ompl-ubuntu-mod.sh \
    && ./install-ompl-ubuntu-mod.sh

RUN pip3 install cython transitions pygraphviz pkgconfig && \
    pip3 install --upgrade numpy

ARG group_id
ARG group_name
ARG user_id
ARG user_name
RUN groupadd -g ${group_id} ${group_name}
RUN useradd --gid ${group_id} --uid ${user_id} -M -d /home/${user_name} ${user_name} -s /bin/bash
RUN echo "${user_name}:${user_name}" | chpasswd

COPY entrypoint.sh /work/entrypoint.sh
RUN chmod +x /work/entrypoint.sh

ENTRYPOINT ["/work/entrypoint.sh"]
CMD ["bash"]