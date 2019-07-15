FROM ros:kinetic
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    apt-get update && apt-get install -y \
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
    ros-kinetic-tf \
    ros-kinetic-tf2 \
    ros-kinetic-sensor-msgs \
    ros-kinetic-rviz \
    ros-kinetic-rqt-plot \
    ros-kinetic-jsk-visualization \
    ros-kinetic-turtlebot3-description \
    ros-kinetic-turtlebot3-gazebo \
    ros-kinetic-robot-state-publisher \
    ros-kinetic-laser-filters \
    ros-kinetic-xacro \
    ros-kinetic-gazebo-ros-pkgs \
    ros-kinetic-ros-numpy \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /work
RUN wget https://ompl.kavrakilab.org/install-ompl-ubuntu.sh \
    && sed "s/make$/make -j4/" install-ompl-ubuntu.sh | sed "s/make update_bindings$/make update_bindings -j4/" > install-ompl-ubuntu-mod.sh \
    && chmod u+x install-ompl-ubuntu-mod.sh \
    && ./install-ompl-ubuntu-mod.sh

RUN pip install cython transitions && \
    pip install --upgrade numpy

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