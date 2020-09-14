ARG ROS_DISTRO=kinetic
FROM ros:${ROS_DISTRO}-ros-core

RUN apt-get -qq update \
  && apt-get install -y --no-install-recommends \
    libxml2-utils \
    python-pip \
    sudo \
    wget \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*
RUN pip install gh-pr-comment catkin_lint

RUN rosdep update \
  && mkdir -p /catkin_ws/src \
  && cd /catkin_ws/src \
  && . /opt/ros/${ROS_DISTRO}/setup.sh \
  && catkin_init_workspace

COPY ./package.xml /catkin_ws/src/mcl_3dl_msgs/package.xml
RUN apt-get -qq update \
  && rosdep install --from-paths /catkin_ws/src --ignore-src --rosdistro=${ROS_DISTRO} -y \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

COPY . /catkin_ws/src/mcl_3dl_msgs
