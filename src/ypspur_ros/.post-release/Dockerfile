ARG ROS_DISTRO=kinetic
FROM ros:${ROS_DISTRO}-ros-core

RUN echo "deb http://packages.ros.org/ros-shadow-fixed/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list
RUN apt-get -qq update \
  && apt-get upgrade -y \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

ARG CACHE_CLEAN

RUN apt-get -qq update \
  && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ypspur-ros \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

ADD https://raw.githubusercontent.com/T-frog/i-cart-mini/master/icart-mini.param /
COPY test.sh /
ENTRYPOINT ["/test.sh"]
