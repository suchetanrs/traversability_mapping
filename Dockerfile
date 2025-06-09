FROM osrf/ros:jazzy-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    libboost-all-dev \
    libeigen3-dev

RUN apt-get update && apt-get install -y \
    libpcl-dev \
    ros-$ROS_DISTRO-pcl-*

RUN apt-get update && apt-get install -y \
    libyaml-cpp-dev

COPY ./ /root/other_ws/src/
RUN cd /root/other_ws && rosdep install --from-paths src --ignore-src -r -y --skip-keys sophus
RUN . /opt/ros/$ROS_DISTRO/setup.sh && cd ~/other_ws && colcon build --symlink-install

# Set default command
CMD ["bash"]
