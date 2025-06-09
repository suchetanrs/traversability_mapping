FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    libboost-all-dev \
    libeigen3-dev

RUN apt-get update && apt-get install -y \
    libpcl-dev \
    ros-humble-pcl-*

RUN apt-get update && apt-get install -y \
    libyaml-cpp-dev

COPY ./ /root/other_ws/src/
RUN cd /root/other_ws && rosdep install --from-paths src --ignore-src -r -y --skip-keys sophus
RUN . /opt/ros/humble/setup.sh && cd ~/other_ws && colcon build --symlink-install

# Set default command
CMD ["bash"]
