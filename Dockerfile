FROM osrf/ros:humble-desktop-full-jammy

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    libboost-all-dev \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    libpcl-dev \
    ros-humble-pcl-* \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y ros-humble-grid-map \
    sudo \
    && rm -rf /var/lib/apt/lists/*

COPY ./ /root/
# check build without ROS sensor_msgs
RUN cd /root/ && sudo chmod +x clean_build.sh && ./clean_build.sh
RUN . /opt/ros/humble/setup.sh && cd && cd traversability_ros_interface && colcon build

# check build with ROS sensor_msgs
RUN . /opt/ros/humble/setup.sh && cd /root/ && sudo chmod +x clean_build.sh && ./clean_build.sh
RUN . /opt/ros/humble/setup.sh && cd && cd traversability_ros_interface && colcon build

# Set default command
CMD ["bash"]
