#!/bin/bash

# SOPHUS

cd ThirdParty/Sophus

echo "Configuring and building Thirdparty/Sophus ..."

mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-std=c++14
make -j2
make install

cd ../../..

# Common
mkdir -p traversability_mapping_common/build
cd traversability_mapping_common/build
cmake ..
make
make install

source /opt/ros/$ROS_DISTRO/setup.bash
sudo apt-get install -y ros-$ROS_DISTRO-sensor-msgs
sudo apt-get install -y ros-$ROS_DISTRO-grid-map
sudo apt-get install -y ros-$ROS_DISTRO-nav-msgs

cd ../..

# Core
mkdir -p traversability_mapping/build
cd traversability_mapping/build
cmake ..
make
make install