#!/bin/bash

set -e

rm -rf ThirdParty/Sophus/build traversability_mapping_common/build traversability_mapping/build 
rm -rf traversability_ros_interface/build traversability_ros_interface/install traversability_ros_interface/log

# SOPHUS

cd ThirdParty/Sophus

echo "Configuring and building Thirdparty/Sophus ..."

mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-std=c++14
make -j2
sudo make install

cd ../../..

# Common
mkdir -p traversability_mapping_common/build
cd traversability_mapping_common/build
cmake ..
make
sudo make install

cd ../..
source /opt/ros/humble/setup.bash
sudo apt-get install ros-humble-grid-map-core ros-humble-grid-map-rviz* -y

# Core
mkdir -p traversability_mapping/build
cd traversability_mapping/build
cmake ..
make
sudo make install
