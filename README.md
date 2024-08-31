# A Global Traversability Mapping Library easily integratable with any SLAM System.

This repository contains the packages responsible for performing global traversability mapping and can be very easily integrated with any SLAM system.

| ![Image 1](images/traversability_map.gif) | ![Image 2](images/gazebo.gif) |
|-------------------------|-------------------------|
| Traversability Map generated with the 3D PointCloud and SLAM | Gazebo world of the map|

## Running the library with ORB-SLAM3 and a Gazebo simulation.

Setup the Gazebo simulation from this repository [Gazebo sim](https://github.com/suchetanrs/scout-husky-gazebo-ros2)

**Important note: Before setting up the below repository. Make sure you checkout on the ```traversability_integration``` branch. This will setup the wrapper along with the traversability mapping during the image build.**

Setup the ORB-SLAM3 Wrapper and the traversability system from this repository [ORB-SLAM3] (https://github.com/suchetanrs/ORB-SLAM3-ROS2-Docker)

## Other information:

To integrate this library with your own SLAM system via CPP, follow this [Wiki](https://github.com/suchetanrs/traversability_mapping/wiki/Integrating-with-your-own-SLAM-System.)