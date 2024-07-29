# A Global Traversability Mapping Library easily integratable with any SLAM System.

This repository contains the packages responsible for performing global traversability mapping and can be very easily integrated with any SLAM system.

| ![Image 1](images/traversability_map.gif) | ![Image 2](images/gazebo.gif) |
|-------------------------|-------------------------|
| Traversability Map generated with the 3D PointCloud and SLAM | Gazebo world of the map|

## Running the library with ORB-SLAM3 and a Gazebo simulation.

Setup the Gazebo simulation from this repository [Gazebo sim](https://github.com/suchetanrs/scout-husky-gazebo-ros2)

**Important note: Before setting up the below repository. Make sure you checkout on the ```traversability_integration``` branch. This will setup the wrapper along with the traversability mapping during the image build.**

Setup the ORB-SLAM3 Wrapper and the traversability system from this repository [ORB-SLAM3] (https://github.com/suchetanrs/ORB-SLAM3-ROS2-Docker)

## Running the library with your own SLAM package.
### Cloning the repository

```git clone https://github.com/suchetanrs/traversability_mapping```

### Building the library.

Run the following to execute a shell script that installs all the dependencies and build the library.
```bash
cd <cloned repository>
./build.sh
```

### Adjusting the parameters
The parameter file for the library is in ```traversability_mapping/params```

In this, you need to provide the transform between the lidar frame and camera (SLAM) frame.
**NOTE: In case you do not intend to provide a LiDAR pointcloud and wish to send your own pointcloud in camera frame, set this transform to 0**

### CMakeLists
You can include this library in the ```CMakeLists.txt``` of your SLAM by doing the following.
Here we assume your SLAM system is named as ```${PROJECT_NAME}```. Change this as you desire.

A compile flag ```-DWITH_TRAVERSABILITY_MAP``` is added so that you can enable or disable this library while running your SLAM system.

```cmake
find_package(traversability_mapping)
if(traversability_mapping_FOUND)
    include_directories(${PROJECT_NAME}
    ${traversability_mapping_INCLUDE_DIRS}
    )
    ADD_DEFINITIONS("-DWITH_TRAVERSABILITY_MAP")
endif(traversability_mapping_FOUND)

if(traversability_mapping_FOUND)
   target_link_libraries(${PROJECT_NAME}
   type_conversion
   traversabilitySystem
   )
endif(traversability_mapping_FOUND)
```

### Using the Traversability Mapping Library
#### Include headers
-----------------------------
```c++
#ifdef WITH_TRAVERSABILITY_MAP
#include "traversability_mapping/System.hpp"
#endif
```

#### Create an instance (You can also use the smart pointers in C++)
-----------------------------
**NOTE: THERE SHOULD BE ONLY ONE INSTANCE OF THIS CLASS CREATED**

You don't need to worry about running this instance in a seperate thread since it is handled internally by the library.

In case you need access to the traversability class pointer across multiple classes in your SLAM system, pass the newly created pointer to the constructors of the class that needs access. Do not create a new instance for it :p

```c++
// private class member
traversability_mapping::System* pTraversability_;

// new instance
pTraversability_ = new traversability_mapping::System();
```

#### Creating a new map for traversability mapping.
-----------------------------
**NOTE: Only one map needs to be created for a single traversability instance. You can create multiple maps in case the SLAM handles loss of tracking by creating a new map.**

```c++
long unsigned int mapid = 0;
pTraversability_->addNewLocalMap(mapid);
```

#### Adding PointClouds to the Buffer.
-----------------------------
The library provides a buffer to store the 3D pointclouds. In case you initialize a KeyFrame without the 3D Pointcloud, it handles it by assigning the Pointcloud with the closest timestamp to the keyframe.
It is better to do this in the PointCloud callback.

**NOTE: This needs to be done continously**

```c++
// pcl is of type sensor_msgs::msg::PointCloud2
pTraversability_->pushToBuffer(pcl); 
```

#### Adding a new KeyFrame.
-----------------------------
Once a map is created, you can add keyframes to the map to generate traversability. This is done by giving a keyframe ID along with the timestamp and an already created map ID.

The timestamps can be passed either as a ```double``` value of the type ```seconds.nanoseconds``` or an ```unsigned long long``` value of only ```nanoseconds```.

The keyFrame ID (kfID) must be a non-negative integer.
This automatically matches the Keyframe with the PointCloud having the closest timestamp in the buffer.
```c++
pTraversability_->addNewKeyFrame(timestamp, kFID, mapid);
```

In case you also want to assign a pointcloud directly to the keyframe instead of using the Pointclouds in the buffer, you can do it by passing a pointcloud of the type ```sensor_msgs::msg::PointCloud2```
```c++
pTraversability_->addNewKeyFrame(timestamp, kFID, mapid, pointCloud);
```

#### Updating the pose of the KeyFrame.
-----------------------------
The poses can be passed either as ```Eigen::Affine3d``` or ```Sophus::SE3f```.

**NOTE: The keyframe pose must be in the map frame.** 
```c++
// Eigen::Affine3d pose;
pTraversability_->updateKeyFrame(kFID, pose);
```
These poses can be updated continously during optimization and loop closing.


### Visualization
#### Visualizing the Occupancy Grid
-----------------------------
You can do this after every keyframe update step.

There is an individual occupancy map for every local Map you create. The following lines will return the occupancy map of the currently active Local Map (That is, the map to which you are adding new keyframes.)

```c++
if(pTraversability_->getLocalMap() != nullptr)
{
    auto OccupancyGrid = pTraversability_->getLocalMap()->getOccupancyMap();
}
```

#### Visualizing the full GridMap.
-----------------------------
You can do this after every keyframe update step.

```c++
if(pTraversability_->getLocalMap() != nullptr)
{
    auto gridmap = pTraversability_->getLocalMap()->getGridMap();
}
```
