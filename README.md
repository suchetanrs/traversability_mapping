# A Global Traversability Mapping Library easily integratable with any SLAM System.

This repository contains the packages responsible for performing realtime global and local traversability mapping and can be very easily integrated with any SLAM system.

| ![Image 1](images/traversability_map.gif) | ![Image 2](images/gazebo.gif) |
|-------------------------|-------------------------|
| Traversability Map generated with the 3D PointCloud and ORB-SLAM 3 | Gazebo world of the map|

## Build status
![Humble Build](https://github.com/suchetanrs/traversability_mapping/actions/workflows/build-humble-image.yml/badge.svg)
![Jazzy Build](https://github.com/suchetanrs/traversability_mapping/actions/workflows/build-jazzy-image.yml/badge.svg)

## Running the library independent of SLAM and with Ground truth poses.

Setup the Gazebo simulation from this repository [Gazebo sim humble](https://github.com/suchetanrs/gz-sim-environment/tree/humble) or [Gazebo sim jazzy](https://github.com/suchetanrs/gz-sim-environment/tree/jazzy)

### Building with colcon (recommended)

Once you are inside the gz-sim-environment docker, run the following:

```cd other_ws/src```

```git clone https://github.com/suchetanrs/traversability_mapping```

```cd .. && rosdep update && rosdep install --from-paths src --ignore-src -r -y --skip-keys sophus```

```colcon build --symlink-install```

```source install/setup.bash```


### Building with cmake

Once you are inside the gz-sim-environment docker, run the following:

```cd && mkdir -p other_ws/src```

```cd other_ws/src```

```git clone https://github.com/suchetanrs/traversability_mapping```

```cd traversability_mapping && sudo chmod +x clean_build.sh && ./clean_build.sh```

```cd ~/other_ws/ && colcon build --symlink-install && source install/setup.bash```

## Launching the mapping

### Method 1: Launch the shell file if you are using the [Gazebo sim humble](https://github.com/suchetanrs/gz-sim-environment/tree/humble) or [Gazebo sim jazzy](https://github.com/suchetanrs/gz-sim-environment/tree/jazzy) repository

```cd ~/other_ws/src/traversability_mapping/```

```sudo chmod +x launch_example.sh && ./launch_example.sh```

### Method 2: Launch manually

To launch the global traversability mapping, run ```ros2 launch traversability_mapping_ros global_gt_traversability_mapping.launch.py```

To launch the local traversability mapping, run
```ros2 launch traversability_mapping_ros local_traversability_mapping.launch.py```

You can now visualize the gridmap and the occupancy map via RViz. To launch rviz run 
```ros2 launch traversability_mapping_ros rviz.launch.py```

## Running the library with ORB-SLAM3 and a Gazebo simulation.

Setup the Gazebo simulation from this repository [Gazebo sim](https://github.com/suchetanrs/gz-sim-environment)

**Important note: Before setting up the below repository. Make sure you checkout on the ```traversability_integration``` branch. This will setup the ORB-SLAM3 wrapper along with the traversability mapping integrations during the image build.**

Setup the ORB-SLAM3 Wrapper and the traversability system from this repository [ORB-SLAM3](https://github.com/suchetanrs/ORB-SLAM3-ROS2-Docker/tree/traversability_integration)

## Other information:

To integrate this library with your own SLAM system via CPP, follow this [Wiki](https://github.com/suchetanrs/traversability_mapping/wiki/Integrating-with-your-own-SLAM-System.)

This work was done at the PNX Lab at ISAE-SUPAERO, Toulouse, France.
