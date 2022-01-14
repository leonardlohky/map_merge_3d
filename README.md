# map_merge_3d

[![Build Status](http://build.ros.org/buildStatus/icon?job=Mdev__map_merge__ubuntu_bionic_amd64)](http://build.ros.org/job/Mdev__map_merge__ubuntu_bionic_amd64)

ROS package for merging multiple 3D point cloud maps. Includes octomap occupancy map generation capability.

## Installing
The following ROS packages are required for this map-merge package:
- [octomap_ros](https://github.com/OctoMap/octomap_ros)
- [octomap_msgs](https://github.com/OctoMap/octomap_msgs)
- [fast_gicp](https://github.com/SMRT-AIST/fast_gicp)

The package is released for ROS Melodic. Build it through `catkin_make` process. `git clone` it to your `catkin_ws/src` folder. After that, build the package through `catkin_make`
```
cd ~/catkin_ws/src
git clone https://github.com/leonardlohky/map_merge_3d
cd ..
catkin_make
```

## Building

The package should build as a standard catkin package. Use rosdep to resolve
dependencies in ROS. The package is intended for ROS Melodic and newer, it
should build on all [supported platforms](http://www.ros.org/reps/rep-0003.html#melodic-morenia-may-2018-may-2023)
of ROS Melodic. Most notably, the package depends on PCL >= 1.8.

Master branch is for the latest ROS.

## Wiki

The package's original documentation by the original author can be found at ROS Wiki.
* [map_merge_3d](http://wiki.ros.org/map_merge_3d)

## Execution

### Map Merger
The package contains two executable ROS nodes:
- map_merger_node: For merging of point cloud maps
- octomap_mapper_node: To generate octomap occupancy map from the merged point cloud

A template launch file can be found under `launch/map_merge.launch`, modify it accordingly to suit the application. To execute the process, simply
run the launch file. Remember to source for the workspace if you haven't.
```
roslaunch map_merge_3d map_merge.launch
```
### Registration Visualization
The `registration_visualization.cpp` executable helps to visualises pair-wise transform estimation between 2 maps. It uses PCL visualiser for the visualisation and can be executed via the following line of code:
```
rosrun map_merge_3d registration_visualisation [--param value] map1.pcd map2.pcd

// Example
rosrun map_merge_3d registration_visualisation [--keypoint_type HARRIS --normal_radius 0.3 --filter_z_min 0.3 --filter_z_max 5.0 --keypoint_threshold 0.005] map1.pcd map2.pcd 
```

## Troubleshooting
While running the package, there is a chance that the node will crash with the following error message:
```
[pcl::PFHEstimation::compute] input_ is empty!
[pcl::PFHEstimation::initCompute] Init failed.
[pcl::PFHEstimation::compute] input_ is empty!
[pcl::PFHEstimation::initCompute] Init failed.
[pcl::PFHEstimation::compute] input_ is empty!
[pcl::PFHEstimation::initCompute] Init failed.
```
This is most likely due to the lack of keypoints, or lack of descriptors. E.g., if your point cloud does not have a RGB value, then using SIFT as the keypoint type WILL NOT work and results in this error. Thus, a solution is to switch to HARRIS as the keypoint type.

## Copyright

The package is licensed under BSD license. See respective files for details.

## Acknowledgement

This package is based on the original [map_merge_3d](https://github.com/hrnr/map-merge) package by hrnr.