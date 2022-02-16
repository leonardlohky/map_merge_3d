# map_merge_3d

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

The package should build as a standard catkin package. Use rosdep to resolve dependencies in ROS. The package is intended for ROS Melodic and newer, it
should build on all [supported platforms](http://www.ros.org/reps/rep-0003.html#melodic-morenia-may-2018-may-2023) of ROS Melodic. Most notably, the package depends on PCL >= 1.8.

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
## Parameters
Most of the parameter descriptions can be found in the original ROS wiki entry. Due to additional parameters not found in the original package, a complete list of parameters for this map merge package is seen below: <br>

|  Parameter Name |       Meaning     |    Values    |
|:---------------:|:-----------------:|:------------:|
| robot_map_topic | Name of robot map topic without namespaces | string, default: `map` | 
| robot_namespace | Fixed part of the robot map topic. Only topics which contain (anywhere) this string are considered for lookup | string, default: `<empty string>` | 
| merged_map_topic | Topic name where merged map is published | string, default: `map` | 
| world_frame | Frame id (in tf tree) which is assigned to published merged map and used as reference frame for tf transforms | string, default: `world` |
| compositing_rate | Rate in Hz. Basic frequency on which the node merges maps and publishes merged map | double, default: `0.3` |  
| discovery_rate | Rate in Hz. Frequency on which this node discovers new robots (maps) | double, default: `0.05` |  
| estimation_rate | Rate in Hz. Frequency on which this node re-estimates transformations between maps | double, default: `0.01` | 
| publish_tf | Whether to publish estimated transforms in the tf tree | bool, default: `true` |   
| resolution | Resolution used for the registration. | double, default: `0.1` |   
| descriptor_radius | Radius for descriptors computation | double, default: `resolution * 8.0` |  
| outliers_min_neighbours | Minimum number of neighbours for a point to be kept in the map during outliers pruning | int, default: `100` |  
| normal_radius | Radius used for estimating normals | double, default: `resolution * 6.0` |  
| keypoint_type | Type of keypoints used. Possible values are SIFT, HARRIS, ISS, NARF | string, default: `HARRIS` |  
| keypoint_threshold | Keypoints with lower response that this value are pruned | double, default: `5.0` | 
| descriptor_type | Type of descriptors used. Possible values are PFH, FPFH, RSD, SHOT, SC3D | string, default: `FPFH` |
| refine_method | Method to refine initial estimated coarse transform. Possible values are ICP, FAST_GICP, FAST_VGICP | string, default: `FAST_VGICP` |    
| estimation_method | Type of descriptors matching algorithm used. Possible values are MATCHING, SAC_IA, NDT | string, default: `MATCHING` |   
| correspondence_method | Method to find correspondence points. Only applicable for MATCHING estimation method. Possible values are KDTREE, RECIPROCAL | string, default: `KDTREE` |     
| refine_transform | Whether to refine estimated transformation or not | bool, default: `true` |     
| inlier_threshold | Inlier threshold used in RANSAC during estimation | double, default: `resolution * 5.0` |    
| max_correspondence_distance | Maximum distance for matched points to be considered the same point | double, default: `inlier_threshold * 2.0` |  
| max_iterations | Maximum iterations for RANSAC | int, default: `100` |  
| matching_k | Number of the nearest descriptors to consider for matching | int, default: `5` |  
| transform_epsilon | The smallest change allowed until ICP convergence | double, default: `1e-2` |  
| confidence_threshold | Minimum confidence in the pair-wise transform estimate to be included in the map-merging graph. Pair-wise transformations with lower confidence are not considered when computing global transforms | double, default: `10.0` |  
| output_resolution | Resolution of the merged global map | double, default: `0.05` | 
| reg_resolution | Resolution for selected refine method | double, default: `1.5` |  
| reg_step_size | Step size for selected refine method | double, default: `0.1` |   
| filter_z_max | Max point z-height cutoff filter for input pointcloud | double, default: `inf` |   
| filter_z_min | Min point z-height cutoff filter for input pointcloud | double, default: `-inf` |   
| reference_frame | Method to determine which node will be used as the global reference frame. Possible values are FIRST, AUTO. FIRST will use the very first node, while AUTO will try to determine the node automatically. Note that the map will jump if AUTO is used | string, default: `AUTO` |  

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