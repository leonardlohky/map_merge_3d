# map_merge_3d

[![Build Status](http://build.ros.org/buildStatus/icon?job=Mdev__map_merge__ubuntu_bionic_amd64)](http://build.ros.org/job/Mdev__map_merge__ubuntu_bionic_amd64)

ROS package for merging 3D point cloud maps.

Installing
----------

The package is released for ROS Melodic. You can install the package through `sudo apt install`:

```
	sudo apt install ros-${ROS_DISTRO}-map-merge-3d
```
Or if you wish to build it through `catkin_make` for modifications later on, `git clone` it to your `catkin_ws/src` folder. After that, build the package through `catkin_make`

Building
--------

The package should build as a standard catkin package. Use rosdep to resolve
dependencies in ROS. The package is intended for ROS Melodic and newer, it
should build on all [supported platforms](http://www.ros.org/reps/rep-0003.html#melodic-morenia-may-2018-may-2023)
of ROS Melodic. Most notably, the package depends on PCL >= 1.8.

You should use the brach specific for your release i.e. `melodic-devel` for
ROS Melodic. Master branch is for the latest ROS.

WIKI
----

The package is documented at ROS Wiki.
* [map_merge_3d](http://wiki.ros.org/map_merge_3d)


TROUBLESHOOTING
----
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

COPYRIGHT
---------

The package is licensed under BSD license. See respective files for details.
