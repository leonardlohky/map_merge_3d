#ifndef MAP_MERGE_OCTOMAP_MAPPER_H_
#define MAP_MERGE_OCTOMAP_MAPPER_H_

#include <ostream>
#include <octomap_ros/conversions.h>
#include <octomap_msgs/conversions.h>
#include <map_merge_3d/typedefs.h>

#include <ros/ros.h>

namespace map_merge_3d
{
/**
 * @defgroup map_merging Map merging
 * @brief High-level map-merging interface.
 * @details High-level interface to estimate transformations between n
 * pointclouds and compositing global map..
 * @{
 */

/**
 * @brief Parameters for map merging high-level interface
 * @details Contains all tunables for estimating transformation between n maps
 * and for compositing the global map
 *
 */
struct OctomapMapperParams {
  double resolution = 0.1;
  double octomap2D_z_min = -std::numeric_limits<double>::infinity();
  double octomap2D_z_max = std::numeric_limits<double>::infinity();
  double octomap3D_z_min = -std::numeric_limits<double>::infinity();
  double octomap3D_z_max = std::numeric_limits<double>::infinity();

  /**
   * @brief Sources parameters from command line arguments
   * @details Uses PCL's command line parser to initialize the parameters.
   * Format is `--param_name <value>`. param_name is the same as the struct
   * member.
   *
   * @param argc arguments count
   * @param argv program arguments
   *
   * @return parameters with values from command line of default values where
   * not provided.
   */
  static OctomapMapperParams fromCommandLine(int argc, char **argv);

  /**
   * @brief Sources parameters from ROS node parameters
   * @details Parameter names are the same as the struct
   * member.
   *
   * @param node ROS node to source parameters from
   * @return parameters with values from ROS params of default values where
   * not provided.
   */
  static OctomapMapperParams fromROSNode(const ros::NodeHandle &node);
};
std::ostream &operator<<(std::ostream &stream, const OctomapMapperParams &params);

/**
 * @brief Generate octomap for 2D occupancy map
 * @details To Generate octomap for 2D occupancy map
 *
 * @param clouds input pointclouds
 * @param params parameters for estimation
 *
 * @return octree
 */
octomap::OcTree octomap2DGenerator(const PointCloudConstPtr &input_cloud,
                                 const OctomapMapperParams &params);

/**
 * @brief Generate octomap for 3D occupancy map
 * @details To Generate octomap for 3D occupancy map
 *
 * @param clouds input pointclouds
 * @param params parameters for estimation
 *
 * @return octree
 */
octomap::OcTree octomap3DGenerator(const PointCloudConstPtr &input_cloud,
                                 const OctomapMapperParams &params);

}

#endif //MAP_MERGE_OCTOMAP_MAPPER_H_