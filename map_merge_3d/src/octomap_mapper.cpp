#include <pcl/console/parse.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>

#include <map_merge_3d/ransac_ground.h>
#include <map_merge_3d/features.h>
#include <map_merge_3d/octomap_mapper.h>

namespace map_merge_3d
{
OctomapMapperParams OctomapMapperParams::fromCommandLine(int argc, char **argv)
{

  OctomapMapperParams params;

  using pcl::console::parse_argument;

  parse_argument(argc, argv, "--resolution", params.resolution);
  parse_argument(argc, argv, "--octomap2D_z_min", params.octomap2D_z_min);
  parse_argument(argc, argv, "--octomap2D_z_max", params.octomap2D_z_max);
  parse_argument(argc, argv, "--octomap3D_z_min", params.octomap3D_z_min);
  parse_argument(argc, argv, "--octomap3D_z_max", params.octomap3D_z_max);

  return params;
}

OctomapMapperParams OctomapMapperParams::fromROSNode(const ros::NodeHandle &n)
{
  OctomapMapperParams params;

  n.getParam("resolution", params.resolution);
  n.getParam("octomap2D_z_min", params.octomap2D_z_min);
  n.getParam("octomap2D_z_max", params.octomap2D_z_max);
  n.getParam("octomap3D_z_min", params.octomap3D_z_min);
  n.getParam("octomap3D_z_max", params.octomap3D_z_max);

  return params;
}

std::ostream &operator<<(std::ostream &stream, const OctomapMapperParams &params)
{
  stream << "resolution: " << params.resolution << std::endl;
  stream << "octomap2D_z_min: " << params.octomap2D_z_min << std::endl;
  stream << "octomap2D_z_max: " << params.octomap2D_z_max << std::endl;
  stream << "octomap3D_z_min: " << params.octomap2D_z_min << std::endl;
  stream << "octomap3D_z_max: " << params.octomap2D_z_max << std::endl;

  return stream;
}

octomap::OcTree octomap2DGenerator(const PointCloudConstPtr &input_cloud,
                                 const OctomapMapperParams &params)
{

  PointCloudPtr filtered_cloud = remove_ground_ransac(input_cloud);

  // create pcl octree - ultra fast
  pcl::octree::OctreePointCloudVoxelCentroid<PointT> octree(params.resolution);
  octree.setInputCloud(filtered_cloud);
  octree.addPointsFromInputCloud();
  std::vector< PointT, Eigen::aligned_allocator<PointT> > voxel_centers;
  octree.getOccupiedVoxelCenters(voxel_centers);

  // create octomap instance from pcl octree
  octomap::OcTree final_octree2D(params.resolution);
  for (const auto & voxel_center: voxel_centers) {
      if (voxel_center.z < params.octomap2D_z_max && voxel_center.z > params.octomap2D_z_min) {
          final_octree2D.updateNode(voxel_center.x,voxel_center.y,voxel_center.z,true,false);
      }
  }
  final_octree2D.updateInnerOccupancy();

  return final_octree2D;

}

octomap::OcTree octomap3DGenerator(const PointCloudConstPtr &input_cloud,
                                 const OctomapMapperParams &params)
{
  // create pcl octree - ultra fast
  pcl::octree::OctreePointCloudVoxelCentroid<PointT> octree(params.resolution);
  octree.setInputCloud(input_cloud);
  octree.addPointsFromInputCloud();
  std::vector< PointT, Eigen::aligned_allocator<PointT> > voxel_centers;
  octree.getOccupiedVoxelCenters(voxel_centers);

  // create octomap instance from pcl octree
  octomap::OcTree final_octree(params.resolution);
  for (const auto & voxel_center: voxel_centers) {
      final_octree.updateNode(voxel_center.x,voxel_center.y,voxel_center.z,true,false);
  }
  final_octree.updateInnerOccupancy();

  return final_octree;

}

}