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

// void insertPointcloudIntoMapImpl(
//     octomap::OcTree octree_,
//     const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {

//   // Then add all the rays from this pointcloud.
//   // We do this as a batch operation - so first get all the keys in a set, then
//   // do the update in batch.
//   octomap::KeySet free_cells, occupied_cells;
//   for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = cloud->begin();
//        it != cloud->end(); ++it) {
//     const octomap::point3d p_G_point(it->x, it->y, it->z);
//     // First, check if we've already checked this.
//     octomap::OcTreeKey key = octree_.coordToKey(p_G_point);

//     if (occupied_cells.find(key) == occupied_cells.end()) {
//       // Check if this is within the allowed sensor range.
//       castRay(p_G_sensor, p_G_point, &free_cells, &occupied_cells);
//     }
//   }

//   // Apply the new free cells and occupied cells from
//   updateOccupancy(&free_cells, &occupied_cells);
// }

// void castRay(const octomap::point3d& sensor_origin,
//                            const octomap::point3d& point,
//                            octomap::KeySet* free_cells,
//                            octomap::KeySet* occupied_cells) {
//   CHECK_NOTNULL(free_cells);
//   CHECK_NOTNULL(occupied_cells);

//   if (params_.sensor_max_range < 0.0 ||
//       (point - sensor_origin).norm() <= params_.sensor_max_range) {
//     // Cast a ray to compute all the free cells.
//     key_ray_.reset();
//     if (octree_->computeRayKeys(sensor_origin, point, key_ray_)) {
//       if (params_.max_free_space == 0.0) {
//         free_cells->insert(key_ray_.begin(), key_ray_.end());
//       } else {
//         for (const auto& key : key_ray_) {
//           octomap::point3d voxel_coordinate = octree_->keyToCoord(key);
//           if ((voxel_coordinate - sensor_origin).norm() <
//                   params_.max_free_space ||
//               voxel_coordinate.z() >
//                   (sensor_origin.z() - params_.min_height_free_space)) {
//             free_cells->insert(key);
//           }
//         }
//       }
//     }
//     // Mark endpoint as occupied.
//     octomap::OcTreeKey key;
//     if (octree_->coordToKeyChecked(point, key)) {
//       occupied_cells->insert(key);
//     }
//   } else {
//     // If the ray is longer than the max range, just update free space.
//     octomap::point3d new_end =
//         sensor_origin +
//         (point - sensor_origin).normalized() * params_.sensor_max_range;
//     key_ray_.reset();
//     if (octree_->computeRayKeys(sensor_origin, new_end, key_ray_)) {
//       if (params_.max_free_space == 0.0) {
//         free_cells->insert(key_ray_.begin(), key_ray_.end());
//       } else {
//         for (const auto& key : key_ray_) {
//           octomap::point3d voxel_coordinate = octree_->keyToCoord(key);
//           if ((voxel_coordinate - sensor_origin).norm() <
//                   params_.max_free_space ||
//               voxel_coordinate.z() >
//                   (sensor_origin.z() - params_.min_height_free_space)) {
//             free_cells->insert(key);
//           }
//         }
//       }
//     }
//   }
// }

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