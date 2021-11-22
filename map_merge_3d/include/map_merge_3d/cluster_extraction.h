#ifndef MAP_MERGE_CLUSTER_EXTRACTION_H_
#define MAP_MERGE_CLUSTER_EXTRACTION_H_

#include <map_merge_3d/enum.h>
#include <map_merge_3d/typedefs.h>

namespace map_merge_3d
{

std::vector<PointCloudPtr> EuclideanClusterExtraction(const PointCloudConstPtr &cloud);
/**
 * @brief Generates clusters from give pointcloud
 * @details Breaks point cloud into smaller chunks
 *
 * @param input input pointcloud
 * @return vector of pointcloud clusters
 */


}  // namespace map_merge_3d

#endif  // MAP_MERGE_CLUSTER_EXTRACTION_H_
