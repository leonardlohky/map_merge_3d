#include <map_merge_3d/map_merging.h>
#include "visualise.h"

#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>

#include <pcl/visualization/pcl_visualizer.h>

using namespace map_merge_3d;

int main(int argc, char **argv)
{
  std::vector<int> pcd_file_indices =
      pcl::console::parse_file_extension_argument(argc, argv, ".pcd");
  std::string output_name = "output.pcd";

  if (pcd_file_indices.size() < 2) {
    pcl::console::print_error("Need at least 2 input files!\n");
    return -1;
  }

  const MapMergingParams params = MapMergingParams::fromCommandLine(argc, argv);
  std::cout << "params: " << std::endl << params << std::endl;

  // load input pointclouds
  std::vector<PointCloudConstPtr> clouds;
  for (int idx : pcd_file_indices) {
    PointCloudPtr cloud(new PointCloud);
    auto file_name = argv[idx];
    if (pcl::io::loadPCDFile<PointT>(file_name, *cloud) < 0) {
      pcl::console::print_error("Error loading pointcloud file %s. Aborting.\n",
                                file_name);
      return -1;
    }
    std::cout << "Loaded pointcloud file of " << cloud->size() << " points." << std::endl;
    clouds.push_back(cloud);
  }

  pcl::console::print_highlight("Estimating transforms.\n");

  std::vector<Eigen::Matrix4f> transforms =
      estimateMapsTransforms(clouds, params);

  pcl::console::print_highlight("Estimated transforms:\n");

  for (const auto &transform : transforms) {
    std::cout << transform << std::endl;
  }

  pcl::console::print_highlight("Compositing clouds and writing to "
                                "output.pcd\n");

  PointCloudPtr result =
      composeMaps(clouds, transforms, params.output_resolution);

  visualisePointCloud(result);
  pcl::io::savePCDFileBinary(output_name, *result);

  std::cout << params.do_stage_2 << std::endl;
  if (params.do_stage_2) {
    std::cout << "performing stage 2 merging" << std::endl;
    std::vector<PointCloudPtr> clusters;
    std::vector<std::vector<PointCloudPtr>> cloud_clusters;
    for (auto cloud : clouds) {
      clusters = EuclideanClusterExtraction(cloud);
      cloud_clusters.emplace_back(std::move(clusters));
    }


  PointCloudPtr result_clusters =
      composeMaps(cloud_clusters, transforms, params.output_resolution);

  visualisePointCloud(result_clusters);
  }


  return 0;
}
