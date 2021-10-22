#include <map_merge_3d/map_merging.h>

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

  pcl::io::savePCDFileBinary(output_name, *result);

  // Initializing point cloud visualizer
  pcl::visualization::PCLVisualizer::Ptr
  viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer_final->setBackgroundColor (0, 0, 0);

  // Coloring and visualizing transformed input cloud (green).
  std::cout << "Output cloud num of points: " << result->size() << std::endl;
  pcl::visualization::PointCloudColorHandlerCustom<PointT>
  output_color (result, 0, 255, 0);
  viewer_final->addPointCloud<PointT> (result, output_color, "output cloud");
  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "output cloud");

  // Starting visualizer
  viewer_final->addCoordinateSystem (1.0, "global");
  viewer_final->initCameraParameters ();

  // Wait until visualizer window is closed.
  while (!viewer_final->wasStopped ())
  {
    viewer_final->spinOnce (100);
  }

  return 0;
}
