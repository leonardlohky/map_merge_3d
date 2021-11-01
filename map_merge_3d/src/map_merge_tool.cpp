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
  viewer_before (new pcl::visualization::PCLVisualizer ("3D Viewer Before Merging"));
  viewer_before->setBackgroundColor (0, 0, 0);

  pcl::visualization::PCLVisualizer::Ptr
  viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer_final->setBackgroundColor (0, 0, 0);

  // Coloring and visualizing transformed input cloud (red).
  std::cout << "Input cloud 1 num of points: " << clouds[0]->size() << std::endl;
  pcl::visualization::PointCloudColorHandlerCustom<PointT>
  input1_color (clouds[0], 255, 0, 0);
  viewer_before->addPointCloud<PointT> (clouds[0], input1_color, "input1 cloud");
  viewer_before->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "input1 cloud");

  // Coloring and visualizing transformed input cloud (blue).
  std::cout << "Input cloud 2 num of points: " << clouds[1]->size() << std::endl;
  pcl::visualization::PointCloudColorHandlerCustom<PointT>
  input2_color (clouds[1], 0, 0, 255);
  viewer_before->addPointCloud<PointT> (clouds[1], input2_color, "input2 cloud");
  viewer_before->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "input2 cloud");

  // Coloring and visualizing transformed input cloud (green).
  std::cout << "Output cloud num of points: " << result->size() << std::endl;
  pcl::visualization::PointCloudColorHandlerCustom<PointT>
  output_color (result, 0, 255, 0);
  viewer_final->addPointCloud<PointT> (result, output_color, "output cloud");
  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "output cloud");

  // Starting visualizer
  viewer_before->addCoordinateSystem (1.0, "global");
  viewer_before->initCameraParameters ();

  viewer_final->addCoordinateSystem (1.0, "global");
  viewer_final->initCameraParameters ();

  // Wait until visualizer window is closed.
  while (!viewer_before->wasStopped () && !viewer_final->wasStopped ())
  {
    viewer_before->spinOnce (100);
    viewer_final->spinOnce (100);
  }

  return 0;
}
