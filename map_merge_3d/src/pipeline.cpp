#include <map_merge_3d/pipeline.h>



// --------------
// -----Main-----
// --------------
int 
main (int argc, char** argv) {
  // Parse Command Line Arguments
  if (pcl::console::find_argument (argc, argv, "-h") >= 0) {
    printUsage (argv[0]);
    return 0;
  }
  
  // Read pcd file
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>& source_cloud = *source_cloud_ptr;
  pcl::PointCloud<pcl::PointXYZ>& target_cloud = *target_cloud_ptr;
  Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());
  std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument (argc, argv, "pcd");

  if (!pcd_filename_indices.empty ()) {
    std::string src_filename = argv[pcd_filename_indices[0]];
    if (pcl::io::loadPCDFile (src_filename, source_cloud) == -1) {
      cerr << "Was not able to open file \""<<src_filename<<"\".\n";
      printUsage (argv[0]);
      return 0;
    }
    std::string tar_filename = argv[pcd_filename_indices[1]];
    if (pcl::io::loadPCDFile (tar_filename, target_cloud) == -1) {
      cerr << "Was not able to open file \""<<tar_filename<<"\".\n";
      printUsage (argv[0]);
      return 0;
    }
    scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (source_cloud.sensor_origin_[0],
                                                               source_cloud.sensor_origin_[1],
                                                               source_cloud.sensor_origin_[2])) *
                        Eigen::Affine3f (source_cloud.sensor_orientation_);
  }
  else {
    cout << "\nNo *.pcd file given.\n\n";
    return 0;
  }
  
  // Downsample input clouds
  /*  
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.setInputCloud(source_cloud_ptr);
  sor.filter(source_cloud);
  sor.setInputCloud(target_cloud_ptr);
  sor.filter(target_cloud);
  */

  // Remove NaN points from point clouds
  // (this is necessary to avoid a segfault when running ICP)
  std::vector<int> nan_idx;
  pcl::removeNaNFromPointCloud(source_cloud, source_cloud, nan_idx);
  pcl::removeNaNFromPointCloud(target_cloud, target_cloud, nan_idx);

  // Estimate cloud normals
  cout << "Computing source cloud normals\n";
  pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
  pcl::PointCloud<pcl::PointNormal>::Ptr src_normals_ptr (new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>& src_normals = *src_normals_ptr;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_xyz (new pcl::search::KdTree<pcl::PointXYZ>());
  ne.setInputCloud(source_cloud_ptr);
  ne.setSearchMethod(tree_xyz);
  ne.setRadiusSearch(0.05);
  ne.compute(*src_normals_ptr);
  for(size_t i = 0;  i < src_normals.points.size(); ++i) {
      src_normals.points[i].x = source_cloud.points[i].x;
      src_normals.points[i].y = source_cloud.points[i].y;
      src_normals.points[i].z = source_cloud.points[i].z;
  }
  cout << "Source cloud normal contains " << src_normals.points.size () << " points\n";

  cout << "Computing target cloud normals\n";
  pcl::PointCloud<pcl::PointNormal>::Ptr tar_normals_ptr (new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>& tar_normals = *tar_normals_ptr;
  ne.setInputCloud(target_cloud_ptr);
  ne.compute(*tar_normals_ptr);
  for(size_t i = 0;  i < tar_normals.points.size(); ++i) {
      tar_normals.points[i].x = target_cloud.points[i].x;
      tar_normals.points[i].y = target_cloud.points[i].y;
      tar_normals.points[i].z = target_cloud.points[i].z;
  }
  cout << "Target cloud normal contains " << tar_normals.points.size () << " points\n";

  // Estimate the SIFT keypoints
  pcl::SIFTKeypoint<pcl::PointNormal, pcl::PointWithScale> sift;
  pcl::PointCloud<pcl::PointWithScale>::Ptr src_keypoints_ptr (new pcl::PointCloud<pcl::PointWithScale>);
  pcl::PointCloud<pcl::PointWithScale>& src_keypoints = *src_keypoints_ptr;
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree_normal(new pcl::search::KdTree<pcl::PointNormal> ());
  sift.setSearchMethod(tree_normal);
  sift.setScales(min_scale, n_octaves, n_scales_per_octave);
  sift.setMinimumContrast(min_contrast);
  sift.setInputCloud(src_normals_ptr);
  sift.compute(src_keypoints);

  cout << "Found " << src_keypoints.points.size () << " SIFT keypoints in source cloud\n";
 
  pcl::PointCloud<pcl::PointWithScale>::Ptr tar_keypoints_ptr (new pcl::PointCloud<pcl::PointWithScale>);
  pcl::PointCloud<pcl::PointWithScale>& tar_keypoints = *tar_keypoints_ptr;
  sift.setInputCloud(tar_normals_ptr);
  sift.compute(tar_keypoints);

  cout << "Found " << tar_keypoints.points.size () << " SIFT keypoints in target cloud\n";
  
  // Extract FPFH features from SIFT keypoints
  pcl::PointCloud<pcl::PointXYZ>::Ptr src_keypoints_xyz (new pcl::PointCloud<pcl::PointXYZ>);                           
  pcl::copyPointCloud (src_keypoints, *src_keypoints_xyz);
  pcl::FPFHEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::FPFHSignature33> fpfh;
  fpfh.setSearchSurface (source_cloud_ptr);
  fpfh.setInputCloud (src_keypoints_xyz);
  fpfh.setInputNormals (src_normals_ptr);
  fpfh.setSearchMethod (tree_xyz);
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr src_features_ptr (new pcl::PointCloud<pcl::FPFHSignature33>());
  pcl::PointCloud<pcl::FPFHSignature33>& src_features = *src_features_ptr;
  fpfh.setRadiusSearch(0.05);
  fpfh.compute(src_features);
  cout << "Computed " << src_features.size() << " FPFH features for source cloud\n";

  pcl::PointCloud<pcl::PointXYZ>::Ptr tar_keypoints_xyz (new pcl::PointCloud<pcl::PointXYZ>);                           
  pcl::copyPointCloud (tar_keypoints, *tar_keypoints_xyz);
  fpfh.setSearchSurface (target_cloud_ptr);
  fpfh.setInputCloud (tar_keypoints_xyz);
  fpfh.setInputNormals (tar_normals_ptr);
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr tar_features_ptr (new pcl::PointCloud<pcl::FPFHSignature33>());
  pcl::PointCloud<pcl::FPFHSignature33>& tar_features = *tar_features_ptr;
  fpfh.compute(tar_features);
  cout << "Computed " << tar_features.size() << " FPFH features for target cloud\n";
  
  // Compute the transformation matrix for alignment
  Eigen::Matrix4f tform = Eigen::Matrix4f::Identity();
  tform = computeInitialAlignment (src_keypoints_ptr, src_features_ptr, tar_keypoints_ptr,
          tar_features_ptr, min_sample_dist, max_correspondence_dist, nr_iters);
  
  /* Uncomment this code to run ICP 
  tform = refineAlignment (source_cloud_ptr, target_cloud_ptr, tform, max_correspondence_distance,
          outlier_rejection_threshold, transformation_epsilon, max_iterations);
  */
 
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>& transformed_cloud = *transformed_cloud_ptr;
  pcl::transformPointCloud(source_cloud, transformed_cloud, tform);
  cout << "Calculated transformation\n";

  // Create 3D viewer and add point clouds
  pcl::visualization::PCLVisualizer viewer ("3D Viewer");
  viewer.setBackgroundColor (0, 0, 0);
  int v1 (0);
  int v2 (1);
  viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
  viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_cloud_color_handler (source_cloud_ptr, 255, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tar_cloud_color_handler (target_cloud_ptr, 0, 255, 255);
  viewer.addPointCloud (source_cloud_ptr, src_cloud_color_handler, "source cloud v1", v1);
  viewer.addPointCloud (target_cloud_ptr, tar_cloud_color_handler, "target cloud v1", v1);
  viewer.addPointCloud (target_cloud_ptr, tar_cloud_color_handler, "target cloud v2", v2);
  viewer.initCameraParameters ();
  setViewerPose (viewer, scene_sensor_pose);
  
  // Show keypoints in 3D viewer
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithScale> src_keypoints_color_handler (src_keypoints_ptr, 255, 0, 0);
  viewer.addPointCloud<pcl::PointWithScale> (src_keypoints_ptr, src_keypoints_color_handler, "source keypoints", v1);
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "source keypoints");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithScale> tar_keypoints_color_handler (tar_keypoints_ptr, 0, 0, 255);
  viewer.addPointCloud<pcl::PointWithScale> (tar_keypoints_ptr, tar_keypoints_color_handler, "target keypoints", v1);
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "target keypoints");

  // Add transformed point cloud to viewer
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tf_cloud_color_handler (transformed_cloud_ptr, 0, 255, 0);
  viewer.addPointCloud<pcl::PointXYZ> (transformed_cloud_ptr, tf_cloud_color_handler, "initial aligned cloud", v2);
  
  //--------------------
  // -----Main loop-----
  //--------------------
  while (!viewer.wasStopped ()) {
    viewer.spinOnce ();
    pcl_sleep(0.01);
  }
}