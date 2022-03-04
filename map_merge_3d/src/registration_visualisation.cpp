#include <map_merge_3d/map_merging.h>

#include "visualise.h"

#include <pcl/common/transforms.h>
#include <pcl/common/time.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>

using namespace map_merge_3d;

static inline void printPointCloud2Summary(const pcl::PCLPointCloud2 &v)
{
  for (size_t i = 0; i < v.fields.size(); ++i) {
    std::cout << "fields[" << i << "]:" << std::endl;
    std::cout << v.fields[i] << std::endl;
  }

  std::cout << "Num of feature descriptors: " << v.data.size() << std::endl;
}

int main(int argc, char **argv)
{
  std::vector<int> pcd_file_indices =
      pcl::console::parse_file_extension_argument(argc, argv, ".pcd");
  if (pcd_file_indices.size() != 3) {
    pcl::console::print_error("Need exactly 3 input files!\n");
    return -1;
  }

  const MapMergingParams params = MapMergingParams::fromCommandLine(argc, argv);
  std::cout << "params: " << std::endl << params << std::endl;

  PointCloudPtr cloud1_full(new PointCloud);
  PointCloudPtr cloud2_full(new PointCloud);
  PointCloudPtr cloud_grd_truth_full(new PointCloud);
  PointCloudPtr cloud1, cloud2, cloud_grd_truth, cloud1_filtered, cloud2_filtered;

  // load input pcd files
  if (pcl::io::loadPCDFile<PointT>(argv[pcd_file_indices[0]], *cloud1_full) <
          0 ||
      pcl::io::loadPCDFile<PointT>(argv[pcd_file_indices[1]], *cloud2_full) <
          0 ||
      pcl::io::loadPCDFile<PointT>(argv[pcd_file_indices[2]], *cloud_grd_truth_full) <
          0) {
    pcl::console::print_error("Error loading input file!\n");
    return -1;
  }
  std::cout << "loaded pointclouds of sizes: " << cloud1_full->size() << ", "
            << cloud2_full->size() << std::endl;

  pcl::console::print_highlight("Downsampling to working resolution.\n");
  {
    pcl::ScopeTime t("downsampling");
    cloud1 = downSample(cloud1_full, params.resolution);
    cloud2 = downSample(cloud2_full, params.resolution);
    cloud_grd_truth = downSample(cloud_grd_truth_full, params.resolution);
  }
  std::cout << "downsampled clouds to: " << cloud1->size() << ", "
            << cloud2->size() << std::endl;

  visualisePointCloud(cloud1);

  pcl::console::print_highlight("Removing outliers.\n");
  {
    pcl::ScopeTime t("removing outliers");
    cloud1 = removeOutliers(cloud1, params.descriptor_radius,
                            params.outliers_min_neighbours);
    cloud2 = removeOutliers(cloud2, params.descriptor_radius,
                            params.outliers_min_neighbours);
    cloud_grd_truth = removeOutliers(cloud_grd_truth, params.descriptor_radius,
                                      params.outliers_min_neighbours);
  }
  std::cout << "remaining points: " << cloud1->size() << ", " << cloud2->size()
            << std::endl;

  visualisePointCloud(cloud1);

  std::cout << "Inserting transform error into src cloud..." << std::endl;
  cloud1 = insertErrorIntoSourceCloud(cloud1);

  std::cout << "Visualize source and target cloud" << std::endl;
  visualisePointClouds(cloud1, cloud2);


  pcl::console::print_highlight("Attempt to make parallel to ground.\n");
  {
    pcl::ScopeTime t("making parallel to ground");
    Eigen::Affine3f xy_transform_cloud1 = getXYPlaneParallelTransform(cloud1);
    Eigen::Affine3f xy_transform_cloud2 = getXYPlaneParallelTransform(cloud2);

    pcl::transformPointCloud(*cloud1, *cloud1, xy_transform_cloud1);
    pcl::transformPointCloud(*cloud2, *cloud2, xy_transform_cloud2);
  }

  visualisePointCloud(cloud1);

  // pcl::console::print_highlight("Getting floor plane.\n");
  // GroundPlane ground_plane = getGroundPlane(cloud1);
  // visualisePointCloud(ground_plane.points);
  // visualisePointClouds(cloud1, ground_plane.points);

  // pcl::console::print_highlight("Filtering ground away.\n");
  // {
  //   pcl::ScopeTime t("filtering off ground points");
  //   cloud1_filtered = removeGround(cloud1);
  //   cloud2_filtered = removeGround(cloud2);
  // }
  // std::cout << "remaining points: " << cloud1_filtered->size() << ", " << cloud2_filtered->size()
  //           << std::endl;

  // visualisePointCloud(cloud1_filtered);

  // pcl::console::print_highlight("Filtering ground away.\n");
  // {
  //   pcl::ScopeTime t("filtering off ground points");
  //   cloud1_filtered = remove_ground_ransac(cloud1);
  //   cloud2_filtered = remove_ground_ransac(cloud2);
  // }
  // std::cout << "remaining points: " << cloud1_filtered->size() << ", " << cloud2_filtered->size()
  //           << std::endl;

  // visualisePointCloud(cloud1_filtered);

  pcl::console::print_highlight("Filtering points by height.\n");
  {
    pcl::ScopeTime t("filtering points by height");
    cloud1_filtered = filterHeight(cloud1, params.filter_z_min,
                            params.filter_z_max);
    cloud2_filtered = filterHeight(cloud2, params.filter_z_min,
                            params.filter_z_max);
  }
  std::cout << "remaining points: " << cloud1_filtered->size() << ", " << cloud2_filtered->size()
            << std::endl;

  visualisePointCloud(cloud1_filtered);

  /* detect normals */
  pcl::console::print_highlight("Computing normals.\n");
  SurfaceNormalsPtr normals1, normals2;
  {
    pcl::ScopeTime t("normals computation");
    normals1 = computeSurfaceNormals(cloud1_filtered, params.normal_radius);
    normals2 = computeSurfaceNormals(cloud2_filtered, params.normal_radius);
  }

  /* detect keypoints */
  pcl::console::print_highlight("Detecting keypoints.\n");
  PointCloudPtr keypoints1, keypoints2;
  {
    pcl::ScopeTime t("keypoints detection");
    keypoints1 = detectKeypoints(cloud1_filtered, normals1, params.keypoint_type,
                                 params.keypoint_threshold,
                                 params.normal_radius, params.resolution);
    keypoints2 = detectKeypoints(cloud2_filtered, normals2, params.keypoint_type,
                                 params.keypoint_threshold,
                                 params.normal_radius, params.resolution);
  }
  std::cout << "keypoints count: " << keypoints1->size() << ", "
            << keypoints2->size() << std::endl;

  /* compute descriptors */
  pcl::console::print_highlight("Computing descriptors.\n");
  LocalDescriptorsPtr descriptors1, descriptors2;
  {
    pcl::ScopeTime t("descriptors computation");
    descriptors1 = computeLocalDescriptors(cloud1_filtered, normals1, keypoints1,
                                           params.descriptor_type,
                                           params.descriptor_radius);
    descriptors2 = computeLocalDescriptors(cloud2_filtered, normals2, keypoints2,
                                           params.descriptor_type,
                                           params.descriptor_radius);
  }

  std::cout << "extracted descriptors:" << std::endl;
  printPointCloud2Summary(*descriptors1);

  std::cout << "Displaying normals" << std::endl;
  visualiseNormals(cloud1_filtered, normals1);
  std::cout << "Displaying keypoints" << std::endl;
  visualiseKeypoints(cloud1_filtered, keypoints1);

  /* compute correspondences */
  pcl::console::print_highlight("Transform estimation using MATCHING.\n");
  CorrespondencesPtr correspondences, inliers;
  Eigen::Matrix4f transform;
  {
    pcl::ScopeTime t("finding correspondences");
    correspondences = findFeatureCorrespondences(descriptors1, descriptors2,
                                                 params.matching_k, 
                                                 params.correspondence_method);
    transform = estimateTransformFromCorrespondences(keypoints1, keypoints2,
                                                     correspondences, inliers,
                                                     params.inlier_threshold);
  }

  std::cout << "cross-matches count: " << correspondences->size() << std::endl;
  std::cout << "inliers count: " << inliers->size() << std::endl;
  std::cout << "MATCHING est score: "
            << transformScore(cloud1_full, cloud2_full, transform,
                              params.max_correspondence_distance)
            << std::endl;

  std::cout << "initial guess transformation: " << std::endl << transform << std::endl;
  std::cout << "Displaying correspondences" << std::endl;
  visualiseCorrespondences(cloud1, keypoints1, cloud2, keypoints2, inliers);
  visualiseTransform(cloud1, cloud2, transform);

  PointCloudPtr cloud_coarse_src(new PointCloud);
  PointCloudPtr merged_cloud_coarse(new PointCloud);
  pcl::transformPointCloud(*cloud1, *cloud_coarse_src, transform);
  *merged_cloud_coarse = *cloud_coarse_src + *cloud2;

  // float similarity_score_coarse = _similarity(merged_cloud_coarse, cloud_grd_truth, 0.5);
  // std::cout << "coarse merge similarity score: " <<  similarity_score_coarse << std::endl;

  pcl::console::print_highlight("Calculating RMSE for coarse alignment...\n");
  double fitness_score_coarse = getICPFitnessScore(merged_cloud_coarse, cloud_grd_truth);
  std::cout << "coarse merge similarity score: " <<  fitness_score_coarse << std::endl;

  visualisePointClouds(merged_cloud_coarse, cloud_grd_truth);

  // pcl::console::print_highlight("Transform estimation using SAC_IA.\n");
  // Eigen::Matrix4f transform_ia;
  // {
  //   pcl::ScopeTime t("initial alignment");
  //   transform_ia = estimateTransformFromDescriptorsSets(
  //       keypoints1, descriptors1, keypoints2, descriptors2,
  //       params.inlier_threshold, params.max_correspondence_distance,
  //       params.max_iterations);
  // }

  // std::cout << "SAC_IA est score: "
  //           << transformScore(cloud1_full, cloud2_full, transform_ia,
  //                             params.max_correspondence_distance)
  //           << std::endl;

  // visualiseTransform(cloud1, cloud2, transform_ia);

  pcl::console::print_highlight("Refining transform with Fast VGICP.\n");
  {
    pcl::ScopeTime t("Fast VGICP alignment");
    transform = estimateTransformFastVGICP(
        cloud1_filtered, cloud2_filtered, transform, params.max_correspondence_distance,
        params.max_iterations, params.transform_epsilon, 
        params.reg_resolution);
  }

  std::cout << "ICP est score: "
            << transformScore(cloud1_full, cloud2_full, transform,
                              params.max_correspondence_distance)
            << std::endl;
  std::cout << "final transformation: " << std::endl << transform << std::endl;

  visualiseTransform(cloud1, cloud2, transform);

  PointCloudPtr cloud_refined_src(new PointCloud);
  PointCloudPtr merged_cloud_fine(new PointCloud);
  pcl::transformPointCloud(*cloud1, *cloud_refined_src, transform);
  *merged_cloud_fine = *cloud_refined_src + *cloud2;

  // float similarity_score_final = _similarity(merged_cloud_fine, cloud_grd_truth, 0.5);
  // std::cout << "final merge similarity score: " <<  similarity_score_final << std::endl;

  pcl::console::print_highlight("Calculating RMSE for refined alignment...\n");
  double fitness_score_final = getICPFitnessScore(merged_cloud_fine, cloud_grd_truth);
  std::cout << "final merge similarity score: " <<  fitness_score_final << std::endl;
  visualisePointClouds(merged_cloud_fine, cloud_grd_truth);

  // std::vector<double> colorList{255.0, 255.0, 255.0};
  // visualisePointCloud(cloud_grd_truth, colorList);

  return 0;
}
