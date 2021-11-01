#include <map_merge_3d/features.h>
#include "dispatch_descriptors.h"

#include <algorithm>

#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/point_representation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

namespace map_merge_3d
{
PointCloudPtr downSample(const PointCloudConstPtr &input, double resolution)
{
  pcl::VoxelGrid<PointT> filter;
  filter.setLeafSize(float(resolution), float(resolution), float(resolution));
  filter.setInputCloud(input);

  PointCloudPtr output(new PointCloud);
  filter.filter(*output);

  return output;
}

/* Use a RadiusOutlierRemoval filter to remove all points with too few local
 * neighbors */
PointCloudPtr removeOutliers(const PointCloudConstPtr &input, double radius,
                             int min_neighbors)
{
  pcl::RadiusOutlierRemoval<PointT> filter;
  filter.setInputCloud(input);
  filter.setRadiusSearch(radius);
  filter.setMinNeighborsInRadius(min_neighbors);

  PointCloudPtr output(new PointCloud);
  filter.filter(*output);

  return output;
}

PointCloudPtr generateClusters(const PointCloudConstPtr &input) 
{
  // // Create the segmentation object for the planar model and set all the parameters
  // std::vector<PointCloudPtr> clouds_clustered;
  // pcl::PointCloud<PointT>::Ptr cloud_f (new pcl::PointCloud<PointT>);
  // pcl::PointCloud<PointT>::Ptr cloud_copy (new pcl::PointCloud<PointT>);
  // pcl::SACSegmentation<PointT> seg;
  // pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  // pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());

  // cloud_copy = input;

  // seg.setOptimizeCoefficients (true);
  // seg.setModelType (pcl::SACMODEL_PLANE);
  // seg.setMethodType (pcl::SAC_RANSAC);
  // seg.setMaxIterations (100);
  // seg.setDistanceThreshold (0.02);

  // int nr_points = (int) input->size();
  // int i = 1;
  // while (cloud_copy->size () > 0.3 * nr_points)
  // {
  //   // Segment the largest planar component from the remaining cloud
  //   seg.setInputCloud (input);
  //   seg.segment (*inliers, *coefficients);
  //   if (inliers->indices.size () == 0)
  //   {
  //     std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
  //     break;
  //   }

  //   // Extract the planar inliers from the input cloud
  //   pcl::ExtractIndices<PointT> extract;
  //   extract.setInputCloud (input);
  //   extract.setIndices (inliers);
  //   extract.setNegative (false);

  //   // Get the points associated with the planar surface
  //   extract.filter (*cloud_plane);
  //   std::cout << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;

  //   // Remove the planar inliers, extract the rest
  //   extract.setNegative (true);
  //   extract.filter (*cloud_f);
  //   *cloud_copy = *cloud_f;
  // }

}

static PointCloudPtr detectKeypointsSIFT(const PointCloudConstPtr &points,
                                         double min_scale, int nr_octaves,
                                         int nr_scales_per_octave,
                                         double min_contrast)
{
  pcl::SIFTKeypoint<PointT, pcl::PointWithScale> detector;
  pcl::search::KdTree<PointT>::Ptr tree_n(new pcl::search::KdTree<PointT>());
  detector.setSearchMethod(tree_n);
  detector.setScales(float(min_scale), nr_octaves, nr_scales_per_octave); //min_scale = resolution apram
  detector.setMinimumContrast(float(min_contrast)); //min_contrast = threshold param
  detector.setInputCloud(points);

  pcl::PointCloud<pcl::PointWithScale> keypoints_temp;
  detector.compute(keypoints_temp);
  PointCloudPtr keypoints(new PointCloud);
  pcl::copyPointCloud(keypoints_temp, *keypoints);

  return keypoints;
}

static PointCloudPtr detectKeypointsHarris(const PointCloudConstPtr &points,
                                           const SurfaceNormalsPtr &normals,
                                           double threshold, double radius)
{
  pcl::HarrisKeypoint3D<PointT, pcl::PointXYZI> detector;
  detector.setInputCloud(points);
  detector.setNormals(normals);
  detector.setNonMaxSupression(true);
  detector.setRefine(true);
  detector.setThreshold(float(threshold));
  detector.setRadius(float(radius)); //normal_radius

  pcl::PointCloud<pcl::PointXYZI> keypoints_temp;
  detector.compute(keypoints_temp);

  PointCloudPtr keypoints(new PointCloud);
  pcl::copyPointCloud(keypoints_temp, *keypoints);

  return keypoints;
}

PointCloudPtr detectKeypoints(const PointCloudConstPtr &points,
                              const SurfaceNormalsPtr &normals, Keypoint type,
                              double threshold, double radius,
                              double resolution)
{
  switch (type) {
    case Keypoint::SIFT:
      return detectKeypointsSIFT(points, resolution, 3, 4, threshold);
    case Keypoint::HARRIS:
      return detectKeypointsHarris(points, normals, threshold, radius);
  }
}

/* functor class implementation for specific descriptor type  */
template <typename DescriptorExtractor, typename DescriptorT>
static LocalDescriptorsPtr
computeLocalDescriptors(const PointCloudConstPtr &points,
                        const SurfaceNormalsPtr &normals,
                        const PointCloudPtr &keypoints, double feature_radius)
{
  DescriptorExtractor descriptor;
  descriptor.setRadiusSearch(feature_radius); // descriptor_radius param
  descriptor.setSearchSurface(points);
  descriptor.setInputNormals(normals);
  descriptor.setInputCloud(keypoints);

  // Compute the features
  typename pcl::PointCloud<DescriptorT>::Ptr descriptors(
      new pcl::PointCloud<DescriptorT>);
  descriptor.compute(*descriptors); //"input_ is empty!" ERROR OCCURS HERE

  // remove invalid descriptors (it might not be possible to compute descriptors
  // for all keypoints)

  // find invalid points
  pcl::DefaultPointRepresentation<DescriptorT> point_rep;
  pcl::IndicesPtr invalid_indices(new std::vector<int>);
  int i = 0;
  for (auto d : *descriptors) {
    if (!point_rep.isValid(d)) {
      invalid_indices->push_back(i);
    }
    ++i;
  }

  // remove invalid descriptors
  pcl::ExtractIndices<DescriptorT> filter;
  filter.setInputCloud(descriptors);
  filter.setIndices(invalid_indices);
  filter.setNegative(true);
  filter.filter(*descriptors);

  // filter also keypoints to keep keypoints and the descriptors synchronized
  pcl::ExtractIndices<PointT> filter_keypoints;
  filter_keypoints.setInputCloud(keypoints);
  filter_keypoints.setIndices(invalid_indices);
  filter_keypoints.setNegative(true);
  filter_keypoints.filter(*keypoints);

  assert(keypoints->size() == descriptors->size());

  // convert to PointCloud2 which is able to hold any descriptors data
  LocalDescriptorsPtr result(new LocalDescriptors);
  pcl::toPCLPointCloud2(*descriptors, *result);

  return result;
}

LocalDescriptorsPtr computeLocalDescriptors(const PointCloudConstPtr &points,
                                            const SurfaceNormalsPtr &normals,
                                            const PointCloudPtr &keypoints,
                                            Descriptor descriptor,
                                            double feature_radius)
{
  // this will be dispatched for all descriptors type
  auto functor = [&](auto descriptor_type) {
    return computeLocalDescriptors<
        typename decltype(descriptor_type)::Estimator,
        typename decltype(descriptor_type)::PointType>(
        points, normals, keypoints, feature_radius);
  }; // create functor class in line 98
  return dispatchForEachDescriptor(descriptor, functor);
}

SurfaceNormalsPtr computeSurfaceNormals(const PointCloudConstPtr &input,
                                        double radius)
{
  pcl::NormalEstimation<PointT, NormalT> estimator;
  estimator.setRadiusSearch(radius);
  estimator.setInputCloud(input);

  SurfaceNormalsPtr normals(new SurfaceNormals);
  estimator.compute(*normals);

  return normals;
}

}  // namespace map_merge_3d
