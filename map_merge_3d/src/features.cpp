#include <map_merge_3d/features.h>
#include "dispatch_descriptors.h"

#include <algorithm>
#include <math.h>

#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/range_image/range_image.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/point_representation.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/progressive_morphological_filter.h>

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

PointCloudPtr filterHeight(const PointCloudConstPtr &input, double z_min,
                           double z_max)
{
  // Create the filtering object
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud (input);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (z_min, z_max);

  PointCloudPtr output(new PointCloud);
  pass.filter(*output);

  return output;
}

PointCloudPtr insertErrorIntoSourceCloud(const PointCloudConstPtr &src_cloud)
{
  Eigen::Affine3f transform_src = Eigen::Affine3f::Identity();

  // Define a translation of 5.0 meters on the x axis.
  transform_src.translation() << 5.0, 0.0, 0.0;

  // The same rotation matrix as before; theta radians around Z axis
  float theta = M_PI/4; // The angle of rotation in radians
  transform_src.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
  std::cout << "Inserted error transform: \n" << transform_src.matrix() << std::endl;

  PointCloudPtr output_src(new PointCloud);
  pcl::transformPointCloud(*src_cloud, *output_src, transform_src);

  return output_src;

}

Eigen::Affine3f getXYPlaneParallelTransform(const PointCloudPtr &input) 
{
  // Find the planar coefficients for floor plane
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr floor_inliers (new pcl::PointIndices);
  pcl::SACSegmentation<PointT> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);
  seg.setInputCloud (input);
  seg.segment (*floor_inliers, *coefficients);
  std::cerr << "Floor Plane Model coefficients: " << coefficients->values[0] << " "
                                    << coefficients->values[1] << " "
                                    << coefficients->values[2] << " "
                                    << coefficients->values[3] << std::endl;

  if (floor_inliers->indices.size() > 512) {
    Eigen::Matrix<float, 1, 3> floor_plane_normal_vector, xy_plane_normal_vector;

    floor_plane_normal_vector[0] = coefficients->values[0];
    floor_plane_normal_vector[1] = coefficients->values[1];
    floor_plane_normal_vector[2] = coefficients->values[2];

    std::cout << floor_plane_normal_vector << std::endl;

    xy_plane_normal_vector[0] = 0.0;
    xy_plane_normal_vector[1] = 0.0;
    xy_plane_normal_vector[2] = 1.0;

    std::cout << xy_plane_normal_vector << std::endl;

    Eigen::Vector3f rotation_vector = xy_plane_normal_vector.cross(floor_plane_normal_vector);
    float theta = -atan2(rotation_vector.norm(), xy_plane_normal_vector.dot(floor_plane_normal_vector));

    Eigen::Affine3f xy_transform = Eigen::Affine3f::Identity();
    xy_transform.translation() << 0, 0, 0;
    xy_transform.rotate (Eigen::AngleAxisf (theta, rotation_vector));
    std::cout << "Floor Plane Transformation matrix: " << std::endl << xy_transform.matrix() << std::endl;
    
    return xy_transform;
  } else {
    return Eigen::Affine3f::Identity();
  }

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


static PointCloudPtr detectKeypointsISS(const PointCloudConstPtr &points,
                                        double resolution, double threshold)
{
  pcl::ISSKeypoint3D<PointT, pcl::PointXYZI> iss_detector;
  pcl::search::KdTree<PointT>::Ptr tree_n(new pcl::search::KdTree<PointT>());
  
  iss_detector.setSearchMethod(tree_n);
  iss_detector.setSalientRadius(6 * resolution);
  iss_detector.setNonMaxRadius(4 * resolution);
  iss_detector.setThreshold21(0.975);
  iss_detector.setThreshold32(0.975);
  iss_detector.setMinNeighbors(5);
  iss_detector.setNumberOfThreads(4);
  iss_detector.setInputCloud(points);

  pcl::PointCloud<pcl::PointXYZI> keypoints_temp;
  iss_detector.compute(keypoints_temp);

  PointCloudPtr keypoints(new PointCloud);
  pcl::copyPointCloud(keypoints_temp, *keypoints);

  return keypoints;
}


static PointCloudPtr detectKeypointsNARF(const PointCloudConstPtr &points,
                                        double resolution, double threshold)
{
	float angular_resolution = 0.5f;
	float support_size = 0.2f;

	PointCloudPtr keyPoints_NARF(new PointCloud);

	PointCloud point_cloud = *points;
	pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
	pcl::RangeImage::CoordinateFrame coordinate_frame = \
	pcl::RangeImage::CAMERA_FRAME;
	Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity ());

	angular_resolution = pcl::deg2rad(angular_resolution);
	scene_sensor_pose = Eigen::Affine3f (\
	Eigen::Translation3f (point_cloud.sensor_origin_[0], \
			point_cloud.sensor_origin_[1], \
			point_cloud.sensor_origin_[2])) * \
	Eigen::Affine3f (point_cloud.sensor_orientation_);

	// -----Create RangeImage from the PointCloud-----
	float noise_level = 0.0;
	float min_range = 0.0f;
	int border_size = 1;
	boost::shared_ptr<pcl::RangeImage> range_image_ptr (new pcl::RangeImage);

	pcl::RangeImage& range_image = *range_image_ptr;
	range_image.createFromPointCloud(point_cloud, angular_resolution, \
	  pcl::deg2rad(360.0f), pcl::deg2rad(180.0f), \
	  scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);

	range_image.integrateFarRanges(far_ranges);

	// --------------------------------
	// -----Extract NARF keypoints-----
	// --------------------------------
	pcl::RangeImageBorderExtractor range_image_border_extractor;
	pcl::NarfKeypoint narf_keypoint_detector (&range_image_border_extractor);
	narf_keypoint_detector.setRangeImage(&range_image);
	narf_keypoint_detector.getParameters().support_size = support_size;

	pcl::PointCloud<int> keypoint_indices;
	narf_keypoint_detector.compute (keypoint_indices);

	PointCloud keypoints = *keyPoints_NARF;
	keypoints.points.resize(keypoint_indices.points.size ());
	
	for (size_t i=0; i<keypoint_indices.points.size(); ++i)
	{
		keypoints.points[i].getVector3fMap() = \
		range_image.points[keypoint_indices.points[i]].getVector3fMap();
	}

  pcl::copyPointCloud(keypoints, *keyPoints_NARF);

  return keyPoints_NARF;
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
    case Keypoint::ISS:
      return detectKeypointsISS(points, resolution, threshold);
    case Keypoint::NARF:
      return detectKeypointsNARF(points, resolution, threshold);
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
