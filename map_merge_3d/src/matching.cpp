#include <map_merge_3d/matching.h>
#include "dispatch_descriptors.h"

#include <pcl/conversions.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_validation_euclidean.h>
#include <pcl/search/kdtree.h>

#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/fast_vgicp.hpp>

namespace map_merge_3d
{
/**
 * @brief Throws an exception if inputs are not containing valid descriptors set
 * for matching
 *
 */
static inline void
assertDescriptorsPair(const LocalDescriptorsPtr &source_descriptors,
                      const LocalDescriptorsPtr &target_descriptors)
{
  if (source_descriptors->fields.empty() ||
      target_descriptors->fields.empty()) {
    throw std::runtime_error("descriptors must contain at least one field with "
                             "descriptors.");
  }
}

// matches reciprocal correspondences among k-nearest matches
template <typename DescriptorT>
static CorrespondencesPtr
findFeatureCorrespondencesKDTree(const LocalDescriptorsPtr &source_descriptors_,
                                 const LocalDescriptorsPtr &target_descriptors_,
                                 size_t k)
{
  typedef pcl::PointCloud<DescriptorT> DescriptorsPointCLoud1;

  typename DescriptorsPointCLoud1::Ptr source_descriptors(
      new DescriptorsPointCLoud1);
  pcl::fromPCLPointCloud2(*source_descriptors_, *source_descriptors);
  typename DescriptorsPointCLoud1::Ptr target_descriptors(
      new DescriptorsPointCLoud1);
  pcl::fromPCLPointCloud2(*target_descriptors_, *target_descriptors);

  CorrespondencesPtr result(new Correspondences);
  result->reserve(source_descriptors->size());

  // Use a KdTree to search for the nearest matches in feature space
  pcl::search::KdTree<DescriptorT> target_search;
  target_search.setInputCloud(target_descriptors);
  target_search.setSortedResults(true);

  pcl::search::KdTree<DescriptorT> source_search;
  source_search.setInputCloud(source_descriptors);
  source_search.setSortedResults(true);

  // forward search results
  std::vector<int> k_indices(k);
  std::vector<float> k_squared_distances(k);
  // backward search results
  std::vector<int> k_indices_back(k);
  std::vector<float> k_squared_distances_back(k);

  for (size_t i = 0; i < source_descriptors->size(); ++i) {
    // source to target match
    target_search.nearestKSearch(*source_descriptors, int(i), k, k_indices,
                                 k_squared_distances);
    // try to cross-match all k source to target matches
    for (size_t j = 0; j < k; ++j) {
      int match = k_indices[j];
      float dist = k_squared_distances[j];
      // target to source match
      source_search.nearestKSearch(*target_descriptors, match, k,
                                   k_indices_back, k_squared_distances_back);
      // seach for original index in back-matches
      for (int back_i : k_indices_back) {
        if (back_i == int(i)) {
          // we have cross match
          result->emplace_back(i, match, dist);

          /* we stop here. pcl::SampleConsensusModelRegistration can not handle
           * multiple matches per point. The current match should already be the
           * best as the results are sorted. */
          j = k;  // stop back matching
          break;
        }
      }
    }
  }

  return result;
}

// matches reciprocal correspondences
template <typename DescriptorT>
static CorrespondencesPtr
findFeatureCorrespondencesReciprocal(const LocalDescriptorsPtr &source_descriptors_,
                                     const LocalDescriptorsPtr &target_descriptors_)
{
  typedef pcl::PointCloud<DescriptorT> DescriptorsPointCLoud1;

  typename DescriptorsPointCLoud1::Ptr source_descriptors(
      new DescriptorsPointCLoud1);
  pcl::fromPCLPointCloud2(*source_descriptors_, *source_descriptors);
  typename DescriptorsPointCLoud1::Ptr target_descriptors(
      new DescriptorsPointCLoud1);
  pcl::fromPCLPointCloud2(*target_descriptors_, *target_descriptors);

  CorrespondencesPtr result(new Correspondences);
  result->reserve(source_descriptors->size());

  pcl::registration::CorrespondenceEstimation<DescriptorT, DescriptorT> est;
  est.setInputSource(source_descriptors);
  est.setInputTarget(target_descriptors);

  est.determineReciprocalCorrespondences(*result);

  return result;
}

CorrespondencesPtr findFeatureCorrespondences(
    const LocalDescriptorsPtr &source_descriptors,
    const LocalDescriptorsPtr &target_descriptors, size_t k,
    CorrespondenceMethod corr_method)
{
  assertDescriptorsPair(source_descriptors, target_descriptors);

  const std::string &name = source_descriptors->fields[0].name;
  switch (corr_method) {
    case CorrespondenceMethod::KDTREE: {
        auto functor = [&](auto descriptor_type) {
          return findFeatureCorrespondencesKDTree<typename decltype(
              descriptor_type)::PointType>(source_descriptors, target_descriptors, k);
        };
        return dispatchForEachDescriptor(name, functor);
    } break;

    case CorrespondenceMethod::RECIPROCAL: {
      auto functor = [&](auto descriptor_type) {
        return findFeatureCorrespondencesReciprocal<typename decltype(
            descriptor_type)::PointType>(source_descriptors, target_descriptors);
      };
      return dispatchForEachDescriptor(name, functor);
    } break;
  }

}

Eigen::Matrix4f estimateTransformFromCorrespondences(
    const PointCloudPtr &source_keypoints,
    const PointCloudPtr &target_keypoints,
    const CorrespondencesPtr &correspondences, CorrespondencesPtr &inliers,
    double inlier_threshold)
{
  Eigen::Matrix4f result;
  inliers.reset(new Correspondences);

  // to reject wrong correspondences
  pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> ransac;
  ransac.setInputSource(source_keypoints);
  ransac.setInputTarget(target_keypoints);
  ransac.setInlierThreshold(inlier_threshold);
  ransac.setMaximumIterations(10000);
  ransac.getRemainingCorrespondences(*correspondences, *inliers);

	std::cout << "Correspondences Rejection:" << std::endl;
	std::cout << (correspondences->size() - inliers->size()) << \
	" Correspondences Are Rejected: " << correspondences->size() << "->" << \
	inliers->size() << std::endl << std::endl;

  // check if a transform solution can be found. unfortunately there is no better
  // way.
  if (ransac.getBestTransformation().isIdentity()) {
    // ransac failed to find a resonable model
    result.setZero();
    inliers->clear();  // ransac will set this to original matches
    return result;
  }

  pcl::registration::TransformationEstimationSVD<PointT, PointT> svd;
  svd.estimateRigidTransformation(*source_keypoints, *target_keypoints,
                                  *inliers, result);

  return result;
}

template <typename DescriptorT>
static Eigen::Matrix4f estimateTransformFromDescriptorsSets(
    const PointCloudPtr &source_keypoints,
    const LocalDescriptorsPtr &source_descriptors_,
    const PointCloudPtr &target_keypoints,
    const LocalDescriptorsPtr &target_descriptors_, double min_sample_distance,
    double max_correspondence_distance, int max_iterations)
{
  // convert to required PointCloudType
  typedef pcl::PointCloud<DescriptorT> DescriptorsPointCLoud1;
  typename DescriptorsPointCLoud1::Ptr source_descriptors(
      new DescriptorsPointCLoud1);
  pcl::fromPCLPointCloud2(*source_descriptors_, *source_descriptors);
  typename DescriptorsPointCLoud1::Ptr target_descriptors(
      new DescriptorsPointCLoud1);
  pcl::fromPCLPointCloud2(*target_descriptors_, *target_descriptors);

  pcl::SampleConsensusInitialAlignment<PointT, PointT, DescriptorT> estimator;
  estimator.setMinSampleDistance(min_sample_distance);
  estimator.setMaxCorrespondenceDistance(max_correspondence_distance);
  estimator.setMaximumIterations(max_iterations);

  estimator.setInputSource(source_keypoints);
  estimator.setSourceFeatures(source_descriptors);

  estimator.setInputTarget(target_keypoints);
  estimator.setTargetFeatures(target_descriptors);

  PointCloud registration_output;
  estimator.align(registration_output);

  return estimator.getFinalTransformation();
}

Eigen::Matrix4f estimateTransformFromDescriptorsSets(
    const PointCloudPtr &source_keypoints,
    const LocalDescriptorsPtr &source_descriptors,
    const PointCloudPtr &target_keypoints,
    const LocalDescriptorsPtr &target_descriptors, double min_sample_distance,
    double max_correspondence_distance, int max_iterations)
{
  assertDescriptorsPair(source_descriptors, target_descriptors);

  const std::string &name = source_descriptors->fields[0].name;
  auto functor = [&](auto descriptor_type) {
    return estimateTransformFromDescriptorsSets<typename decltype(
        descriptor_type)::PointType>(
        source_keypoints, source_descriptors, target_keypoints,
        target_descriptors, min_sample_distance, max_correspondence_distance,
        max_iterations);
  };
  return dispatchForEachDescriptor(name, functor);
}

Eigen::Matrix4f estimateTransformICP(const PointCloudPtr &source_points,
                                     const PointCloudPtr &target_points,
                                     const Eigen::Matrix4f &initial_guess,
                                     double max_correspondence_distance,
                                     double outlier_rejection_threshold,
                                     int max_iterations,
                                     double transformation_epsilon)
{
  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setMaxCorrespondenceDistance(max_correspondence_distance);
  icp.setRANSACOutlierRejectionThreshold(outlier_rejection_threshold);
  icp.setTransformationEpsilon(transformation_epsilon);
  icp.setMaximumIterations(max_iterations);

  PointCloudPtr source_points_transformed(new PointCloud);
  pcl::transformPointCloud(*source_points, *source_points_transformed,
                           initial_guess);

  icp.setInputSource(source_points);
  icp.setInputTarget(target_points);

  PointCloud registration_output;
  icp.align(registration_output);

  return icp.getFinalTransformation();
}

Eigen::Matrix4f estimateTransformNDT(const PointCloudPtr &source_points,
                                     const PointCloudPtr &target_points,
                                     int max_iterations,
                                     double transformation_epsilon,
                                     double ndt_resolution,
                                     double ndt_step_size)
{
  // Initializing Normal Distributions Transform (NDT).
  pcl::NormalDistributionsTransform<PointT, PointT> ndt;

  // Setting scale dependent NDT parameters
  ndt.setTransformationEpsilon(transformation_epsilon); // Setting minimum transformation difference for termination condition.
  ndt.setStepSize(ndt_step_size);   // Setting maximum step size for More-Thuente line search.
  ndt.setResolution(ndt_resolution);

  // Setting max number of registration iterations.
  ndt.setMaximumIterations(max_iterations);
  ndt.setInputSource(source_points);
  ndt.setInputTarget(target_points);

  PointCloud registration_output;
  ndt.align(registration_output);

  return ndt.getFinalTransformation();
}


Eigen::Matrix4f estimateTransformFastGICP(const PointCloudPtr &source_points,
                                          const PointCloudPtr &target_points,
                                          const Eigen::Matrix4f &initial_guess,
                                          double max_correspondence_distance,
                                          int max_iterations,
                                          double transformation_epsilon) 
{

  fast_gicp::FastGICP<PointT, PointT>::Ptr gicp(new fast_gicp::FastGICP<PointT, PointT>());

  gicp->setNumThreads(0);
  gicp->setTransformationEpsilon(transformation_epsilon);
  gicp->setMaximumIterations(max_iterations);
  gicp->setMaxCorrespondenceDistance(max_correspondence_distance);
  gicp->setCorrespondenceRandomness(20);

  if (!initial_guess.isZero(0)) {
    PointCloudPtr source_points_transformed(new PointCloud);
    pcl::transformPointCloud(*source_points, *source_points_transformed,
                            initial_guess);
    gicp->setInputSource(source_points_transformed);
    gicp->setInputTarget(target_points);
  
    PointCloud registration_output;
    gicp->align(registration_output);

    return gicp->getFinalTransformation() * initial_guess;
  } else {
    std::cout << "Initial guess all zero. Dropping initial guess for refinement." << std::endl;
    gicp->setInputSource(source_points);
    gicp->setInputTarget(target_points);
    
    PointCloud registration_output;
    gicp->align(registration_output);

    return gicp->getFinalTransformation();
  }

}

Eigen::Matrix4f estimateTransformFastVGICP(const PointCloudPtr &source_points,
                                           const PointCloudPtr &target_points,
                                           const Eigen::Matrix4f &initial_guess,
                                           double max_correspondence_distance,
                                           int max_iterations,
                                           double transformation_epsilon,
                                           double reg_resolution)
{
  
  fast_gicp::FastVGICP<PointT, PointT>::Ptr vgicp(new fast_gicp::FastVGICP<PointT, PointT>());

  // vgicp->setResolution(reg_resolution); // default is 1.0
  vgicp->setNumThreads(0);
  vgicp->setTransformationEpsilon(transformation_epsilon);
  vgicp->setMaximumIterations(max_iterations);
  vgicp->setMaxCorrespondenceDistance(max_correspondence_distance);
  vgicp->setCorrespondenceRandomness(20);

  if (!initial_guess.isZero(0)) {
    PointCloudPtr source_points_transformed(new PointCloud);
    pcl::transformPointCloud(*source_points, *source_points_transformed,
                            initial_guess);
    vgicp->setInputSource(source_points_transformed);
    vgicp->setInputTarget(target_points);
  
    PointCloud registration_output;
    vgicp->align(registration_output);

    return vgicp->getFinalTransformation() * initial_guess;
  } else {
    std::cout << "Initial guess all zero. Dropping initial guess for refinement." << std::endl;
    vgicp->setInputSource(source_points);
    vgicp->setInputTarget(target_points);
    
    PointCloud registration_output;
    vgicp->align(registration_output);

    return vgicp->getFinalTransformation();
  }

}

Eigen::Matrix4f estimateTransform(
    const PointCloudPtr &source_points, const PointCloudPtr &source_keypoints,
    const LocalDescriptorsPtr &source_descriptors,
    const PointCloudPtr &target_points, const PointCloudPtr &target_keypoints,
    const LocalDescriptorsPtr &target_descriptors, EstimationMethod method,
    bool refine, RefineMethod refine_method, double inlier_threshold, double max_correspondence_distance,
    int max_iterations, size_t matching_k, double transform_epsilon, 
    double reg_resolution, double reg_step_size,
    CorrespondenceMethod corr_method)
{
  Eigen::Matrix4f transform;

  switch (method) {
    case EstimationMethod::MATCHING: {
      CorrespondencesPtr inliers;
      CorrespondencesPtr correspondences = findFeatureCorrespondences(
          source_descriptors, target_descriptors, matching_k, corr_method);
      transform = estimateTransformFromCorrespondences(
          source_keypoints, target_keypoints, correspondences, inliers,
          inlier_threshold);
    } break;
    case EstimationMethod::SAC_IA: {
      transform = estimateTransformFromDescriptorsSets(
          source_keypoints, source_descriptors, target_keypoints,
          target_descriptors, inlier_threshold, max_correspondence_distance,
          max_iterations);
    } break;
    case EstimationMethod::NDT: {
        transform = estimateTransformNDT(
            source_points, target_points, max_iterations, 
            transform_epsilon, reg_resolution, reg_step_size);
      } break;
  }

  if (refine) {

    switch (refine_method) {
      case RefineMethod::ICP: {
        std::cout << "Using ICP refine method" << std::endl;
        transform = estimateTransformICP(
            source_points, target_points, transform, max_correspondence_distance,
            inlier_threshold, max_iterations, transform_epsilon);
      } break;

      case RefineMethod::FAST_GICP: {
        std::cout << "Using FAST_GICP refine method" << std::endl;
        transform = estimateTransformFastGICP(
            source_points, target_points, transform, 
            max_correspondence_distance, max_iterations, transform_epsilon);
      } break;
      case RefineMethod::FAST_VGICP: {
        std::cout << "Using FAST_VGICP refine method" << std::endl;
        transform = estimateTransformFastVGICP(
            source_points, target_points, transform, 
            max_correspondence_distance, max_iterations, transform_epsilon,
            reg_resolution);
      } break;
    }

  }

  return transform;
}

double transformScore(const PointCloudPtr &source_points,
                      const PointCloudPtr &target_points,
                      const Eigen::Matrix4f &transform, double max_distance)
{
  pcl::registration::TransformationValidationEuclidean<PointT, PointT> validator;
  validator.setMaxRange(max_distance);

  return validator.validateTransformation(source_points, target_points,
                                          transform);
}

float nearestDistance(const TreeT& tree, const PointT& pt)
{
  const int k = 1;
  std::vector<int> indices (k);
  std::vector<float> sqr_distances (k);

  tree.nearestKSearch(pt, k, indices, sqr_distances);

  return sqr_distances[0];
}

// compare cloudB to cloudA
// use threshold for identifying outliers and not considering those for the similarity
// a good value for threshold is 5 * <cloud_resolution>, e.g. 10cm for a cloud with 2cm resolution
float _similarity(const PointCloudConstPtr& cloudA, const PointCloudConstPtr& cloudB, float threshold)
{
  // compare B to A
  int num_outlier = 0;
  pcl::search::KdTree<PointT> tree;
  tree.setInputCloud(cloudA);
  auto sum = std::accumulate(cloudB->begin(), cloudB->end(), 0.0f, [&](auto current_sum, const auto& pt) {
    const auto dist = nearestDistance(tree, pt);

    if(dist < threshold)
    {
      return current_sum + dist;
    }
    else
    {
      num_outlier++;
      return current_sum;
    }
  });

  return sum / (cloudB->size() - num_outlier);
}

double getICPFitnessScore(const PointCloudConstPtr& source_points, const PointCloudConstPtr& target_points) 
{
  pcl::IterativeClosestPoint<PointT, PointT> icp;

  icp.setInputSource(source_points);
  icp.setInputTarget(target_points);

  PointCloud registration_output;
  icp.align(registration_output);

  return icp.getFitnessScore();

}

}  // namespace map_merge_3d

