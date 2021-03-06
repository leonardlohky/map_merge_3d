#include <map_merge_3d/features.h>
#include <map_merge_3d/map_merging.h>
#include "graph.h"

#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>

namespace map_merge_3d
{

std::vector<TransformEstimate> transformsEst_;

MapMergingParams MapMergingParams::fromCommandLine(int argc, char **argv)
{
  MapMergingParams params;

  using pcl::console::parse_argument;

  parse_argument(argc, argv, "--resolution", params.resolution);
  parse_argument(argc, argv, "--descriptor_radius", params.descriptor_radius);
  parse_argument(argc, argv, "--outliers_min_neighbours",
                 params.outliers_min_neighbours);
  parse_argument(argc, argv, "--normal_radius", params.normal_radius);
  std::string keypoint_type;
  parse_argument(argc, argv, "--keypoint_type", keypoint_type);
  if (!keypoint_type.empty()) {
    params.keypoint_type = enums::from_string<Keypoint>(keypoint_type);
  }
  parse_argument(argc, argv, "--keypoint_threshold", params.keypoint_threshold);
  std::string descriptor_type;
  parse_argument(argc, argv, "--descriptor_type", descriptor_type);
  if (!descriptor_type.empty()) {
    params.descriptor_type = enums::from_string<Descriptor>(descriptor_type);
  }
  std::string estimation_method;
  parse_argument(argc, argv, "--estimation_method", estimation_method);
  if (!estimation_method.empty()) {
    params.estimation_method =
        enums::from_string<EstimationMethod>(estimation_method);
  }
  std::string correspondence_method;
  parse_argument(argc, argv, "--correspondence_method", correspondence_method);
  if (!correspondence_method.empty()) {
    params.correspondence_method =
        enums::from_string<CorrespondenceMethod>(correspondence_method);
  }
  parse_argument(argc, argv, "--refine_transform", params.refine_transform);
  std::string refine_method;
  parse_argument(argc, argv, "--refine_method", refine_method);
  if (!refine_method.empty()) {
    params.refine_method =
        enums::from_string<RefineMethod>(refine_method);
  }
  parse_argument(argc, argv, "--inlier_threshold", params.inlier_threshold);
  parse_argument(argc, argv, "--max_correspondence_distance",
                 params.max_correspondence_distance);
  parse_argument(argc, argv, "--max_iterations", params.max_iterations);
  int matching_k = -1;
  parse_argument(argc, argv, "--matching_k", matching_k);
  if (matching_k > 0) {
    params.matching_k = size_t(matching_k);
  }
  parse_argument(argc, argv, "--transform_epsilon", params.transform_epsilon);
  parse_argument(argc, argv, "--confidence_threshold",
                 params.confidence_threshold);
  parse_argument(argc, argv, "--output_resolution", params.output_resolution);
  parse_argument(argc, argv, "--reg_resolution", params.reg_resolution);
  parse_argument(argc, argv, "--reg_step_size", params.reg_step_size);
  parse_argument(argc, argv, "--do_stage_2", params.do_stage_2);
  parse_argument(argc, argv, "--filter_z_min", params.filter_z_min);
  parse_argument(argc, argv, "--filter_z_max", params.filter_z_max);
  parse_argument(argc, argv, "--reference_frame", params.reference_frame);

  return params;
}

MapMergingParams MapMergingParams::fromROSNode(const ros::NodeHandle &n)
{
  MapMergingParams params;

  n.getParam("resolution", params.resolution);
  n.getParam("descriptor_radius", params.descriptor_radius);
  n.getParam("outliers_min_neighbours",
                 params.outliers_min_neighbours);
  n.getParam("normal_radius", params.normal_radius);
  std::string keypoint_type;
  n.getParam("keypoint_type", keypoint_type);
  if (!keypoint_type.empty()) {
    params.keypoint_type = enums::from_string<Keypoint>(keypoint_type);
  }
  n.getParam("keypoint_threshold", params.keypoint_threshold);
  std::string descriptor_type;
  n.getParam("descriptor_type", descriptor_type);
  if (!descriptor_type.empty()) {
    params.descriptor_type = enums::from_string<Descriptor>(descriptor_type);
  }
  std::string estimation_method;
  n.getParam("estimation_method", estimation_method);
  if (!estimation_method.empty()) {
    params.estimation_method =
        enums::from_string<EstimationMethod>(estimation_method);
  }
  std::string correspondence_method;
  n.getParam("correspondence_method", correspondence_method);
  if (!correspondence_method.empty()) {
    params.correspondence_method =
        enums::from_string<CorrespondenceMethod>(correspondence_method);
  }
  n.getParam("refine_transform", params.refine_transform);
  std::string refine_method;
  n.getParam("refine_method", refine_method);
  if (!refine_method.empty()) {
    params.refine_method =
        enums::from_string<RefineMethod>(refine_method);
  }
  n.getParam("inlier_threshold", params.inlier_threshold);
  n.getParam("max_correspondence_distance",
                 params.max_correspondence_distance);
  n.getParam("max_iterations", params.max_iterations);
  int matching_k = -1;
  n.getParam("matching_k", matching_k);
  if (matching_k > 0) {
    params.matching_k = size_t(matching_k);
  }
  n.getParam("transform_epsilon", params.transform_epsilon);
  n.getParam("confidence_threshold",
                 params.confidence_threshold);
  n.getParam("output_resolution", params.output_resolution);
  n.getParam("reg_resolution", params.reg_resolution);
  n.getParam("reg_step_size", params.reg_step_size);
  n.getParam("do_stage_2", params.do_stage_2);
  n.getParam("filter_z_min", params.filter_z_min);
  n.getParam("filter_z_max", params.filter_z_max);
  n.getParam("reference_frame", params.reference_frame);

  return params;
}

std::ostream &operator<<(std::ostream &stream, const MapMergingParams &params)
{
  stream << "resolution: " << params.resolution << std::endl;
  stream << "descriptor_radius: " << params.descriptor_radius << std::endl;
  stream << "outliers_min_neighbours: " << params.outliers_min_neighbours
         << std::endl;
  stream << "normal_radius: " << params.normal_radius << std::endl;
  stream << "keypoint_type: " << params.keypoint_type << std::endl;
  stream << "keypoint_threshold: " << params.keypoint_threshold << std::endl;
  stream << "descriptor_type: " << params.descriptor_type << std::endl;
  stream << "estimation_method: " << params.estimation_method << std::endl;
  stream << "correspondence_method: " << params.correspondence_method << std::endl;
  stream << "refine_transform: " << params.refine_transform << std::endl;
  stream << "refine_method: " << params.refine_method << std::endl;
  stream << "inlier_threshold: " << params.inlier_threshold << std::endl;
  stream << "max_correspondence_distance: "
         << params.max_correspondence_distance << std::endl;
  stream << "max_iterations: " << params.max_iterations << std::endl;
  stream << "matching_k: " << params.matching_k << std::endl;
  stream << "transform_epsilon: " << params.transform_epsilon << std::endl;
  stream << "confidence_threshold: " << params.confidence_threshold << std::endl;
  stream << "output_resolution: " << params.output_resolution << std::endl;
  stream << "reg_resolution: " << params.reg_resolution << std::endl;
  stream << "reg_step_size: " << params.reg_step_size << std::endl;
  stream << "do_stage_2: " << params.do_stage_2 << std::endl;
  stream << "filter_z_min: " << params.filter_z_min << std::endl;
  stream << "filter_z_max: " << params.filter_z_max << std::endl;
  stream << "reference_frame: " << params.reference_frame << std::endl;

  return stream;
}

/**
 * @brief Updates transformation for current pairwise_transforms based on confidence
 * @details Checks whether the confidence score of each pairwise transform
 * exceeds the threshold. If not, use the previous pairwise transform instead.
 * Memorize new current pairwise_transforms.
 *
 * @param pairwise_transforms transforms to look
 * @param confidence_threshold confidence threshold value
 */
static inline void
updatePairwiseTransform(std::vector<TransformEstimate> &pairwise_transforms,
                        double confidence_threshold)
{
  std::vector<TransformEstimate> prev_transformsEst = transformsEst_;

  if (prev_transformsEst.size() == 0) {
    transformsEst_ = pairwise_transforms;
  } else {
    for (auto &est : pairwise_transforms) {
      for (auto &prev_est : prev_transformsEst) {
        if (est.source_idx == prev_est.source_idx && est.target_idx == prev_est.target_idx && est.confidence < confidence_threshold) {
          std::cout << "Pairwise transform between " << est.target_idx << " and " << est.source_idx <<
          " confidence score below threshold, taking previous transform instead\n" << std::endl;
          std::cout << "Old pairwise transform: \n" << est.transform << std::endl;
          est = prev_est;
          std::cout << "New pairwise transform: \n" << est.transform << std::endl;
        }
      }
    }

    transformsEst_ = pairwise_transforms;
  }

}

/**
 * @brief Finds transformation between from and to in pairwise_transforms
 * @details May return either transform present in pairwise_transforms or
 * inverse of suitable transform that represent transform between from and to
 * nodes.
 *
 * @param pairwise_transforms transform to look
 * @param from source index
 * @param to target index
 * @param confidence_threshold confidence threshold value
 * @return Required transform or zero matrix if the transform could not be
 * found.
 */
static inline Eigen::Matrix4f
getTransform(const std::vector<TransformEstimate> &pairwise_transforms,
             size_t from, size_t to, double confidence_threshold)
{

  for (const auto &est : pairwise_transforms) {
    if (est.source_idx == from && est.target_idx == to && est.confidence > confidence_threshold) {
      return est.transform.inverse();
    }
    if (est.source_idx == to && est.target_idx == from && est.confidence > confidence_threshold) {
      return est.transform;
    }
  }

  std::cerr << "Pairwise transform between " << from << " and " << to <<
          " confidence score is below threshold. Unable to include in merged result\n" << std::endl;

  return Eigen::Matrix4f::Zero();
}

static inline std::vector<Eigen::Matrix4f> computeGlobalTransforms(
    const std::vector<TransformEstimate> &pairwise_transforms,
    double confidence_threshold,
    std::string reference_frame_method)
{
  // consider only largest conncted component
  std::vector<TransformEstimate> component =
      largestConnectedComponent(pairwise_transforms, confidence_threshold);

  // find maximum spanning tree
  Graph span_tree;
  std::vector<size_t> span_tree_centers;
  // uses number of inliers as weights
  findMaxSpanningTree(component, span_tree, span_tree_centers, reference_frame_method);

  // size of the largest connected component
  const size_t nodes_count = numberOfNodesInEstimates(pairwise_transforms);
  // index of the node taken as the reference frame
  const size_t reference_frame = span_tree_centers[0];
  // init all transforms as invalid
  std::vector<Eigen::Matrix4f> global_transforms(nodes_count,
                                                 Eigen::Matrix4f::Zero());
  // reference frame always has identity transform
  global_transforms[reference_frame] = Eigen::Matrix4f::Identity();
  // compute global transforms by chaining them together
  span_tree.walkBreadthFirst(
      span_tree_centers[0],
      [&global_transforms, &component, &confidence_threshold](const GraphEdge &edge) {
        global_transforms[edge.to] =
            global_transforms[edge.from] *
            getTransform(component, edge.from, edge.to, confidence_threshold);
      });

  return global_transforms;
}

std::vector<Eigen::Matrix4f>
estimateMapsTransforms(const std::vector<PointCloudConstPtr> &clouds,
                       const MapMergingParams &params)
{
  if (clouds.empty()) {
    return {};
  }
  if (clouds.size() == 1) {
    return {Eigen::Matrix4f::Identity()};
  }

  // per cloud data extracted for transform estimation
  std::vector<PointCloudPtr> clouds_resized;
  std::vector<SurfaceNormalsPtr> normals;
  std::vector<PointCloudPtr> keypoints;
  std::vector<LocalDescriptorsPtr> descriptors;
  clouds_resized.reserve(clouds.size());
  normals.reserve(clouds.size());
  keypoints.reserve(clouds.size());
  descriptors.reserve(clouds.size());

  /* compute per-cloud features */

  // resize clouds to registration resolution
  for (auto &cloud : clouds) {
    PointCloudPtr resized = downSample(cloud, params.resolution);
    clouds_resized.emplace_back(std::move(resized));
  }

  // remove noise, floor and ceiling points (this reduces number of keypoints)
  for (auto &cloud : clouds_resized) {
    cloud = removeOutliers(cloud, params.descriptor_radius,
                           params.outliers_min_neighbours);
    // cloud = remove_ground_ransac(cloud);
    cloud = filterHeight(cloud, params.filter_z_min,
                         params.filter_z_max);
  }

  // compute normals
  for (const auto &cloud : clouds_resized) {
    auto cloud_normals = computeSurfaceNormals(cloud, params.normal_radius);
    normals.emplace_back(std::move(cloud_normals));  // should have the same size as the input cloud->size()
  }

  // detect keypoints
  for (size_t i = 0; i < clouds_resized.size(); ++i) {
    auto cloud_keypoints = detectKeypoints(
        clouds_resized[i], normals[i], params.keypoint_type,
        params.keypoint_threshold, params.normal_radius, params.resolution);
    std::cout << "Found " << cloud_keypoints->size() << " " << params.keypoint_type << " keypoints in cloud" << std::endl;
    keypoints.emplace_back(std::move(cloud_keypoints));
  }

  // Extract feature descriptors from keypoints
  for (size_t i = 0; i < clouds_resized.size(); ++i) {
    auto cloud_descriptors = computeLocalDescriptors(
        clouds_resized[i], normals[i], keypoints[i], params.descriptor_type,
        params.descriptor_radius);
    descriptors.emplace_back(std::move(cloud_descriptors));
  }

  /* estimate pairwise transforms */

  std::vector<TransformEstimate> pairwise_transforms;
  // generate pairs
  for (size_t i = 0; i < clouds.size() - 1; ++i) {
    for (size_t j = i + 1; j < clouds.size(); ++j) {
      if (keypoints[i]->size() > 0 && keypoints[j]->size() > 0) {
        pairwise_transforms.emplace_back(i, j);
      }
    }
  }

  for (auto &estimate : pairwise_transforms) {
    size_t i = estimate.source_idx;
    size_t j = estimate.target_idx;
    estimate.transform = estimateTransform(
        clouds_resized[i], keypoints[i], descriptors[i], clouds_resized[j],
        keypoints[j], descriptors[j], params.estimation_method,
        params.refine_transform, params.refine_method, params.inlier_threshold,
        params.max_correspondence_distance, params.max_iterations,
        params.matching_k, params.transform_epsilon, 
        params.reg_resolution, params.reg_step_size,
        params.correspondence_method);
    estimate.confidence =
        1. / transformScore(clouds_resized[i], clouds_resized[j],
                            estimate.transform,
                            params.max_correspondence_distance);

    std::cout << "Estimation score: " << estimate.confidence << std::endl;

  }

  updatePairwiseTransform(pairwise_transforms, params.confidence_threshold);

  std::vector<Eigen::Matrix4f> global_transforms =
      computeGlobalTransforms(pairwise_transforms, params.confidence_threshold, 
                              params.reference_frame);

  return global_transforms;
}

PointCloudPtr composeMaps(const std::vector<PointCloudConstPtr> &clouds,
                          const std::vector<Eigen::Matrix4f> &transforms,
                          double resolution)
{
  if (clouds.empty()) {
    return nullptr;
  }

  if (clouds.size() != transforms.size()) {
    throw new std::runtime_error("composeMaps: clouds and transforms size must "
                                 "be the same.");
  }

  std::cout << "Attempting to merge " << clouds.size()<< " pointcloud maps" << std::endl;
  PointCloudPtr result(new PointCloud);
  PointCloudPtr cloud_aligned(new PointCloud);
  for (size_t i = 0; i < clouds.size(); ++i) {
    if (transforms[i].isZero()) {
      continue;
    }
    pcl::transformPointCloud(*clouds[i], *cloud_aligned, transforms[i]);
    std::cout << "Pointcloud " << i << " contains " << cloud_aligned->size() << " points" << std::endl;
    *result += *cloud_aligned;
  }

  // voxelize result cloud to required resolution
  result = downSample(result, resolution);

  return result;
}

PointCloudPtr composeMaps(const std::vector<std::vector<PointCloudPtr>> &cloud_clusters,
                          const std::vector<Eigen::Matrix4f> &transforms,
                          double resolution) 
{
  if (cloud_clusters.empty()) {
    return nullptr;
  }

  if (cloud_clusters.size() != transforms.size()) {
    throw new std::runtime_error("composeMaps: clouds and transforms size must "
                                 "be the same.");
  }

  std::cout << "Found " << cloud_clusters.size()<< " clusters for pointcloud" << std::endl;
  PointCloudPtr result(new PointCloud);
  PointCloudPtr cloud_aligned(new PointCloud);
  for (size_t i = 0; i < cloud_clusters.size(); ++i) {
    for (size_t j = 0; j < cloud_clusters[i].size(); j++) {
      if (transforms[i].isZero()) {
        continue;
      }

      pcl::transformPointCloud(*cloud_clusters[i][j], *cloud_aligned, transforms[i]);
      *result += *cloud_aligned;

    }
  }

  // voxelize result cloud to required resolution
  result = downSample(result, resolution);

  return result;

}

}  // namespace map_merge_3d
