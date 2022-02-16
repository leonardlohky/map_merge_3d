#include<bits/stdc++.h>

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/transforms.h>

#include <map_merge_3d/typedefs.h>
#include<map_merge_3d/ransac_ground.h>

#include <Eigen/Geometry>

namespace map_merge_3d 
{

GroundPlane getGroundPlane(const PointCloudConstPtr &input)
{
  pcl::SACSegmentation<PointT> seg;
  ModelCoeffPtr coefficients (new pcl::ModelCoefficients);
  PointIndicesPtr ground (new pcl::PointIndices);

  // Set up parameters for our segmentation/ extraction scheme
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE); //only want points perpendicular to a given axis
  seg.setMaxIterations(1000); // this is key (default is 50 and that sucks)
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.2); // keep points within 0.5 m of the plane

  // because we want a specific plane (X-Y Plane)
  Eigen::Vector3f axis = Eigen::Vector3f(0.0,0.0,1.0); //z axis
  seg.setAxis(axis);
  seg.setEpsAngle(30.0f * (M_PI/180.0f)); // plane can be within 30 degrees of X-Y plane
  seg.setProbability(0.95);
  seg.setInputCloud(input);
  seg.segment(*ground, *coefficients);

  // Create the filtering object
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(input);
  extract.setIndices(ground);

  // Extract non-ground returns
  PointCloudPtr points(new PointCloud);
  extract.setNegative(false);
  extract.filter(*points);

  GroundPlane output;
  output.ground = ground;
  output.coefficients = coefficients;
  output.points = points;

  return output;
}

PointCloudPtr removeGround(const PointCloudConstPtr &input)
{

  GroundPlane planeInfo = getGroundPlane(input);

  // to remove points below and within the plane
  PointCloudPtr filtered(new PointCloud);
  for(int i = 0; i < planeInfo.points->size(); i++)
  {
    PointT point = planeInfo.points->points[i];

    if(point.x * planeInfo.coefficients->values[0] + point.y * planeInfo.coefficients->values[1] + 
        point.z *  planeInfo.coefficients->values[2] +  planeInfo.coefficients->values[3] > 0)
    {
      filtered->points.push_back(point);
    }
  }

  return filtered;

}

// This helper function finds indices of points that are considered inliers,
// given a plane description and a condition on distance from the plane.
std::vector<size_t> find_inlier_indices(
    const PointCloudConstPtr &input_cloud_ptr,
    const Plane& plane,
    std::function<bool(float)> condition_z_fn)
{
    typedef Eigen::Transform<float, 3, Eigen::Affine, Eigen::DontAlign> Transform3f;

    auto base_point = plane.base_point;
    auto normal = plane.normal;

    // Before rotation of the coordinate frame we need to relocate the point cloud to
    // the position of base_point of the plane.
    Transform3f world_to_ransac_base = Transform3f::Identity();
    world_to_ransac_base.translate(-base_point);
    PointCloudPtr ransac_base_cloud_ptr (new PointCloud);
    pcl::transformPointCloud(*input_cloud_ptr, *ransac_base_cloud_ptr, world_to_ransac_base);

    // We are going to use a quaternion to determine the rotation transform
    // which is required to rotate a coordinate system that plane's normal
    // becomes aligned with Z coordinate axis.
    auto rotate_to_plane_quat = Eigen::Quaternionf::FromTwoVectors(
        normal,
        Eigen::Vector3f::UnitZ()
    ).normalized();

    // Now we can create a rotation transform and align the cloud that
    // the candidate plane matches XY plane.
    Transform3f ransac_base_to_ransac = Transform3f::Identity();
    ransac_base_to_ransac.rotate(rotate_to_plane_quat);
    PointCloudPtr aligned_cloud_ptr (new PointCloud);
    pcl::transformPointCloud(*ransac_base_cloud_ptr, *aligned_cloud_ptr, ransac_base_to_ransac);

    // Once the point cloud is transformed into the plane coordinates,
    // We can apply a simple criterion on Z coordinate to find inliers.
    std::vector<size_t> indices;
    for (size_t i_point = 0; i_point < aligned_cloud_ptr->size(); i_point++)
    {
        const auto &p = (aligned_cloud_ptr->points)[i_point];
        if (condition_z_fn(p.z))
        {
            indices.push_back(i_point);
        }
    }
    return indices;
}


// This function performs plane detection with RANSAC sampling of planes
// that lie on triplets of points randomly sampled from the cloud.
// Among all trials the plane that is picked is the one that has the highest
// number of inliers. Inlier points are then removed as belonging to the ground.
PointCloudPtr remove_ground_ransac(const PointCloudConstPtr &input_cloud_ptr)
{
    // Threshold for rough point dropping by Z coordinate (meters)
    const float rough_filter_thr = 0.5f;
    // How much to decimate the input cloud for RANSAC sampling and inlier counting
    const size_t decimation_rate = 10;

    // Tolerance threshold on the distance of an inlier to the plane (meters)
    const float ransac_tolerance = 0.1f;
    // After the final plane is found this is the threshold below which all
    // points are discarded as belonging to the ground.
    const float remove_ground_threshold = 0.2f;

    // To reduce the number of outliers (non-ground points) we can roughly crop
    // the point cloud by Z coordinate in the range (-rough_filter_thr, rough_filter_thr).
    // Simultaneously we perform decimation of the remaining points since the full
    // point cloud is excessive for RANSAC.
    std::mt19937::result_type decimation_seed = 41;
    std::mt19937 rng_decimation(decimation_seed);
    auto decimation_gen = std::bind(
        std::uniform_int_distribution<size_t>(0, decimation_rate), rng_decimation);

    PointCloudPtr filtered_ptr(new PointCloud);
    for (auto &p : input_cloud_ptr->points)
    {
        if ((p.z > -rough_filter_thr) && (p.z < rough_filter_thr))
        {
            // Use random number generator to avoid introducing patterns
            // (which are possible with structured subsampling
            // like picking each Nth point).
            if (decimation_gen() == 0)
            {
                filtered_ptr->points.push_back(p);
            }
        }
    }

    // We need a random number generator for sampling triplets of points.
    std::mt19937::result_type sampling_seed = 42;
    std::mt19937 sampling_rng(sampling_seed);
    auto random_index_gen = std::bind(
        std::uniform_int_distribution<size_t>(0, filtered_ptr->size()), sampling_rng);

    // Number of RANSAC trials
    const size_t num_iterations = 25;
    // The best plane is determined by a pair of (number of inliers, plane specification)
    typedef std::pair<size_t, Plane> BestPair;
    auto best = std::unique_ptr<BestPair>();
    for (size_t i_iter = 0; i_iter < num_iterations; i_iter++)
    {
        // Sample 3 random points.
        // pa is special in the sense that is becomes an anchor - a base_point of the plane
        Eigen::Vector3f pa = (*filtered_ptr)[random_index_gen()].getVector3fMap();
        Eigen::Vector3f pb = (*filtered_ptr)[random_index_gen()].getVector3fMap();
        Eigen::Vector3f pc = (*filtered_ptr)[random_index_gen()].getVector3fMap();

        // Here we figure out the normal to the plane which can be easily calculated
        // as a normalized cross product.
        auto vb = pb - pa;
        auto vc = pc - pa;
        Eigen::Vector3f normal = vb.cross(vc).normalized();

        // Flip the normal if points down
        if (normal.dot(Eigen::Vector3f::UnitZ()) < 0)
        {
            normal = -normal;
        }

        Plane plane{pa, normal};

        // Call find_inlier_indices to retrieve inlier indices.
        // We will need only the number of inliers.
        auto inlier_indices = find_inlier_indices(filtered_ptr, plane,
            [&ransac_tolerance](float z) -> bool {
                return (z >= -ransac_tolerance) && (z <= ransac_tolerance);
            });

        // If new best plane is found, update the best
        bool found_new_best = false;
        if (best)
        {
            if (inlier_indices.size() > best->first)
            {
                found_new_best = true;
            }
        }
        else
        {
            // For the first trial update anyway
            found_new_best = true;
        }

        if (found_new_best)
        {
            best = std::unique_ptr<BestPair>(new BestPair{inlier_indices.size(), plane});
        }
    }

    // For the best plane filter out all the points that are
    // below the plane + remove_ground_threshold.
    PointCloudPtr cloud_no_ground_ptr (new PointCloud);
    if (best)
    {
        auto inlier_indices = find_inlier_indices(input_cloud_ptr, best->second,
            [&remove_ground_threshold](float z) -> bool {
                return z <= remove_ground_threshold;
            });
        std::unordered_set<size_t> inlier_set(inlier_indices.begin(), inlier_indices.end());
        for (size_t i_point = 0; i_point < input_cloud_ptr->size(); i_point++)
        {
            bool extract_non_ground = true;
            if ((inlier_set.find(i_point) == inlier_set.end()) == extract_non_ground)
            {
                const auto &p = (input_cloud_ptr->points)[i_point];
                cloud_no_ground_ptr->points.push_back(p);
            }
        }
    }
    else
    {
        *cloud_no_ground_ptr = *input_cloud_ptr;
    }

    return cloud_no_ground_ptr;
}


} // namespace map_merge_3d