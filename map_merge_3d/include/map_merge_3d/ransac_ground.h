#ifndef MAP_MERGE_RANSAC_GROUND_H_
#define MAP_MERGE_RANSAC_GROUND_H_

#include <map_merge_3d/typedefs.h>

namespace map_merge_3d 
{

// A plane is represented with a point on the plane (base_point)
// and a normal vector to the plane.
struct Plane
{
    Eigen::Vector3f base_point;
    Eigen::Vector3f normal;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct GroundPlane {
    PointIndicesPtr ground;
    ModelCoeffPtr coefficients;
    PointCloudPtr points;
};

GroundPlane getGroundPlane(const PointCloudConstPtr &input);
PointCloudPtr removeGround(const PointCloudConstPtr &input);

// This helper function finds indices of points that are considered inliers,
// given a plane description and a condition on distance from the plane.
std::vector<size_t> find_inlier_indices(
    const PointCloudConstPtr &input_cloud_ptr,
    const Plane& plane,
    std::function<bool(float)> condition_z_fn);


// This function performs plane detection with RANSAC sampling of planes
// that lie on triplets of points randomly sampled from the cloud.
// Among all trials the plane that is picked is the one that has the highest
// number of inliers. Inlier points are then removed as belonging to the ground.
PointCloudPtr remove_ground_ransac(const PointCloudConstPtr &input_cloud_ptr);

} // namespace map_merge_3d

#endif //MAP_MERGE_RANSAC_GROUND_H_