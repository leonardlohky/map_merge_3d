#ifndef MAP_MERGE_OCTOMAP_MAPPER_NODE_H_
#define MAP_MERGE_OCTOMAP_MAPPER_NODE_H_

#include <mutex>
#include <forward_list>

#include <ros/ros.h>

#include <map_merge_3d/octomap_mapper.h>
#include <map_merge_3d/typedefs.h>

namespace map_merge_3d
{

class OctomapMapper
{   
private:

    ros::Subscriber pointcloud_sub;

    ros::NodeHandle node_;


    /* node parameters */
    std::string world_frame_;
    // octomap mapper parameters
    OctomapMapperParams octomap_mapper_params_;

    // publishing
    ros::Publisher octomap2D_publisher_;
    ros::Publisher octomap3D_publisher_;

    // owns maps -- iterator safe
    size_t subscriptions_size_;
    std::mutex subscriptions_mutex_;

public:
    OctomapMapper();

    /**
     * @brief Generates and publishes 2D and 3D octomap
     * from merged pointcloud result
     * @details This function is thread-safe
     */
    void publishOctomap(const PointCloudConstPtr& msg);

};

} // namespace map_merge_3d

#endif  // MAP_MERGE_OCTOMAP_MAPPER_NODE_H_