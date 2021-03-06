#include <map_merge_3d/octomap_mapper_node.h>

#include <pcl_ros/point_cloud.h>
#include <ros/assert.h>
#include <ros/console.h>

namespace map_merge_3d
{
OctomapMapper::OctomapMapper() : subscriptions_size_(0)
{
    ros::NodeHandle private_nh("~");
    std::string merged_map_topic;
    std::string floor_plane_topic;
    std::string octomap2D_topic;
    std::string octomap3D_topic;

    private_nh.param<std::string>("merged_map_topic", merged_map_topic, "map_merge/map");
    private_nh.param<std::string>("floor_plane_topic", floor_plane_topic, "map_merge/floor_plane");
    private_nh.param<std::string>("octomap2D_topic", octomap2D_topic, "octomap2D");
    private_nh.param<std::string>("octomap3D_topic", octomap3D_topic, "octomap3D");
    private_nh.param<std::string>("world_frame", world_frame_, "world");

    // registration parameters
    octomap_mapper_params_ = OctomapMapperParams::fromROSNode(private_nh);

    /* publishing */
    floor_plane_publisher_ =
        node_.advertise<PointCloud>(floor_plane_topic, 50, true);
    octomap2D_publisher_ =
        node_.advertise<octomap_msgs::Octomap>(octomap2D_topic, 50, true);
    octomap3D_publisher_ =
        node_.advertise<octomap_msgs::Octomap>(octomap3D_topic, 50, true);

    ROS_INFO("Subscribing to MAP topic: %s.", merged_map_topic.c_str());
    pointcloud_sub = node_.subscribe<PointCloud>(
        merged_map_topic, 50, [this](const PointCloudConstPtr& msg) {
            publishOctomap(msg);
        });

}

void OctomapMapper::publishOctomap(const PointCloudConstPtr& msg)
{
    octomap::OcTree final_octree_2D = octomap2DGenerator(msg, octomap_mapper_params_);
    octomap::OcTree final_octree_3D = octomap3DGenerator(msg, octomap_mapper_params_);

    if (octomap3D_publisher_.getNumSubscribers() != 0) {
    octomap_msgs::Octomap octomap_fullmsg_3D;
    octomap_msgs::fullMapToMsg(final_octree_3D, octomap_fullmsg_3D);
    octomap_fullmsg_3D.header.frame_id = "map";
    octomap_fullmsg_3D.header.stamp = ros::Time::now();
    octomap3D_publisher_.publish(octomap_fullmsg_3D);

  }

  if (octomap2D_publisher_.getNumSubscribers() != 0) {
    octomap_msgs::Octomap octomap_msg_2D;
    octomap_msgs::binaryMapToMsg(final_octree_2D, octomap_msg_2D);
    octomap_msg_2D.header.frame_id = "map";
    octomap_msg_2D.header.stamp = ros::Time::now();
    octomap2D_publisher_.publish(octomap_msg_2D);

  }

}

} // namespace map_merge_3d

int main(int argc, char** argv)
{
  ros::init(argc, argv, "octomap_mapper");
  // this package is still in development -- start with debugging enabled
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Info)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  map_merge_3d::OctomapMapper octomap_mapper_node;
  // use all threads for spinning
  ros::MultiThreadedSpinner spinner;
  spinner.spin();
  return 0;
}