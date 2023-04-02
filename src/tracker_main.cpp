#include "simple_object_tracker/tracker.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_cluster_node");
    ros::NodeHandle nh;

    Tracker tracker_node(nh);

    tracker_node.spin();

    return 0;
}