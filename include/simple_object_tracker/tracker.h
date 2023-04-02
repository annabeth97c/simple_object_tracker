#include <ros/ros.h>
#include <simple_object_tracker/ObjectList.h>
#include <simple_object_tracker/Object.h>
#include <Eigen/Dense>
#include <vector>
#include <simple_object_tracker/extended_kalman_filter.hpp>
#include <simple_object_tracker/tracked_object.hpp>
#include <simple_object_tracker/association_heuristics.hpp>
#include <simple_object_tracker/hungarian_algorithm.hpp>

class Tracker {
public:
    Tracker(ros::NodeHandle& nh);
    void spin();
    
private:
    ros::NodeHandle nh_;
    ros::Subscriber object_list_sub_;
    ros::Publisher tracked_object_list_pub_;
    ros::Time last_callback_time_;

    double position_weight_ = 1.0;
    double velocity_weight_ = 0.0;
    AssociationHeuristics cost_factory{position_weight_,velocity_weight_};

    int tracked_object_counter;

    // Core logic
    void objectListCallback(const simple_object_tracker::ObjectList::ConstPtr& object_list_msg);
    void updateObjects(const simple_object_tracker::ObjectList::ConstPtr& object_list_msg);
    void predictTrackedObjects();
    
    std::vector<TrackedObject> tracked_objects_;
    simple_object_tracker::ObjectList tracked_object_list;

    friend class TrackerTest;
};