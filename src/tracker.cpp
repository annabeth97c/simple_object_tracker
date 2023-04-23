#include "simple_object_tracker/tracker.h"

Tracker::Tracker(ros::NodeHandle& nh) : nh_(nh) {
    tracked_object_counter = 0;
    if (nh.getNamespace().empty()) {
        throw std::invalid_argument("Invalid NodeHandle provided");
    }
    object_list_sub_ = nh_.subscribe<simple_object_tracker::ObjectList>("object_list_topic", 10, &Tracker::objectListCallback, this);
    tracked_object_list_pub_ = nh_.advertise<simple_object_tracker::ObjectList>("tracked_object_list_topic", 10);
    last_callback_time_ = ros::Time::now();

}

void Tracker::spin() {
    ros::Rate r(10);
    while (ros::ok())
    {
        ros::spinOnce();

        ros::Time current_time = ros::Time::now();
        double elapsed_time = (current_time - last_callback_time_).toSec();

        tracked_object_list.objects.clear();

        // Step 4: If no callback since the last callback was processed, predict the current position of all tracked objects using EKF
        if (elapsed_time > 1.0) {
            predictTrackedObjects();

            tracked_object_list_pub_.publish(tracked_object_list);
        }

        // Publish the tracked object list
        r.sleep();
    }
}

void Tracker::objectListCallback(const simple_object_tracker::ObjectList::ConstPtr& object_list_msg) {
    if (!object_list_msg) {
        ROS_WARN("Received null object list");
        return;
    }
    ros::Time current_time = ros::Time::now();
    last_callback_time_ = current_time;
    
    // Step 1: Associate new detections with existing tracks using Hungarian Algorithm
    // Step 2: Update tracked objects using Extended Kalman Filter
    // Step 3: Spawn new tracks for any unassociated detections
    updateObjects(object_list_msg);
}

void Tracker::updateObjects(const simple_object_tracker::ObjectList::ConstPtr& curr_objects) {
    if (!curr_objects) {
        ROS_WARN("Received null object list");
        return;
    }

    // Convert current objects to vector of TrackedObject
    std::vector<TrackedObject> curr_tracked_objects;
    for (const simple_object_tracker::Object& obj : curr_objects->objects) {
        curr_tracked_objects.emplace_back(TrackedObject(obj));
    }

    // Build cost matrix for Hungarian algorithm
    std::vector<std::vector<double>> cost_matrix;
    cost_matrix.reserve(tracked_objects_.size());

    for (const TrackedObject& prev_obj : tracked_objects_) {
        std::vector<double> costs;
        costs.reserve(curr_tracked_objects.size());

        for (const TrackedObject& curr_obj : curr_tracked_objects) {
            costs.push_back(cost_factory.association_cost(prev_obj, curr_obj));
        }
        cost_matrix.push_back(costs);
    }

    std::vector<int> assignments;

    if (!tracked_objects_.empty() && !curr_tracked_objects.empty())
    {
        // Use Hungarian algorithm to assign current objects to previous objects
        assignments = hungarian(cost_matrix);
    }
    else
    {
        assignments.assign(tracked_objects_.size(), -1);
    }

    // Assign current objects to previous objects
    for (size_t i = 0; i < tracked_objects_.size(); i++) {
        if (assignments[i+1] != -1) {
            tracked_objects_[i].update(curr_tracked_objects[assignments[i+1] - 1]);
            curr_tracked_objects[assignments[i+1] - 1].set_is_assigned(true);
        } else {
            tracked_objects_[i].markMissed();
        }
    }

    // Add unassigned current objects as new tracked objects
    for (size_t i = 0; i < curr_tracked_objects.size(); i++) {
        if (!curr_tracked_objects[i].isAssigned()) {
            curr_tracked_objects[i].set_id(tracked_object_counter);
            curr_tracked_objects[i].set_is_assigned(true);
            tracked_objects_.push_back(curr_tracked_objects[i]);
            tracked_object_counter++;
        }
    }

}

void Tracker::predictTrackedObjects() {
    // Get the current time
    ros::Time current_time = ros::Time::now();

    // Iterate through all tracked objects
    for (TrackedObject& obj : tracked_objects_) {
        ros::Time last_state_change_time = obj.isPredicted() ? obj.last_predict_time_stamp() : obj.last_update_time_stamp();
        // Compute the time since the last update
        double dt = (current_time - obj.last_update_time_stamp()).toSec();

        obj.predict(dt, current_time);

        // Add the Object message to the tracked object list
        tracked_object_list.objects.push_back(obj.toObjectMessage());
    }
}