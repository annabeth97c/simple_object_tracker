#include <ros/ros.h>
#include <gtest/gtest.h>
#include <simple_object_tracker/tracker.h>

class TrackerTest : public testing::Test {
public:
    TrackerTest() {}

protected:
    void objectListCallback(Tracker& tracker, simple_object_tracker::ObjectList object_list) {
        return tracker.objectListCallback(boost::make_shared<const simple_object_tracker::ObjectList>(object_list));
    }

    void updateObjects(Tracker& tracker, simple_object_tracker::ObjectList object_list_msg) {
      return tracker.updateObjects(boost::make_shared<const simple_object_tracker::ObjectList>(object_list_msg));
    }

    void predictTrackedObjects(Tracker& tracker) {
      return tracker.predictTrackedObjects();
    }

    std::vector<TrackedObject> get_tracked_objects(Tracker& tracker) {
      return tracker.tracked_objects_;
    }

    void set_tracked_objects(Tracker& tracker, std::vector<TrackedObject> tracked_objects_new_) {
      tracker.tracked_objects_ = tracked_objects_new_;
    }
};

TEST_F(TrackerTest, TestObjectListCallback) {

  // Create a fake object list
  simple_object_tracker::ObjectList object_list;
  simple_object_tracker::Object object;
  // object.obj_id = 1;
  object.pose.position.x = 1.0;
  object.pose.position.y = 2.0;
  object.pose.position.z = 0.0;
  object_list.objects.push_back(object);

  // // Create a tracker instance
  ros::NodeHandle nh;
  Tracker tracker(nh);

  // // Call the object list callback
  objectListCallback(tracker, object_list);

  std::vector<TrackedObject> tracked_objects = get_tracked_objects(tracker);

  // Assert that the tracker now has one tracked object with the same ID and position as the object in the object list
  ASSERT_EQ(tracked_objects.size(), 1);
  ASSERT_EQ(tracked_objects[0].id(), 0);

  ASSERT_DOUBLE_EQ(tracked_objects[0].centroid().position.x, 1.0);
  ASSERT_DOUBLE_EQ(tracked_objects[0].centroid().position.y, 2.0);
  ASSERT_DOUBLE_EQ(tracked_objects[0].centroid().position.z, 0.0);

}

TEST_F(TrackerTest, TestPredictTrackedObjects) {
  // Create a tracker instance
  ros::NodeHandle nh;
  Tracker tracker(nh);

  simple_object_tracker::Object object;
  object.id = 1;
  object.pose.position.x = 0.0;
  object.pose.position.y = 0.0;
  object.pose.position.z = 0.0;

  // Create a tracked object with ID 1 and position (0, 0, 0)
  TrackedObject tracked_object(object);

  std::vector<TrackedObject> tracked_objects = get_tracked_objects(tracker);
  tracked_objects.push_back(tracked_object);
  set_tracked_objects(tracker, tracked_objects);

  // Call the predictTrackedObjects method
  predictTrackedObjects(tracker);

  tracked_objects.clear();
  tracked_objects = get_tracked_objects(tracker);

  // Assert that the tracked object's position has been updated
  ASSERT_DOUBLE_EQ(tracked_objects[0].centroid().position.x, 0.0);
  ASSERT_DOUBLE_EQ(tracked_objects[0].centroid().position.y, 0.0);
  ASSERT_DOUBLE_EQ(tracked_objects[0].centroid().position.z, 0.0);
}

TEST_F(TrackerTest, costMatrixTest) {
  // Create a Tracker object
  ros::NodeHandle nh;
  Tracker tracker(nh);

  // Create two example TrackedObjects to use for testing
  simple_object_tracker::Object obj1;
  obj1.pose.position.x = 1.0;
  obj1.pose.position.y = 2.0;
  obj1.pose.position.z = 3.0;
  TrackedObject prev_obj(obj1);

  simple_object_tracker::Object obj2;
  obj2.pose.position.x = 4.0;
  obj2.pose.position.y = 5.0;
  obj2.pose.position.z = 6.0;
  TrackedObject curr_obj(obj2);

  // Set the expected cost value based on the distance between the objects
  double expected_cost = sqrt(pow(curr_obj.centroid().position.x - prev_obj.centroid().position.x, 2.0) + pow(curr_obj.centroid().position.y - prev_obj.centroid().position.y, 2.0) + pow(curr_obj.centroid().position.z - prev_obj.centroid().position.z, 2.0));

  // Call the association_cost function to get the actual cost value
  AssociationHeuristics cost_factory(1.0, 0.0);
  double actual_cost = cost_factory.association_cost(prev_obj, curr_obj);

  // Verify that the actual cost value matches the expected cost value
  ASSERT_NEAR(expected_cost, actual_cost, 0.0001);
}


// Test that the Hungarian Algorithm correctly associates new detections with existing tracks
TEST_F(TrackerTest, HungarianAssociationTest) {
    ros::NodeHandle nh;
    Tracker tracker(nh);

    // Create some initial tracked objects
    simple_object_tracker::ObjectList object_list_msg;
    simple_object_tracker::Object obj1;
    obj1.pose.position.x = 1.0;
    obj1.pose.position.y = 2.0;
    obj1.pose.position.z = 0.5;

    object_list_msg.objects.push_back(obj1);

    simple_object_tracker::Object obj2;
    obj2.pose.position.x = 5.0;
    obj2.pose.position.y = 6.0;
    obj2.pose.position.z = 0.3;

    object_list_msg.objects.push_back(obj2);

    // tracker.updateObjects(object_list_msg);
    updateObjects(tracker, object_list_msg);

    // Create a new object list with the first object moved slightly
    simple_object_tracker::ObjectList object_list_msg2;
    simple_object_tracker::Object obj3;
    obj3.pose.position.x = 1.1;
    obj3.pose.position.y = 2.0;
    obj3.pose.position.z = 0.5;

    object_list_msg2.objects.push_back(obj3);

    simple_object_tracker::Object obj4;
    obj4.pose.position.x = 5.0;
    obj4.pose.position.y = 6.0;
    obj4.pose.position.z = 0.3;

    object_list_msg2.objects.push_back(obj4);

    // tracker.updateObjects(object_list_msg2);
    updateObjects(tracker, object_list_msg2);

    std::vector<TrackedObject> tracked_objects = get_tracked_objects(tracker);

    // Verify that the first tracked object has been updated with the new position
    ASSERT_EQ(tracked_objects[0].centroid().position.x, 1.1);
    ASSERT_EQ(tracked_objects[0].centroid().position.y, 2.0);

    // Verify that the second tracked object has not been updated
    ASSERT_EQ(tracked_objects[1].centroid().position.x, 5.0);
    ASSERT_EQ(tracked_objects[1].centroid().position.y, 6.0);
}

// Test that the Hungarian Algorithm correctly associates new detections with existing tracks
TEST_F(TrackerTest, missingObjectTest) {
    ros::NodeHandle nh;
    Tracker tracker(nh);

    // Create some initial tracked objects
    simple_object_tracker::ObjectList object_list_msg;
    simple_object_tracker::Object obj1;
    obj1.pose.position.x = 1.0;
    obj1.pose.position.y = 2.0;
    obj1.pose.position.z = 0.5;

    object_list_msg.objects.push_back(obj1);

    simple_object_tracker::Object obj2;
    obj2.pose.position.x = 5.0;
    obj2.pose.position.y = 6.0;
    obj2.pose.position.z = 0.3;

    object_list_msg.objects.push_back(obj2);

    updateObjects(tracker, object_list_msg);

    // Create a new object list with the first object moved slightly
    simple_object_tracker::ObjectList object_list_msg2;
    simple_object_tracker::Object obj3;
    obj3.pose.position.x = 1.1;
    obj3.pose.position.y = 2.0;
    obj3.pose.position.z = 0.5;

    object_list_msg2.objects.push_back(obj3);

    simple_object_tracker::Object obj4;
    obj4.pose.position.x = 5.0;
    obj4.pose.position.y = 6.0;
    obj4.pose.position.z = 0.3;

    object_list_msg2.objects.push_back(obj4);

    updateObjects(tracker, object_list_msg2);

    // Create a new object list with the first object moved slightly
    simple_object_tracker::ObjectList object_list_msg3;
    simple_object_tracker::Object obj5;
    obj5.pose.position.x = 1.2;
    obj5.pose.position.y = 2.0;
    obj5.pose.position.z = 0.5;

    object_list_msg3.objects.push_back(obj5);

    updateObjects(tracker, object_list_msg3);

    std::vector<TrackedObject> tracked_objects = get_tracked_objects(tracker);

    ASSERT_EQ(tracked_objects.size(), 2);

    // // Verify that the first tracked object has been updated with the new position
    ASSERT_EQ(tracked_objects[0].centroid().position.x, 1.2);
    ASSERT_EQ(tracked_objects[0].centroid().position.y, 2.0);

    // // Verify that the second tracked object has not been updated
    ASSERT_EQ(tracked_objects[1].centroid().position.x, 5.0);
    ASSERT_EQ(tracked_objects[1].centroid().position.y, 6.0);
    ASSERT_EQ(tracked_objects[1].isAssigned(), false);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "tracker_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}