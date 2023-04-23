class AssociationHeuristics {
public:
    AssociationHeuristics(double position_weight, double velocity_weight)
        : position_weight_(position_weight), velocity_weight_(velocity_weight) {}

    double association_cost(const TrackedObject& tracked_obj, const TrackedObject& detected_obj) {
        // Calculate the Euclidean distance between the predicted position of the tracked object
        // and the detected object, weighted by the position weight.
        double position_cost = position_weight_ * euclidean_distance(tracked_obj.centroid(), detected_obj.centroid());

        // Calculate the Euclidean distance between the predicted velocity of the tracked object
        // and the detected object, weighted by the velocity weight.

        // Combine the position and velocity costs into a single cost.
        double total_cost = position_cost;

        return total_cost;
    }

    double euclidean_distance(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b) {
        Eigen::Vector3d pos_a(a.position.x, a.position.y, a.position.z);
        Eigen::Vector3d pos_b(b.position.x, b.position.y, b.position.z);

        double dist = (pos_b - pos_a).norm();
        return dist;
    }

private:
    double position_weight_;
    double velocity_weight_;
};