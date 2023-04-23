class TrackedObject {
public:
    TrackedObject() {}

    TrackedObject(const simple_object_tracker::Object& obj) :
        frame_id_(std::string("odom")),
        id_(obj.id), 
        centroid_(obj.pose), 
        velocity_(obj.velocity.linear),
        init_time_stamp_(obj.header.stamp),
        last_update_time_stamp_(obj.header.stamp),
        last_predict_time_stamp_(obj.header.stamp),
        num_missed_frames_(0),
        is_assigned_(false),
        is_predicted_(false)
    {
        // Initialize the Kalman filter
        int state_dim = 6;
        int meas_dim = 3;
        Eigen::VectorXd x0(state_dim);
        x0 << centroid_.position.x, centroid_.position.y, centroid_.position.z, velocity_.x, velocity_.y, velocity_.z;
        Eigen::MatrixXd P0(state_dim, state_dim);
        P0 << 1, 0, 0, 0, 0, 0,
              0, 1, 0, 0, 0, 0,
              0, 0, 1, 0, 0, 0,
              0, 0, 0, 1, 0, 0,
              0, 0, 0, 0, 1, 0,
              0, 0, 0, 0, 0, 1;
        Eigen::MatrixXd Q(state_dim, state_dim);
        Q.setIdentity();
        Eigen::MatrixXd R(meas_dim, meas_dim);
        R.setIdentity();
        ekf_ = ExtendedKalmanFilter(x0, P0, Q, R);
    }

    void predict(double dt, ros::Time stamp) {
        // Compute the state transition matrix F
        Eigen::MatrixXd F(6, 6);
        F << 1, 0, 0, dt, 0, 0,
             0, 1, 0, 0, dt, 0,
             0, 0, 1, 0, 0, dt,
             0, 0, 0, 1, 0, 0,
             0, 0, 0, 0, 1, 0,
             0, 0, 0, 0, 0, 1;
        
        // Compute the control input matrix B
        Eigen::MatrixXd B(6, 3);
        B << 0.5 * dt * dt, 0, 0,
             0, 0.5 * dt * dt, 0,
             0, 0, 0.5 * dt * dt,
             dt, 0, 0,
             0, dt, 0,
             0, 0, dt;

        // Compute the control input vector u (assuming constant velocity)
        Eigen::VectorXd u(3);
        u << 0, 0, 0;

        // Predict the state
        ekf_.predict(F, B * u);
        last_predict_time_stamp_ = stamp;

        Eigen::VectorXd state = ekf_.getState();

        centroid_.position.x = state(0);
        centroid_.position.y = state(1);
        centroid_.position.z = state(2);

        velocity_.x = state(3);
        velocity_.y = state(4);
        velocity_.z = state(5);

        is_predicted_ = true;

    }

    void update(const TrackedObject& obj) {
        // Compute the measurement matrix H
        // Compute the measurement matrix H
        Eigen::MatrixXd H(3, 6);
        H << 1, 0, 0, 0, 0, 0,
             0, 1, 0, 0, 0, 0,
             0, 0, 1, 0, 0, 0;

        // Compute the measurement vector z
        Eigen::VectorXd z(3);
        z << obj.centroid().position.x, obj.centroid().position.y, obj.centroid().position.z;

        // Update the state
        ekf_.update(z, H);
        
        // Update the other object properties
        // harm_level_ = obj.harm_level;
        centroid_ = obj.centroid();
        velocity_ = obj.velocity();
        last_update_time_stamp_ = obj.last_update_time_stamp();
        num_missed_frames_ = 0;
        is_assigned_ = true;
        is_predicted_ = false;
    }

    void markMissed() {
        ++num_missed_frames_;
        is_assigned_ = false;
    }

    simple_object_tracker::Object toObjectMessage() {

        // Create a new Object message to publish
        simple_object_tracker::Object obj_msg;
        obj_msg.id = id();

        obj_msg.pose = centroid();
        obj_msg.velocity.linear = velocity();
        
        obj_msg.header.stamp = is_predicted_ ? last_predict_time_stamp() : last_update_time_stamp();
        obj_msg.header.frame_id = frame_id();

        return obj_msg;
    }

    std::string frame_id() const { return frame_id_; }
    int id() const { return id_; }
    geometry_msgs::Pose centroid() const { return centroid_; }
    geometry_msgs::Vector3 velocity() const { return velocity_; }
    ExtendedKalmanFilter& ekf() { return ekf_; }
    ros::Time init_time_stamp() const { return init_time_stamp_; }
    ros::Time last_update_time_stamp() const { return last_update_time_stamp_; }
    ros::Time last_predict_time_stamp() const { return last_predict_time_stamp_; }
    int num_missed_frames() const { return num_missed_frames_; }
    bool isAssigned() const { return is_assigned_; }
    bool isPredicted() const { return is_predicted_; }

    void set_frame_id(const std::string& frame) { frame_id_ = frame; }
    void set_id(int id) { id_ = id; }
    void set_is_assigned(bool _is_assigned) { is_assigned_ = _is_assigned; }

private:
    std::string frame_id_;
    int id_;
    // int harm_level_;
    std::string class_;
    geometry_msgs::Pose centroid_;
    geometry_msgs::Vector3 velocity_;
    ExtendedKalmanFilter ekf_;
    ros::Time init_time_stamp_;
    ros::Time last_update_time_stamp_;
    ros::Time last_predict_time_stamp_;
    int num_missed_frames_;
    bool is_assigned_;
    bool is_predicted_;
};