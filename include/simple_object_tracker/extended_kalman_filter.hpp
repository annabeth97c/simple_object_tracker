class ExtendedKalmanFilter {
public:
    ExtendedKalmanFilter() {}
    ExtendedKalmanFilter(VectorXd x0, MatrixXd P0, MatrixXd Q, MatrixXd R) {
        init(x0, P0, Q, R);
    }

    void init(VectorXd x0, MatrixXd P0, MatrixXd Q, MatrixXd R) {
        x_ = x0;
        P_ = P0;
        Q_ = Q;
        R_ = R;
    }

    void predict(MatrixXd F, VectorXd u) {
        x_ = F * x_;
        P_ = F * P_ * F.transpose() + Q_;
    }

    void update(VectorXd z, MatrixXd H) {
        VectorXd y = z - H * x_;
        MatrixXd S = H * P_ * H.transpose() + R_;
        MatrixXd K = P_ * H.transpose() * S.inverse();
        x_ = x_ + K * y;
        P_ = (MatrixXd::Identity(x_.size(), x_.size()) - K * H) * P_;
    }

    VectorXd getState() {
        return x_;
    }

    Eigen::VectorXd getMeasurementNoise() {
        Eigen::VectorXd v(2);
        v.setRandom();
        v *= sqrt(R_(0, 0));
        return v;
    }

private:
    Eigen::VectorXd x_;
    Eigen::MatrixXd P_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd R_;
};