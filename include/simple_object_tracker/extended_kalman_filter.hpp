class ExtendedKalmanFilter {
public:
    ExtendedKalmanFilter() {}
    ExtendedKalmanFilter(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R) {
        init(x0, P0, Q, R);
    }

    void init(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R) {
        x_ = x0;
        P_ = P0;
        Q_ = Q;
        R_ = R;
    }

    void predict(const Eigen::MatrixXd& F, const Eigen::VectorXd& u) {
        x_ = F * x_;
        P_ = F * P_ * F.transpose() + Q_;
    }

    void update(const Eigen::VectorXd& z, const Eigen::MatrixXd& H) {
        Eigen::VectorXd y = z - H * x_;
        Eigen::MatrixXd S = H * P_ * H.transpose() + R_;
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
        x_ = x_ + K * y;
        P_ = (Eigen::MatrixXd::Identity(x_.size(), x_.size()) - K * H) * P_;
    }

    Eigen::VectorXd getState() {
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