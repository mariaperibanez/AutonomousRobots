#pragma once
#include <Eigen/Dense>

class KalmanFilter {
public:
    KalmanFilter() = default;

    KalmanFilter(const Eigen::Vector3d& initial_position, const Eigen::Matrix3d& initial_uncertainty)
        : state_(initial_position), uncertainty_(initial_uncertainty) {}

    void predict(const Eigen::Matrix3d& process_noise) {
        // Prediction step: state remains the same, uncertainty increases
        uncertainty_ += process_noise;
    }

    void update(const Eigen::Vector3d& measurement, const Eigen::Matrix3d& measurement_noise) {
        // Compute Kalman gain
        Eigen::Matrix3d kalman_gain = uncertainty_ * (uncertainty_ + measurement_noise).inverse();

        // Update state with measurement
        state_ += kalman_gain * (measurement - state_);

        // Update uncertainty
        uncertainty_ = (Eigen::Matrix3d::Identity() - kalman_gain) * uncertainty_;
    }

    Eigen::Vector3d getState() const {
        return state_;
    }

private:
    Eigen::Vector3d state_;          // Estimated position
    Eigen::Matrix3d uncertainty_;   // Associated uncertainty
};