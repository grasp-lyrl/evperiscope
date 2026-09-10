#pragma once

#include <vector>
#include <array>
#include <optional>
#include <cmath>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

namespace epa {

/**
 * Extended Kalman Filter for 6DoF quadrotor pose estimation (Constant Velocity).
 *
 * State: [x, y, z, roll, pitch, yaw, vx, vy, vz, v_roll, v_pitch, v_yaw]
 * Measurement: 4 propeller centroid pixel positions (8 values)
 */
class QuadPoseFilter {
public:
    using Vec6 = Eigen::Matrix<double, 6, 1>;
    using Vec12 = Eigen::Matrix<double, 12, 1>;
    using Mat12 = Eigen::Matrix<double, 12, 12>;
    using Mat3 = Eigen::Matrix3d;
    using Vec3 = Eigen::Vector3d;

    struct Pose {
        double x, y, z, roll, pitch, yaw;
    };

    struct Twist {
        double vx, vy, vz, v_roll, v_pitch, v_yaw;
    };

    struct AxesPoints {
        std::array<int, 2> origin;    // (row, col)
        std::array<int, 2> x_tip;
        std::array<int, 2> y_tip;
        std::array<int, 2> z_tip;
    };

    /**
     * @param camera_matrix 3x3 camera intrinsic matrix
     * @param front_arm_length Front arm length (meters)
     * @param rear_arm_length Rear arm length (meters)
     * @param front_back_offset X offset from center (meters)
     * @param use_brute_force_ordering Use brute-force centroid ordering
     */
    QuadPoseFilter(const Eigen::Matrix3d& camera_matrix,
                   double front_arm_length = 0.15,
                   double rear_arm_length = 0.12,
                   double front_back_offset = 0.1,
                   bool use_brute_force_ordering = false);

    /**
     * Update filter with new centroid observations.
     * @param centroids 4 centroids in (row, col) format
     * @param dt Time step (seconds)
     * @return true if update succeeded
     */
    bool update(const std::vector<Eigen::Vector2f>& centroids, double dt = 0.01);

    /// Get current pose estimate (returns nullopt if not initialized)
    std::optional<Pose> getPose() const;

    /// Get current velocity estimate
    std::optional<Twist> getTwist() const;

    /// Get projected model centroids in (row, col) format
    std::optional<std::vector<Eigen::Vector2d>> getProjectedCentroids() const;

    /// Get projected body axes for visualization
    std::optional<AxesPoints> getAxes(double length = 0.2) const;

    bool isInitialized() const { return initialized_; }

private:
    // Camera intrinsics
    Eigen::Matrix3d camera_matrix_;
    double fx_, fy_, cx_, cy_;

    // Quadrotor model points in body frame (FL, FR, RR, RL)
    Eigen::Matrix<double, 4, 3> model_points_;

    // State and covariance
    Vec12 state_;
    Mat12 P_;
    bool initialized_;
    bool use_brute_force_ordering_;
    mutable std::mutex state_mutex_;

    // EKF tuning
    Mat12 Q_;
    Eigen::Matrix<double, 8, 8> R_;

    // Core EKF methods
    bool initialize(const std::vector<Eigen::Vector2f>& centroids);
    void predict(double dt);

    // Centroid ordering
    std::optional<Eigen::Matrix<double, 4, 2>> orderCentroids(
        const std::vector<Eigen::Vector2f>& centroids);
    std::optional<Eigen::Matrix<double, 4, 2>> orderCentroidsBruteForce(
        const std::vector<Eigen::Vector2f>& centroids);
    Eigen::Matrix<double, 4, 2> orderCentroidsByDistance(
        const std::vector<Eigen::Vector2f>& centroids);

    // Math helpers
    static Mat3 eulerToRotationMatrix(double roll, double pitch, double yaw);

    static void computeRotationDerivatives(
        double roll, double pitch, double yaw,
        const Eigen::Matrix<double, 4, 3>& points,
        Eigen::Matrix<double, 4, 3>& d_roll,
        Eigen::Matrix<double, 4, 3>& d_pitch,
        Eigen::Matrix<double, 4, 3>& d_yaw);

    Eigen::Matrix<double, 4, 2> projectPoints(
        const Eigen::Matrix<double, Eigen::Dynamic, 3>& points_3d,
        const Vec3& rvec_euler, const Vec3& tvec) const;

    Eigen::Matrix<double, 8, 6> computeJacobian6DOF(
        const Eigen::Matrix<double, 4, 3>& points_3d,
        const Vec6& state6) const;
};

}  // namespace epa
