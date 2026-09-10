#include "epa/quad_pose_filter.hpp"
#include <algorithm>
#include <numeric>

namespace epa {

QuadPoseFilter::QuadPoseFilter(const Eigen::Matrix3d& camera_matrix,
                               double front_arm_length,
                               double rear_arm_length,
                               double front_back_offset,
                               bool use_brute_force_ordering)
    : camera_matrix_(camera_matrix),
      initialized_(false),
      use_brute_force_ordering_(use_brute_force_ordering) {
    fx_ = camera_matrix_(0, 0);
    fy_ = camera_matrix_(1, 1);
    cx_ = camera_matrix_(0, 2);
    cy_ = camera_matrix_(1, 2);

    // Build 3D model points in body frame (X forward, Y left, Z up)
    // Order: FL, FR, RR, RL
    model_points_ << front_back_offset,  front_arm_length, 0.0,
                     front_back_offset, -front_arm_length, 0.0,
                    -front_back_offset, -rear_arm_length,  0.0,
                    -front_back_offset,  rear_arm_length,  0.0;

    // Process noise Q (12x12)
    Q_ = Mat12::Identity() * 0.001;
    Q_.block<6, 6>(6, 6) *= 10.0;

    // Measurement noise R (8x8)
    Eigen::VectorXd r_diag(8);
    for (int i = 0; i < 4; ++i) {
        r_diag(2 * i) = 5.0;
        r_diag(2 * i + 1) = 5.0;
    }
    R_ = r_diag.asDiagonal();

    state_ = Vec12::Zero();
    P_ = Mat12::Identity();
}

// ---------- Euler / Rotation helpers ----------

QuadPoseFilter::Mat3 QuadPoseFilter::eulerToRotationMatrix(double roll, double pitch, double yaw) {
    double cr = std::cos(roll),  sr = std::sin(roll);
    double cp = std::cos(pitch), sp = std::sin(pitch);
    double cy = std::cos(yaw),   sy = std::sin(yaw);

    Mat3 R;
    R(0, 0) = cy * cp;
    R(0, 1) = cy * sp * sr - sy * cr;
    R(0, 2) = cy * sp * cr + sy * sr;
    R(1, 0) = sy * cp;
    R(1, 1) = sy * sp * sr + cy * cr;
    R(1, 2) = sy * sp * cr - cy * sr;
    R(2, 0) = -sp;
    R(2, 1) = cp * sr;
    R(2, 2) = cp * cr;
    return R;
}

void QuadPoseFilter::computeRotationDerivatives(
    double roll, double pitch, double yaw,
    const Eigen::Matrix<double, 4, 3>& points,
    Eigen::Matrix<double, 4, 3>& d_roll,
    Eigen::Matrix<double, 4, 3>& d_pitch,
    Eigen::Matrix<double, 4, 3>& d_yaw) {

    double cr = std::cos(roll),  sr = std::sin(roll);
    double cp = std::cos(pitch), sp = std::sin(pitch);
    double cy = std::cos(yaw),   sy = std::sin(yaw);

    Mat3 dR_dr;
    dR_dr(0, 0) = 0;
    dR_dr(0, 1) = cy * sp * cr + sy * sr;
    dR_dr(0, 2) = -cy * sp * sr + sy * cr;
    dR_dr(1, 0) = 0;
    dR_dr(1, 1) = sy * sp * cr - cy * sr;
    dR_dr(1, 2) = -sy * sp * sr - cy * cr;
    dR_dr(2, 0) = 0;
    dR_dr(2, 1) = cp * cr;
    dR_dr(2, 2) = -cp * sr;

    Mat3 dR_dp;
    dR_dp(0, 0) = -cy * sp;
    dR_dp(0, 1) = cy * cp * sr;
    dR_dp(0, 2) = cy * cp * cr;
    dR_dp(1, 0) = -sy * sp;
    dR_dp(1, 1) = sy * cp * sr;
    dR_dp(1, 2) = sy * cp * cr;
    dR_dp(2, 0) = -cp;
    dR_dp(2, 1) = -sp * sr;
    dR_dp(2, 2) = -sp * cr;

    Mat3 dR_dy;
    dR_dy(0, 0) = -sy * cp;
    dR_dy(0, 1) = -sy * sp * sr - cy * cr;
    dR_dy(0, 2) = -sy * sp * cr + cy * sr;
    dR_dy(1, 0) = cy * cp;
    dR_dy(1, 1) = cy * sp * sr - sy * cr;
    dR_dy(1, 2) = cy * sp * cr + sy * sr;
    dR_dy(2, 0) = 0;
    dR_dy(2, 1) = 0;
    dR_dy(2, 2) = 0;

    for (int i = 0; i < 4; ++i) {
        Vec3 p = points.row(i).transpose();
        d_roll.row(i)  = (dR_dr * p).transpose();
        d_pitch.row(i) = (dR_dp * p).transpose();
        d_yaw.row(i)   = (dR_dy * p).transpose();
    }
}

Eigen::Matrix<double, 4, 2> QuadPoseFilter::projectPoints(
    const Eigen::Matrix<double, Eigen::Dynamic, 3>& points_3d,
    const Vec3& rvec_euler, const Vec3& tvec) const {

    Mat3 R = eulerToRotationMatrix(rvec_euler(0), rvec_euler(1), rvec_euler(2));

    int n = static_cast<int>(points_3d.rows());
    Eigen::Matrix<double, 4, 2> pts_2d;
    pts_2d.setZero();

    for (int i = 0; i < n; ++i) {
        Vec3 p_cam = R * points_3d.row(i).transpose() + tvec;
        if (p_cam(2) > 0) {
            pts_2d(i, 0) = fx_ * p_cam(0) / p_cam(2) + cx_;  // u (col)
            pts_2d(i, 1) = fy_ * p_cam(1) / p_cam(2) + cy_;  // v (row)
        } else {
            pts_2d(i, 0) = -1;
            pts_2d(i, 1) = -1;
        }
    }
    return pts_2d;
}

Eigen::Matrix<double, 8, 6> QuadPoseFilter::computeJacobian6DOF(
    const Eigen::Matrix<double, 4, 3>& points_3d,
    const Vec6& state6) const {

    double x = state6(0), y = state6(1), z = state6(2);
    double roll = state6(3), pitch = state6(4), yaw = state6(5);
    Vec3 tvec(x, y, z);

    Mat3 R = eulerToRotationMatrix(roll, pitch, yaw);

    Eigen::Matrix<double, 4, 3> dP_droll, dP_dpitch, dP_dyaw;
    computeRotationDerivatives(roll, pitch, yaw, points_3d, dP_droll, dP_dpitch, dP_dyaw);

    Eigen::Matrix<double, 8, 6> J = Eigen::Matrix<double, 8, 6>::Zero();

    for (int i = 0; i < 4; ++i) {
        Vec3 p_cam = R * points_3d.row(i).transpose() + tvec;
        double X = p_cam(0), Y = p_cam(1), Z = p_cam(2);

        if (Z > 0.1) {
            double Z_inv = 1.0 / Z;
            double Z_inv2 = Z_inv * Z_inv;

            double du_dX = fx_ * Z_inv;
            double du_dZ = -fx_ * X * Z_inv2;
            double dv_dY = fy_ * Z_inv;
            double dv_dZ = -fy_ * Y * Z_inv2;

            // Translation derivatives
            J(2 * i, 0) = du_dX;     // du/dx
            J(2 * i, 2) = du_dZ;     // du/dz
            J(2 * i + 1, 1) = dv_dY; // dv/dy
            J(2 * i + 1, 2) = dv_dZ; // dv/dz

            // Roll
            J(2 * i, 3) = du_dX * dP_droll(i, 0) + du_dZ * dP_droll(i, 2);
            J(2 * i + 1, 3) = dv_dY * dP_droll(i, 1) + dv_dZ * dP_droll(i, 2);

            // Pitch
            J(2 * i, 4) = du_dX * dP_dpitch(i, 0) + du_dZ * dP_dpitch(i, 2);
            J(2 * i + 1, 4) = dv_dY * dP_dpitch(i, 1) + dv_dZ * dP_dpitch(i, 2);

            // Yaw
            J(2 * i, 5) = du_dX * dP_dyaw(i, 0) + du_dZ * dP_dyaw(i, 2);
            J(2 * i + 1, 5) = dv_dY * dP_dyaw(i, 1) + dv_dZ * dP_dyaw(i, 2);
        }
    }
    return J;
}

// ---------- Centroid Ordering ----------

std::optional<Eigen::Matrix<double, 4, 2>> QuadPoseFilter::orderCentroidsBruteForce(
    const std::vector<Eigen::Vector2f>& centroids) {

    // Convert to OpenCV format (col, row) for solvePnP
    std::vector<cv::Point2d> all_pts(4);
    for (int i = 0; i < 4; ++i) {
        all_pts[i] = cv::Point2d(centroids[i](1), centroids[i](0));  // (x=col, y=row)
    }

    // Model points for OpenCV
    std::vector<cv::Point3d> model_cv(4);
    for (int i = 0; i < 4; ++i) {
        model_cv[i] = cv::Point3d(model_points_(i, 0), model_points_(i, 1), model_points_(i, 2));
    }

    cv::Mat cam_mat(3, 3, CV_64F);
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            cam_mat.at<double>(r, c) = camera_matrix_(r, c);

    // Try all 24 permutations
    int perm[4] = {0, 1, 2, 3};
    double best_error = std::numeric_limits<double>::infinity();
    Eigen::Matrix<double, 4, 2> best_ordered;
    bool found = false;

    do {
        std::vector<cv::Point2d> img_pts(4);
        for (int i = 0; i < 4; ++i) {
            img_pts[i] = all_pts[perm[i]];
        }

        cv::Mat rvec, tvec;
        bool success = cv::solvePnP(model_cv, img_pts, cam_mat, cv::noArray(),
                                    rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
        if (!success || tvec.at<double>(2) < 0.1) continue;

        std::vector<cv::Point2d> reproj;
        cv::projectPoints(model_cv, rvec, tvec, cam_mat, cv::noArray(), reproj);

        double error = 0;
        for (int i = 0; i < 4; ++i) {
            double dx = reproj[i].x - img_pts[i].x;
            double dy = reproj[i].y - img_pts[i].y;
            error += std::sqrt(dx * dx + dy * dy);
        }
        error /= 4.0;

        if (error < best_error) {
            best_error = error;
            for (int i = 0; i < 4; ++i) {
                // Store in (row, col) format
                best_ordered(i, 0) = centroids[perm[i]](0);
                best_ordered(i, 1) = centroids[perm[i]](1);
            }
            found = true;
        }
    } while (std::next_permutation(perm, perm + 4));

    if (found) return best_ordered;
    return std::nullopt;
}

Eigen::Matrix<double, 4, 2> QuadPoseFilter::orderCentroidsByDistance(
    const std::vector<Eigen::Vector2f>& centroids) {

    // Convert to (col, row) = (x, y) for geometry
    Eigen::Matrix<double, 4, 2> points;  // (x=col, y=row)
    for (int i = 0; i < 4; ++i) {
        points(i, 0) = centroids[i](1);  // col -> x
        points(i, 1) = centroids[i](0);  // row -> y
    }

    Eigen::Vector2d center = points.colwise().mean();

    // Angular sort (CCW)
    Eigen::Vector4d angles;
    for (int i = 0; i < 4; ++i) {
        angles(i) = std::atan2(points(i, 1) - center(1), points(i, 0) - center(0));
        if (angles(i) < 0) angles(i) += 2.0 * M_PI;
    }

    // Sort indices by angle
    std::array<int, 4> idx = {0, 1, 2, 3};
    std::sort(idx.begin(), idx.end(), [&](int a, int b) { return angles(a) < angles(b); });

    // Find front pair (max adjacent distance)
    std::array<double, 4> adj_dists;
    for (int i = 0; i < 4; ++i) {
        int j = (i + 1) % 4;
        double dx = points(idx[i], 0) - points(idx[j], 0);
        double dy = points(idx[i], 1) - points(idx[j], 1);
        adj_dists[i] = std::sqrt(dx * dx + dy * dy);
    }

    int max_idx = static_cast<int>(
        std::max_element(adj_dists.begin(), adj_dists.end()) - adj_dists.begin());

    // Front pair indices
    int fi0 = idx[max_idx];
    int fi1 = idx[(max_idx + 1) % 4];

    // Compute yaw from front pair center
    Eigen::Vector2d front_center;
    front_center(0) = (points(fi0, 0) + points(fi1, 0)) / 2.0;
    front_center(1) = (points(fi0, 1) + points(fi1, 1)) / 2.0;
    Eigen::Vector2d forward_vec = front_center - center;
    double yaw = std::atan2(forward_vec(1), forward_vec(0));
    if (yaw < 0) yaw += 2.0 * M_PI;

    // Sort angles relative to yaw to get FL, FR, RR, RL ordering
    // Reconstruct sorted sequence using the same logic as Python
    std::array<double, 5> all_angles;
    for (int i = 0; i < 4; ++i) all_angles[i] = angles(i);
    all_angles[4] = yaw;

    std::array<int, 5> all_sort_idx = {0, 1, 2, 3, 4};
    std::sort(all_sort_idx.begin(), all_sort_idx.end(),
              [&](int a, int b) { return all_angles[a] < all_angles[b]; });

    int yaw_pos = -1;
    for (int i = 0; i < 5; ++i) {
        if (all_sort_idx[i] == 4) { yaw_pos = i; break; }
    }

    std::array<int, 4> seq;
    for (int i = 0; i < 4; ++i) {
        seq[i] = all_sort_idx[(yaw_pos + 1 + i) % 5];
    }

    // Filter out the yaw index (4) - in practice it shouldn't appear,
    // but keep only point indices < 4
    std::vector<int> ordered_idx;
    for (int s : seq) {
        if (s < 4) ordered_idx.push_back(s);
    }

    // Map to [FL, FR, RR, RL]
    Eigen::Matrix<double, 4, 2> ordered;  // (row, col)
    if (static_cast<int>(ordered_idx.size()) >= 4) {
        // ordered[0]=FL, ordered[3]=FR, ordered[2]=RR, ordered[1]=RL
        ordered(0, 0) = points(ordered_idx[0], 1); ordered(0, 1) = points(ordered_idx[0], 0);
        ordered(1, 0) = points(ordered_idx[3], 1); ordered(1, 1) = points(ordered_idx[3], 0);
        ordered(2, 0) = points(ordered_idx[2], 1); ordered(2, 1) = points(ordered_idx[2], 0);
        ordered(3, 0) = points(ordered_idx[1], 1); ordered(3, 1) = points(ordered_idx[1], 0);
    } else {
        // Fallback: just use input order
        for (int i = 0; i < 4; ++i) {
            ordered(i, 0) = centroids[i](0);
            ordered(i, 1) = centroids[i](1);
        }
    }

    return ordered;
}

std::optional<Eigen::Matrix<double, 4, 2>> QuadPoseFilter::orderCentroids(
    const std::vector<Eigen::Vector2f>& centroids) {
    if (static_cast<int>(centroids.size()) != 4) return std::nullopt;
    if (use_brute_force_ordering_) {
        return orderCentroidsBruteForce(centroids);
    }
    return orderCentroidsByDistance(centroids);
}

// ---------- EKF ----------

bool QuadPoseFilter::initialize(const std::vector<Eigen::Vector2f>& centroids) {
    auto ordered_opt = orderCentroids(centroids);
    if (!ordered_opt) return false;
    auto ordered = *ordered_opt;

    // Convert (row, col) → (col, row) for solvePnP image points
    std::vector<cv::Point2d> img_pts(4);
    for (int i = 0; i < 4; ++i) {
        img_pts[i] = cv::Point2d(ordered(i, 1), ordered(i, 0));
    }

    std::vector<cv::Point3d> model_cv(4);
    for (int i = 0; i < 4; ++i) {
        model_cv[i] = cv::Point3d(model_points_(i, 0), model_points_(i, 1), model_points_(i, 2));
    }

    cv::Mat cam_mat(3, 3, CV_64F);
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            cam_mat.at<double>(r, c) = camera_matrix_(r, c);

    cv::Mat rvec, tvec;
    bool success = cv::solvePnP(model_cv, img_pts, cam_mat, cv::noArray(),
                                rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
    if (!success) return false;

    // Convert Rodrigues to Euler
    cv::Mat rot_mat;
    cv::Rodrigues(rvec, rot_mat);

    double sy = std::sqrt(rot_mat.at<double>(0, 0) * rot_mat.at<double>(0, 0) +
                          rot_mat.at<double>(1, 0) * rot_mat.at<double>(1, 0));
    double roll, pitch, yaw_val;
    if (sy > 1e-6) {
        roll = std::atan2(rot_mat.at<double>(2, 1), rot_mat.at<double>(2, 2));
        pitch = std::atan2(-rot_mat.at<double>(2, 0), sy);
        yaw_val = std::atan2(rot_mat.at<double>(1, 0), rot_mat.at<double>(0, 0));
    } else {
        roll = std::atan2(-rot_mat.at<double>(1, 2), rot_mat.at<double>(1, 1));
        pitch = std::atan2(-rot_mat.at<double>(2, 0), sy);
        yaw_val = 0;
    }

    // Sanity checks for initialization
    const double MAX_ROLL_PITCH_DEG = 20.0;
    const double MAX_ROLL_PITCH_RAD = MAX_ROLL_PITCH_DEG * M_PI / 180.0;
    const double MAX_ALTITUDE = 5.0;
    const double MIN_ALTITUDE = 0.0;

    double z = tvec.at<double>(2);

    // Check roll and pitch are within limits
    if (std::abs(roll) > MAX_ROLL_PITCH_RAD || std::abs(pitch) > MAX_ROLL_PITCH_RAD) {
        return false;  // Roll/pitch out of range
    }

    // Check altitude is within limits
    if (z < MIN_ALTITUDE || z > MAX_ALTITUDE) {
        return false;  // Altitude out of range
    }

    // Initialize 12-state vector
    state_ = Vec12::Zero();
    state_(0) = tvec.at<double>(0);
    state_(1) = tvec.at<double>(1);
    state_(2) = tvec.at<double>(2);
    state_(3) = roll;
    state_(4) = pitch;
    state_(5) = yaw_val;

    P_ = Mat12::Identity() * 0.1;
    P_.block<6, 6>(6, 6) = Eigen::Matrix<double, 6, 6>::Identity() * 1.0;

    initialized_ = true;
    return true;
}

void QuadPoseFilter::predict(double dt) {
    if (!initialized_) return;

    // State transition F (constant velocity)
    Mat12 F = Mat12::Identity();
    for (int i = 0; i < 6; ++i) {
        F(i, i + 6) = dt;
    }

    state_ = F * state_;
    P_ = F * P_ * F.transpose() + Q_;
}

bool QuadPoseFilter::update(const std::vector<Eigen::Vector2f>& centroids, double dt) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!initialized_) return initialize(centroids);

    auto ordered_opt = orderCentroids(centroids);
    if (!ordered_opt) return false;
    auto ordered = *ordered_opt;

    predict(dt);

    // Project current state
    Vec6 state6 = state_.head<6>();
    Vec3 rvec_euler(state6(3), state6(4), state6(5));
    Vec3 tvec_val(state6(0), state6(1), state6(2));

    auto predicted_pts = projectPoints(model_points_, rvec_euler, tvec_val);

    // Residual: z_meas - h(x)
    // Both in (row, col) format
    Eigen::Matrix<double, 8, 1> z_meas;
    Eigen::Matrix<double, 8, 1> h_pred;
    for (int i = 0; i < 4; ++i) {
        z_meas(2 * i)     = ordered(i, 0);  // row
        z_meas(2 * i + 1) = ordered(i, 1);  // col
        h_pred(2 * i)     = predicted_pts(i, 1);  // y -> row
        h_pred(2 * i + 1) = predicted_pts(i, 0);  // x -> col
    }

    Eigen::Matrix<double, 8, 1> y_residual = z_meas - h_pred;

    // Jacobian H (8x6 w.r.t. state6)
    auto H_6dof = computeJacobian6DOF(model_points_, state6);

    // Pad to 8x12 (measurement does not depend on velocities)
    Eigen::Matrix<double, 8, 12> H_12 = Eigen::Matrix<double, 8, 12>::Zero();
    H_12.block<8, 6>(0, 0) = H_6dof;

    // Swap rows for (row, col) output format
    Eigen::Matrix<double, 8, 12> H_rc = Eigen::Matrix<double, 8, 12>::Zero();
    for (int i = 0; i < 4; ++i) {
        H_rc.row(2 * i)     = H_12.row(2 * i + 1);  // row <- v
        H_rc.row(2 * i + 1) = H_12.row(2 * i);      // col <- u
    }

    // Kalman update
    Eigen::Matrix<double, 8, 8> S = H_rc * P_ * H_rc.transpose() + R_;
    Eigen::FullPivLU<Eigen::Matrix<double, 8, 8>> lu(S);
    if (!lu.isInvertible()) return false;
    Eigen::Matrix<double, 8, 8> S_inv = lu.inverse();

    Eigen::Matrix<double, 12, 8> K = P_ * H_rc.transpose() * S_inv;
    state_ += K * y_residual;

    // Joseph form covariance update
    Mat12 IKH = Mat12::Identity() - K * H_rc;
    P_ = IKH * P_ * IKH.transpose() + K * R_ * K.transpose();

    return true;
}

// ---------- Accessors ----------

std::optional<QuadPoseFilter::Pose> QuadPoseFilter::getPose() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!initialized_) return std::nullopt;
    return Pose{state_(0), state_(1), state_(2), state_(3), state_(4), state_(5)};
}

std::optional<QuadPoseFilter::Twist> QuadPoseFilter::getTwist() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!initialized_) return std::nullopt;
    return Twist{state_(6), state_(7), state_(8), state_(9), state_(10), state_(11)};
}

std::optional<std::vector<Eigen::Vector2d>> QuadPoseFilter::getProjectedCentroids() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!initialized_) return std::nullopt;
    Vec3 rvec(state_(3), state_(4), state_(5));
    Vec3 tvec(state_(0), state_(1), state_(2));
    auto pts = projectPoints(model_points_, rvec, tvec);

    std::vector<Eigen::Vector2d> result(4);
    for (int i = 0; i < 4; ++i) {
        result[i] = Eigen::Vector2d(pts(i, 1), pts(i, 0));  // (row, col)
    }
    return result;
}

std::optional<QuadPoseFilter::AxesPoints> QuadPoseFilter::getAxes(double length) const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!initialized_) return std::nullopt;
    Vec3 rvec(state_(3), state_(4), state_(5));
    Vec3 tvec(state_(0), state_(1), state_(2));

    Eigen::Matrix<double, 4, 3> axes_3d;
    axes_3d << 0.0, 0.0, 0.0,
               length, 0.0, 0.0,
               0.0, length, 0.0,
               0.0, 0.0, length;

    auto pts = projectPoints(axes_3d, rvec, tvec);

    AxesPoints ax;
    ax.origin = {static_cast<int>(pts(0, 1)), static_cast<int>(pts(0, 0))};
    ax.x_tip  = {static_cast<int>(pts(1, 1)), static_cast<int>(pts(1, 0))};
    ax.y_tip  = {static_cast<int>(pts(2, 1)), static_cast<int>(pts(2, 0))};
    ax.z_tip  = {static_cast<int>(pts(3, 1)), static_cast<int>(pts(3, 0))};
    return ax;
}

}  // namespace epa
