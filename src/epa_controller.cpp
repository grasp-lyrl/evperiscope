#include "epa/epa_controller.hpp"
#include <rclcpp_components/register_node_macro.hpp>

#include <H5Cpp.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace epa {

EpaController::EpaController(const rclcpp::NodeOptions& options)
    : Node("epa_controller", options), t_current_(0.0), frame_count_(0) {
    this->declare_parameter<std::string>("input_mode", "h5");
    this->declare_parameter<std::string>("h5_filepath", "");
    this->declare_parameter<std::string>("calib_file", "");
    this->declare_parameter<double>("tau", 0.1);
    this->declare_parameter<double>("duration", 0.01);
    this->declare_parameter<double>("t_start", 15.0);
    this->declare_parameter<int>("n_centroids", 4);
    // Landing params
    this->declare_parameter<double>("landing_altitude", 0.5);
    this->declare_parameter<double>("landing_xy_thresh", 0.05);
    this->declare_parameter<double>("landing_z_thresh", 0.10);
    this->declare_parameter<double>("landing_hold_time", 3.0);
    this->declare_parameter<double>("landing_descent_rate", 0.3);
    this->declare_parameter<double>("landing_timeout", 30.0);
    this->declare_parameter<double>("landing_pose_grace", 0.5);

    // Quadrotor geometry
    this->declare_parameter<double>("front_arm_length", 0.13125);
    this->declare_parameter<double>("rear_arm_length", 0.11625);
    this->declare_parameter<double>("front_back_offset", 0.09625);

    // PID controller parameters
    this->declare_parameter<double>("pid_kp", 1.0);
    this->declare_parameter<double>("pid_ki", 0.0);
    this->declare_parameter<double>("pid_kd", 0.1);
    this->declare_parameter<double>("pid_kp_yaw", 0.5);
    this->declare_parameter<double>("pid_ki_yaw", 0.0);
    this->declare_parameter<double>("pid_kd_yaw", 0.05);
    this->declare_parameter<double>("target_x", 0.0);
    this->declare_parameter<double>("target_y", 0.0);
    this->declare_parameter<double>("target_z", 1.0);
    this->declare_parameter<double>("target_yaw", 0.0);
    this->declare_parameter<double>("max_vel_xy", 0.5);
    this->declare_parameter<double>("max_vel_z", 0.5);
    this->declare_parameter<double>("max_yaw_rate", 0.5);
    this->declare_parameter<bool>("controller_enabled", false);

    // Read parameters
    input_mode_ = this->get_parameter("input_mode").as_string();
    h5_filepath_ = this->get_parameter("h5_filepath").as_string();
    tau_ = this->get_parameter("tau").as_double();
    duration_ = this->get_parameter("duration").as_double();
    t_start_param_ = this->get_parameter("t_start").as_double();
    int n_centroids = this->get_parameter("n_centroids").as_int();

    landing_altitude_ = this->get_parameter("landing_altitude").as_double();
    landing_xy_thresh_ = this->get_parameter("landing_xy_thresh").as_double();
    landing_z_thresh_ = this->get_parameter("landing_z_thresh").as_double();
    landing_hold_time_ = this->get_parameter("landing_hold_time").as_double();
    landing_descent_rate_ = this->get_parameter("landing_descent_rate").as_double();
    landing_timeout_ = this->get_parameter("landing_timeout").as_double();
    landing_pose_grace_ = this->get_parameter("landing_pose_grace").as_double();

    double front_arm = this->get_parameter("front_arm_length").as_double();
    double rear_arm = this->get_parameter("rear_arm_length").as_double();
    double fb_offset = this->get_parameter("front_back_offset").as_double();

    // Load camera calibration from XML file
    std::string calib_file = this->get_parameter("calib_file").as_string();
    cv::Mat K_cv, dist_coeffs;
    loadCalibration(calib_file, K_cv, dist_coeffs);

    // Build undistortion lookup table and get rectified intrinsics
    buildUndistortMap(K_cv, dist_coeffs);

    // Initialize components with rectified intrinsics
    detector_ = std::make_unique<EvPropDet>(image_h_, image_w_, tau_, n_centroids);
    pose_filter_ = std::make_unique<QuadPoseFilter>(K_rect_, front_arm, rear_arm, fb_offset);

    // Publishers
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("epa/pose", 10);
    debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("epa/debug_image", 10);
    vel_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/epa/tracking/velocity", 10);

    // GUI <-> Controller comms
    tracking_nudge_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
        "/epa/tracking/position_nudge", 10,
        std::bind(&EpaController::positionNudgeCallback, this, std::placeholders::_1));

    land_sequence_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "/epa/cmd/land_sequence",
        std::bind(&EpaController::landSequenceHandler, this, std::placeholders::_1, std::placeholders::_2));

    abort_landing_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "/epa/cmd/abort_landing",
        std::bind(&EpaController::abortLandingHandler, this, std::placeholders::_1, std::placeholders::_2));

    interface_land_client_ = this->create_client<std_srvs::srv::Trigger>("/epa/cmd/land");
    landing_status_pub_ = this->create_publisher<std_msgs::msg::String>("/epa/landing_status", 10);

    // Initialize PID controller
    {
        double kp = this->get_parameter("pid_kp").as_double();
        double ki = this->get_parameter("pid_ki").as_double();
        double kd = this->get_parameter("pid_kd").as_double();
        double kp_yaw = this->get_parameter("pid_kp_yaw").as_double();
        double ki_yaw = this->get_parameter("pid_ki_yaw").as_double();
        double kd_yaw = this->get_parameter("pid_kd_yaw").as_double();
        double max_v_xy = this->get_parameter("max_vel_xy").as_double();
        double max_v_z = this->get_parameter("max_vel_z").as_double();
        double max_yr = this->get_parameter("max_yaw_rate").as_double();

        QuadController::PidGains gains_xy{kp, ki, kd};
        QuadController::PidGains gains_z{kp, ki, kd};
        QuadController::PidGains gains_yaw{kp_yaw, ki_yaw, kd_yaw};

        quad_controller_ = std::make_unique<QuadController>(
            gains_xy, gains_z, gains_yaw, max_v_xy, max_v_z, max_yr);

        double tx = this->get_parameter("target_x").as_double();
        double ty = this->get_parameter("target_y").as_double();
        double tz = this->get_parameter("target_z").as_double();
        double tyaw = this->get_parameter("target_yaw").as_double();
        quad_controller_->setTarget(tx, ty, tz, tyaw);

        bool enabled = this->get_parameter("controller_enabled").as_bool();
        quad_controller_->setEnabled(enabled);

        RCLCPP_INFO(this->get_logger(),
                    "PID controller: enabled=%d, target=(%.2f, %.2f, %.2f, %.1f°)",
                    enabled, tx, ty, tz, tyaw * 180.0 / M_PI);
    }

    RCLCPP_INFO(this->get_logger(), "EPA Controller starting in '%s' mode", input_mode_.c_str());

    if (input_mode_ == "h5") {
        initH5Mode();
    } else if (input_mode_ == "topic") {
        initTopicMode();
    } else {
        RCLCPP_ERROR(this->get_logger(), "Unknown input_mode: '%s'. Use 'h5' or 'topic'.",
                     input_mode_.c_str());
    }
}

EpaController::~EpaController() {
    if (input_mode_ == "topic") {
        is_running_ = false;
        queue_cv_.notify_all();
        if (processing_thread_.joinable()) {
            processing_thread_.join();
        }
    }
}

// ---------- H5 Mode ----------

void EpaController::initH5Mode() {
    if (h5_filepath_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "h5_filepath parameter is required for h5 mode");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Loading H5 file: %s", h5_filepath_.c_str());

    try {
        H5::H5File file(h5_filepath_, H5F_ACC_RDONLY);
        H5::Group events = file.openGroup("events");

        // Get sizes
        {
            H5::DataSet ds = events.openDataSet("t");
            H5::DataSpace space = ds.getSpace();
            hsize_t dims[1];
            space.getSimpleExtentDims(dims);
            size_t n = dims[0];

            // Read raw arrays
            std::vector<int64_t> t_raw(n);
            std::vector<uint16_t> x_raw(n), y_raw(n);
            std::vector<uint8_t> p_raw(n);

            events.openDataSet("t").read(t_raw.data(), H5::PredType::NATIVE_INT64);
            events.openDataSet("x").read(x_raw.data(), H5::PredType::NATIVE_UINT16);
            events.openDataSet("y").read(y_raw.data(), H5::PredType::NATIVE_UINT16);
            events.openDataSet("p").read(p_raw.data(), H5::PredType::NATIVE_UINT8);

            RCLCPP_INFO(this->get_logger(), "Loaded %zu events", n);

            // Convert timestamps to seconds (relative to first event, microseconds -> seconds)
            double t0 = static_cast<double>(t_raw[0]);
            all_t_.resize(n);
            all_p_.resize(n);
            all_x_.resize(n);
            all_y_.resize(n);

            for (size_t i = 0; i < n; ++i) {
                all_t_[i] = (static_cast<double>(t_raw[i]) - t0) / 1e6;
                all_p_[i] = static_cast<float>(p_raw[i]);

                // Undistort event coordinates via lookup table
                int raw_x = static_cast<int>(x_raw[i]);
                int raw_y = static_cast<int>(y_raw[i]);
                if (raw_x >= 0 && raw_x < image_w_ && raw_y >= 0 && raw_y < image_h_) {
                    all_x_[i] = static_cast<int>(std::round(
                        undist_map_x_.at<float>(raw_y, raw_x)));
                    all_y_[i] = static_cast<int>(std::round(
                        undist_map_y_.at<float>(raw_y, raw_x)));
                } else {
                    all_x_[i] = raw_x;
                    all_y_[i] = raw_y;
                }
            }

            RCLCPP_INFO(this->get_logger(), "Total time duration: %.3f seconds",
                        all_t_.back());
        }

        file.close();
    } catch (const H5::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load H5 file: %s", e.getCDetailMsg());
        return;
    }

    t_current_ = t_start_param_;
    frame_count_ = 0;

    // Create timer at ~100Hz (10ms period matching the batch duration)
    auto period = std::chrono::duration<double>(duration_);
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&EpaController::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "H5 mode initialized. Starting at t=%.3f s", t_start_param_);
}

void EpaController::timerCallback() {
    double t_end = t_current_ + duration_;

    // Check if we've reached the end of data
    if (all_t_.empty() || t_end > all_t_.back()) {
        RCLCPP_INFO(this->get_logger(), "Reached end of data after %d frames", frame_count_);
        timer_->cancel();
        return;
    }

    // Extract event slice using binary search
    auto it_start = std::lower_bound(all_t_.begin(), all_t_.end(), t_current_);
    auto it_end = std::lower_bound(all_t_.begin(), all_t_.end(), t_end);

    int idx_start = static_cast<int>(it_start - all_t_.begin());
    int idx_end = static_cast<int>(it_end - all_t_.begin());
    int n_events = idx_end - idx_start;

    RCLCPP_INFO(this->get_logger(),
                "Frame %d: Processing events in t=[%.3f, %.3f) s | events=%d",
                frame_count_, t_current_, t_end, n_events);

    if (n_events > 0) {
        processEventBatch(all_p_.data() + idx_start,
                          all_t_.data() + idx_start,
                          all_x_.data() + idx_start,
                          all_y_.data() + idx_start,
                          n_events, t_end);
    }

    t_current_ += duration_;
    frame_count_++;
}

// ---------- Topic Mode ----------
void EpaController::initTopicMode() {
    RCLCPP_INFO(this->get_logger(),
                "Topic mode initialized. Subscribing to 'events' topic.");

    event_sub_ = this->create_subscription<dv_ros2_msgs::msg::EventArray>(
        "events", 10,
        std::bind(&EpaController::eventCallback, this, std::placeholders::_1));

    is_running_ = true;
    processing_thread_ = std::thread(&EpaController::processingLoop, this);

    int64_t delta_time_us = static_cast<int64_t>(duration_ * 1e6);
    slicer_.doEveryTimeInterval(
        dv::Duration(delta_time_us),
        std::bind(&EpaController::slicerCallback, this, std::placeholders::_1)
    );
}

void EpaController::eventCallback(const dv_ros2_msgs::msg::EventArray::SharedPtr msg) {
    if (!msg || msg->events.empty()) return;

    try {
        slicer_.accept(dv_ros2_msgs::toEventStore(*msg));
    } catch (const std::out_of_range& e) {
        RCLCPP_WARN_STREAM(this->get_logger(), "Event out of range: " << e.what());
    }
}

void EpaController::slicerCallback(const dv::EventStore& events) {
    if (events.isEmpty()) return;
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        event_queue_.push(events);
    }
    queue_cv_.notify_one();
}

void EpaController::processingLoop() {
    while (is_running_) {
        dv::EventStore events;
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_cv_.wait(lock, [this]() { return !event_queue_.empty() || !is_running_; });

            if (!is_running_ && event_queue_.empty()) {
                break;
            }

            events = event_queue_.front();
            event_queue_.pop();
        }

        int n = static_cast<int>(events.size());
        if (n == 0) continue;

        // Prepare arrays for detector
        std::vector<float> p(n);
        std::vector<double> t(n);
        std::vector<int> x(n), y(n);

        // Fill arrays from sliced events
        int i = 0;
        for (const auto& e : events) {
            p[i] = e.polarity() ? 1.0f : 0.0f;
            t[i] = static_cast<double>(e.timestamp()) * 1e-6; // timestamps in seconds

            int raw_x = static_cast<int>(e.x());
            int raw_y = static_cast<int>(e.y());
            if (raw_x >= 0 && raw_x < image_w_ && raw_y >= 0 && raw_y < image_h_) {
                x[i] = static_cast<int>(std::round(undist_map_x_.at<float>(raw_y, raw_x)));
                y[i] = static_cast<int>(std::round(undist_map_y_.at<float>(raw_y, raw_x)));
            } else {
                x[i] = raw_x;
                y[i] = raw_y;
            }
            i++;
        }

        double t_end = static_cast<double>(events.getHighestTime() * 1e-6);
        processEventBatch(p.data(), t.data(), x.data(), y.data(), n, t_end);
    }
}

double EpaController::measuredDt(double t_end, double& last_t) {
    double dt = duration_;

    if (last_t >= 0.0) {
        const double min_dt = 0.1 * duration_;
        const double max_dt = 10.0 * duration_;
        dt = t_end - last_t;

        if (dt < min_dt || dt > max_dt) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Measured interval %.4f s outside [%.4f, %.4f] s, clamping",
                                 dt, min_dt, max_dt);
            dt = std::clamp(dt, min_dt, max_dt);
        }
    }

    last_t = t_end;
    return dt;
}

/**
 * Detect, estimate pose, publish, and run one control step for a batch of
 * events. Shared by the H5 timer and the topic-mode worker thread; only one
 * of the two is ever active, so no locking is needed for the detector state.
 */
void EpaController::processEventBatch(const float* p, const double* t,
                                      const int* x, const int* y,
                                      int n, double t_end) {
    // Debug images are expensive, so cap them at roughly 10 Hz
    const bool render_debug = (last_debug_time_ < 0.0 || t_end - last_debug_time_ >= 0.1);

    auto start_time = std::chrono::high_resolution_clock::now();
    auto result = detector_->processFrame(p, t, x, y, n, t_end, render_debug);
    auto detect_end = std::chrono::high_resolution_clock::now();
    double detect_ms = std::chrono::duration<double, std::milli>(detect_end - start_time).count();

    double pose_ms = 0.0;
    if (result.is_stable && !result.centroids.empty()) {
        auto pose_start = std::chrono::high_resolution_clock::now();
        pose_filter_->update(result.centroids, measuredDt(t_end, last_pose_update_t_));
        auto pose_end = std::chrono::high_resolution_clock::now();
        pose_ms = std::chrono::duration<double, std::milli>(pose_end - pose_start).count();

        auto pose = pose_filter_->getPose();
        if (render_debug) {
            drawPoseOverlay(result.image, pose);
        }

        if (pose) {
            publishPose(t_end);
            runControlStep(pose, measuredDt(t_end, last_control_t_));
        }
    } else {
        bool landing;
        {
            std::lock_guard<std::mutex> lock(control_mutex_);
            landing = (ctrl_state_ != ControllerState::TRACKING);
        }
        // Tick the landing state machine anyway so it can react to pose loss
        if (landing) {
            runControlStep(std::nullopt, measuredDt(t_end, last_control_t_));
        }
    }

    if (render_debug) {
        char timing_buf[128];
        snprintf(timing_buf, sizeof(timing_buf),
                 "t=%.3fs | det:%.1fms | pose:%.1fms", t_end, detect_ms, pose_ms);
        cv::putText(result.image, timing_buf, cv::Point(10, image_h_ - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
        publishDebugImage(result.image, t_end);
        last_debug_time_ = t_end;
    }

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "t=%.3fs, detect=%.2fms, pose=%.2fms, stable=%d",
                         t_end, detect_ms, pose_ms, result.is_stable);
}

/// Draw the reprojected model points, pose readout and body X axis.
void EpaController::drawPoseOverlay(cv::Mat& image,
                                    const std::optional<QuadPoseFilter::Pose>& pose) {
    if (image.empty()) return;

    auto projected = pose_filter_->getProjectedCentroids();
    if (projected) {
        for (const auto& pt : *projected) {
            cv::circle(image, cv::Point(static_cast<int>(pt(1)), static_cast<int>(pt(0))),
                       8, cv::Scalar(0, 255, 0), 2);
        }
    }

    if (!pose) return;

    char buf[128];
    snprintf(buf, sizeof(buf), "Pos: %.2f %.2f %.2f", pose->x, pose->y, pose->z);
    cv::putText(image, buf, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX,
                0.6, cv::Scalar(255, 255, 255), 2);

    snprintf(buf, sizeof(buf), "Att: %.1f %.1f %.1f",
             pose->roll * 180.0 / M_PI, pose->pitch * 180.0 / M_PI,
             pose->yaw * 180.0 / M_PI);
    cv::putText(image, buf, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX,
                0.6, cv::Scalar(255, 255, 255), 2);

    auto axes = pose_filter_->getAxes(0.2);
    if (axes) {
        cv::arrowedLine(image,
                        cv::Point(axes->origin[1], axes->origin[0]),
                        cv::Point(axes->x_tip[1], axes->x_tip[0]),
                        cv::Scalar(0, 0, 255), 3, cv::LINE_8, 0, 0.2);
    }
}

// ---------- Control state machine ----------

void EpaController::runControlStep(const std::optional<QuadPoseFilter::Pose>& pose, double dt) {
    if (!quad_controller_) return;

    std::lock_guard<std::mutex> lock(control_mutex_);
    double now = this->now().seconds();

    switch (ctrl_state_) {
    // ---- Normal tracking ----
    case ControllerState::TRACKING: {
        if (pose && quad_controller_->isEnabled()) {
            auto vel_cmd = quad_controller_->compute(*pose, dt);
            vel_cmd_pub_->publish(vel_cmd);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "Vel cmd: x=%.2f, y=%.2f, z=%.2f, yaw=%.2f",
                        vel_cmd.linear.x, vel_cmd.linear.y, vel_cmd.linear.z, vel_cmd.angular.z);
        }
        break;
    }

    // ---- Landing: descend ----
    case ControllerState::LAND_DESCEND: {
        // Check timeout
        if (now - land_sequence_start_time_ >= landing_timeout_) {
            RCLCPP_WARN(this->get_logger(), "Landing timed out during DESCEND after %.1fs",
                        now - land_sequence_start_time_);
            abortLandSequence(pose);
            return;
        }

        // Ramp Z target down
        auto tgt = quad_controller_->getTarget();
        tgt.z -= landing_descent_rate_ * dt;
        if (tgt.z <= landing_altitude_) {
            tgt.z = landing_altitude_;
            ctrl_state_ = ControllerState::LAND_HOLD;
            RCLCPP_INFO(this->get_logger(),
                        "Descent complete, entering LAND_HOLD at z=%.2f", tgt.z);
        }
        quad_controller_->setTarget(tgt);

        // Still run PID while descending
        if (pose) {
            auto vel_cmd = quad_controller_->compute(*pose, dt);
            vel_cmd_pub_->publish(vel_cmd);
        }

        // Publish status
        if (now - land_last_status_time_ >= 0.5) {
            auto tgt_now = quad_controller_->getTarget();
            char buf[256];
            snprintf(buf, sizeof(buf),
                     "phase=DESCEND target_z=%.2f timeout=%.0f/%.0f",
                     tgt_now.z, now - land_sequence_start_time_, landing_timeout_);
            publishLandingStatus(buf);
            land_last_status_time_ = now;
        }
        break;
    }

    // ---- Landing: hold ----
    case ControllerState::LAND_HOLD: {
        // Check timeout
        if (now - land_sequence_start_time_ >= landing_timeout_) {
            RCLCPP_WARN(this->get_logger(), "Landing timed out during HOLD after %.1fs",
                        now - land_sequence_start_time_);
            abortLandSequence(pose);
            return;
        }

        auto tgt = quad_controller_->getTarget();

        if (!pose) {
            // Pose lost — apply grace period
            if (land_pose_lost_time_ < 0.0) {
                land_pose_lost_time_ = now;
            }
            if (now - land_pose_lost_time_ >= landing_pose_grace_) {
                RCLCPP_WARN(this->get_logger(),
                            "Pose lost for %.1fs (> grace %.1fs), resetting hold timer",
                            now - land_pose_lost_time_, landing_pose_grace_);
                land_hold_satisfied_start_ = -1.0;
                land_pose_lost_time_ = -1.0;
            }
            // Keep publishing last velocity (PID holds state internally)
            break;
        }

        // Pose valid — clear pose-lost tracker
        land_pose_lost_time_ = -1.0;

        // Run PID
        auto vel_cmd = quad_controller_->compute(*pose, dt);
        vel_cmd_pub_->publish(vel_cmd);

        // Check thresholds
        double dx = pose->x - tgt.x;
        double dy = pose->y - tgt.y;
        double dz = pose->z - tgt.z;
        double dist_xy = std::sqrt(dx * dx + dy * dy);

        if (dist_xy <= landing_xy_thresh_ && std::abs(dz) <= landing_z_thresh_) {
            if (land_hold_satisfied_start_ < 0.0) {
                land_hold_satisfied_start_ = now;
                RCLCPP_INFO(this->get_logger(),
                            "Position within thresholds, starting %.1fs hold", landing_hold_time_);
            }
            if (now - land_hold_satisfied_start_ >= landing_hold_time_) {
                RCLCPP_INFO(this->get_logger(),
                            "Hold satisfied for %.1fs, transitioning to LAND_COMMIT",
                            landing_hold_time_);
                ctrl_state_ = ControllerState::LAND_COMMIT;
                land_commit_count_ = 0;
                // Disable PID — we're about to hand off to PX4
                quad_controller_->setEnabled(false);
                return;
            }
        } else {
            if (land_hold_satisfied_start_ > 0.0) {
                RCLCPP_INFO(this->get_logger(),
                            "Position drifted (xy=%.3f z=%.3f), resetting hold timer",
                            dist_xy, dz);
            }
            land_hold_satisfied_start_ = -1.0;
        }

        // Publish status
        if (now - land_last_status_time_ >= 0.5) {
            double hold_progress = (land_hold_satisfied_start_ > 0.0)
                                 ? (now - land_hold_satisfied_start_) : 0.0;
            char buf[256];
            snprintf(buf, sizeof(buf),
                     "phase=HOLD xy_err=%.3f z_err=%.3f hold=%.1f/%.1f timeout=%.0f/%.0f",
                     dist_xy, dz, hold_progress, landing_hold_time_,
                     now - land_sequence_start_time_, landing_timeout_);
            publishLandingStatus(buf);
            land_last_status_time_ = now;
        }
        break;
    }

    // ---- Landing: commit ----
    case ControllerState::LAND_COMMIT: {
        // Send zero-velocity for several cycles to ensure PX4 receives it
        vel_cmd_pub_->publish(geometry_msgs::msg::Twist());
        land_commit_count_++;

        // After 10 cycles, request PX4 LAND and return to TRACKING (idle)
        if (land_commit_count_ >= 10) {
            RCLCPP_INFO(this->get_logger(),
                        "Zero-vel flushed (%d msgs), requesting PX4 LAND via interface",
                        land_commit_count_);
            requestInterfaceLand();
            ctrl_state_ = ControllerState::TRACKING;
            publishLandingStatus("complete");
        }
        break;
    }
    }  // switch
}

void EpaController::beginLandSequence(const std::optional<QuadPoseFilter::Pose>& pose) {
    if (!quad_controller_ || !pose_filter_) {
        RCLCPP_WARN(this->get_logger(), "Cannot begin landing: controller or pose filter unavailable");
        return;
    }

    // Determine starting Z from current pose or controller target
    double start_z = landing_altitude_;
    if (pose) {
        start_z = pose->z;
    } else {
        start_z = quad_controller_->getTarget().z;
    }
    if (start_z <= landing_altitude_) {
        start_z = landing_altitude_;
    }

    // Set target to center at current altitude
    QuadController::Target tgt;
    tgt.x = 0.0;
    tgt.y = 0.0;
    tgt.z = start_z;
    tgt.yaw = 0.0;
    quad_controller_->setTarget(tgt);
    quad_controller_->setEnabled(true);

    // Reset landing state
    land_sequence_start_time_ = this->now().seconds();
    land_hold_satisfied_start_ = -1.0;
    land_pose_lost_time_ = -1.0;
    land_last_status_time_ = 0.0;
    land_commit_count_ = 0;

    // Skip descent if already at landing altitude
    if (start_z <= landing_altitude_) {
        ctrl_state_ = ControllerState::LAND_HOLD;
        RCLCPP_INFO(this->get_logger(),
                    "Already at landing altitude (z=%.2f), entering LAND_HOLD directly", start_z);
    } else {
        ctrl_state_ = ControllerState::LAND_DESCEND;
        RCLCPP_INFO(this->get_logger(),
                    "Landing sequence started: descending from z=%.2f to z=%.2f at %.2f m/s",
                    start_z, landing_altitude_, landing_descent_rate_);
    }
}

void EpaController::abortLandSequence(const std::optional<QuadPoseFilter::Pose>& pose) {
    RCLCPP_INFO(this->get_logger(), "Landing sequence aborted, returning to TRACKING hold");

    // Hold current position
    if (quad_controller_ && pose) {
        QuadController::Target hold_tgt;
        hold_tgt.x = pose->x;
        hold_tgt.y = pose->y;
        hold_tgt.z = pose->z;
        hold_tgt.yaw = pose->yaw;
        quad_controller_->setTarget(hold_tgt);
        // Keep controller enabled so it holds
    }

    ctrl_state_ = ControllerState::TRACKING;
    publishLandingStatus("aborted");
}

// ---------- GUI callbacks ----------

void EpaController::positionNudgeCallback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
    if (!quad_controller_) return;

    std::lock_guard<std::mutex> lock(control_mutex_);

    // Nudges only apply during TRACKING
    if (ctrl_state_ != ControllerState::TRACKING) {
        RCLCPP_WARN(this->get_logger(), "Ignoring position nudge during landing sequence");
        return;
    }

    // Treat the incoming vector as a relative delta in meters (dx, dy, dz)
    auto target = quad_controller_->getTarget();
    double dx = msg->x;
    double dy = msg->y;
    double dz = msg->z;

    // Apply delta to current target
    target.x += dx;
    target.y += dy;
    target.z += dz;
    quad_controller_->setTarget(target);

    RCLCPP_INFO(this->get_logger(), "Position nudge applied: dx=%.3f dy=%.3f dz=%.3f -> target=(%.2f,%.2f,%.2f)",
                dx, dy, dz, target.x, target.y, target.z);
}

void EpaController::landSequenceHandler(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    (void)req;
    std::lock_guard<std::mutex> lock(control_mutex_);

    if (ctrl_state_ != ControllerState::TRACKING) {
        res->success = false;
        res->message = "Landing sequence already in progress";
        return;
    }

    auto pose = pose_filter_ ? pose_filter_->getPose() : std::nullopt;
    beginLandSequence(pose);

    res->success = true;
    res->message = "Landing sequence started";
}

void EpaController::abortLandingHandler(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    (void)req;
    std::lock_guard<std::mutex> lock(control_mutex_);

    if (ctrl_state_ == ControllerState::TRACKING) {
        res->success = false;
        res->message = "No landing sequence in progress";
        return;
    }

    auto pose = pose_filter_ ? pose_filter_->getPose() : std::nullopt;
    abortLandSequence(pose);

    res->success = true;
    res->message = "Landing aborted, holding position";
}

void EpaController::publishLandingStatus(const std::string& detail) {
    auto msg = std_msgs::msg::String();
    msg.data = detail;
    landing_status_pub_->publish(msg);
}

void EpaController::requestInterfaceLand() {
    if (!interface_land_client_->wait_for_service(std::chrono::milliseconds(100))) {
        RCLCPP_ERROR(this->get_logger(),
                     "Interface land service not available");
        return;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    interface_land_client_->async_send_request(request,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
            try {
                auto result = future.get();
                if (result && result->success) {
                    RCLCPP_INFO(this->get_logger(), "PX4 LAND requested via interface — success");
                } else {
                    RCLCPP_WARN(this->get_logger(), "Interface land service returned failure: %s",
                                result ? result->message.c_str() : "null response");
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Exception in interface land callback: %s", e.what());
            }
        });
}

// ---------- Publishing ----------

void EpaController::publishPose(double timestamp) {
    auto pose_opt = pose_filter_->getPose();
    if (!pose_opt) return;

    auto msg = geometry_msgs::msg::PoseStamped();
    msg.header.stamp = rclcpp::Time(static_cast<int64_t>(timestamp * 1e9));
    msg.header.frame_id = "camera";

    msg.pose.position.x = pose_opt->x;
    msg.pose.position.y = pose_opt->y;
    msg.pose.position.z = pose_opt->z;

    // Convert Euler angles to quaternion (ZYX convention)
    double cr = std::cos(pose_opt->roll / 2.0);
    double sr = std::sin(pose_opt->roll / 2.0);
    double cp = std::cos(pose_opt->pitch / 2.0);
    double sp = std::sin(pose_opt->pitch / 2.0);
    double cy = std::cos(pose_opt->yaw / 2.0);
    double sy = std::sin(pose_opt->yaw / 2.0);

    msg.pose.orientation.w = cr * cp * cy + sr * sp * sy;
    msg.pose.orientation.x = sr * cp * cy - cr * sp * sy;
    msg.pose.orientation.y = cr * sp * cy + sr * cp * sy;
    msg.pose.orientation.z = cr * cp * sy - sr * sp * cy;

    pose_pub_->publish(msg);
}

void EpaController::publishDebugImage(const cv::Mat& image, double timestamp) {
    auto msg = cv_bridge::CvImage(
        std_msgs::msg::Header(),
        "bgr8",
        image
    ).toImageMsg();

    msg->header.stamp = rclcpp::Time(static_cast<int64_t>(timestamp * 1e9));
    msg->header.frame_id = "camera";

    debug_image_pub_->publish(*msg);
}

// ---------- Undistortion ----------

void EpaController::loadCalibration(const std::string& filepath,
                               cv::Mat& K, cv::Mat& dist_coeffs) {
    if (filepath.empty()) {
        RCLCPP_ERROR(this->get_logger(), "calib_file parameter is empty");
        throw std::runtime_error("calib_file parameter is required");
    }

    cv::FileStorage fs(filepath, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Cannot open calibration file: %s", filepath.c_str());
        throw std::runtime_error("Failed to open calibration file: " + filepath);
    }

    // Find the camera node (first child with camera_matrix)
    cv::FileNode cam_node;
    for (auto it = fs.root().begin(); it != fs.root().end(); ++it) {
        cv::FileNode n = *it;
        if (!n["camera_matrix"].empty()) {
            cam_node = n;
            break;
        }
    }

    if (cam_node.empty()) {
        RCLCPP_ERROR(this->get_logger(), "No camera node found in calibration file");
        throw std::runtime_error("No camera node in calibration file");
    }

    cam_node["camera_matrix"] >> K;
    cam_node["distortion_coefficients"] >> dist_coeffs;
    image_w_ = static_cast<int>(cam_node["image_width"]);
    image_h_ = static_cast<int>(cam_node["image_height"]);

    fs.release();

    RCLCPP_INFO(this->get_logger(),
                "Loaded calibration from %s: %dx%d, fx=%.2f fy=%.2f",
                filepath.c_str(), image_w_, image_h_,
                K.at<double>(0, 0), K.at<double>(1, 1));
}

void EpaController::buildUndistortMap(const cv::Mat& K, const cv::Mat& dist_coeffs) {
    RCLCPP_INFO(this->get_logger(), "Building undistortion lookup table (%dx%d)...",
                image_w_, image_h_);

    // Get optimal new camera matrix (alpha=0: crop to valid pixels)
    cv::Mat K_new = cv::getOptimalNewCameraMatrix(
        K, dist_coeffs, cv::Size(image_w_, image_h_), 0.0);

    // Build all pixel coordinates as a (N x 1 x 2) matrix
    int n_pixels = image_h_ * image_w_;
    cv::Mat pixel_coords(n_pixels, 1, CV_32FC2);
    for (int y = 0; y < image_h_; ++y) {
        for (int x = 0; x < image_w_; ++x) {
            pixel_coords.at<cv::Vec2f>(y * image_w_ + x) =
                cv::Vec2f(static_cast<float>(x), static_cast<float>(y));
        }
    }

    // Undistort all points at once
    cv::Mat undistorted;
    cv::undistortPoints(pixel_coords, undistorted, K, dist_coeffs, cv::noArray(), K_new);

    // Store as lookup maps
    undist_map_x_ = cv::Mat(image_h_, image_w_, CV_32F);
    undist_map_y_ = cv::Mat(image_h_, image_w_, CV_32F);

    for (int y = 0; y < image_h_; ++y) {
        for (int x = 0; x < image_w_; ++x) {
            cv::Vec2f pt = undistorted.at<cv::Vec2f>(y * image_w_ + x);
            undist_map_x_.at<float>(y, x) = pt[0];  // undistorted x
            undist_map_y_.at<float>(y, x) = pt[1];  // undistorted y
        }
    }

    // Store rectified intrinsics as Eigen matrix for pose filter
    K_rect_ = Eigen::Matrix3d::Zero();
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            K_rect_(r, c) = K_new.at<double>(r, c);

    RCLCPP_INFO(this->get_logger(),
                "Rectified intrinsics: fx=%.2f fy=%.2f cx=%.2f cy=%.2f",
                K_rect_(0, 0), K_rect_(1, 1), K_rect_(0, 2), K_rect_(1, 2));
}

}  // namespace epa

RCLCPP_COMPONENTS_REGISTER_NODE(epa::EpaController)
