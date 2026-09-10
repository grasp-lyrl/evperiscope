#pragma once

#include <string>
#include <memory>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <atomic>
#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <dv_ros2_msgs/msg/event_array.hpp>
#include <dv_ros2_messaging/messaging.hpp>
#include <dv-processing/processing.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>

#include "epa/ev_prop_det.hpp"
#include "epa/quad_pose_filter.hpp"
#include "epa/quad_controller.hpp"

namespace epa {

/// Unified controller state covering both tracking and landing phases.
enum class ControllerState : uint8_t {
    TRACKING,       ///< Normal PID tracking to the configured target
    LAND_DESCEND,   ///< Smoothly ramping Z target down to landing_altitude
    LAND_HOLD,      ///< Holding position, waiting for XY+Z thresholds
    LAND_COMMIT,    ///< Disabling PID, flushing zero-vel, requesting PX4 LAND
};

/**
 * ROS 2 node for event-based propeller pose estimation and tracking.
 *
 * Undistorts incoming events, detects the four propeller centroids, runs the
 * EKF, publishes the resulting pose, and drives the tracking / landing state
 * machine that produces velocity commands for EpaInterface.
 *
 * Two input modes, selected by the `input_mode` parameter:
 *   - "h5":    replays events from an HDF5 file on a wall timer
 *   - "topic": subscribes to a live event topic, processed on a worker thread
 */
class EpaController : public rclcpp::Node {
public:
    explicit EpaController(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~EpaController() override;

private:
    // --- Parameters ---
    std::string input_mode_;           // "h5" or "topic"
    std::string h5_filepath_;
    int image_h_, image_w_;
    double tau_;
    double duration_;                  // Time window per batch (seconds)
    double t_start_param_;             // Start time in H5 data (seconds)

    // --- Undistortion ---
    cv::Mat undist_map_x_;  ///< Per-pixel undistorted x lookup (h x w, float)
    cv::Mat undist_map_y_;  ///< Per-pixel undistorted y lookup (h x w, float)
    Eigen::Matrix3d K_rect_; ///< Rectified camera intrinsics

    /// Load camera calibration from OpenCV FileStorage XML
    void loadCalibration(const std::string& filepath,
                         cv::Mat& K, cv::Mat& dist_coeffs);

    /// Build per-pixel undistortion lookup table from distortion coefficients
    void buildUndistortMap(const cv::Mat& K, const cv::Mat& dist_coeffs);

    // --- Components ---
    std::unique_ptr<EvPropDet> detector_;
    std::unique_ptr<QuadPoseFilter> pose_filter_;
    std::unique_ptr<QuadController> quad_controller_;

    // --- Publishers ---
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_cmd_pub_;

    // --- Subscriptions / Services for GUI <-> Controller communication ---
    // Only receives nudges when quadrotor is in TRACKING mode
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr tracking_nudge_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr land_sequence_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr abort_landing_srv_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr interface_land_client_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr landing_status_pub_;

    // --- H5 mode ---
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<float> all_p_;
    std::vector<double> all_t_;
    std::vector<int> all_x_, all_y_;
    double t_current_;
    int frame_count_;
    double last_debug_time_{-1.0};

    // --- Timing ---
    // The EKF and the PID both need the interval that actually elapsed, which
    // is not the nominal batch duration once batches run late or are dropped.
    // Event timestamps are used rather than wall time, so an H5 replay gives
    // exactly the same result as the live run it was recorded from.
    double last_pose_update_t_{-1.0};  ///< Event time of the last EKF update
    double last_control_t_{-1.0};      ///< Event time of the last control step

    /**
     * Interval since @p last_t, clamped to a plausible range around the
     * nominal batch duration, and updates @p last_t to @p t_end.
     * Falls back to the nominal duration on the first call.
     */
    double measuredDt(double t_end, double& last_t);

    // --- Landing parameters (tunable) ---
    double landing_altitude_{0.5};
    double landing_xy_thresh_{0.05};
    double landing_z_thresh_{0.10};
    double landing_hold_time_{3.0};
    double landing_descent_rate_{0.3};
    double landing_timeout_{30.0};
    double landing_pose_grace_{0.5};

    // --- Controller state machine ---
    ControllerState ctrl_state_{ControllerState::TRACKING};
    double land_sequence_start_time_{0.0};   ///< Wall time when landing was initiated
    double land_hold_satisfied_start_{-1.0}; ///< Wall time when hold thresholds were first met
    double land_pose_lost_time_{-1.0};       ///< Wall time when pose was first lost during hold
    double land_last_status_time_{0.0};      ///< Wall time of last status publish
    int    land_commit_count_{0};            ///< Zero-vel messages sent during COMMIT phase

    void initH5Mode();
    void timerCallback();

    // --- Topic mode ---
    rclcpp::Subscription<dv_ros2_msgs::msg::EventArray>::SharedPtr event_sub_;
    dv::EventStreamSlicer slicer_;
    
    std::thread processing_thread_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::queue<dv::EventStore> event_queue_;
    std::atomic<bool> is_running_{false};

    void initTopicMode();
    void eventCallback(const dv_ros2_msgs::msg::EventArray::SharedPtr msg);
    void slicerCallback(const dv::EventStore& events);
    void processingLoop();

    /**
     * Detect, estimate pose, publish and run one control step for a batch of
     * undistorted events. Shared by both input modes.
     * @param p,t,x,y Event polarity, timestamp (s), column and row arrays
     * @param n       Number of events in the batch
     * @param t_end   End time of the batch (seconds)
     */
    void processEventBatch(const float* p, const double* t,
                           const int* x, const int* y, int n, double t_end);

    /// Draw reprojected model points, pose readout and body X axis onto @p image.
    void drawPoseOverlay(cv::Mat& image,
                         const std::optional<QuadPoseFilter::Pose>& pose);

    // --- Control state machine ---
    /// Guards ctrl_state_ and the land_* variables, which the processing
    /// thread and the executor's service callbacks both touch.
    std::mutex control_mutex_;

    /**
     * Run one step of the tracking/landing state machine.
     * Called from processEventBatch() after the pose is updated.
     * @param pose Current pose estimate (may be nullopt)
     * @param dt   Time step for PID (seconds)
     */
    void runControlStep(const std::optional<QuadPoseFilter::Pose>& pose, double dt);

    /// Transition to landing and reset landing state. Call with control_mutex_ held.
    void beginLandSequence(const std::optional<QuadPoseFilter::Pose>& pose);

    /// Abort landing and return to TRACKING. Call with control_mutex_ held.
    void abortLandSequence(const std::optional<QuadPoseFilter::Pose>& pose);

    // --- Helpers ---
    void publishPose(double timestamp);
    void publishDebugImage(const cv::Mat& image, double timestamp);

    // GUI callbacks
    void positionNudgeCallback(const geometry_msgs::msg::Vector3::SharedPtr msg);
    // Land sequence service handler
    void landSequenceHandler(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    // Abort landing service handler
    void abortLandingHandler(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    /// Publish landing status diagnostics
    void publishLandingStatus(const std::string& detail);
    /// Request PX4 LAND via the interface node (called async from COMMIT phase)
    void requestInterfaceLand();
};

}  // namespace epa
