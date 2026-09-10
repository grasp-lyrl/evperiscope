#pragma once

#include <string>
#include <optional>
#include <functional>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/estimator_status.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>

namespace epa {

/**
 * EPA Interface Node
 *
 * Bridges ZED VIO (FLU) → MAVROS vision pose (ENU), provides arming /
 * offboard / takeoff / land / loiter commands via MAVROS service clients, and
 * publishes position or velocity setpoints depending on the active mode.
 *
 * All MAVROS and ZED topics are looked up under the `robot_namespace`
 * parameter (default "kestrel1").
 *
 * takeoff() arms and starts streaming setpoints, but deliberately leaves the
 * switch into OFFBOARD to the safety operator. Set `auto_offboard` to true to
 * have it request OFFBOARD itself one second after arming.
 */
class EpaInterface : public rclcpp::Node {
public:
    using Callback = std::function<void(bool)>;

    explicit EpaInterface(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    // --- Public command interface (called by GUI or external nodes) ---
    void arm(Callback cb = nullptr);
    void disarm(Callback cb = nullptr);
    void offboard(Callback cb = nullptr);
    void takeoff(double height, Callback cb = nullptr);
    void land(Callback cb = nullptr);
    void loiter(Callback cb = nullptr);
    void setPosition(double x, double y, double z);
    void updateSetpointRelative(double dx, double dy, double dz);
    void setVelocity(double vx, double vy, double vz);

private:
    // --- Subscribers ---
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr zed_pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr mavros_odom_sub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<mavros_msgs::msg::EstimatorStatus>::SharedPtr estimator_sub_;

    // GUI command subscribers
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr takeoff_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr nudge_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_cmd_sub_;

    // --- Publishers for controller comms (gated by mode) ---
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr tracking_nudge_pub_;

    // --- GUI Mode Services ---
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_arm_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_disarm_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_offboard_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_land_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_abort_landing_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_tracking_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_position_;

    // EPA tracking velocity subscriber
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr tracking_vel_sub_;

    // --- Publishers ---
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vision_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;

    // --- Service clients ---
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    // Client to request controller-managed landing sequence
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr controller_land_client_;
    // Client to request controller abort landing
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr controller_abort_client_;
    // Subscription to controller landing status
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr landing_status_sub_;

    // --- Timers ---
    rclcpp::TimerBase::SharedPtr cmd_timer_;
    /// One-shot delay between arming and the OFFBOARD request, used only when
    /// auto_offboard_ is set; must outlive the arming callback that creates it.
    rclcpp::TimerBase::SharedPtr offboard_delay_timer_;

    // --- State ---
    std::string robot_ns_;   ///< Robot namespace for MAVROS / ZED topics
    bool auto_offboard_;     ///< Let takeoff() request OFFBOARD itself
    mavros_msgs::msg::State current_state_;
    std::optional<nav_msgs::msg::Odometry> current_odom_;
    geometry_msgs::msg::PoseStamped target_pose_;
    geometry_msgs::msg::TwistStamped target_velocity_;
    geometry_msgs::msg::Twist tracking_velocity_;    ///< Latest velocity from epa tracking
    enum class ControlMode { POSITION, VELOCITY, TRACKING };
    ControlMode control_mode_;
    bool takeoff_triggered_;
    rclcpp::Time last_tracking_vel_time_;             ///< Timestamp of last tracking velocity msg
    static constexpr double kTrackingTimeoutSec = 0.5; ///< Max age before stream considered stale
    bool vio_healthy_;

    // --- Callbacks ---
    void stateCallback(const mavros_msgs::msg::State::SharedPtr msg);
    void estimatorCallback(const mavros_msgs::msg::EstimatorStatus::SharedPtr msg);
    void zedPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void mavrosOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void cmdLoop();

    // GUI command callbacks
    void takeoffCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void positionNudgeCallback(const geometry_msgs::msg::Vector3::SharedPtr msg);
    void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void trackingVelocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    // Helper to forward nudges to controller if in TRACKING mode
    void forwardNudgeToController(const geometry_msgs::msg::Vector3& nudge);

    // GUI service callbacks
    void srvArmCb(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void srvDisarmCb(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void srvOffboardCb(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void srvLandCb(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void srvAbortLandingCb(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void srvTrackingCb(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void srvPositionCb(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res);

    /// Check if the tracking velocity stream is actively being received
    bool isTrackingStreamActive();

    // --- Helpers ---
    /// Quaternion multiply: q1 * q2 (Hamilton convention, [x,y,z,w])
    static std::array<double, 4> quatMultiply(const std::array<double, 4>& q1,
                                               const std::array<double, 4>& q2);
    /// Euler ZYX to quaternion [x,y,z,w]
    static std::array<double, 4> eulerToQuat(double roll, double pitch, double yaw);
    /// Quaternion [x,y,z,w] to Euler ZYX, returns [roll, pitch, yaw]
    static std::array<double, 3> quatToEuler(double qx, double qy, double qz, double qw);
};

}  // namespace epa
