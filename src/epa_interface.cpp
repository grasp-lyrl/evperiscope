#include "epa/epa_interface.hpp"
#include <rclcpp_components/register_node_macro.hpp>

#include <chrono>

namespace epa {

EpaInterface::EpaInterface(const rclcpp::NodeOptions& options)
    : Node("epa_interface", options),
      control_mode_(ControlMode::POSITION),
      takeoff_triggered_(false),
      last_tracking_vel_time_(0, 0, RCL_ROS_TIME),
      vio_healthy_(false) {

    // All MAVROS and ZED topics live under the robot namespace
    robot_ns_ = this->declare_parameter<std::string>("robot_namespace", "kestrel1");
    const std::string ns = "/" + robot_ns_;
    RCLCPP_INFO(this->get_logger(), "Using robot namespace '%s'", ns.c_str());

    // Off by default: mode changes are the safety operator's call
    auto_offboard_ = this->declare_parameter<bool>("auto_offboard", false);
    if (auto_offboard_) {
        RCLCPP_WARN(this->get_logger(),
                    "auto_offboard is enabled: takeoff will request OFFBOARD by itself");
    }

    // --- Subscribers ---
    zed_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        ns + "/zed_node/pose", 10,
        std::bind(&EpaInterface::zedPoseCallback, this, std::placeholders::_1));

    mavros_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        ns + "/mavros/local_position/odom", 10,
        std::bind(&EpaInterface::mavrosOdomCallback, this, std::placeholders::_1));

    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
        ns + "/mavros/state", 10,
        std::bind(&EpaInterface::stateCallback, this, std::placeholders::_1));

    estimator_sub_ = this->create_subscription<mavros_msgs::msg::EstimatorStatus>(
        ns + "/mavros/estimator_status", 10,
        std::bind(&EpaInterface::estimatorCallback, this, std::placeholders::_1));

    srv_arm_ = this->create_service<std_srvs::srv::Trigger>(
        "/epa/cmd/arm", std::bind(&EpaInterface::srvArmCb, this, std::placeholders::_1, std::placeholders::_2));
    srv_disarm_ = this->create_service<std_srvs::srv::Trigger>(
        "/epa/cmd/disarm", std::bind(&EpaInterface::srvDisarmCb, this, std::placeholders::_1, std::placeholders::_2));
    srv_offboard_ = this->create_service<std_srvs::srv::Trigger>(
        "/epa/cmd/offboard", std::bind(&EpaInterface::srvOffboardCb, this, std::placeholders::_1, std::placeholders::_2));
    srv_land_ = this->create_service<std_srvs::srv::Trigger>(
        "/epa/cmd/land", std::bind(&EpaInterface::srvLandCb, this, std::placeholders::_1, std::placeholders::_2));
    srv_abort_landing_ = this->create_service<std_srvs::srv::Trigger>(
        "/epa/cmd/abort_landing_gui",
        std::bind(&EpaInterface::srvAbortLandingCb, this, std::placeholders::_1, std::placeholders::_2));
    srv_tracking_ = this->create_service<std_srvs::srv::Trigger>(
        "/epa/cmd/tracking", std::bind(&EpaInterface::srvTrackingCb, this, std::placeholders::_1, std::placeholders::_2));
    srv_position_ = this->create_service<std_srvs::srv::Trigger>(
        "/epa/cmd/position", std::bind(&EpaInterface::srvPositionCb, this, std::placeholders::_1, std::placeholders::_2));

    takeoff_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/epa/cmd/takeoff", 10,
        std::bind(&EpaInterface::takeoffCallback, this, std::placeholders::_1));

    nudge_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
        "/epa/cmd/position_nudge", 10,
        std::bind(&EpaInterface::positionNudgeCallback, this, std::placeholders::_1));

    vel_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/epa/cmd/velocity", 10,
        std::bind(&EpaInterface::velocityCallback, this, std::placeholders::_1));

    tracking_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/epa/tracking/velocity", 10,
        std::bind(&EpaInterface::trackingVelocityCallback, this, std::placeholders::_1));

    // --- Publishers ---
    local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        ns + "/mavros/setpoint_position/local", 10);

    velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        ns + "/mavros/setpoint_velocity/cmd_vel", 10);

    vision_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        ns + "/mavros/vision_pose/pose", 10);

    // --- Service clients ---
    arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>(
        ns + "/mavros/cmd/arming");
    set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>(
        ns + "/mavros/set_mode");

    // Controller land sequence client (only used in TRACKING mode)
    controller_land_client_ = this->create_client<std_srvs::srv::Trigger>("/epa/cmd/land_sequence");
    // Controller abort landing client
    controller_abort_client_ = this->create_client<std_srvs::srv::Trigger>("/epa/cmd/abort_landing");
    // Subscribe to landing status from controller
    landing_status_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/epa/landing_status", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            RCLCPP_INFO(this->get_logger(), "Landing status: %s", msg->data.c_str());
        });

    // --- Publishers for tracking-mode controller comms ---
    tracking_nudge_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
        "/epa/tracking/position_nudge", 10);

    // --- Target pose init ---
    target_pose_.pose.position.x = 0.0;
    target_pose_.pose.position.y = 0.0;
    target_pose_.pose.position.z = 0.0;
    target_pose_.pose.orientation.w = 1.0;

    // --- 20 Hz command loop ---
    cmd_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&EpaInterface::cmdLoop, this));

    RCLCPP_INFO(this->get_logger(), "EPA Interface initialized");
}

// ============================================================
// Subscriber callbacks
// ============================================================

void EpaInterface::stateCallback(const mavros_msgs::msg::State::SharedPtr msg) {
    current_state_ = *msg;
}

void EpaInterface::estimatorCallback(
    const mavros_msgs::msg::EstimatorStatus::SharedPtr msg) {
    vio_healthy_ = (msg->pos_horiz_rel_status_flag ||
                    msg->pos_horiz_abs_status_flag) &&
                   !msg->accel_error_status_flag;
}

void EpaInterface::zedPoseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // Convert ZED FLU → ENU for MAVROS
    double zed_x = msg->pose.position.x;   // forward
    double zed_y = msg->pose.position.y;   // left
    double zed_z = msg->pose.position.z;   // up

    // FLU → ENU: East = -left, North = forward, Up = up
    double enu_x = -zed_y;
    double enu_y =  zed_x;
    double enu_z =  zed_z;

    // Rotate orientation by 90° yaw (FLU → ENU)
    auto q_rot = eulerToQuat(0.0, 0.0, M_PI / 2.0);  // [x,y,z,w]
    std::array<double, 4> zed_quat = {
        msg->pose.orientation.x, msg->pose.orientation.y,
        msg->pose.orientation.z, msg->pose.orientation.w};
    auto enu_quat = quatMultiply(q_rot, zed_quat);

    geometry_msgs::msg::PoseStamped vision_pose;
    vision_pose.header.stamp = msg->header.stamp;
    vision_pose.header.frame_id = "odom";
    vision_pose.pose.position.x = enu_x;
    vision_pose.pose.position.y = enu_y;
    vision_pose.pose.position.z = enu_z;
    vision_pose.pose.orientation.x = enu_quat[0];
    vision_pose.pose.orientation.y = enu_quat[1];
    vision_pose.pose.orientation.z = enu_quat[2];
    vision_pose.pose.orientation.w = enu_quat[3];

    vision_pose_pub_->publish(vision_pose);
}

void EpaInterface::mavrosOdomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_odom_ = *msg;
}

void EpaInterface::cmdLoop() {
    if (!takeoff_triggered_) return;

    if (control_mode_ == ControlMode::POSITION) {
        target_pose_.header.stamp = this->get_clock()->now();
        target_pose_.header.frame_id = "map";

        local_pos_pub_->publish(target_pose_);
    } else if (control_mode_ == ControlMode::VELOCITY) {
        if (!current_odom_) return;

        auto q = current_odom_->pose.pose.orientation;
        auto euler = quatToEuler(q.x, q.y, q.z, q.w);
        double yaw = euler[2];

        double vx = target_velocity_.twist.linear.x;
        double vy = target_velocity_.twist.linear.y;

        double enu_vx = vx * std::cos(yaw) - vy * std::sin(yaw);
        double enu_vy = vx * std::sin(yaw) + vy * std::cos(yaw);

        geometry_msgs::msg::TwistStamped msg;
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "map";
        msg.twist.linear.x = enu_vx;
        msg.twist.linear.y = enu_vy;
        msg.twist.linear.z = target_velocity_.twist.linear.z;

        velocity_pub_->publish(msg);

        // Keep position target up to date so switching to position holds steady
        target_pose_.pose.position.x = current_odom_->pose.pose.position.x;
        target_pose_.pose.position.y = current_odom_->pose.pose.position.y;
        target_pose_.pose.position.z = current_odom_->pose.pose.position.z;
    } else if (control_mode_ == ControlMode::TRACKING) {
        if (!current_odom_) return;

        // Check that the tracking stream is still alive
        if (!isTrackingStreamActive()) {
            RCLCPP_WARN(this->get_logger(),
                        "Tracking velocity stream timed out, switching to POSITION hold");
            control_mode_ = ControlMode::POSITION;
            return;
        }

        // Transform body-frame velocity to ENU (same rotation as manual VELOCITY mode)
        auto q = current_odom_->pose.pose.orientation;
        auto euler = quatToEuler(q.x, q.y, q.z, q.w);
        double yaw = euler[2];

        double vx = tracking_velocity_.linear.x;
        double vy = tracking_velocity_.linear.y;

        double enu_vx = vx * std::cos(yaw) - vy * std::sin(yaw);
        double enu_vy = vx * std::sin(yaw) + vy * std::cos(yaw);

        geometry_msgs::msg::TwistStamped msg;
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "map";
        msg.twist.linear.x = enu_vx;
        msg.twist.linear.y = enu_vy;
        msg.twist.linear.z = tracking_velocity_.linear.z;
        msg.twist.angular.z = tracking_velocity_.angular.z;

        velocity_pub_->publish(msg);

        // Keep position target up to date so switching to position holds steady
        target_pose_.pose.position.x = current_odom_->pose.pose.position.x;
        target_pose_.pose.position.y = current_odom_->pose.pose.position.y;
        target_pose_.pose.position.z = current_odom_->pose.pose.position.z;
    }
}

// ============================================================
// GUI command callbacks
// ============================================================

void EpaInterface::srvArmCb(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    (void)req;
    this->arm();
    res->success = true;
    res->message = "Arm requested";
}

void EpaInterface::srvDisarmCb(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    (void)req;
    this->disarm();
    res->success = true;
    res->message = "Disarm requested";
}

void EpaInterface::srvOffboardCb(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    (void)req;
    this->offboard();
    res->success = true;
    res->message = "Offboard requested";
}

void EpaInterface::srvLandCb(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    (void)req;
    // If in TRACKING mode and controller land sequence service is available, use it
    if (control_mode_ == ControlMode::TRACKING &&
        controller_land_client_ && controller_land_client_->wait_for_service(std::chrono::milliseconds(100))) {
        // Send the request asynchronously to avoid deadlocking the executor
        auto land_req = std::make_shared<std_srvs::srv::Trigger::Request>();
        controller_land_client_->async_send_request(land_req,
            [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
                try {
                    auto result = future.get();
                    if (result && result->success) {
                        RCLCPP_INFO(this->get_logger(), "Controller land_sequence accepted");
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Controller land_sequence rejected: %s",
                                    result ? result->message.c_str() : "null");
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Controller land_sequence exception: %s", e.what());
                }
            });
        res->success = true;
        res->message = "Landing sequence requested via controller";
        return;
    }

    // Fallback: send PX4 land directly (used in non-TRACKING modes or if controller unavailable)
    this->land();
    res->success = true;
    res->message = control_mode_ == ControlMode::TRACKING ?
                   "Land requested (direct fallback from TRACKING)" : "Land requested (direct PX4 land)";
}

void EpaInterface::srvAbortLandingCb(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    (void)req;
    if (controller_abort_client_ && controller_abort_client_->wait_for_service(std::chrono::milliseconds(100))) {
        auto abort_req = std::make_shared<std_srvs::srv::Trigger::Request>();
        controller_abort_client_->async_send_request(abort_req,
            [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
                try {
                    auto result = future.get();
                    RCLCPP_INFO(this->get_logger(), "Abort landing result: %s",
                                result ? result->message.c_str() : "null");
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Abort landing exception: %s", e.what());
                }
            });
        res->success = true;
        res->message = "Abort landing forwarded to controller";
    } else {
        res->success = false;
        res->message = "Controller abort service not available";
    }
}

void EpaInterface::srvTrackingCb(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    (void)req;
    if (isTrackingStreamActive()) {
        control_mode_ = ControlMode::TRACKING;
        RCLCPP_INFO(this->get_logger(), "Switched to TRACKING mode");
        res->success = true;
        res->message = "Switched to TRACKING mode";
    } else {
        RCLCPP_WARN(this->get_logger(),
                    "Cannot switch to TRACKING mode: no active velocity stream from epa_node");
        res->success = false;
        res->message = "Inactive velocity stream";
    }
}

void EpaInterface::srvPositionCb(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    (void)req;
    control_mode_ = ControlMode::POSITION;
    RCLCPP_INFO(this->get_logger(), "Switched to POSITION mode");
    res->success = true;
    res->message = "Switched to POSITION mode";
}

void EpaInterface::takeoffCallback(
    const std_msgs::msg::Float64::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "GUI takeoff: height=%.2f", msg->data);
    takeoff(msg->data);
}

void EpaInterface::positionNudgeCallback(
    const geometry_msgs::msg::Vector3::SharedPtr msg) {
    if (control_mode_ == ControlMode::TRACKING) {
        // In tracking mode, forward nudge to controller
        forwardNudgeToController(*msg);
    } else if (control_mode_ == ControlMode::POSITION) {
        // In position mode, nudge the position setpoint directly
        updateSetpointRelative(msg->x, msg->y, msg->z);
    }
    // In VELOCITY mode, ignore nudges
}

void EpaInterface::forwardNudgeToController(const geometry_msgs::msg::Vector3& nudge) {
    tracking_nudge_pub_->publish(nudge);
    RCLCPP_DEBUG(this->get_logger(), "Forwarded nudge to controller: dx=%.3f dy=%.3f dz=%.3f",
                 nudge.x, nudge.y, nudge.z);
}

void EpaInterface::velocityCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
    double vx = msg->linear.x;
    double vy = msg->linear.y;
    double vz = msg->linear.z;

    if (std::abs(vx) + std::abs(vy) + std::abs(vz) > 0.01) {
        setVelocity(vx, vy, vz);
    } else {
        // Zero velocity → switch to position hold
        setVelocity(0.0, 0.0, 0.0);
        control_mode_ = ControlMode::POSITION;
    }
}

// ============================================================
// Commands
// ============================================================

void EpaInterface::arm(Callback cb) {
    if (!arming_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(this->get_logger(), "Arming service timeout");
        if (cb) cb(false);
        return;
    }

    auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    req->value = true;

    arming_client_->async_send_request(req,
        [this, cb](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
            try {
                auto result = future.get();
                bool success = result ? result->success : false;
                if (success) {
                    RCLCPP_INFO(this->get_logger(), "Armed successfully");
                } else {
                    RCLCPP_WARN(this->get_logger(), "Arming failed");
                }
                if (cb) cb(success);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Arming exception: %s", e.what());
                if (cb) cb(false);
            }
        });
}

void EpaInterface::disarm(Callback cb) {
    if (!arming_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(this->get_logger(), "Disarming service timeout");
        if (cb) cb(false);
        return;
    }

    auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    req->value = false;

    arming_client_->async_send_request(req,
        [this, cb](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
            try {
                auto result = future.get();
                bool success = result ? result->success : false;
                if (success) {
                    RCLCPP_INFO(this->get_logger(), "Disarmed successfully");
                } else {
                    RCLCPP_WARN(this->get_logger(), "Disarming failed");
                }
                if (cb) cb(success);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Disarming exception: %s", e.what());
                if (cb) cb(false);
            }
        });
}

void EpaInterface::offboard(Callback cb) {
    if (!set_mode_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(this->get_logger(), "Set mode service timeout");
        if (cb) cb(false);
        return;
    }

    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req->custom_mode = "OFFBOARD";

    set_mode_client_->async_send_request(req,
        [this, cb](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
            try {
                auto result = future.get();
                bool success = result ? result->mode_sent : false;
                if (success) {
                    RCLCPP_INFO(this->get_logger(), "OFFBOARD mode set");
                } else {
                    RCLCPP_WARN(this->get_logger(), "Failed to set OFFBOARD mode");
                }
                if (cb) cb(success);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Set mode exception: %s", e.what());
                if (cb) cb(false);
            }
        });
}

void EpaInterface::takeoff(double height, Callback cb) {
    if (!current_odom_) {
        RCLCPP_ERROR(this->get_logger(), "No odometry data for takeoff initialization");
        if (cb) cb(false);
        return;
    }

    const auto& odom_pose = current_odom_->pose.pose;

    // Set target position (current + height offset)
    target_pose_.pose.position.x = odom_pose.position.x;
    target_pose_.pose.position.y = odom_pose.position.y;
    target_pose_.pose.position.z = odom_pose.position.z + height;

    // Preserve current yaw
    auto euler = quatToEuler(odom_pose.orientation.x, odom_pose.orientation.y,
                              odom_pose.orientation.z, odom_pose.orientation.w);
    auto target_quat = eulerToQuat(0.0, 0.0, euler[2]);
    target_pose_.pose.orientation.x = target_quat[0];
    target_pose_.pose.orientation.y = target_quat[1];
    target_pose_.pose.orientation.z = target_quat[2];
    target_pose_.pose.orientation.w = target_quat[3];

    RCLCPP_INFO(this->get_logger(),
                "Takeoff from (%.2f, %.2f) to Z=%.2f",
                target_pose_.pose.position.x,
                target_pose_.pose.position.y,
                target_pose_.pose.position.z);

    takeoff_triggered_ = true;

    arm([this, cb](bool armed) {
        if (!armed) {
            if (cb) cb(false);
            return;
        }

        if (!auto_offboard_) {
            // Default path: setpoints are now streaming, but entering OFFBOARD
            // is left to the safety operator.
            RCLCPP_INFO(this->get_logger(),
                        "Armed and streaming setpoints; waiting for the operator "
                        "to request OFFBOARD");
            if (cb) cb(true);
            return;
        }

        // Opt-in path: give PX4 a second of setpoints, then request OFFBOARD.
        // The timer must be kept alive in a member, otherwise it is destroyed
        // as soon as this callback returns and never fires.
        offboard_delay_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this, cb]() {
                offboard_delay_timer_->cancel();  // one-shot
                this->offboard(cb);
            });
    });
}

void EpaInterface::land(Callback cb) {
    if (!set_mode_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(this->get_logger(), "Set mode service timeout");
        if (cb) cb(false);
        return;
    }

    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req->custom_mode = "AUTO.LAND";

    set_mode_client_->async_send_request(req,
        [this, cb](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
            try {
                auto result = future.get();
                bool success = result ? result->mode_sent : false;
                if (success) {
                    RCLCPP_INFO(this->get_logger(), "Landing initiated");
                } else {
                    RCLCPP_WARN(this->get_logger(), "Failed to initiate landing");
                }
                if (cb) cb(success);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Land exception: %s", e.what());
                if (cb) cb(false);
            }
        });
}

void EpaInterface::loiter(Callback cb) {
    if (!set_mode_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(this->get_logger(), "Set mode service timeout");
        if (cb) cb(false);
        return;
    }

    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req->custom_mode = "AUTO.LOITER";

    set_mode_client_->async_send_request(req,
        [this, cb](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
            try {
                auto result = future.get();
                bool success = result ? result->mode_sent : false;
                if (success) {
                    RCLCPP_INFO(this->get_logger(), "Loiter mode set");
                } else {
                    RCLCPP_WARN(this->get_logger(), "Failed to set loiter mode");
                }
                if (cb) cb(success);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Loiter exception: %s", e.what());
                if (cb) cb(false);
            }
        });
}

void EpaInterface::setPosition(double x, double y, double z) {
    target_pose_.pose.position.x = x;
    target_pose_.pose.position.y = y;
    target_pose_.pose.position.z = z;
    control_mode_ = ControlMode::POSITION;
}

void EpaInterface::updateSetpointRelative(double dx, double dy, double dz) {
    if (!current_odom_) return;

    auto q = current_odom_->pose.pose.orientation;
    auto euler = quatToEuler(q.x, q.y, q.z, q.w);
    double yaw = euler[2];

    double body_dx = dx * std::cos(yaw) - dy * std::sin(yaw);
    double body_dy = dx * std::sin(yaw) + dy * std::cos(yaw);

    target_pose_.pose.position.x += body_dx;
    target_pose_.pose.position.y += body_dy;
    target_pose_.pose.position.z += dz;
    control_mode_ = ControlMode::POSITION;
}

void EpaInterface::setVelocity(double vx, double vy, double vz) {
    target_velocity_.twist.linear.x = vx;
    target_velocity_.twist.linear.y = vy;
    target_velocity_.twist.linear.z = vz;
    control_mode_ = ControlMode::VELOCITY;
}

void EpaInterface::trackingVelocityCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
    tracking_velocity_ = *msg;
    last_tracking_vel_time_ = this->get_clock()->now();
}

bool EpaInterface::isTrackingStreamActive() {
    if (last_tracking_vel_time_.nanoseconds() == 0) {
        return false;  // Never received a tracking velocity message
    }
    double age = (this->get_clock()->now() - last_tracking_vel_time_).seconds();
    return age < kTrackingTimeoutSec;
}

// ============================================================
// Quaternion / Euler helpers (no tf2 dependency needed)
// ============================================================

std::array<double, 4> EpaInterface::quatMultiply(
    const std::array<double, 4>& q1,
    const std::array<double, 4>& q2) {
    // Hamilton product: q1 * q2,  layout [x, y, z, w]
    double x1 = q1[0], y1 = q1[1], z1 = q1[2], w1 = q1[3];
    double x2 = q2[0], y2 = q2[1], z2 = q2[2], w2 = q2[3];
    return {
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    };
}

std::array<double, 4> EpaInterface::eulerToQuat(double roll, double pitch, double yaw) {
    double cr = std::cos(roll / 2.0),  sr = std::sin(roll / 2.0);
    double cp = std::cos(pitch / 2.0), sp = std::sin(pitch / 2.0);
    double cy = std::cos(yaw / 2.0),   sy = std::sin(yaw / 2.0);
    return {
        sr*cp*cy - cr*sp*sy,   // x
        cr*sp*cy + sr*cp*sy,   // y
        cr*cp*sy - sr*sp*cy,   // z
        cr*cp*cy + sr*sp*sy    // w
    };
}

std::array<double, 3> EpaInterface::quatToEuler(
    double qx, double qy, double qz, double qw) {
    // Roll (x-axis)
    double sinr_cosp = 2.0 * (qw*qx + qy*qz);
    double cosr_cosp = 1.0 - 2.0 * (qx*qx + qy*qy);
    double roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis)
    double sinp = 2.0 * (qw*qy - qz*qx);
    double pitch;
    if (std::abs(sinp) >= 1.0)
        pitch = std::copysign(M_PI / 2.0, sinp);
    else
        pitch = std::asin(sinp);

    // Yaw (z-axis)
    double siny_cosp = 2.0 * (qw*qz + qx*qy);
    double cosy_cosp = 1.0 - 2.0 * (qy*qy + qz*qz);
    double yaw = std::atan2(siny_cosp, cosy_cosp);

    return {roll, pitch, yaw};
}

}  // namespace epa

RCLCPP_COMPONENTS_REGISTER_NODE(epa::EpaInterface)
