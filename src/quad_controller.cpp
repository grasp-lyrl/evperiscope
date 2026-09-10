#include "epa/quad_controller.hpp"

#include <algorithm>

namespace epa {

QuadController::QuadController(const PidGains& gains_xy,
                               const PidGains& gains_z,
                               const PidGains& gains_yaw,
                                                             double max_vel_xy,
                                                             double max_vel_z,
                               double max_yaw_rate)
    : gains_xy_(gains_xy),
      gains_z_(gains_z),
      gains_yaw_(gains_yaw),
            max_vel_xy_(max_vel_xy),
            max_vel_z_(max_vel_z),
      max_yaw_rate_(max_yaw_rate),
      enabled_(false) {}

void QuadController::setTarget(double x, double y, double z, double yaw)
{
    std::lock_guard<std::mutex> lock(mutex_);
    target_ = {x, y, z, yaw};
}

void QuadController::setTarget(const Target& target)
{
    std::lock_guard<std::mutex> lock(mutex_);
    target_ = target;
}

void QuadController::setEnabled(bool enabled)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (enabled && !enabled_)
    {
        state_x_ = {};
        state_y_ = {};
        state_z_ = {};
        state_yaw_ = {};
    }
    enabled_ = enabled;
}

bool QuadController::isEnabled() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return enabled_;
}

void QuadController::reset()
{
    std::lock_guard<std::mutex> lock(mutex_);
    state_x_ = {};
    state_y_ = {};
    state_z_ = {};
    state_yaw_ = {};
}

QuadController::Target QuadController::getTarget() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return target_;
}

geometry_msgs::msg::Twist QuadController::compute(const QuadPoseFilter::Pose& pose, double dt)
{
    geometry_msgs::msg::Twist twist;
    std::lock_guard<std::mutex> lock(mutex_);

    if (!enabled_ || dt <= 0.0) {
        return twist;
    }

    // compute errors (target - current)
    double ex = target_.x - pose.x;
    double ey = target_.y - pose.y;
    double ez = target_.z - pose.z;
    double eyaw = wrapAngle(target_.yaw - pose.yaw);

    // per-axis PID (in camera frame)
    double vel_cam_x = pidStep(ex, dt, gains_xy_, state_x_, max_vel_xy_);
    double vel_cam_y = pidStep(ey, dt, gains_xy_, state_y_, max_vel_xy_);
    double vel_cam_z = pidStep(ez, dt, gains_z_, state_z_, max_vel_z_);
    double yaw_rate  = pidStep(eyaw, dt, gains_yaw_, state_yaw_, max_yaw_rate_);

    // rotate camera-frame XY velocity into quad body frame by -yaw
    double cy = std::cos(-pose.yaw);
    double sy = std::sin(-pose.yaw);
    twist.linear.x = vel_cam_x * cy - vel_cam_y * sy;
    twist.linear.y = vel_cam_x * sy + vel_cam_y * cy;
    twist.linear.z = vel_cam_z;
    twist.angular.z = yaw_rate;

    return twist;
}

double QuadController::pidStep(double error, double dt,
                               const PidGains& gains,
                               PidState& state,
                               double max_output)
{
    // proportional
    double p = gains.kp * error;

    // integral with anti-windup
    state.integral += error * dt;
    state.integral = std::clamp(state.integral, -kIntegralMax, kIntegralMax);
    double i = gains.ki * state.integral;

    // derivative (skip on first call to avoid spike)
    double d = 0.0;
    if (!state.first) {
        d = gains.kd * (error - state.prev_error) / dt;
    }
    state.prev_error = error;
    state.first = false;

    double output = p + i + d;
    return std::clamp(output, -max_output, max_output);
}

double QuadController::wrapAngle(double angle)
{
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

} // namespace epa
