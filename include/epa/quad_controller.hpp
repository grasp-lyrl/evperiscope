#pragma once

#include <array>
#include <cmath>
#include <mutex>
#include <geometry_msgs/msg/twist.hpp>

#include "epa/quad_pose_filter.hpp"

namespace epa {

/**
 * Per-axis PID controller for quadrotor position/yaw tracking.
 *
 * Consumes QuadPoseFilter::Pose (camera frame), outputs a velocity Twist
 * to be published on /epa/cmd/velocity for the controller interface.
 */
class QuadController
{
public:
    struct PidGains
    {
        double kp = 1.0;
        double ki = 0.0;
        double kd = 0.1;
    };

    struct Target
    {
        double x = 0.0;
        double y = 0.0;
        double z = 1.0;
        double yaw = 0.0;
    };

    /**
     * @param gains_xy  PID gains for X and Y axes
     * @param gains_z   PID gains for Z axis
     * @param gains_yaw PID gains for yaw
    * @param max_vel_xy   Max horizontal linear velocity output (m/s)
    * @param max_vel_z    Max vertical linear velocity output (m/s)
    * @param max_yaw_rate Max yaw rate output (rad/s)
     */
    QuadController(const PidGains& gains_xy,
                   const PidGains& gains_z,
                   const PidGains& gains_yaw,
                double max_vel_xy = 0.5,
                double max_vel_z = 0.5,
                   double max_yaw_rate = 0.5);

    /// Set the target pose (camera frame)
    void setTarget(double x, double y, double z, double yaw);
    void setTarget(const Target& target);

    /// Enable / disable the controller (disabled → zero output)
    void setEnabled(bool enabled);
    bool isEnabled() const;

    /// Run one PID step. Returns velocity command twist.
    geometry_msgs::msg::Twist compute(const QuadPoseFilter::Pose& pose, double dt);

    /// Reset all integrators and derivative state
    void reset();

    /// Get the current target
    Target getTarget() const;

private:
    struct PidState
    {
        double integral = 0.0;
        double prev_error = 0.0;
        bool first = true;
    };

    /// run a single-axis PID step
    double pidStep(double error, double dt, const PidGains& gains,
                   PidState& state, double max_output);

    /// wrap angle to [-pi, pi]
    static double wrapAngle(double angle);

    // per-axis gains
    PidGains gains_xy_;
    PidGains gains_z_;
    PidGains gains_yaw_;

    // per-axis state
    PidState state_x_;
    PidState state_y_;
    PidState state_z_;
    PidState state_yaw_;

    Target target_;
    double max_vel_xy_;
    double max_vel_z_;
    double max_yaw_rate_;
    bool enabled_ = false;
    mutable std::mutex mutex_;

    // anti-windup clamp for integral
    static constexpr double kIntegralMax = 2.0;
};

} // namespace epa
