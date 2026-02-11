#include "dead_reckoning_rk4.h"
#include <cmath>

constexpr double PI = 3.14159265358979323846;

deadReckoningRK4::deadReckoningRK4(double wheel_base, double wheel_radius)
    : x_(0.0), y_(0.0), theta_(0.0), wheel_base_(wheel_base), wheel_radius_(wheel_radius) 
    {

    }

void deadReckoningRK4::setInitialPose(double x, double y, double theta) 
{
    x_ = x; y_ = y; theta_ = theta;
}

void deadReckoningRK4::update(double left_rpm, double right_rpm, double dt) 
{
    auto velocities = [this](double l_rpm, double r_rpm) 
    {
        double v_l = (l_rpm / 60.0) * 2.0 * PI * wheel_radius_;
        double v_r = (r_rpm / 60.0) * 2.0 * PI * wheel_radius_;
        double v = (v_r + v_l) / 2.0;
        double w = (v_r - v_l) / wheel_base_;
        return std::make_pair(v, w);
    };

    // k1
    auto [v1, w1] = velocities(left_rpm, right_rpm);
    double dx1 = v1 * std::cos(theta_);
    double dy1 = v1 * std::sin(theta_);
    double dtheta1 = w1;

    // k2
    auto [v2, w2] = velocities(left_rpm, right_rpm);
    double theta_k2 = theta_ + dtheta1 * dt / 2;
    double dx2 = v2 * std::cos(theta_k2);
    double dy2 = v2 * std::sin(theta_k2);
    double dtheta2 = w2;

    // k3
    auto [v3, w3] = velocities(left_rpm, right_rpm);
    double theta_k3 = theta_ + dtheta2 * dt / 2;
    double dx3 = v3 * std::cos(theta_k3);
    double dy3 = v3 * std::sin(theta_k3);
    double dtheta3 = w3;

    // k4
    auto [v4, w4] = velocities(left_rpm, right_rpm);
    double theta_k4 = theta_ + dtheta3 * dt;
    double dx4 = v4 * std::cos(theta_k4);
    double dy4 = v4 * std::sin(theta_k4);
    double dtheta4 = w4;

    // Combine
    x_ += dt / 6.0 * (dx1 + 2 * dx2 + 2 * dx3 + dx4);
    y_ += dt / 6.0 * (dy1 + 2 * dy2 + 2 * dy3 + dy4);
    theta_ += dt / 6.0 * (dtheta1 + 2 * dtheta2 + 2 * dtheta3 + dtheta4);
}

void deadReckoningRK4::getPose(double& x, double& y, double& theta) const 
{
    x = x_; y = y_; theta = theta_;
}