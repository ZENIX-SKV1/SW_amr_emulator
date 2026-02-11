#include "dead_reckoning_rk2.h"
#include <cmath>

constexpr double PI = 3.14159265358979323846;

deadReckoningRK2::deadReckoningRK2(double wheel_base, double wheel_radius)
    : x_(0.0), y_(0.0), theta_(0.0), wheel_base_(wheel_base), wheel_radius_(wheel_radius) 
    {

    }

void deadReckoningRK2::setInitialPose(double x, double y, double theta) 
{
    x_ = x; y_ = y; theta_ = theta;
}

void deadReckoningRK2::update(double left_rpm, double right_rpm, double dt) 
{
    auto linear_angular_velocity = [this](double l_rpm, double r_rpm)
    {
        double v_l = (l_rpm / 60.0) * 2.0 * PI * wheel_radius_;
        double v_r = (r_rpm / 60.0) * 2.0 * PI * wheel_radius_;
        double v = (v_r + v_l) / 2.0;
        double w = (v_r - v_l) / wheel_base_;
        return std::make_pair(v, w);
    };

    auto [v1, w1] = linear_angular_velocity(left_rpm, right_rpm);

    // 1단계: Euler estimate of theta at midpoint
    double theta_mid = theta_ + w1 * dt * 0.5;

    // 2단계: Compute v, w at midpoint (assuming constant inputs, v,w same)
    double v2 = v1;
    double w2 = w1;

    // Update using midpoint slope
    x_ += v2 * std::cos(theta_mid) * dt;
    y_ += v2 * std::sin(theta_mid) * dt;
    theta_ += w2 * dt;
}

void deadReckoningRK2::getPose(double& x, double& y, double& theta) const 
{
    x = x_; y = y_; theta = theta_;
}