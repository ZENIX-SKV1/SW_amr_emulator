#include "dead_reckoning_euler.h"
#include <iostream>
#include <cmath>

constexpr double PI = 3.14159265358979323846;

deadReckoningEuler::deadReckoningEuler(double wheel_base, double wheel_radius, double rpm_noise_std)
    : x_(0.0), y_(0.0), theta_(0.0), wheel_base_(wheel_base), wheel_radius_(wheel_radius),
     generator_(std::random_device{}()),noise_dist_(0.0, rpm_noise_std) 
    {
        std::cout << "rpm_noise_std_ : " << rpm_noise_std_ << std::endl;
    }

void deadReckoningEuler::setInitialPose(double x, double y, double theta) 
{
    x_ = x; y_ = y; theta_ = theta;
}

void deadReckoningEuler::update(double left_rpm, double right_rpm, double dt) 
{
    // std::cout << "noise_dist_ : " << noise_dist_(generator_) << std::endl;

    double noisy_left_rpm = left_rpm + noise_dist_(generator_);
    double noisy_right_rpm = right_rpm + noise_dist_(generator_);

    // std::cout <<"dr lrpm : " << left_rpm << " rrpm : " << right_rpm <<std::endl;
    double v_l = (noisy_left_rpm / 60.0) * 2.0 * PI * wheel_radius_;
    double v_r = (noisy_right_rpm / 60.0) * 2.0 * PI * wheel_radius_;
    // std::cout <<"dr v_l : " << v_l << " v_r : " << v_r <<std::endl;

    double v = (v_r + v_l) / 2.0;
    double w = (v_r - v_l) / wheel_base_;

    // std::cout <<"v : " << v << " w : " << w <<std::endl;
    x_ += v * std::cos(theta_) * dt;
    y_ += v * std::sin(theta_) * dt;
    theta_ += w * dt;
}



void deadReckoningEuler::getPose(double& x, double& y, double& theta) const 
{
    x = x_; y = y_; theta = theta_;
}