#include "sd_dead_reckoning_euler.h"
#include <iostream>
#include <cmath>

constexpr double PI = 3.14159265358979323846;

SDdeadReckoningEuler::SDdeadReckoningEuler(double wheel_base, double wheel_radius, double rpm_noise_std)
    : x_(0.0), y_(0.0), theta_(0.0), wheel_base_(wheel_base), wheel_radius_(wheel_radius),
     generator_(std::random_device{}()),noise_dist_(0.0, rpm_noise_std) 
    {
        std::cout << "rpm_noise_std_ : " << rpm_noise_std_ << std::endl;
    }

void SDdeadReckoningEuler::setInitialPose(double x, double y, double theta) 
{
    x_ = x; y_ = y; theta_ = theta;
}

void SDdeadReckoningEuler::update(double rpm, double steering_angle_, double dt) 
{
    double v = (2.0 * M_PI * wheel_radius_ * rpm) / 60.0;
    v += noise_dist_(generator_);

    double w = v / wheel_base_ * std::tan(steering_angle_);
    x_ += v * std::cos(theta_) * dt;
    y_ += v * std::sin(theta_) * dt;
    theta_ += w * dt;
}


void SDdeadReckoningEuler::getPose(double& x, double& y, double& theta) const 
{
    x = x_; y = y_; theta = theta_;
}