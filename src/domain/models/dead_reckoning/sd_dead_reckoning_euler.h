#pragma once
#include "idead_reckoning.h"
#include <random>

class SDdeadReckoningEuler : public ideadReckoningModel 
{
public:
    // deadReckoningEuler(double wheel_base, double wheel_radius);
    SDdeadReckoningEuler(double wheel_base, double wheel_radius, double rpm_noise_std);

    void setInitialPose(double x, double y, double theta) override;
    // void update(double left_rpm, double right_rpm, double dt) override;
    void update(double rpm, double steering_angle, double dt) override;
    void getPose(double& x, double& y, double& theta) const override;

private:
    double x_, y_, theta_;
    double steering_angle_;
    double wheel_base_, wheel_radius_;
    double rpm_noise_std_; 
    mutable std::mt19937 generator_;
    mutable std::normal_distribution<double> noise_dist_;    
};