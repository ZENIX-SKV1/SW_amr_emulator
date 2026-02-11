#pragma once
#include <vector>
class INavigation 
{
public:
    virtual ~INavigation() = default;
    virtual void setTarget(double x, double y) = 0;
    virtual void setArcTarget(double x, double y, double center_x, double center_y, double radius, double start_angle, double end_angle, bool clockwise) = 0;
    // virtual void update(double current_x, double current_y, double current_theta,double& out_linear, double& out_angular) = 0;
    virtual void update(double current_x, double current_y, double current_theta,
                            double& out_linear, double& out_angular,
                            const std::vector<std::pair<double, double>>& other_robot_positions) = 0;
    
};