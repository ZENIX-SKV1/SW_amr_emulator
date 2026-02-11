#pragma once

class ILocalizer 
{
public:
    virtual ~ILocalizer() = default;

    virtual void setInitialPose(double x, double y, double theta) = 0;
    virtual void update(double left_rpm, double right_rpm, double dt) = 0;
    virtual void getPose(double& x, double& y, double& theta) const = 0;
};