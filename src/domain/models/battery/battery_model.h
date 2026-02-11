#pragma once

class IBatteryModel 
{
public:
    virtual ~IBatteryModel() = default;

    virtual void update(double dt, double linear_vel, double angular_vel, bool is_charging) = 0;
    virtual double getCapacityPercent() const = 0;
};