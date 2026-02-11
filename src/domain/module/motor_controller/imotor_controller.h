#pragma once
#include <memory> 
#include "acceleration_model.h"

class IMotorController 
{
public:
    virtual ~IMotorController() = default;
    virtual void setAccelerationModel(std::shared_ptr<AccelerationModel> model) = 0;
    virtual void setVelocity(double linear, double angular) = 0;
    virtual void update(double dt) = 0;
    virtual void getRPM(double& left_rpm, double& right_rpm) const = 0;
    virtual void setMaxSpeed(double max_speed) = 0;
    virtual double getLinearVelocity() const = 0;
    virtual double getAngularVelocity() const = 0;
};