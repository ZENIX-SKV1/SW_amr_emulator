#pragma once
#include "yaml_config.h"
#include "imotor_controller.h"
#include <iostream>
#include <memory>

class SDMotorController : public IMotorController
{
public:
    // MotorController(double wheel_base, double max_speed, double max_angular_speed, double wheel_radius, double max_accel, double max_angular_accel);
    SDMotorController(const AmrConfig& config);
    
    void setAccelerationModel(std::shared_ptr<AccelerationModel> model) override;
    void setVelocity(double linear, double angular) override;
    void update(double dt) override;
    void getRPM(double& left_rpm, double& right_rpm) const override;
    void setMaxSpeed(double max_speed)  override;
    double getLinearVelocity() const override;
    double getAngularVelocity() const override;

private:
    std::shared_ptr<AccelerationModel> acceleration_model_;

    double linear_vel_cmd_, angular_vel_cmd_;
    double linear_vel_actual_, angular_vel_actual_;
    double rpm_, steering_angular_;
    double wheel_base_, max_speed_, max_angular_speed_, wheel_radius_;
    double max_accel_, max_angular_accel_;

    double linear_speed_cmd_, angular_speed_cmd_ ;
    double front_wheel_rpm;
    double max_steering_angle_;

    void convertWheelSpeedToRPM(double wheel_speed, double& rpm) const;
    void convertAngularSpeedToSteeringAngle(double linear_speed, double angular_speed, double wheel_base, double& steering_angle) const;   
};