#pragma once

#include "acceleration_model.h" // 추상 클래스 포함
#include <string>

class SDAccelerationModel : public AccelerationModel 
{
public:
    SDAccelerationModel(
        double max_velocity_accel,         // 휠 최대 가속도
        double max_velocity_decel,         // 휠 최대 감속도
        double max_steering_accel,         // 조향 최대 가속도
        double max_steering_decel          // 조향 최대 감속도
    );

    double applyAcceleration(double current_velocity, double target_velocity, double dt) override;
    double applyAngularAcceleration(double current_angular_speed, double target_angular_speed, double dt) override;
    bool loadParametersFromYaml(const std::string& yaml_file_path) override;

private:
    double max_velocity_accel_;
    double max_velocity_decel_;
    double max_steering_accel_;
    double max_steering_decel_;
};
