#pragma once

#include "acceleration_model.h" // 추상 클래스 포함
#include <string>

class DDAccelerationWithoutDynamicsModel : public AccelerationModel 
{
public:
    DDAccelerationWithoutDynamicsModel(
        double max_acceleration,
        double max_deceleration,
        double max_angular_acceleration,
        double max_angular_deceleration
    );

    double applyAcceleration(double current_speed, double target_speed, double dt) override;
    double applyAngularAcceleration(double current_angular_speed, double target_angular_speed, double dt) override;
    bool loadParametersFromYaml(const std::string& yaml_file_path) override;

private:
    double max_acceleration_;   // 최대 선형 가속도 
    double max_deceleration_;   // 최대 선형 감속도 
    double max_angular_acceleration_; // 최대 각가속도
    double max_angular_deceleration_; // 최대 각감속도
};
