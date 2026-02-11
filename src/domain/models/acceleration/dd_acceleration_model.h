#pragma once

#include "acceleration_model.h" // 추상 클래스 포함
#include <string>

class DDAccelerationModel : public AccelerationModel 
{
public:
    DDAccelerationModel(
        double mass_vehicle,
        double load_weight,
        double max_torque,
        double friction_coeff,
        double max_speed,
        double max_acceleration,
        double max_deceleration,
        double wheel_radius,
        double max_angular_acceleration,
        double max_angular_deceleration
    );

    double applyAcceleration(double current_speed, double target_speed, double dt) override;
    double applyAngularAcceleration(double current_angular_speed, double target_angular_speed, double dt) override;
    bool loadParametersFromYaml(const std::string& yaml_file_path) override;

private:
    double mass_vehicle_;       // 차량 질량
    double load_weight_;        // 적재 중량
    double max_torque_;         // 최대 토크 (Nm)
    double friction_coeff_;     // 마찰 계수
    double max_speed_;          // 최대 선형 속도
    double max_acceleration_;   // 최대 선형 가속도 
    double max_deceleration_;   // 최대 선형 감속도 
    double wheel_radius_;       // 휠 반경 (m)
    double max_angular_acceleration_; // 최대 각가속도
    double max_angular_deceleration_; // 최대 각감속도

    const double GRAVITY = 9.81; // 중력 가속도 (m/s^2)
};
