#include "sd_acceleration_model.h"
#include <algorithm> 
#include <cmath>     
#include <iostream>  

SDAccelerationModel::SDAccelerationModel(
        double max_velocity_accel,         // 휠 최대 가속도
        double max_velocity_decel,         // 휠 최대 감속도
        double max_steering_accel,         // 조향 최대 가속도
        double max_steering_decel          // 조향 최대 감속도
) :
    max_velocity_accel_(max_velocity_accel),
    max_velocity_decel_(max_velocity_decel),
    max_steering_accel_(max_steering_accel),
    max_steering_decel_(max_steering_decel)
    {}

//유효성 확인 필요....(실제 테스트agv로 실험하고 피드백 반영하자)
double SDAccelerationModel::applyAcceleration(double current_speed, double target_speed, double dt) 
{
    double diff = target_speed - current_speed;
    double max_delta = (diff > 0 ? max_velocity_accel_ : max_velocity_decel_) * dt;
    if (diff > max_delta)
        return current_speed + max_delta;
    else if (diff < -max_delta)
        return current_speed - max_delta;
    else
        return target_speed;
}

double SDAccelerationModel::applyAngularAcceleration(double current_angular_speed, double target_angular_speed, double dt) 
{
    double diff = target_angular_speed - current_angular_speed;
    double max_delta = (diff > 0 ? max_steering_accel_ : max_steering_decel_) * dt;
    if (diff > max_delta)
        return current_angular_speed + max_delta;
    else if (diff < -max_delta)
        return current_angular_speed - max_delta;
    else
        return target_angular_speed;
}

bool SDAccelerationModel::loadParametersFromYaml(const std::string& yaml_file_path) 
{

    std::cout << "Loading parameters from YAML file: " << yaml_file_path << " (Not yet implemented in this example)" << std::endl;
    return false;
}