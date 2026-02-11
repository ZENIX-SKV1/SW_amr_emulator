#include "dd_acceleration_model.h"
#include <algorithm> 
#include <cmath>     
#include <iostream>  

DDAccelerationModel::DDAccelerationModel(
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
) :
    mass_vehicle_(mass_vehicle),
    load_weight_(load_weight),
    max_torque_(max_torque),
    friction_coeff_(friction_coeff),
    max_speed_(max_speed),
    max_acceleration_(max_acceleration),
    max_deceleration_(max_deceleration),
    wheel_radius_(wheel_radius), 
    max_angular_acceleration_(max_angular_acceleration),
    max_angular_deceleration_(max_angular_deceleration)
{
    if (mass_vehicle_ <= 0 || load_weight_ < 0 || max_torque_ <= 0 || friction_coeff_ < 0 ||
        max_speed_ <= 0 || max_acceleration_ <= 0 || max_deceleration_ <= 0 ||
        wheel_radius_ <= 0 || 
        max_angular_acceleration_ <= 0 || max_angular_deceleration_ <= 0) {
        std::cerr << "Warning: DDAccelerationModel initialized with non-positive parameters. This may lead to unexpected behavior." << std::endl;
    }
}

//유효성 확인 필요....(실제 테스트agv로 실험하고 피드백 반영하자)
double DDAccelerationModel::applyAcceleration(double current_speed, double target_speed, double dt) 
{
    double total_mass = mass_vehicle_ + load_weight_;

    double drive_force = (wheel_radius_ > 0) ? max_torque_ / wheel_radius_ : 0.0;

    double gravity_force = total_mass * GRAVITY;
    double friction_force = gravity_force * friction_coeff_;

    double speed_diff = target_speed - current_speed;
    double net_force = 0.0;

    if (speed_diff > 0) 
    { 
        net_force = std::max(0.0, drive_force - friction_force);
    } 
    else if (speed_diff < 0) 
    { 
        net_force = -friction_force;
    } 
    else 
    { 
        return current_speed;
    }

    double acceleration = net_force / total_mass;

    if (acceleration > 0) 
    { 
        acceleration = std::min(acceleration, max_acceleration_);
    } 
    else if (acceleration < 0) 
    { 
        acceleration = std::max(acceleration, -max_deceleration_); 
    }

    double new_speed = current_speed + acceleration * dt;

    new_speed = std::clamp(new_speed, 0.0, max_speed_);

    if (speed_diff > 0 && new_speed > target_speed) 
    {
        new_speed = target_speed;
    } 
    else if(speed_diff < 0 && new_speed < target_speed) 
    {
        new_speed = target_speed;
    }


    return new_speed;
}

double DDAccelerationModel::applyAngularAcceleration(double current_angular_speed, double target_angular_speed, double dt) 
{
    double effective_angular_acceleration;

    if(target_angular_speed > current_angular_speed) 
    { // 각가속
        effective_angular_acceleration = max_angular_acceleration_;
    } 
    else if(target_angular_speed < current_angular_speed) 
    { // 각감속
        effective_angular_acceleration = -max_angular_deceleration_;
    } 
    else 
    { 
        return current_angular_speed;
    }

    double next_angular_speed = current_angular_speed + effective_angular_acceleration * dt;

    if (effective_angular_acceleration > 0) 
    { // 각가속 중
        next_angular_speed = std::min(next_angular_speed, target_angular_speed);
    } 
    else 
    { // 각감속 중
        next_angular_speed = std::max(next_angular_speed, target_angular_speed);
    }

    return next_angular_speed;
}

bool DDAccelerationModel::loadParametersFromYaml(const std::string& yaml_file_path) 
{

    std::cout << "Loading parameters from YAML file: " << yaml_file_path << " (Not yet implemented in this example)" << std::endl;
    return false;
}