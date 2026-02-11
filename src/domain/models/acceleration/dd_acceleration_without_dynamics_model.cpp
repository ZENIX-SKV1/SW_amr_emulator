#include "dd_acceleration_without_dynamics_model.h"
#include <algorithm> 
#include <cmath>     
#include <iostream>  

DDAccelerationWithoutDynamicsModel::DDAccelerationWithoutDynamicsModel(
    double max_acceleration,
    double max_deceleration,
    double max_angular_acceleration,
    double max_angular_deceleration
) :
    max_acceleration_(max_acceleration),
    max_deceleration_(max_deceleration),
    max_angular_acceleration_(max_angular_acceleration),
    max_angular_deceleration_(max_angular_deceleration)
{
    std::cout << "[DDAccelerationWithoutDynamicsModel] max_acc : " << max_acceleration_ << " max_dec : " << max_deceleration_
        << " max_acc_ang : " << max_angular_acceleration_ << " max_dec_ang : " << max_angular_deceleration_ << std::endl;
}

double DDAccelerationWithoutDynamicsModel::applyAcceleration(double current_speed, double target_speed, double dt) 
{
        // std::cout << "DDAccelerationWithoutDynamicsModel : " << max_acceleration_ << " " << max_deceleration_ << std::endl;

        double speed_diff = target_speed - current_speed;
        double max_delta = (speed_diff > 0 ? max_acceleration_ : max_deceleration_) * dt;
        if (speed_diff > max_delta)
        {
            return current_speed + max_delta;
        }
        else if (speed_diff < -max_delta)
        {
            return current_speed - max_delta;
        }
        else
        {
            return target_speed;
        }
}

double DDAccelerationWithoutDynamicsModel::applyAngularAcceleration(double current_angular_speed, double target_angular_speed, double dt) 
{
    double ang_diff = target_angular_speed - current_angular_speed;
        
    double max_ang_delta = (ang_diff > 0 ? max_angular_acceleration_ : max_angular_deceleration_) * dt;

    if (ang_diff > max_ang_delta)
    {
        return current_angular_speed + max_ang_delta;
    }
    else if (ang_diff < -max_ang_delta)
    {
        return current_angular_speed - max_ang_delta;
    }
    else
    {
        return target_angular_speed;
    }
}

bool DDAccelerationWithoutDynamicsModel::loadParametersFromYaml(const std::string& yaml_file_path) 
{

    std::cout << "Loading parameters from YAML file: " << yaml_file_path << " (Not yet implemented in this example)" << std::endl;
    return false;
}