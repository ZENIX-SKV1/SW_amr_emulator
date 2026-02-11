#ifndef ACCELERATION_MODEL_HPP
#define ACCELERATION_MODEL_HPP

#include <string>
#include <memory>

class AccelerationModel 
{
public:
    virtual ~AccelerationModel() = default;
    virtual double applyAcceleration(double current_speed, double target_speed, double dt) = 0;
    virtual double applyAngularAcceleration(double current_angular_speed, double target_angular_speed, double dt) = 0;
    virtual bool loadParametersFromYaml(const std::string& yaml_file_path) = 0;
};

#endif // ACCELERATION_MODEL_HPP
