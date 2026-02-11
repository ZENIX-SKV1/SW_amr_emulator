#include "dead_reckoning_model_factory.h"
#include <stdexcept>
#include <iostream>

std::shared_ptr<ideadReckoningModel> DeadReckoningModelFactory::create(
    const std::string& vehicle_type,
    const AmrConfig& config
) 
{
    if (vehicle_type == "differential_drive" || vehicle_type == "dd") 
    {
        double wheel_base = config.amr_params.wheel_base;
        double wheel_radius = config.amr_params.wheel_radius;

        if (wheel_base <= 0.0 || wheel_radius <= 0.0) 
        {
            throw std::invalid_argument("Invalid wheel_base or wheel_radius in config");
        }

        std::cout << "success to set deadReckoningEuler " << wheel_base << " " << wheel_radius << std::endl;
        return std::make_shared<deadReckoningEuler>(wheel_base, wheel_radius, config.amr_params.max_rpm_deviation);
    }
    else if (vehicle_type == "steering_drive") 
    {
        double wheel_base = config.amr_params.wheel_base;
        double wheel_radius = config.amr_params.wheel_radius;

        if (wheel_base <= 0.0 || wheel_radius <= 0.0) 
        {
            throw std::invalid_argument("Invalid wheel_base or wheel_radius in config");
        }

        std::cout << "success to set SDdeadReckoningEuler " << wheel_base << " " << wheel_radius << std::endl;
        return std::make_shared<SDdeadReckoningEuler>(wheel_base, wheel_radius, config.amr_params.max_rpm_deviation);
    }
    else 
    {
        std::cerr << "[DeadReckoningModelFactory] Unknown model_type: '" << vehicle_type << "'" << std::endl;
        throw std::invalid_argument("Unknown DeadReckoningModel type: " + vehicle_type);
    }
}
