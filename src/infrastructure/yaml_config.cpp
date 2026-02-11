#include "yaml_config.h"
#include <yaml-cpp/yaml.h>
#include <iostream>

AmrConfig YamlConfig::load(const std::string& filename) 
{
    YAML::Node config = YAML::LoadFile(filename);
    AmrConfig cfg;
    cfg.amr_count = config["amr_count"].as<int>();
    cfg.base_port = config["base_port"].as<int>();

    // Load protocol_type
    if (config["protocol_type"]) 
    {
        cfg.protocol_type = config["protocol_type"].as<std::string>();
    } 
    else 
    {
        cfg.protocol_type = "vda5050";
        std::cerr << "Warning: 'protocol_type' not found in config. Defaulting to 'vda5050'." << std::endl;
    }    

    cfg.amr_params.wheel_radius = config["amr_params"]["wheel_radius"].as<double>();
    cfg.amr_params.wheel_base = config["amr_params"]["wheel_base"].as<double>();
    cfg.amr_params.max_speed = config["amr_params"]["max_speed"].as<double>();
    cfg.amr_params.min_speed = config["amr_params"]["min_speed"].as<double>();
    cfg.amr_params.angularSpeedMax = config["amr_params"]["angularSpeedMax"].as<double>();
    cfg.amr_params.angularSpeedMin = config["amr_params"]["angularSpeedMin"].as<double>();
    cfg.amr_params.accelerationMax = config["amr_params"]["accelerationMax"].as<double>();
    cfg.amr_params.decelerationMax = config["amr_params"]["decelerationMax"].as<double>();
    cfg.amr_params.max_angular_acceleration = config["amr_params"]["max_angular_acceleration"].as<double>();
    cfg.amr_params.max_angular_deceleration = config["amr_params"]["max_angular_deceleration"].as<double>();
    cfg.amr_params.mass_vehicle = config["amr_params"]["mass_vehicle"].as<double>();
    cfg.amr_params.load_weight = config["amr_params"]["load_weight"].as<double>();
    cfg.amr_params.max_torque = config["amr_params"]["max_torque"].as<double>();
    cfg.amr_params.friction_coeff = config["amr_params"]["friction_coeff"].as<double>();
    cfg.amr_params.max_rpm_deviation = config["amr_params"]["max_rpm_deviation"].as<double>();
    cfg.amr_params.heightMax = config["amr_params"]["heightMax"].as<double>();
    cfg.amr_params.heightMin = config["amr_params"]["heightMin"].as<double>();
    cfg.amr_params.width = config["amr_params"]["width"].as<double>();
    cfg.amr_params.length = config["amr_params"]["length"].as<double>();

    if(config["speedup_ratio"])
    {
        cfg.speedup_ratio = config["speedup_ratio"].as<double>();
    }
    else
    {
        cfg.speedup_ratio  = 1.0;
    }

    cfg.mqtt.visualization_publish_period = config["mqtt"]["visualization_publish_period"].as<double>();
    cfg.mqtt.state_publish_period = config["mqtt"]["state_publish_period"].as<double>();
    cfg.control_period = config["control_period"].as<double>();
    cfg.vehicle_type = config["vehicle_type"].as<std::string>();

    cfg.battery_params.idle_discharge_per_sec = config["battery_params"]["idle_discharge_per_sec"].as<double>();
    cfg.battery_params.acceleration_factor = config["battery_params"]["acceleration_factor"].as<double>();
    cfg.battery_params.angular_slope = config["battery_params"]["angular_slope"].as<double>();
    cfg.battery_params.charge_stop_threshold = config["battery_params"]["charge_stop_threshold"].as<double>();
    cfg.battery_params.linear_slope = config["battery_params"]["linear_slope"].as<double>();
    cfg.battery_params.max_charge_per_sec = config["battery_params"]["max_charge_per_sec"].as<double>();
    
    cfg.mqtt.server_address = config["mqtt"]["server_address"].as<std::string>();
    
    cfg.initial_pose.x = config["initial_pose"]["x"].as<double>();
    cfg.initial_pose.y = config["initial_pose"]["y"].as<double>();
    cfg.initial_pose.heading = config["initial_pose"]["heading"].as<double>();

    return cfg;
}