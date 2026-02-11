#pragma once
#include <string>
struct AmrParams 
{
    double wheel_diameter;
    double wheel_base;
    double angularSpeedMax;
    double angularSpeedMin;
    double wheel_radius;
    double mass_vehicle;      // kg
    double load_weight;       // kg
    double max_torque;        // Nm
    double friction_coeff;    // (0~1)
    double max_speed;         // m/s
    double min_speed;         // m/s
    double accelerationMax;  // m/s^2
    double decelerationMax;  // m/s^2    
    double max_angular_acceleration;  // m/s^2
    double max_angular_deceleration;  // m/s^2
    double max_rpm_deviation;  //rpm noise level(standard deviation)
    double heightMin;
    double heightMax;
    double width;
    double length;
};

struct BatteryParams 
{
    double idle_discharge_per_sec;
    double max_charge_per_sec;
    double charge_stop_threshold;
    double linear_slope;
    double angular_slope;
    double acceleration_factor;
};

struct Mqtt
{
    std::string server_address;
    double visualization_publish_period = 0.05;
    double state_publish_period = 1.0;    
};

struct Pose
{
    double x;
    double y;
    double heading;
};

struct AmrConfig 
{
    int amr_count;
    int base_port;
    std::string protocol_type;   
    std::string vehicle_type;  
    std::string dead_reckoning_model;  
    double speedup_ratio = 1.0;
    double control_period = 0.01;
    AmrParams amr_params;
    BatteryParams battery_params;
    Mqtt mqtt;
    Pose initial_pose;
};

class YamlConfig 
{
public:
    static AmrConfig load(const std::string& filename);
};