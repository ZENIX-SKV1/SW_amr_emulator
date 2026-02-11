#include "sd_motor_controller.h"
#include <algorithm>
#include <cmath>
constexpr double PI = 3.14159265358979323846;

using namespace std;

SDMotorController::SDMotorController(const AmrConfig& config)
    : linear_vel_cmd_(0), angular_vel_cmd_(0),
      linear_vel_actual_(0), angular_vel_actual_(0),
      max_steering_angle_(30.0),
      wheel_base_(config.amr_params.wheel_base), max_speed_(config.amr_params.max_speed), max_angular_speed_(config.amr_params.angularSpeedMax),
      wheel_radius_(config.amr_params.wheel_radius)
{
    cout << "wheel_base_ : " << wheel_base_ << endl;
    cout << "max_speed_ : " << max_speed_ << endl;
    cout << "max_angular_speed_ : " << max_angular_speed_ << endl;
    cout << "wheel_radius_ : " << wheel_radius_ << endl;
}

void SDMotorController::setAccelerationModel(std::shared_ptr<AccelerationModel> model) {
    acceleration_model_ = model;
}

void SDMotorController::setMaxSpeed(double max_speed)
{
    max_speed_ = max_speed;
}

void SDMotorController::setVelocity(double linear_speed, double angular_speed) 
{
    // cout << "[SD_MotorController::setVelocity] linear : " << linear << " angular :" <<angular << endl; 
    linear_speed_cmd_ = linear_speed; 
    angular_speed_cmd_ = angular_speed;
    // 제한 걸기
    linear_speed_cmd_ = std::clamp(linear_speed_cmd_, -max_speed_, max_speed_);    
    // angular_speed_cmd_ = std::clamp(angular_speed_cmd_, -max_steering_angle_, max_steering_angle_);
}

double SDMotorController::getLinearVelocity() const
{
    // std::cout << " SDMotorController::getLinearVelocity(): " << linear_vel_actual_<<std::endl;
    return linear_vel_actual_;
}
double SDMotorController::getAngularVelocity() const
{
    // std::cout << " SDMotorController::getAngularVelocity(): " << angular_vel_actual_<<std::endl;
    return angular_vel_actual_;
}
    

void SDMotorController::update(double dt) 
{
    if (acceleration_model_) 
    {
        // 가감속 모델을 사용하여 실제 속도 업데이트
        linear_vel_actual_ = acceleration_model_->applyAcceleration(linear_vel_actual_, linear_speed_cmd_, dt);
        angular_vel_actual_ = acceleration_model_->applyAngularAcceleration(angular_vel_actual_, angular_speed_cmd_, dt);
    } 
    else 
    {
        // 가감속 모델이 설정되지 않았다면, 목표 속도로 즉시 변경 (디버그 또는 폴백)
        linear_vel_actual_ = linear_speed_cmd_;
        angular_vel_actual_ = angular_speed_cmd_;
        // std::cerr << "Warning: No AccelerationModel set for MotorController. Speeds updated instantly." << std::endl;
    }
    
    // 휠 속도를 RPM으로 변환
    convertWheelSpeedToRPM(linear_vel_actual_, rpm_);
    convertAngularSpeedToSteeringAngle(linear_vel_actual_, angular_vel_actual_, wheel_base_, steering_angular_);
}

void SDMotorController::getRPM(double& front_wheel_rpm, double& front_steering_angle) const 
{
    front_wheel_rpm = rpm_;
    front_steering_angle = steering_angular_;
}

void SDMotorController::convertWheelSpeedToRPM(double wheel_speed, double& rpm) const 
{
    if (wheel_radius_ > 0) 
    {
        rpm = (wheel_speed / (2.0 * M_PI * wheel_radius_)) * 60.0;
    } 
    else
    {
        rpm = 0.0;
        std::cerr << "Error: Wheel radius is zero or negative when converting to RPM." << std::endl;
    }
}

void SDMotorController::convertAngularSpeedToSteeringAngle(double linear_speed, double angular_speed, double wheel_base, double& steering_angle) const 
{
    if (std::abs(linear_speed) > 1e-5) 
    {
        steering_angle = std::atan(wheel_base_ * angular_speed / linear_speed);
    } 
    else 
    {
        steering_angle = 0.0;
    }

    steering_angle = std::clamp(steering_angle, -max_steering_angle_, max_steering_angle_);    
}

