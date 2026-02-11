#include "imotor_controller.h"
#include "motor_controller.h"
#include "dd_acceleration_model.h"
#include <iostream>
#include <memory>
#include <iomanip>
#include "yaml_config.h"

#define STEP_COUNT  50

int main() 
{
    std::cout << "--- MotorController Test with DDAccelerationModel ---" << std::endl;
    std::string config_path = "../config/amr_params.yaml";

    //AmrConfig 파라미터설정(실제로는 YAML 파일에서 로드)
    AmrConfig config = YamlConfig::load(config_path);

     std:: cout << "mass_vehicle "<< config.amr_params.mass_vehicle << std::endl;
    std:: cout << "load_weight "<< config.amr_params.load_weight << std::endl;
    std:: cout << "max_torque "<< config.amr_params.max_torque << std::endl;
    std:: cout << "friction_coeff "<< config.amr_params.friction_coeff << std::endl;
    std:: cout << "max_speed "<< config.amr_params.max_speed << std::endl;
    // std:: cout << "max_acceleration "<< config.amr_params.max_acceleration << std::endl;
    // std:: cout << "max_deceleration "<< config.amr_params.max_deceleration << std::endl;
    // std:: cout << "wheel_radius "<< config.amr_params.wheel_radius << std::endl;
    std:: cout << "max_angular_acceleration "<< config.amr_params.max_angular_acceleration << std::endl;
    std:: cout << "max_angular_deceleration "<< config.amr_params.max_angular_deceleration << std::endl;

    // DDAccelerationModel 인스턴스 생성
    std::shared_ptr<AccelerationModel> dd_model = std::make_shared<DDAccelerationModel>(
        config.amr_params.mass_vehicle,
        config.amr_params.load_weight,
        config.amr_params.max_torque,
        config.amr_params.friction_coeff,
        config.amr_params.max_speed, 
        // config.amr_params.max_acceleration,
        // config.amr_params.max_deceleration,
        // config.amr_params.wheel_radius, 
        config.amr_params.max_angular_acceleration,
        config.amr_params.max_angular_deceleration
    );  

    std::cout << "DDAccelerationModel created with parameters." << std::endl;

    // MotorController 인스턴스 생성
    MotorController motor_controller(config);
    std::cout << "MotorController created with config." << std::endl;

    // MotorController에 가감속 모델 설정 (주입)
    motor_controller.setAccelerationModel(dd_model);
    std::cout << "AccelerationModel set to MotorController." << std::endl;

    // 시뮬레이션 루프
    double dt = 0.05; 
    int total_steps = 100; 

    std::cout << "\n--- Scenario 1: Accelerating forward to 1.5 m/s, 0.0 rad/s ---" << std::endl;
    motor_controller.setVelocity(5.5, 0.0); 

    for (int i = 0; i < total_steps; ++i) {
        motor_controller.update(dt); 
    }

    std::cout << "\n--- Scenario 1: Accelerating forward to 0.5 m/s, 0.0 rad/s ---" << std::endl;
    motor_controller.setVelocity(0.5, 0.0); 

    for (int i = 0; i < total_steps; ++i) {
        motor_controller.update(dt); 
    }

    std::cout << "\n--- Test Completed ---" << std::endl;

    return 0;
}