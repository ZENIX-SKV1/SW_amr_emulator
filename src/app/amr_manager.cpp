#include "amr_manager.h"
#include "sd_acceleration_model.h" 
#include "dd_acceleration_model.h" 
#include "dd_acceleration_without_dynamics_model.h" 
#include "motor_controller.h"
#include "sd_motor_controller.h"
#include "navigation.h"
#include "vcu.h"
#include "localizer.h"
#include "sd_localizer.h"
#include "dead_reckoning_model_factory.h"
#include "battery_model_simple.h"
#include <iostream>
#include <unistd.h>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

// 생성자: 여러 AMR 인스턴스 및 관련 객체 생성 후 초기화
AmrManager::AmrManager(const AmrConfig& config)
    : config_(config)
{
   for (int i = 0; i < config_.amr_count; ++i)
   {
        int port = config_.base_port + i;

        std::stringstream ss;
        ss << std::setw(4) << std::setfill('0') << i;        
        std::string agv_id = ss.str();

        auto amr = createSingleAmr(i, config_);

        // 초기 위치 설정 (예: YAML config 선언 값 또는 하드코딩)
        // double init_x = 0.0, init_y = 0.0, init_theta = 0.0;
    
        amr->getVcu()->setInitialPose(config_.initial_pose.x, config_.initial_pose.y, config_.initial_pose.heading);

        amrs_.push_back(std::move(amr));

        std::cout << "port : " << port << std::endl;

        auto protocol = createProtocol(config_.protocol_type, config_.mqtt.server_address , agv_id, amrs_.back().get(), config_);
        if (protocol)
            protocols_.push_back(std::move(protocol));

        setupTcpServer(port, i);
   }
}

bool AmrManager::isVda5050OrderMessage(const std::string& msg) 
{
    try 
    {
        nlohmann::json j = nlohmann::json::parse(msg);
        // 필수 필드 존재 + nodes 배열 확인
        return j.contains("headerId")
            && j.contains("timestamp")
            && j.contains("orderId")
            && j.contains("nodes")
            && j["nodes"].is_array()
            && !j["nodes"].empty();
    } 
    catch (const nlohmann::json::parse_error& e) 
    {
        std::cerr << "[isVda5050OrderMessage] JSON parse error: " << e.what() << std::endl;
        return false;
    }
}

bool AmrManager::isCustomTcpProtocolMessage(const std::string& msg) 
{
    try 
    {
        nlohmann::json j = nlohmann::json::parse(msg);
        // Basic check for Custom TCP command fields
        return j.contains("command") && j.contains("agvId");
    } 
    catch (const nlohmann::json::parse_error& e) 
    {
        return false;
    }
}

// 개별 AMR 생성
std::unique_ptr<Amr> AmrManager::createSingleAmr(int id, const AmrConfig& config)
{
    std::unique_ptr<IMotorController> motor;

    if(config.vehicle_type == "steering_drive")
    {
        motor = std::make_unique<SDMotorController>(config);
    }
    else
    {
        motor = std::make_unique<MotorController>(config);
    }

    // 다이나믹스 파라미터 체크
    bool use_dyn_model = (config.amr_params.mass_vehicle > 0 &&
                          config.amr_params.max_torque > 0 &&
                          config.amr_params.accelerationMax > 0 &&
                          config.amr_params.decelerationMax > 0);

    if (use_dyn_model) 
    {
        std::cout << " acc model : acc_model" << std::endl;
        
        auto acc_model = std::make_shared<SDAccelerationModel>(
            config.amr_params.accelerationMax,
            config.amr_params.decelerationMax,
            config.amr_params.max_angular_acceleration,
            config.amr_params.max_angular_deceleration
        );
        motor->setAccelerationModel(acc_model);
    } 
    else
    {
        std::cout << " acc model : acc_model_simple" << std::endl;
        // 다이나믹스 제외한 단순 모델 사용
        auto acc_model_simple = std::make_shared<DDAccelerationWithoutDynamicsModel>(
            config.amr_params.accelerationMax,
            config.amr_params.decelerationMax,
            config.amr_params.max_angular_acceleration,
            config.amr_params.max_angular_deceleration
        );
        motor->setAccelerationModel(acc_model_simple);
    }

    auto navigation = std::make_unique<Navigation>();

    std::shared_ptr<ideadReckoningModel> dr_model;

    try 
    {
        std::cout <<"try dr model : " << config.vehicle_type << std::endl;
        dr_model = DeadReckoningModelFactory::create(config.vehicle_type, config);
    } 
    catch (const std::exception&) 
    {
        std::cout <<"catch dr model : " << config.vehicle_type << std::endl;
        dr_model = DeadReckoningModelFactory::create("differential_drive", config);
    }

    auto localizer = std::make_unique<SDLocalizer>(dr_model);
    auto vcu = std::make_unique<Vcu>(std::move(motor), std::move(navigation), std::move(localizer));
    
    // auto battery_model = std::make_unique<BatteryModelSimple>(100.0, 0.01, 0.5, 95.0);
    auto battery_model = std::make_unique<BatteryModelSimple>(
        100.0,
        config.battery_params.idle_discharge_per_sec,
        config.battery_params.max_charge_per_sec,
        config.battery_params.charge_stop_threshold,
        config.battery_params.linear_slope,
        config.battery_params.angular_slope,
        config.battery_params.acceleration_factor
    );

    return std::make_unique<Amr>(id, std::move(vcu), std::move(battery_model));
}

static std::string makeLogFilePath(const std::string &log_dir, const std::string &agv_id)
{
    using namespace std::chrono;
    auto now   = system_clock::now();
    std::time_t now_c = system_clock::to_time_t(now);

    std::tm local_tm;
#ifdef _WIN32
    localtime_s(&local_tm, &now_c);
#else
    localtime_r(&now_c, &local_tm);
#endif

    std::ostringstream ts;
    // YYYYMMDD_HHMMSS
    ts << std::put_time(&local_tm, "%Y%m%d_%H%M%S");

    std::ostringstream path;
    path << log_dir << "/vda5050_" << agv_id << "_" << ts.str() << ".log";
    return path.str();
}

// 프로토콜 생성 및 초기화
std::unique_ptr<IProtocol> AmrManager::createProtocol(const std::string& protocol_type, const std::string& server_address, const std::string& agv_id, Amr* amr, const AmrConfig& config)
{
    if (protocol_type == "vda5050")
    {
        auto vdaProto = std::make_unique<Vda5050Protocol>(config);
        vdaProto->setAgvId(agv_id);
        vdaProto->useDefaultConfig(server_address);
        vdaProto->setAmr(amr);

        const std::string log_dir  = "logs";
        const std::string log_path = makeLogFilePath(log_dir, agv_id);
        // const std::string log_path = log_dir + "/vda5050_" + agv_id + ".log";
        vdaProto->enableLogging(log_path);

        std::cout << "[AmrManager] AMR " << agv_id << " configured for VDA 5050 protocol. Log: " << log_path << std::endl;
        return vdaProto;
    }
    else if(protocol_type == "custom_tcp") 
    {
        // custom TCP 프로토콜 초기화 필요시 여기 구현
        std::cout << "[AmrManager] AMR " << agv_id << " configured for Custom TCP protocol." << std::endl;
        return nullptr;
    }
    else
    {
        std::cerr << "[AmrManager] Error: Unknown protocol type '" << protocol_type << "' for AMR " << agv_id 
                  << ". Defaulting to Custom TCP." << std::endl;
        return nullptr;
    }
}

// TCP 서버 생성 및 명령 핸들러 등록
void AmrManager::setupTcpServer(int port, int amr_idx)
{
    auto server = std::make_unique<TcpServer>(port);

    server->setCommandHandler([this, amr_idx](const std::string& msg) 
    {
        std::cout << "[TCP Server] Received message on port for AMR " << amr_idx << ":\n" << msg << std::endl;

        if (amr_idx >= protocols_.size()) 
        {
            std::cerr << "[AmrManager] Error: Protocol handler not found for AMR index " << amr_idx << std::endl;
            return;
        }

        IProtocol* currentProtocol = protocols_[amr_idx].get();
        if (!currentProtocol) 
        {
            std::cerr << "[AmrManager] Error: Protocol pointer is null for AMR index " << amr_idx << std::endl;
            return;
        }

        std::cout << "[TCP Received] size: " << msg.size() << ", content:\n" << msg << std::endl;   

        if (currentProtocol->getProtocolType() == "vda5050" && isVda5050OrderMessage(msg))
        {
            std::cout << "[AmrManager] Forwarding VDA5050 Order message to protocol handler for AMR " << amrs_[amr_idx]->getState() << std::endl;
            
            currentProtocol->handleMessage(msg, amrs_[amr_idx].get());
        }
        else if (currentProtocol->getProtocolType() == "custom_tcp" && isCustomTcpProtocolMessage(msg))
        {
            // currentProtocol->handleMessage(msg, amrs_[amr_idx].get());
        }
        else
        {
            std::cerr << "[AmrManager] Error: Mismatch between configured protocol and incoming message for AMR " 
                      << amrs_[amr_idx]->getState() << ". Ignoring message: " << msg.substr(0, std::min((size_t)100, msg.length())) << "..." << std::endl;
        }
    });

    servers_.push_back(std::move(server));
}

void AmrManager::startAll() 
{
    for (auto& proto : protocols_) 
    {
        proto->start();
    }    
    
    for (auto& s : servers_) 
        s->start();
}

void AmrManager::stopAll() 
{
    for (auto& s : servers_) 
        s->stop();
}

size_t AmrManager::getProtocolCount() const 
{
    return protocols_.size();
}

IProtocol* AmrManager::getProtocol(size_t index) const 
{
    if (index < protocols_.size()) 
    {
        return protocols_[index].get();
    }
    return nullptr;
}