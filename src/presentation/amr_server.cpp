#include "amr_server.h"
#include "yaml_config.h"
#include "amr_manager.h" // AmrManager 클래스의 올바른 경로로 변경
#include <thread>
#include <chrono>
#include <iostream>


void AmrServerApp::run(const std::string& config_path)
{
    AmrConfig config = YamlConfig::load(config_path);
    AmrManager manager(config);
    manager.startAll();

    const double speedup = config.speedup_ratio;
    const double dt_control = config.control_period;       // 내부 제어 주기(10ms)
    const double dt_state = config.mqtt.state_publish_period;           // state topic (1초)
    const double dt_vis = config.mqtt.visualization_publish_period;            // visualization topic (50ms)
    const double dt_master = 0.01;        // 최소 단위 루프(가급적 작게)

    double sim_time = 0.0;
    double next_motor_update = 0.0;
    double next_state_pub = 0.0;
    double next_vis_pub = 0.0;

    std::cout << "[AmrServer] Starting with speedup_ratio: " << speedup << std::endl;
    
    auto start_time = std::chrono::high_resolution_clock::now();
    auto last_time = start_time;

    while (true)
    {
        sim_time += dt_master * speedup;
        auto& amrs = manager.getAmrs();
       
        // 내부 제어: dt_internal 마다 호출
        if (sim_time >= next_motor_update) 
        {
            // 모든 AMR 위치 수집
            std::vector<std::pair<double, double>> all_positions;
            for (auto& amr : amrs)
            {
                double x=0, y=0, theta=0;
                amr->getVcu()->getEstimatedPose(x, y, theta);
                all_positions.emplace_back(x, y);
            }

            size_t idx = 0;
            for (auto& amr : amrs)
            {
                std::vector<std::pair<double, double>> other_positions;
                other_positions.reserve(all_positions.size() - 1);

                for (size_t i = 0; i < all_positions.size(); ++i)
                {
                    if (i != idx)
                    {
                        other_positions.push_back(all_positions[i]);
                    }
                }

                bool is_charging = false;  // 필요시 충전 상태 로직 구현
                amr->updateBattery(dt_control, is_charging);
                
                amr->step(dt_control, other_positions);

                // 노드 도착 이벤트 확인 및 즉시 상태 발행
                if (amr->needsImmediateStatePublish())
                {
                    if (idx < manager.getProtocolCount()) 
                    {
                        auto* protocol = manager.getProtocol(idx);
                        if (protocol) 
                        {
                            protocol->publishStateMessage(amr.get());
                            std::cout << "[AmrServer] Immediate state published for AMR" << amr->getId() << " after node arrival." << std::endl;
                            // 발행 후 플래그를 리셋
                            amr->resetImmediateStatePublishFlag();
                        }
                    }
                }                
                ++idx;                
            }
            next_motor_update += dt_control;
        }

        if (sim_time >= next_state_pub) 
        {
            for (size_t i = 0; i < amrs.size(); ++i) 
            {
                if (i < manager.getProtocolCount()) 
                {
                    auto* protocol = manager.getProtocol(i);
                    if (protocol) 
                    {
                        // std::cout << "publishStateMessage" << std::endl;
                        protocol->publishStateMessage(amrs[i].get());
                    }
                }
            }
            next_state_pub += dt_state;
        }

        // visualization topic publish: dt_vis 마다 호출
        if (sim_time >= next_vis_pub && dt_vis > 0.0) 
        {
            for (size_t i = 0; i < amrs.size(); ++i) 
            {
                if (i < manager.getProtocolCount()) 
                {
                    auto* protocol = manager.getProtocol(i);
                    if (protocol)
                    {
                        // std::cout << "publishVisualizationMessage : " << i << std::endl;
                        protocol->publishVisualizationMessage(amrs[i].get());
                    }
                }
            }
            next_vis_pub += dt_vis;
        }
        
        std::this_thread::sleep_for(std::chrono::duration<double>(dt_master / speedup));
        // 실제 sleep 시간은 speedup 반영 (dt_master만큼 시뮬 타임 진행)
    }
}



