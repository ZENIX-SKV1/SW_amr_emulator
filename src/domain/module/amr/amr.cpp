//amr.cpp - VDA5050 Enhanced Version
#include "amr.h"
#include <cmath>
#include <iostream>
#include <iomanip>  
#include <algorithm>

Amr::Amr(int id, std::unique_ptr<Vcu> vcu, std::unique_ptr<IBatteryModel> battery_model) 
: id_(id), 
  vcu_(std::move(vcu)), 
  battery_model_(std::move(battery_model)), 
  cur_idx_(0),
  is_paused_(false),
  max_speed_override_(0.0),
  operating_mode_("AUTOMATIC")
{ 
    std::cout << "[AMR" << id_ << "] Initialized with VDA5050 enhancements" << std::endl;
}

// ========================================================================
// 배터리 관리
// ========================================================================

void Amr::updateBattery(double dt, bool is_charging)
{
    double linear_vel = 0.0, angular_vel = 0.0;
    if (vcu_)
    {
        linear_vel = vcu_->getMotor().getLinearVelocity();     
        angular_vel = vcu_->getMotor().getAngularVelocity();   
    }
    battery_model_->update(dt, linear_vel, angular_vel, is_charging);
}

double Amr::getBatteryPercent() const
{
    if (battery_model_)
        return battery_model_->getCapacityPercent();
    return 0.0;
}

double Amr::getBatteryVoltage() const
{
    // IBatteryModel에 getVoltage()가 없는 경우 기본값 반환
    // TODO: IBatteryModel에 getVoltage() 메서드 추가 시 구현
    if (battery_model_)
    {
        // 배터리 용량을 기반으로 전압 추정 (간단한 모델)
        double capacity = battery_model_->getCapacityPercent();
        return 48.0 * (capacity / 100.0);  // 48V 배터리 가정
    }
    return 0.0;
}

// ========================================================================
// Order 관리
// ========================================================================

void Amr::setVcuTargetFromEdge(const EdgeInfo& edge, const std::vector<NodeInfo>& nodes, 
                               const std::vector<NodeInfo>& all_nodes, double wheel_base)
{
    // target_node 찾기
    const NodeInfo* target_node = nullptr;
    const NodeInfo* start_node = nullptr;
    const NodeInfo* center_node = nullptr;

    auto it = std::find_if(nodes.begin(), nodes.end(),
        [&](const NodeInfo& n) { return n.nodeId == edge.endNodeId; });
    if (it != nodes.end())
        target_node = &(*it);

    auto sit = std::find_if(all_nodes.begin(), all_nodes.end(),
        [&](const NodeInfo& n) { return n.nodeId == edge.startNodeId; });        
    if (sit != all_nodes.end())
    {
        start_node = &(*sit);
    }

    // amr.cpp 내의 적절한 위치 (예: 새로운 Edge 진입 시)
    double cur_x, cur_y, cur_theta;
    vcu_->getEstimatedPose(cur_x, cur_y, cur_theta); // 현재 로봇의 실시간 자세 획득    

    if(edge.hasTurnCenter)
    {
        std::cout << "[AMR" << id_ << "] Curved edge: " << edge.edgeId << std::endl;
        std::cout << "  turnCenter: (" << edge.turnCenter.x << ", " << edge.turnCenter.y << ")" << std::endl;
        
        vcu_->setTargetPosition(
            start_node ? start_node->x : 0.0,
            start_node ? start_node->y : 0.0,
            cur_theta,
            target_node ? target_node->x : 0.0,
            target_node ? target_node->y : 0.0,
            edge.turnCenter.x,  
            edge.turnCenter.y,  
            true,  // hasTurnCenter
            wheel_base
        );
    }
    else  // 직선 경로
    {
        std::cout << "[AMR" << id_ << "] Straight edge: " << edge.edgeId << std::endl;
        
        vcu_->setTargetPosition(
            start_node ? start_node->x : 0.0,
            start_node ? start_node->y : 0.0,
            cur_theta,
            target_node ? target_node->x : 0.0,
            target_node ? target_node->y : 0.0,
            0.0, 
            0.0, 
            false,  // hasTurnCenter
            wheel_base
        );
    }    
}



// void Amr::setVcuTargetFromEdge(const EdgeInfo& edge, const std::vector<NodeInfo>& nodes, 
//                                const std::vector<NodeInfo>& all_nodes, double wheel_base)
// {
//     // target_node 찾기
//     const NodeInfo* target_node = nullptr;
//     const NodeInfo* start_node = nullptr;
//     const NodeInfo* center_node = nullptr;

//     auto it = std::find_if(nodes.begin(), nodes.end(),
//         [&](const NodeInfo& n) { return n.nodeId == edge.endNodeId; });
//     if (it != nodes.end())
//         target_node = &(*it);

//     auto sit = std::find_if(all_nodes.begin(), all_nodes.end(),
//         [&](const NodeInfo& n) { return n.nodeId == edge.startNodeId; });        
//     if (sit != all_nodes.end())
//     {
//         start_node = &(*sit);
//     }

//     if(edge.hasTurnCenter)
//     {
//         std::cout << "[AMR" << id_ << "] Curved edge: " << edge.edgeId << std::endl;
//         std::cout << "  turnCenter: (" << edge.turnCenter.x << ", " << edge.turnCenter.y << ")" << std::endl;
        
//         vcu_->setTargetPosition(
//             start_node ? start_node->x : 0.0,
//             start_node ? start_node->y : 0.0,
//             target_node ? target_node->x : 0.0,
//             target_node ? target_node->y : 0.0,
//             edge.turnCenter.x,  
//             edge.turnCenter.y,  
//             true,  // hasTurnCenter
//             wheel_base
//         );
//     }
//     else  // 직선 경로
//     {
//         std::cout << "[AMR" << id_ << "] Straight edge: " << edge.edgeId << std::endl;
        
//         vcu_->setTargetPosition(
//             start_node ? start_node->x : 0.0,
//             start_node ? start_node->y : 0.0,
//             target_node ? target_node->x : 0.0,
//             target_node ? target_node->y : 0.0,
//             0.0, 
//             0.0, 
//             false,  // hasTurnCenter
//             wheel_base
//         );
//     }    
// }

void Amr::setOrder(const std::vector<NodeInfo>& nodes, const std::vector<EdgeInfo>& edges, 
                  const std::vector<NodeInfo>& all_nodes, double wheel_base)
{
    std::cout << "[AMR" << id_ << "] setOrder called - NEW ORDER" << std::endl;
    
    nodes_ = nodes;
    edges_ = edges;
    all_nodes_ = all_nodes; 

    cur_edge_idx_ = 0;
    is_angle_adjusting_ = false;
    wheel_base_ = wheel_base;

    completed_nodes_.clear();
    completed_edges_.clear();
    executing_actions_.clear();  // Action 상태도 초기화

    if (!nodes_.empty())
    {
        // 시작 노드(리스트의 첫 번째)를 완료목록에 즉시 추가
        completed_nodes_.push_back(nodes_.front()); 
    }


    if (!edges_.empty() && vcu_)
    {
        // std::cout << "[AMR" << id_ << "] Starting order with edge: " << edges_[cur_edge_idx_].edgeId << std::endl;
        // std::cout << "[AMR" << id_ << "] Nodes for driving (endNodes): " << nodes_.size() << std::endl;
        // std::cout << "[AMR" << id_ << "] All nodes available (including centerNodes): " << all_nodes_.size() << std::endl;
                
        const EdgeInfo& edge = edges_[cur_edge_idx_];
        setVcuTargetFromEdge(edge, nodes_, all_nodes_, wheel_base);
        
        // 일시 정지 상태 해제
        is_paused_ = false;
    }
}

void Amr::updateOrder(const std::vector<NodeInfo>& nodes, const std::vector<EdgeInfo>& edges,
                     const std::vector<NodeInfo>& all_nodes, double wheel_base)
{
    std::cout << "[AMR" << id_ << "] updateOrder called - ORDER UPDATE (Merge)" << std::endl;
    
    // 현재 진행 중인 edge의 sequenceId 저장
    int current_edge_sequence = -1;
    if (cur_edge_idx_ < edges_.size())
    {
        current_edge_sequence = edges_[cur_edge_idx_].sequenceId;
        std::cout << "[AMR" << id_ << "] Currently processing edge with sequenceId: " 
                  << current_edge_sequence << std::endl;
    }
    
    // 진행 상황 저장
    auto saved_completed_nodes = completed_nodes_;
    auto saved_completed_edges = completed_edges_;
    
    // 새 Order 데이터로 업데이트
    nodes_ = nodes;
    edges_ = edges;
    all_nodes_ = all_nodes;
    wheel_base_ = wheel_base;
    
    // 새 edges에서 현재 처리 중인 edge 찾기
    bool found_current_edge = false;
    for (size_t i = 0; i < edges_.size(); ++i)
    {
        if (edges_[i].sequenceId == current_edge_sequence)
        {
            cur_edge_idx_ = i;
            found_current_edge = true;
            std::cout << "[AMR" << id_ << "] Found current edge in new order at index: " 
                      << i << " (sequenceId: " << current_edge_sequence << ")" << std::endl;
            break;
        }
    }
    
    if (found_current_edge)
    {
        // 진행 상황 복원
        completed_nodes_ = saved_completed_nodes;
        completed_edges_ = saved_completed_edges;
        
        std::cout << "[AMR" << id_ << "] Order updated. Continuing from edge index: " 
                  << cur_edge_idx_ << " (" << edges_[cur_edge_idx_].edgeId << ")" << std::endl;
        
        // VCU 목표는 유지 (이미 주행 중이므로 재설정하지 않음)
        // 만약 재설정이 필요하다면 주석 해제:
        // if (vcu_)
        // {
        //     const EdgeInfo& edge = edges_[cur_edge_idx_];
        //     setVcuTargetFromEdge(edge, nodes_, all_nodes_, wheel_base);
        // }
    }
    else if (current_edge_sequence == -1)
    {
        // 아직 주행 시작 전 (IDLE 상태에서 Order Update 수신)
        std::cout << "[AMR" << id_ << "] Order updated before driving started. Starting from beginning." << std::endl;
        
        cur_edge_idx_ = 0;
        completed_nodes_ = saved_completed_nodes;
        completed_edges_ = saved_completed_edges;
        
        if (!edges_.empty() && vcu_)
        {
            const EdgeInfo& edge = edges_[cur_edge_idx_];
            setVcuTargetFromEdge(edge, nodes_, all_nodes_, wheel_base);
            std::cout << "[AMR" << id_ << "] Set target to first edge: " << edge.edgeId << std::endl;
        }
    }
    else
    {
        // 현재 edge가 새 Order에 없음 - 가장 가까운 edge 찾기
        std::cout << "[AMR" << id_ << "] WARNING: Current edge (seq=" << current_edge_sequence 
                  << ") not found in new order!" << std::endl;
        
        // 현재 sequenceId보다 큰 첫 번째 edge 찾기
        bool found_next_edge = false;
        for (size_t i = 0; i < edges_.size(); ++i)
        {
            if (edges_[i].sequenceId > current_edge_sequence)
            {
                cur_edge_idx_ = i;
                found_next_edge = true;
                std::cout << "[AMR" << id_ << "] Moving to next available edge: " 
                          << edges_[i].edgeId << " (seq=" << edges_[i].sequenceId << ")" << std::endl;
                break;
            }
        }
        
        if (!found_next_edge)
        {
            // 새 Order에 더 이상 진행할 edge가 없음
            std::cout << "[AMR" << id_ << "] No more edges in new order. Order complete." << std::endl;
            nodes_.clear();
            edges_.clear();
            all_nodes_.clear();
            cur_edge_idx_ = 0;
            return;
        }
        
        // 새 edge로 VCU 목표 설정
        completed_nodes_ = saved_completed_nodes;
        completed_edges_ = saved_completed_edges;
        
        if (vcu_)
        {
            const EdgeInfo& edge = edges_[cur_edge_idx_];
            setVcuTargetFromEdge(edge, nodes_, all_nodes_, wheel_base);
        }
    }
    
    std::cout << "[AMR" << id_ << "] updateOrder completed. Total edges: " << edges_.size() 
              << ", Current edge index: " << cur_edge_idx_ << std::endl;
}



void Amr::cancelOrder()
{
    std::cout << "[AMR" << id_ << "] Order cancellation requested" << std::endl;
    
    // 1. 모든 오더 관련 데이터 초기화
    nodes_.clear();
    edges_.clear();
    all_nodes_.clear();
    cur_edge_idx_ = 0;
    cur_idx_ = 0;
    is_angle_adjusting_ = false;
    
    // 2. 완료 리스트도 초기화 (새로운 오더를 위해)
    completed_nodes_.clear();
    completed_edges_.clear();
    executing_actions_.clear();
    
    // 3. 일시 정지 상태 해제
    is_paused_ = false;
    
    // 4. VCU를 Idle 상태로 전환 (즉시 정지)
    if (vcu_)
    {
        double current_x = 0.0, current_y = 0.0, current_theta = 0.0;
        vcu_->getEstimatedPose(current_x, current_y, current_theta);
        
        // 현재 위치를 목표로 설정하여 정지
        vcu_->getNavigation().setTarget(current_x, current_y);
        
        // 모터를 0으로 설정
        vcu_->getMotor().setVelocity(0.0, 0.0);
        
        std::cout << "[AMR" << id_ << "] Motion stopped at position (" 
                  << current_x << ", " << current_y << ")" << std::endl;
    }
    
    //즉시 State 발행 플래그 설정
    needs_immediate_state_publish_ = true;
    
    std::cout << "[AMR" << id_ << "] Order cancelled - AMR is now IDLE" << std::endl;
}

// ========================================================================
// Node/Edge 완료 추적
// ========================================================================

std::vector<NodeInfo> Amr::getCurrentNodes() const
{
    std::vector<NodeInfo> current;
    
    if (edges_.empty() || cur_edge_idx_ >= edges_.size())
        return current;
    
    // 현재 엣지의 endNode를 현재 목표 노드로 간주
    const EdgeInfo& cur_edge = edges_[cur_edge_idx_];
    
    auto it = std::find_if(nodes_.begin(), nodes_.end(),
        [&](const NodeInfo& n) { return n.nodeId == cur_edge.endNodeId; });
    
    if (it != nodes_.end())
    {
        current.push_back(*it);
    }
    
    return current;
}

std::vector<NodeInfo> Amr::getCompletedNodes() const
{
    return completed_nodes_;
}

std::vector<EdgeInfo> Amr::getCurrentEdges() const
{
    std::vector<EdgeInfo> current;
    
    if (cur_edge_idx_ < edges_.size())
    {
        current.push_back(edges_[cur_edge_idx_]);
    }
    
    return current;
}

std::vector<EdgeInfo> Amr::getCompletedEdges() const
{
    return completed_edges_;
}

std::string Amr::getLastNodeId() const
{
    if (completed_nodes_.empty())
        return "";
    
    return completed_nodes_.back().nodeId;
}

int Amr::getLastNodeSequenceId() const
{
    if (completed_nodes_.empty())
        return 0;
    
    return completed_nodes_.back().sequenceId;
}

void Amr::markNodeAsCompleted(const NodeInfo& node)
{
    std::cout << "[AMR" << id_ << "] markNodeAsCompleted called for: '" << node.nodeId 
              << "' (sequenceId: " << node.sequenceId << ")" << std::endl;
    
    // 중복 체크
    for (const auto& completed_node : completed_nodes_)
    {
        if (completed_node.nodeId == node.nodeId)
        {
            std::cout << "[AMR" << id_ << "] Node '" << node.nodeId 
                      << "' already marked as completed. Skipping." << std::endl;
            return;
        }
    }
    
    completed_nodes_.push_back(node);
    
    std::cout << "[AMR" << id_ << "] Node marked as completed. Total completed nodes: " 
              << completed_nodes_.size() << std::endl;
    
    // 즉시 State 발행 플래그 설정
    needs_immediate_state_publish_ = true;
}

// ========================================================================
// VDA5050 개선: Pause/Resume 기능
// ========================================================================

void Amr::pause()
{
    if (is_paused_)
    {
        std::cout << "[AMR" << id_ << "] Already paused" << std::endl;
        return;
    }
    
    is_paused_ = true;
    
    // VCU를 정지 상태로 전환
    if (vcu_)
    {
        vcu_->getMotor().setVelocity(0.0, 0.0);
    }
    
    std::cout << "[AMR" << id_ << "] PAUSED" << std::endl;
    needs_immediate_state_publish_ = true;
}

void Amr::resume()
{
    if (!is_paused_)
    {
        std::cout << "[AMR" << id_ << "] Not paused" << std::endl;
        return;
    }
    
    is_paused_ = false;
    
    std::cout << "[AMR" << id_ << "] RESUMED" << std::endl;
    needs_immediate_state_publish_ = true;
}

bool Amr::isDriving() const
{
    if (!vcu_ || is_paused_ || edges_.empty())
        return false;
    
    // VCU Motor의 속도가 0이 아니면 주행 중
    double linear_vel = vcu_->getMotor().getLinearVelocity();
    double angular_vel = vcu_->getMotor().getAngularVelocity();
    
    return (std::abs(linear_vel) > 0.01 || std::abs(angular_vel) > 0.01);
}

void Amr::setMaxSpeed(double speed)
{
    max_speed_override_ = speed;
    
    std::cout << "[AMR" << id_ << "] Max speed override set to: " << speed << " m/s" << std::endl;
    
    // 현재 주행 중인 경우 즉시 적용
    if (vcu_ && !edges_.empty() && cur_edge_idx_ < edges_.size())
    {
        vcu_->getMotor().setMaxSpeed(speed);
    }
    
    needs_immediate_state_publish_ = true;
}

std::string Amr::getOperatingMode() const
{
    return operating_mode_;
}

// ========================================================================
// VDA5050 개선: Action 처리
// ========================================================================

std::string Amr::getActionParameter(const ActionInfo& action, const std::string& key, 
                                   const std::string& default_value) const
{
    // actionParameters는 nlohmann::json 배열
    if (action.actionParameters.is_array())
    {
        for (const auto& param : action.actionParameters)
        {
            if (param.is_object() && param.contains("key"))
            {
                std::string param_key = param["key"].get<std::string>();
                if (param_key == key)
                {
                    if (param.contains("value"))
                    {
                        // value가 문자열이면 그대로 반환, 아니면 dump()
                        if (param["value"].is_string())
                        {
                            return param["value"].get<std::string>();
                        }
                        else
                        {
                            return param["value"].dump();
                        }
                    }
                }
            }
        }
    }
    return default_value;
}

void Amr::executeAction(const ActionInfo& action)
{
    std::cout << "[AMR" << id_ << "] Executing action: " << action.actionType 
              << " (ID: " << action.actionId << ")" << std::endl;
    
    // Action 타입에 따라 처리
    if (action.actionType == "pick")
    {
        executePick(action);
    }
    else if (action.actionType == "drop")
    {
        executeDrop(action);
    }
    else if (action.actionType == "charge")
    {
        executeCharge(action);
    }
    else if (action.actionType == "wait")
    {
        executeWait(action);
    }
    else
    {
        std::cout << "[AMR" << id_ << "] Unknown action type: " << action.actionType << std::endl;
        
        // 알 수 없는 Action은 즉시 완료로 처리
        ExecutingAction exec_action;
        exec_action.info = action;
        exec_action.status = "FINISHED";
        exec_action.start_time = 0.0;
        exec_action.duration = 0.0;
        executing_actions_.push_back(exec_action);
    }
    
    needs_immediate_state_publish_ = true;
}

void Amr::executePick(const ActionInfo& action)
{
    std::cout << "[AMR" << id_ << "] Executing PICK action" << std::endl;
    
    // 파라미터 파싱
    std::string station_type = getActionParameter(action, "stationType", "");
    std::string load_type = getActionParameter(action, "loadType", "");
    std::string load_id = getActionParameter(action, "loadId", "");
    
    std::cout << "[AMR" << id_ << "]   - stationType: " << station_type << std::endl;
    std::cout << "[AMR" << id_ << "]   - loadType: " << load_type << std::endl;
    std::cout << "[AMR" << id_ << "]   - loadId: " << load_id << std::endl;
    
    // 실제 pick 로직은 하드웨어 인터페이스에 따라 구현
    // 여기서는 시뮬레이션을 위해 일정 시간 대기
    
    ExecutingAction exec_action;
    exec_action.info = action;
    exec_action.status = "RUNNING";
    exec_action.start_time = 0.0;  // step()에서 실제 시간 관리
    exec_action.duration = 30.0;    // 30초 소요
    executing_actions_.push_back(exec_action);
    
    std::cout << "[AMR" << id_ << "] PICK action started (estimated 30s)" << std::endl;
}

void Amr::executeDrop(const ActionInfo& action)
{
    std::cout << "[AMR" << id_ << "] Executing DROP action" << std::endl;
    
    // 파라미터 파싱
    std::string station_type = getActionParameter(action, "stationType", "");
    std::string load_type = getActionParameter(action, "loadType", "");
    std::string load_id = getActionParameter(action, "loadId", "");
    
    std::cout << "[AMR" << id_ << "]   - stationType: " << station_type << std::endl;
    std::cout << "[AMR" << id_ << "]   - loadType: " << load_type << std::endl;
    std::cout << "[AMR" << id_ << "]   - loadId: " << load_id << std::endl;
    
    ExecutingAction exec_action;
    exec_action.info = action;
    exec_action.status = "RUNNING";
    exec_action.start_time = 0.0;
    exec_action.duration = 30.0;    // 30초 소요
    executing_actions_.push_back(exec_action);
    
    std::cout << "[AMR" << id_ << "] DROP action started (estimated 30s)" << std::endl;
}

void Amr::executeCharge(const ActionInfo& action)
{
    std::cout << "[AMR" << id_ << "] Executing CHARGE action" << std::endl;
    
    ExecutingAction exec_action;
    exec_action.info = action;
    exec_action.status = "RUNNING";
    exec_action.start_time = 0.0;
    exec_action.duration = 60.0;    // 60초 충전 (시뮬레이션)
    executing_actions_.push_back(exec_action);
    
    std::cout << "[AMR" << id_ << "] CHARGE action started (estimated 60s)" << std::endl;
}

void Amr::executeWait(const ActionInfo& action)
{
    std::cout << "[AMR" << id_ << "] Executing WAIT action" << std::endl;
    
    // duration 파라미터 파싱
    std::string duration_str = getActionParameter(action, "duration", "10");
    double duration = 10.0;
    try
    {
        duration = std::stod(duration_str);
    }
    catch (...)
    {
        std::cerr << "[AMR" << id_ << "] Invalid duration parameter, using default 10s" << std::endl;
    }
    
    ExecutingAction exec_action;
    exec_action.info = action;
    exec_action.status = "RUNNING";
    exec_action.start_time = 0.0;
    exec_action.duration = duration;
    executing_actions_.push_back(exec_action);
    
    std::cout << "[AMR" << id_ << "] WAIT action started (duration: " << duration << "s)" << std::endl;
}

void Amr::updateExecutingActions(double dt)
{
    for (auto& exec_action : executing_actions_)
    {
        if (exec_action.status == "RUNNING")
        {
            exec_action.start_time += dt;
            
            if (exec_action.start_time >= exec_action.duration)
            {
                exec_action.status = "FINISHED";
                exec_action.start_time = exec_action.duration;
                
                std::cout << "[AMR" << id_ << "] Action FINISHED: " << exec_action.info.actionType 
                          << " (ID: " << exec_action.info.actionId << ")" << std::endl;
                
                needs_immediate_state_publish_ = true;
            }
        }
    }
    
    // 완료된 Action 제거 (일정 시간 후)
    executing_actions_.erase(
        std::remove_if(executing_actions_.begin(), executing_actions_.end(),
            [](const ExecutingAction& a) { 
                return a.status == "FINISHED" && a.start_time > a.duration + 5.0; 
            }),
        executing_actions_.end()
    );
}

// amr.cpp - Part 2: Step function and utilities

// ========================================================================
// 메인 Step 함수 (VDA5050 개선 반영)
// ========================================================================

void Amr::step(double dt, const std::vector<std::pair<double, double>>& other_robot_positions)
{
    constexpr double reach_threshold = 0.01;
    constexpr double angle_threshold = 0.1;
    constexpr double PI = 3.14159265358979323846;
    static double reach_distance_radius = 0.1;
    static double angle_area_radius = 0.1;

    double cur_x, cur_y, cur_theta;
    vcu_->getEstimatedPose(cur_x, cur_y, cur_theta);

    // 1. Action 실행 상태 업데이트
    updateExecutingActions(dt);
    
    // 2. 일시 정지 상태 확인
    if (is_paused_)
    {
        // 일시 정지 중에는 정지 상태 유지
        if (vcu_)
        {
            vcu_->getMotor().setVelocity(0.0, 0.0);
        }
        return;
    }

    // 3. Order가 없으면 Idle
    if (edges_.empty() || cur_edge_idx_ >= edges_.size() || !vcu_)
    {
        vcu_->Idle(dt);
        return;
    }
    
    // 4. VCU 업데이트
    vcu_->update(dt, other_robot_positions);

    // 5. 현재 목표 노드 확인
    const EdgeInfo& cur_edge = edges_[cur_edge_idx_];
    const NodeInfo* target_node = nullptr;

    auto it = std::find_if(nodes_.begin(), nodes_.end(),
        [&](const NodeInfo& n) { return n.nodeId == cur_edge.endNodeId; });
    
    if (it != nodes_.end())
    {
        target_node = &(*it);
    }
    else
    {
        std::cerr << "[AMR" << id_ << "] ERROR: Target node '" << cur_edge.endNodeId 
                  << "' not found for edge '" << cur_edge.edgeId << "'" << std::endl;
        vcu_->Idle(dt);
        return;
    }

    if (!target_node)
    {
        std::cerr << "[AMR" << id_ << "] ERROR: target_node is null" << std::endl;
        vcu_->Idle(dt);
        return;
    }

    // 6. 목표 노드까지 거리 계산
    double dx = target_node->x - cur_x;
    double dy = target_node->y - cur_y;
    double dist = std::hypot(dx, dy);

    // std::cout << target_node->x <<" "<< cur_x << " " << target_node->y <<" "<< cur_y << " " << cur_edge.hasTurnCenter << std::endl;

    // 7. 속도 제한 설정 (VDA5050 개선: 동적 속도 참조)
    double max_speed = (max_speed_override_ > 0.0) ? max_speed_override_ : cur_edge.maxSpeed;
    vcu_->getMotor().setMaxSpeed(max_speed);
    
    // 8. 도착 판정 거리 설정
    if (cur_edge.hasTurnCenter)
    {
        reach_distance_radius = 0.8;
        angle_area_radius = 0.8;
    }
    else
    {
        reach_distance_radius = 0.1;
        angle_area_radius = 0.1;
    }
    
    // 9. 노드 도착 처리
    if (dist < reach_distance_radius)
    {
        std::cout << "[AMR" << id_ << "] Arrived at node: " << target_node->nodeId 
                  << " (current completed: " << completed_nodes_.size() << ")" << std::endl;

        // 엣지를 먼저 추가
        completed_edges_.push_back(cur_edge);
        
        // 중복 체크
        bool is_duplicate = false;
        
        for (size_t i = 0; i < completed_nodes_.size(); ++i)
        {
            if (completed_nodes_[i].nodeId == target_node->nodeId)
            {
                is_duplicate = true;
                break;
            }
        }
        
        // 중복 체크 후 노드 추가
        if (!is_duplicate)
        {
            completed_nodes_.push_back(*target_node);
        }
        
        needs_immediate_state_publish_ = true;
        // 다음 엣지로 이동
        cur_edge_idx_++;
        is_angle_adjusting_ = false;

        // 10. Order 완료 확인
        if (cur_edge_idx_ >= edges_.size())
        {
            std::cout << "[AMR" << id_ << "] All edges completed. Order finished." << std::endl;
            std::cout << "[AMR" << id_ << "] Final stats: " 
                      << completed_nodes_.size() << " nodes, " 
                      << completed_edges_.size() << " edges" << std::endl;
            
            // Order 데이터 초기화
            nodes_.clear();
            edges_.clear();
            all_nodes_.clear();
            cur_edge_idx_ = 0;
            executing_actions_.clear();
            // // 다음 오더를 위해 비움
            // completed_nodes_.clear(); 
            // completed_edges_.clear();             
            
            vcu_->Idle(dt);
            
            needs_immediate_state_publish_ = true;
            
            return;
        }

        // 11. 다음 엣지 시작
        const EdgeInfo& next_edge = edges_[cur_edge_idx_];
        std::cout << "[AMR" << id_ << "] Moving to next edge: " << next_edge.edgeId 
                  << " (idx: " << cur_edge_idx_ << "/" << edges_.size() << ")" << std::endl;
        
        setVcuTargetFromEdge(next_edge, nodes_, all_nodes_, wheel_base_);
    }
}

// ========================================================================
// 유틸리티 함수
// ========================================================================

NodeInfo Amr::calculateTangentPoint(const Line& line, const NodeInfo& center, double radius, 
                                    const NodeInfo& ref, bool preferCloser) 
{
    double A = line.A, B = line.B, C = line.C;
    double cx = center.x, cy = center.y;
    double denom = A*A + B*B;
    double dist = (A*cx + B*cy + C) / sqrt(denom);

    // 중심에서 직선으로 내린 수선의 발
    double x0 = cx - A * (A*cx + B*cy + C) / denom;
    double y0 = cy - B * (A*cx + B*cy + C) / denom;

    double d2 = radius*radius - ((A*cx + B*cy + C)*(A*cx + B*cy + C))/denom;

    NodeInfo pt;

    if(d2 < 0) 
    {
        // 접점 없음(fallback)
        pt.x = x0; pt.y = y0;
    } 
    else 
    {
        double mult = std::sqrt(d2 / denom);

        // 직선의 방향벡터(B, -A)
        double dx = B / sqrt(denom);
        double dy = -A / sqrt(denom);

        NodeInfo p1; 
        p1.x = x0 + mult * dx;
        p1.y = y0 + mult * dy;

        NodeInfo p2; 
        p2.x = x0 - mult * dx;
        p2.y = y0 - mult * dy;

        // ref와 가까운 쪽, 아니면 preferCloser
        double dist1 = hypot(p1.x - ref.x, p1.y - ref.y);
        double dist2 = hypot(p2.x - ref.x, p2.y - ref.y);
        pt = (preferCloser == (dist1 < dist2)) ? p1 : p2;
    }
    return pt;
}

const NodeInfo* Amr::findNodeById(const std::vector<NodeInfo>& nodes, const std::string& id)
{
    auto it = std::find_if(nodes.begin(), nodes.end(), [&](const NodeInfo& n) {
        return n.nodeId == id;
    });
    return (it != nodes.end()) ? &(*it) : nullptr;
}

std::string Amr::getState() const 
{
    if (is_paused_)
    {
        return "AMR" + std::to_string(id_) + ": Paused";
    }
    
    if (nodes_.empty()) 
    {
        return "AMR" + std::to_string(id_) + ": Idle";
    }
    
    if (cur_edge_idx_ < edges_.size())
    {
        return "AMR" + std::to_string(id_) + " driving edge: " + edges_[cur_edge_idx_].edgeId;
    }
    
    return "AMR" + std::to_string(id_) + " at node: " + getLastNodeId();
}

IVcu* Amr::getVcu()
{
    return vcu_.get();
}