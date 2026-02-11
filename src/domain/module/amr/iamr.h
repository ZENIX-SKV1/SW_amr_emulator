#pragma once
#include <vector>
#include <string>
#include <iostream>  // 추가: std::cout을 위한 헤더
#include "node_edge_info.h"

struct AmrNode 
{
    std::string id;
    double x;
    double y;
};

struct AmrEdge 
{
    std::string id;
    std::string from;
    std::string to;
};

class IVcu;  // 전방 선언

class IAmr {
public:
    virtual ~IAmr() = default;
    
    // ========================================================================
    // 기존 메서드 (필수)
    // ========================================================================
    virtual void step(double dt, const std::vector<std::pair<double, double>>& other_robot_positions) = 0;
    virtual std::string getState() const = 0;
    virtual std::vector<NodeInfo> getNodes() const = 0;
    virtual std::size_t getCurIdx() const = 0;
    virtual IVcu* getVcu() = 0;
    
    // Order 관리
    virtual void setOrder(const std::vector<NodeInfo>& nodes, 
                         const std::vector<EdgeInfo>& edges, 
                         const std::vector<NodeInfo>& all_nodes, 
                         double wheel_base) = 0;
    virtual void cancelOrder() = 0;
    
    // Node/Edge 완료 추적
    virtual void markNodeAsCompleted(const NodeInfo& node) = 0;
    virtual std::vector<NodeInfo> getCurrentNodes() const = 0;
    virtual std::vector<EdgeInfo> getCurrentEdges() const = 0;
    virtual std::vector<NodeInfo> getCompletedNodes() const = 0;
    virtual std::vector<EdgeInfo> getCompletedEdges() const = 0;
    virtual std::string getLastNodeId() const = 0;
    virtual int getLastNodeSequenceId() const = 0;
    
    // 배터리 관리
    virtual void updateBattery(double dt, bool is_charging) = 0;
    virtual double getBatteryPercent() const = 0;
    
    // VDA5050 표준에서 필요한 배터리 전압 (기본 구현 제공)
    virtual double getBatteryVoltage() const 
    { 
        return 0.0;  // 기본값
    }
    
    // ========================================================================
    // VDA5050 개선을 위한 새로운 메서드 (기본 구현 제공)
    // ========================================================================
    
    /**
     * @brief Order Update 처리 (Merge 방식)
     * 기본 구현: setOrder()를 호출
     */
    virtual void updateOrder(const std::vector<NodeInfo>& nodes, 
                            const std::vector<EdgeInfo>& edges,
                            const std::vector<NodeInfo>& all_nodes,
                            double wheel_base) 
    {
        // 기본 구현: setOrder와 동일하게 처리
        setOrder(nodes, edges, all_nodes, wheel_base);
    }
    
    /**
     * @brief Action 실행
     */
    virtual void executeAction(const ActionInfo& action) 
    {
        // 기본 구현: 로그만 출력
        std::cout << "[IAmr] Action requested: " << action.actionType 
                  << " (ID: " << action.actionId << ")" << std::endl;
    }
    
    /**
     * @brief 일시 정지 상태 확인
     */
    virtual bool isPaused() const 
    { 
        return false;  // 기본값
    }
    
    /**
     * @brief AMR 일시 정지
     */
    virtual void pause() 
    {
        std::cout << "[IAmr] Pause requested" << std::endl;
    }
    
    /**
     * @brief AMR 재개
     */
    virtual void resume() 
    {
        std::cout << "[IAmr] Resume requested" << std::endl;
    }
    
    /**
     * @brief 주행 중인지 확인
     */
    virtual bool isDriving() const 
    {
        return false;  // 기본값
    }
    
    /**
     * @brief 최대 속도 설정
     */
    virtual void setMaxSpeed(double speed) 
    {
        std::cout << "[IAmr] Max speed set to: " << speed << " m/s" << std::endl;
    }
    
    /**
     * @brief 운영 모드 가져오기
     */
    virtual std::string getOperatingMode() const 
    {
        return "AUTOMATIC";  // 기본값
    }
    
    /**
     * @brief 즉시 State 발행 필요 여부 확인
     */
    virtual bool needsImmediateStatePublish() const 
    {
        return false;  // 기본값
    }
    
    /**
     * @brief 즉시 State 발행 플래그 리셋
     */
    virtual void resetImmediateStatePublishFlag() 
    {
        // 기본 구현: 아무것도 하지 않음
    }
};