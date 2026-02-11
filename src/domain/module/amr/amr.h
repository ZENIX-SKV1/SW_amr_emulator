#pragma once
#include "iamr.h"
#include "ivcu.h"
#include "vcu.h"
#include "battery_model.h"
#include <vector>
#include <string>
#include <memory>

// 직선 일반형 방정식 계산
struct Line {
    double A, B, C;
};

class Amr : public IAmr 
{
public:
    Amr(int id, std::unique_ptr<Vcu> vcu, std::unique_ptr<IBatteryModel> battery_model);
    
    // ========================================================================
    // 기존 메서드 구현
    // ========================================================================
    std::string getId() const { return "amr_" + std::to_string(id_); }
    
    void setOrder(const std::vector<NodeInfo>& nodes, 
                 const std::vector<EdgeInfo>& edges, 
                 const std::vector<NodeInfo>& all_nodes, 
                 double wheel_base) override;
    
    std::string getState() const override;
    std::vector<NodeInfo> getNodes() const override { return nodes_; }
    std::size_t getCurIdx() const override { return cur_idx_; }
    
    void step(double dt, const std::vector<std::pair<double, double>>& other_robot_positions) override;
    void updateBattery(double dt, bool is_charging) override;
    double getBatteryPercent() const override;
    double getBatteryVoltage() const override;
    
    std::vector<NodeInfo> getCurrentNodes() const override;
    std::vector<NodeInfo> getCompletedNodes() const override;
    std::vector<EdgeInfo> getCurrentEdges() const override;
    std::vector<EdgeInfo> getCompletedEdges() const override;
    std::string getLastNodeId() const override;
    int getLastNodeSequenceId() const override;
    
    void markNodeAsCompleted(const NodeInfo& node) override;
    void cancelOrder() override;
    
    IVcu* getVcu() override;
    
    // ========================================================================
    // VDA5050 개선을 위한 새로운 메서드 구현
    // ========================================================================
    
    /**
     * @brief Order Update 처리 (Merge 방식)
     */
    void updateOrder(const std::vector<NodeInfo>& nodes, 
                    const std::vector<EdgeInfo>& edges,
                    const std::vector<NodeInfo>& all_nodes,
                    double wheel_base) override;
    
    /**
     * @brief Action 실행
     */
    void executeAction(const ActionInfo& action) override;
    
    /**
     * @brief 일시 정지 상태 확인
     */
    bool isPaused() const override { return is_paused_; }
    
    /**
     * @brief AMR 일시 정지
     */
    void pause() override;
    
    /**
     * @brief AMR 재개
     */
    void resume() override;
    
    /**
     * @brief 주행 중인지 확인
     */
    bool isDriving() const override;
    
    /**
     * @brief 최대 속도 설정
     */
    void setMaxSpeed(double speed) override;
    
    /**
     * @brief 운영 모드 가져오기
     */
    std::string getOperatingMode() const override;
    
    /**
     * @brief 즉시 State 발행 필요 여부
     */
    bool needsImmediateStatePublish() const override { 
        return needs_immediate_state_publish_; 
    }
    
    /**
     * @brief 즉시 State 발행 플래그 리셋
     */
    void resetImmediateStatePublishFlag() override { 
        needs_immediate_state_publish_ = false; 
    }

private:
    // ========================================================================
    // 기존 멤버 변수
    // ========================================================================
    int id_;
    bool is_angle_adjusting_ = false;
    bool needs_immediate_state_publish_ = false;
    double wheel_base_;
    
    std::vector<NodeInfo> nodes_;
    std::vector<EdgeInfo> edges_;
    std::size_t cur_idx_ = 0;
    std::size_t cur_edge_idx_ = 0;
    
    std::unique_ptr<Vcu> vcu_;
    std::unique_ptr<IBatteryModel> battery_model_;
    
    std::vector<NodeInfo> completed_nodes_;
    std::vector<EdgeInfo> completed_edges_;
    std::vector<NodeInfo> all_nodes_;
    
    // ========================================================================
    // VDA5050 개선을 위한 새로운 멤버 변수
    // ========================================================================
    bool is_paused_ = false;
    double max_speed_override_ = 0.0;
    std::string operating_mode_ = "AUTOMATIC";
    
    // Action 실행 상태 추적
    struct ExecutingAction {
        ActionInfo info;
        std::string status;
        double start_time;
        double duration;
    };
    std::vector<ExecutingAction> executing_actions_;
    
    // ========================================================================
    // Private 메서드
    // ========================================================================
    Line getLineFromPoints(const NodeInfo& p1, const NodeInfo& p2);
    NodeInfo calculateTangentPoint(const Line& line, const NodeInfo& center, 
                                   double radius, const NodeInfo& ref, bool preferCloser);
    const NodeInfo* findNodeById(const std::vector<NodeInfo>& nodes, const std::string& id);
    void setVcuTargetFromEdge(const EdgeInfo& edge, const std::vector<NodeInfo>& nodes, 
                             const std::vector<NodeInfo>& all_nodes, double wheel_base);
    
    // VDA5050 개선
    void updateExecutingActions(double dt);
    void executePick(const ActionInfo& action);
    void executeDrop(const ActionInfo& action);
    void executeCharge(const ActionInfo& action);
    void executeWait(const ActionInfo& action);
    std::string getActionParameter(const ActionInfo& action, const std::string& key, 
                                   const std::string& default_value = "") const;
};