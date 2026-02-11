#pragma once
#include "node_edge_info.h"
#include "imotor_controller.h"
#include "inavigation.h"
#include "ilocalizer.h"
#include <vector>

class IVcu 
{
public:
    virtual ~IVcu() = default;
    virtual void setTargetPosition(double start_x, double start_y, double start_theta, double target_x, double target_y, double center_x, double center_y, bool hasTurnCenter, double wheel_base) = 0;
    // virtual void setTargetPosition(double start_x, double start_y, double target_x, double target_y, double center_x, double center_y, bool hasTurnCenter, double wheel_base) = 0;

    virtual void update(double dt, const std::vector<std::pair<double, double>>& other_robot_positions) = 0;

    virtual void Idle(double dt) = 0;

    virtual void updateNodes(const std::vector<NodeInfo>& nodes) = 0;
    virtual void updateEdges(const std::vector<EdgeInfo>& edges) = 0;
    
    virtual void getEstimatedPose(double& x, double& y, double& theta) const = 0;
    virtual void setInitialPose(double x, double y, double theta) = 0;    

    virtual IMotorController& getMotor() = 0;
    virtual INavigation& getNavigation() = 0;
    virtual ILocalizer& getLocalizer() = 0;
};
