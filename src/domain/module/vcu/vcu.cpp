// vcu.cpp 
#include "vcu.h"
#include <iostream> 
#include <cmath>

Vcu::Vcu(std::unique_ptr<IMotorController> motor, std::unique_ptr<INavigation> nav, std::unique_ptr<ILocalizer> localizer)
    : motor_(std::move(motor)), navigation_(std::move(nav)), localizer_(std::move(localizer)), target_x_(0), target_y_(0) 
{
}

// void Vcu::setTargetPosition(double start_x, double start_y, double target_x, double target_y, double center_x, double center_y, bool hasTurnCenter, double wheel_base)
// {
//     target_x_ = target_x;
//     target_y_ = target_y;

//     if(!hasTurnCenter)
//     {
//         navigation_->setTarget(target_x_, target_y_);
//     }
//     else
//     {
//         // std::cout << "[VCU] Target ARC position set to (" << target_x_ << ", " << target_y_ << "), theta=" << target_theta_ << 
//         // "cx : " << center_x << "cy : " << center_y << "hasTurnCenter : " << hasTurnCenter <<std::endl;
//         double radius = std::hypot(target_x - center_x, target_y - center_y);
        
//         // double radius = 27.1;
//         double start_angle = std::atan2(start_y - center_y, start_x - center_x);
//         double end_angle = std::atan2(target_y - center_y, target_x - center_x);

//         // bool clockwise = false;
//         // 외적을 이용한 CW / CCW 판별
//         // 중심점에서 시작점을 향하는 벡터 (v1)와 중심점에서 끝점을 향하는 벡터 (v2)
//         double v1x = start_x - center_x;
//         double v1y = start_y - center_y;
//         double v2x = target_x - center_x;
//         double v2y = target_y - center_y;

//         // 2D 외적: x1*y2 - y1*x2
//         double cross_product = v1x * v2y - v1y * v2x;

//         // cross_product > 0 이면 CCW(좌회전), < 0 이면 CW(우회전)
//         bool clockwise = (cross_product < 0);        
//         // std::cout << " radius "<< radius << " " << "start_angle : " << start_angle << " " << "end_angle : " << end_angle << std::endl;

//         navigation_->setArcTarget(target_x_, target_y_, center_x, center_y, radius, start_angle, end_angle, clockwise);
//     }
// }

void Vcu::setTargetPosition(double start_x, double start_y, double start_theta, double target_x, double target_y, double center_x, double center_y, bool hasTurnCenter, double wheel_base)
{
    target_x_ = target_x;
    target_y_ = target_y;

    if(!hasTurnCenter) 
    {
        navigation_->setTarget(target_x_, target_y_);
    }
    else 
    {
        double radius = std::hypot(target_x - center_x, target_y - center_y);
        double start_angle = std::atan2(start_y - center_y, start_x - center_x);
        double end_angle = std::atan2(target_y - center_y, target_x - center_x);

        // // 벡터 기반 외적 계산 (더 안전한 방식)
        // double v1x = start_x - center_x;
        // double v1y = start_y - center_y;
        // double v2x = target_x - center_x;
        // double v2y = target_y - center_y;

        // // 외적이 0에 너무 가까우면(직선에 가까운 호) 이전 방향 유지 혹은 기본값
        // double cross_product = v1x * v2y - v1y * v2x;
        
        // // 0보다 작으면 시계방향(CW, 우회전), 크면 반시계방향(CCW, 좌회전)
        // bool clockwise = (cross_product < 0);

        // // [디버그용] 방향 판별 로그 추가
        // std::cout << "[VCU] Arc Direction: " << (clockwise ? "RIGHT (CW)" : "LEFT (CCW)") 
        //           << " CrossProduct: " << cross_product << std::endl;

// 외적 대신 '중심점이 로봇의 어느 쪽에 있는가'로 판별 (더 확실함)
        double dx = center_x - start_x;
        double dy = center_y - start_y;
        
        // 로봇 헤딩 기준 로컬 좌표계로 중심점 변환
        double local_center_y = -std::sin(start_theta) * dx + std::cos(start_theta) * dy;
        
        // 중심점이 오른쪽에 있으면 좌회전(CCW), 왼쪽에 있으면 우회전(CW)
        // (Ackermann 기하학 기준)
        // bool clockwise = (local_center_y > 0); 
        bool clockwise = (local_center_y < 0); 

        std::cout << "[DEBUG] StartPose: (" << start_x << ", " << start_y << ", " << start_theta << ")" << std::endl;
        std::cout << "[DEBUG] LocalCenterY: " << local_center_y << " -> IsCW: " << (clockwise ? "YES" : "NO") << std::endl;

        std::cout << "[VCU] Corrected Direction: " << (clockwise ? "RIGHT (CW)" : "LEFT (CCW)") << std::endl;        

        navigation_->setArcTarget(target_x_, target_y_, center_x, center_y, radius, start_angle, end_angle, clockwise);
    }
}

void Vcu::update(double dt, const std::vector<std::pair<double, double>>& other_robot_positions)
{
    double left_rpm, right_rpm;
    motor_->getRPM(left_rpm, right_rpm);

    localizer_->update(left_rpm, right_rpm, dt);
    
    double current_x = 0.0, current_y = 0.0, current_theta = 0.0;
    localizer_->getPose(current_x, current_y, current_theta);

    double linear_vel_cmd = 0.0, angular_vel_cmd = 0.0;
    navigation_->update(current_x, current_y, current_theta, linear_vel_cmd, angular_vel_cmd, other_robot_positions);
    motor_->setVelocity(linear_vel_cmd, angular_vel_cmd);
    motor_->update(dt);
}

void Vcu::Idle(double dt)
{

    double current_x = 0.0, current_y = 0.0, current_theta = 0.0;
    localizer_->getPose(current_x, current_y, current_theta);
    navigation_->setTarget(current_x, current_y);
    motor_->setVelocity(0.0, 0.0);
    motor_->update(dt);
    // std::cout << "Vcu::Idle()" << current_x << " " << current_y << " " << current_theta << std::endl;
}

void Vcu::setInitialPose(double x, double y, double theta)
{
    if (localizer_)
    {
        localizer_->setInitialPose(x, y, theta);
    }
}

void Vcu::getEstimatedPose(double& x, double& y, double& theta) const
{
    if (localizer_)
    {
        localizer_->getPose(x, y, theta);
    }
    else
    {
        x = y = theta = 0.0;
    }
}

IMotorController& Vcu::getMotor() 
{ 
    return *motor_; 
}

INavigation& Vcu::getNavigation() 
{ 
    return *navigation_; 
}

ILocalizer& Vcu::getLocalizer()
{
    return *localizer_;
}

void Vcu::updateNodes(const std::vector<NodeInfo>& nodes) 
{
    std::cout << "[VCU] updateNodes called with " << nodes.size() << " nodes." << std::endl;
}

void Vcu::updateEdges(const std::vector<EdgeInfo>& edges) 
{
    std::cout << "[VCU] updateEdges called with " << edges.size() << " edges." << std::endl;
    
    for (const auto& edge : edges) 
    {
        std::cout << " - EdgeId: " << edge.edgeId 
                  << ", StartNodeId: " << edge.startNodeId 
                  << ", EndNodeId: " << edge.endNodeId << std::endl;
    }
}

