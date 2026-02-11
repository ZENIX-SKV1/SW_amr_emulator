#include "navigation.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <iomanip>  

constexpr double PI = 3.14159265358979323846;

Navigation::Navigation() : target_x_(50), target_y_(50) 
{

}

void Navigation::setTarget(double x, double y)
{
    target_x_ = x;
    target_y_ = y;
    use_arc_ = false;
}

void Navigation::setArcTarget(double x, double y, double center_x, double center_y, double radius, double start_angle, double end_angle, bool clockwise)
{
    target_x_ = x;
    target_y_ = y;
    arc_center_x_ = center_x;
    arc_center_y_ = center_y;
    arc_radius_ = radius;
    arc_start_angle_ = start_angle;
    arc_end_angle_ = end_angle;
    arc_clockwise_ = clockwise;
    use_arc_ = true;
}

// void Navigation::update(double current_x, double current_y, double current_theta,
//                         double& out_linear, double& out_angular)
void Navigation::update(double current_x, double current_y, double current_theta,
                        double& out_linear, double& out_angular,
                        const std::vector<std::pair<double, double>>& other_robot_positions)
{
    constexpr double position_reach_threshold = 0.01;   // 위치 도달 임계 거리 (20cm)
    constexpr double angle_reach_threshold = 0.01;     // 회전 도달 임계 각도 (약 3도)

    constexpr double collision_radius = 5.0;            // 충돌 감지 반경 (1m)
    constexpr double slow_down_radius = 15.0;            // 감속 시작 반경 (2m)
    constexpr double max_linear_speed = 10.0;
    constexpr double max_angular_speed = 1.0;

    constexpr double fov_angle = M_PI / 3.0;            // 전방 시야각 (예: 60도 = PI/3)

    double min_dist_in_fov = std::numeric_limits<double>::max();

    // 전방 시야 내 최소 거리 계산
    for (const auto& pos : other_robot_positions)
    {
        double dx = pos.first - current_x;
        double dy = pos.second - current_y;
        double dist = std::sqrt(dx*dx + dy*dy);

        // 로봇 전방 방향 벡터: (cos(current_theta), sin(current_theta))
        // 대상 방향 벡터: (dx, dy)
        // 내적 계산하여 각도 구하기
        double direction_dot = (dx * std::cos(current_theta) + dy * std::sin(current_theta)) / dist;
        double angle_to_obj = std::acos(std::clamp(direction_dot, -1.0, 1.0)); // 대상과 전방 사이 각도

        if (angle_to_obj <= fov_angle / 2.0)
        {
            if (dist < min_dist_in_fov)
            {
                min_dist_in_fov = dist;
            }
        }
    }

    //충돌 및 감속 영역에 따른 속도 비율 계산
    double speed_scale = 1.0;
    // if (min_dist_to_robot < collision_radius)
    if (min_dist_in_fov < collision_radius)
    {
        speed_scale = 0.0;  // 충돌 영역 내 정지
        // std::cout << "[Navigation] Collision imminent! Robot stopped." << std::endl;
    }
    else if (min_dist_in_fov < slow_down_radius)
    {
        speed_scale = ((min_dist_in_fov - collision_radius) / (slow_down_radius - collision_radius)) * 0.1;
        // std::cout << "[Navigation] Slowing down due to nearby robot. Speed scale: " << speed_scale << std::endl;
    }
    else
    {
        speed_scale = 1.0;  // 정상 주행
    }    

    if(!use_arc_)
    {
        double dx = target_x_ - current_x;
        double dy = target_y_ - current_y;
        double distance = std::sqrt(dx * dx + dy * dy);
        
        double target_angle = std::atan2(dy, dx);
        double angle_diff = target_angle - current_theta;
        while (angle_diff > M_PI)  angle_diff -= 2.0 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;

        out_linear = std::clamp(distance, 0.0, 10.0) * speed_scale;
        out_angular = std::clamp(angle_diff, -1.0, 1.0) * speed_scale;

        // std::cout << target_x_ << " " << target_y_ << " " << current_x << " " << current_y << " " << target_angle << " " << current_theta << std::endl;

    }
    else
    {
        // std::cout << "go arc : " << std::endl;
        updateArc(current_x, current_y, current_theta, out_linear, out_angular);
    }

    // std::cout << "navigation : " << target_x_ << " " << target_y_ << " " << current_x << " " << current_y << " " << current_theta << " " << out_linear << " " << out_angular << std::endl;
}

void Navigation::computeTargetControl(double current_x, double current_y, double current_theta,
                                      double speed_scale, double& out_linear, double& out_angular)
{
    double dx = target_x_ - current_x;
    double dy = target_y_ - current_y;
    double distance = std::sqrt(dx * dx + dy * dy);

    double target_angle = std::atan2(dy, dx);
    double angle_diff = target_angle - current_theta;
    while (angle_diff > M_PI)  angle_diff -= 2.0 * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;

    out_linear = std::clamp(distance, 0.0, 10.0) * speed_scale;
    out_angular = std::clamp(angle_diff, -1.0, 1.0) * speed_scale;
}

// double Navigation::calculatePurePursuit(double current_x, double current_y, 
//                                        double current_theta, 
//                                        double target_x, double target_y, 
//                                        double look_ahead_dist)
// {
//     double dx = target_x - current_x;
//     double dy = target_y - current_y;

//     // 로컬 좌표계 변환
//     double local_x = std::cos(current_theta) * dx + std::sin(current_theta) * dy;
//     double local_y = -std::sin(current_theta) * dx + std::cos(current_theta) * dy;

//     // ★ 뒤로 가는 경우 방어 (더 강력하게)
//     if (local_x < 0.1) {
//         std::cout << "[PurePursuit] Target behind robot, forcing forward" << std::endl;
//         local_x = 0.1;
//     }
    
//     // ★ 곡률 계산 (분모에 안전 장치)
//     double denominator = look_ahead_dist * look_ahead_dist;
//     if (denominator < 0.01) {
//         denominator = 0.01;
//     }
    
//     double curvature = (2.0 * local_y) / denominator;
    
//     // ★ 곡률 제한 (너무 급격한 값 방지)
//     double max_curvature = 2.0;  // 최소 0.5m 반경
//     curvature = std::clamp(curvature, -max_curvature, max_curvature);
    
//     return curvature;
// }

double Navigation::calculatePurePursuit(double cx, double cy, double ct, 
                                       double lx, double ly, double ld)
{
    double dx = lx - cx;
    double dy = ly - cy;

    // 로봇 로컬 좌표계 변환
    double local_x = std::cos(ct) * dx + std::sin(ct) * dy;
    double local_y = -std::sin(ct) * dx + std::cos(ct) * dy;

    // [수정 전] if (local_x < 0) { ... 강제 직진 ... } <- 이 부분이 범인입니다.

    // [수정 후] local_x가 음수더라도 곡률 계산을 수행함. 
    // 단, 분모(ld^2)가 0이 되지 않도록 주의
    if (ld < 0.1) ld = 0.1; 
    
    double curvature = (2.0 * local_y) / (ld * ld);

    // 로그에서 본 Clamping 로직이 여기서 동작하도록 유도
    return curvature;
}


// double Navigation::calculatePurePursuit(double current_x, double current_y, double current_theta, 
//                                         double target_x, double target_y, double look_ahead_dist)
// {
//     double dx = target_x - current_x;
//     double dy = target_y - current_y;

//     double local_x = std::cos(current_theta) * dx + std::sin(current_theta) * dy;
//     double local_y = -std::sin(current_theta) * dx + std::cos(current_theta) * dy;

//     // 뒤를 돌아보지 않게만 최소한으로 방어
//     if (local_x < 0.01) local_x = 0.01; 

//     double curvature = (2.0 * local_y) / (look_ahead_dist * look_ahead_dist);
//     return curvature;
// }

// void Navigation::updateArc(double current_x, double current_y, double current_theta,
//                            double& out_linear, double& out_angular)
// {
//     const double arc_follow_speed = 2.0;  // 원호 주행 선속도 예시

//     // 1.Look-ahead 설정 (속도에 비례하도록 설정하거나 고정값 사용)
//     // 1m/s 저속 주행 시에는 0.8m ~ 1.2m 정도가 적당합니다.
//     double look_ahead_dist = 1.0; 

//     // 2.현재 로봇 위치에서 원 궤적 위의 Look-ahead 포인트 계산
//     // 현재 로봇이 원의 중심으로부터 몇 도 위치에 있는지 계산
//     double current_angle = std::atan2(current_y - arc_center_y_, current_x - arc_center_x_);
    
//     // 호의 길이 s = r * theta 이므로, theta = s / r
//     double angle_step = look_ahead_dist / arc_radius_;
    
//     // 진행 방향에 따른 목표 각도 설정
//     double target_angle = arc_clockwise_ ? (current_angle - angle_step) : (current_angle + angle_step);

//     // 궤적 위의 실제 목표 좌표(Look-ahead Point)
//     double lx = arc_center_x_ + arc_radius_ * std::cos(target_angle);
//     double ly = arc_center_y_ + arc_radius_ * std::sin(target_angle);

//     // 3.별도 함수를 통해 곡률 계산
//     double curvature = calculatePurePursuit(current_x, current_y, current_theta, lx, ly, look_ahead_dist);

//     // 4.최종 제어 값 출력
//     out_linear = arc_follow_speed; // 설정된 선속도 (1.0m/s 등)
//     out_angular = out_linear * curvature; // w = v * k

//     // [선택 사항] 각속도 제한 (급격한 회전 방지)
//     double max_angular = 1.5; 
//     out_angular = std::clamp(out_angular, -max_angular, max_angular);
// }

void Navigation::updateArc(double current_x, double current_y, double current_theta,
                           double& out_linear, double& out_angular)
{
    // ★ 1. 반경에 비례한 동적 look-ahead 거리
    double look_ahead_dist = std::clamp(arc_radius_ * 0.3, 0.5, 2.0);
    
    // ★ 2. 반경에 따른 속도 조정
    double arc_follow_speed = 2.0;
    if (arc_radius_ < 10.0) {
        arc_follow_speed = std::max(1.0, arc_radius_ * 0.2);
    }

    // 3. 현재 로봇 위치에서 원 궤적까지의 오차 계산
    double dx_to_center = current_x - arc_center_x_;
    double dy_to_center = current_y - arc_center_y_;
    double dist_to_center = std::sqrt(dx_to_center*dx_to_center + dy_to_center*dy_to_center);
    double radius_error = arc_radius_ - dist_to_center;
    
    // ★ 4. 반경 오차가 크면 직접 복귀 제어 (Pure Pursuit 대신)
    if (std::abs(radius_error) > arc_radius_ * 0.3) {
        std::cout << "[Navigation] Large radius error: " << radius_error 
                  << "m. Applying correction mode." << std::endl;
        
        // 원 위로 복귀하는 제어
        double angle_to_center = std::atan2(dy_to_center, dx_to_center);
        double target_angle = angle_to_center + (radius_error > 0 ? M_PI : 0);
        
        double angle_diff = target_angle - current_theta;
        while (angle_diff > M_PI)  angle_diff -= 2.0 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;
        
        out_linear = std::min(1.0, std::abs(radius_error));
        out_angular = std::clamp(angle_diff * 2.0, -1.0, 1.0);
        return;
    }

    // 5. Pure Pursuit 계산
    double current_angle = std::atan2(current_y - arc_center_y_, current_x - arc_center_x_);
    double angle_step = look_ahead_dist / arc_radius_;

    double target_angle = arc_clockwise_ ? (current_angle - angle_step) : (current_angle + angle_step);
    // std::cout << "[updateArc] target_angle : " << target_angle << std::endl;


    // ★ 추가: target_angle이 불연속 구간을 넘지 않도록 보정
    double angle_to_target = target_angle - current_angle;
    while (angle_to_target > M_PI) angle_to_target -= 2.0 * M_PI;
    while (angle_to_target < -M_PI) angle_to_target += 2.0 * M_PI;
    target_angle = current_angle + angle_to_target;

    double lx = arc_center_x_ + arc_radius_ * std::cos(target_angle);
    double ly = arc_center_y_ + arc_radius_ * std::sin(target_angle);

    double curvature = calculatePurePursuit(current_x, current_y, current_theta, lx, ly, look_ahead_dist);

    // 6. 곡률 제한 (발산 방지)
    double max_curvature = 1.0 / arc_radius_ * 1.5;  // 이론 곡률의 1.5배까지만
    if (std::abs(curvature) > max_curvature) {
        std::cout << "[Navigation] Curvature clamped: " << curvature 
                  << " -> " << std::copysign(max_curvature, curvature) << std::endl;
        curvature = std::copysign(max_curvature, curvature);
    }
    // double max_curvature = 1.0 / arc_radius_ * 5.0;  // 이론 곡률의 1.5배까지만

    out_linear = arc_follow_speed;
    out_angular = out_linear * curvature;

    // ★ 7. 각속도 최종 제한
    double max_angular = std::min(1.5, arc_follow_speed / arc_radius_ * 2.0);
    out_angular = std::clamp(out_angular, -max_angular, max_angular);
    
    // ★ 8. 디버그 출력
    static int debug_counter = 0;
    // if (++debug_counter % 10 == 0) {
    //     std::cout << std::fixed << std::setprecision(2)
    //               << "[Arc] R=" << arc_radius_ 
    //               << " err=" << radius_error
    //               << " LA=" << look_ahead_dist
    //               << " k=" << curvature 
    //               << " v=" << out_linear 
    //               << " w=" << out_angular << std::endl;
    // }
}

// void Navigation::updateArc(double current_x, double current_y, double current_theta, double& out_linear, double& out_angular)
// {
//     const double arc_follow_speed = 2.0;  // 원호 주행 선속도 예시

//     // 원호 중심과 현재 위치 거리
//     double dx = current_x - arc_center_x_;
//     double dy = current_y - arc_center_y_;
//     double distance_to_center = std::sqrt(dx*dx + dy*dy);

//     // 원호 반경과 현재 거리 오차
//     double radius_error = arc_radius_ - distance_to_center;

//     // 현재 위치 각도 w.r.t 원호 중심
//     double pos_angle = std::atan2(dy, dx);

//     // 목표 각도 진행 계산 (시계방향 or 반시계방향)
//     double angle_diff = arc_end_angle_ - pos_angle;

//     if (arc_clockwise_) 
//     {
//         if (angle_diff > 0)
//             angle_diff -= 2.0 * M_PI;
//     } 
//     else 
//     {
//         if (angle_diff < 0)
//             angle_diff += 2.0 * M_PI;
//     }

//     // const double angle_threshold = 0.05; // 3도 정도 도달 허용
//     // if (std::abs(angle_diff) < angle_threshold) 
//     // {
//     //     // 원호 끝점 도달 시 원호 모드 종료 (직선 모드 전환 or 정지)
//     //     use_arc_ = false;
//     //     out_linear = 0.0;
//     //     out_angular = 0.0;
//     //     return;
//     // }

//     // 조향은 원호의 중심 방향으로 회전: 원호 반경과 각속도 관계
//     // 각속도 w = v / r
//     double linear_speed = arc_follow_speed;
//     double angular_speed = linear_speed / arc_radius_;

//     if (arc_clockwise_)
//         angular_speed = -angular_speed;

//     // radius_error 등을 반영해 약간 보정 
//     // radius_error가 크면 속도 줄이거나 각속도 조정
//     applyRadiusErrorCorrection(radius_error, linear_speed, angular_speed);

//     out_linear = linear_speed;
//     out_angular = angular_speed;

//     // std::cout << "cx: " << current_x << " acx: " << arc_center_x_
//     // << " cy: " << current_y << " acy: " << arc_center_y_
//     // <<  " dtc: " << distance_to_center << " re: " << radius_error
//     // << " ad: " << angle_diff << " ls: " << linear_speed << " as: " << angular_speed << std::endl;
// }

// void Navigation::Idle((double current_x, double current_y, double current_theta)
// {
//     target_x_ = current_x;
//     target_y_ = current_y;
//     target_t
// }


void Navigation::applyRadiusErrorCorrection(double radius_error, double& linear_speed, double& angular_speed)
{
    const double max_radius_error = 1.5;
    const double max_speed_reduction = 0.5;
    const double max_angular_increase = 0.5;

    double error_ratio = std::min(std::abs(radius_error) / max_radius_error, 1.0);

    // 선속도 보정
    linear_speed *= (1.0 - max_speed_reduction * error_ratio);

    // 각속도 보정
    // double correction_factor = 1.0 + (max_angular_increase - 1.0) * error_ratio;
    double correction_factor = 1.0 + (max_angular_increase - 1.0) * error_ratio;
    angular_speed *= correction_factor;
}
